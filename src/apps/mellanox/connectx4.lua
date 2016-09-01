--go@ git up
--- Device driver for the Mellanox ConnectX-4 series Ethernet controller.

-- This driver is written using these main reference sources:
-- 
--   PRM: Mellanox Adapter Programmer's Reference Manual
--        This document will be made available on Mellanox's website.
--        Has not happened yet (as of 2016-05-24).
--
--   mlx5_core: Linux kernel driver for ConnectX-4. This has been
--        developed by Mellanox.
--
--   Hexdumps: The Linux kernel driver has the capability to run in
--        debug mode and to output hexdumps showing the exact
--        interactions with the card. This driver has a similar
--        capability. This makes it possible to directly compare
--        driver behavior directly via hexdumps i.e. independently of
--        the source code.

-- Implementation notes:
--
--   RESET: This driver performs a PCIe reset of the device prior to
--        initialization. This is instead of performing the software
--        deinitialization procedure. The main reason for this is
--        simplicity and keeping the code minimal.
--
--        Relatedly, reloading the mlx5_core driver in Linux 4.4.8
--        does not seem to consistently succeed in reinitializing the
--        device. This may be due to bugs in the driver and/or firmware. 
--        Skipping the soft-reset would seem to reduce our driver's
--        exposure to such problems.
--
--        In the future we could consider implementing the software
--        reset if this is found to be important for some purpose.
--
--   SIGNATURE:
--        Command signatures fields: Are they useful? Are they used?
--
--        Usefulness - command signature is an 8-bit value calculated
--        with a simple xor. What does this protect and how effective
--        is it? Curious because PCIe is already performing a more
--        robust checksum. Perhaps the signature is designed to catch
--        driver bugs? Or host memory corruption? Enquiring minds
--        would like to know...
--
--        Used - the Linux driver has code for signatures but seems to
--        hard-code this as disabled at least in certain instances.
--        Likewise the card is accepting at least some commands from
--        this driver without signatures. It seems potentially futile
--        to calculate and include command signatures if they are not
--        actually being verified by the device.
--
--   DRIVER VERSION: This driver does /not/ identify itself via the
--        command SET_DRIVER_VERSION. That interation could lead to
--        hazards in the spirit of HTTP User-Agent where the adapter
--        firmware would behave differently depending on how the
--        driver identifies itself.
--
--        This decision could be revisited in the future when the
--        motivation for this mechanism is better understood. (The
--        card and firmware being used for initial development is not
--        asking the driver to identify itself anyway.)

local qsize = tonumber(os.getenv("SNABB_QSIZE"))
local log_qsize = math.log(qsize) / math.log(2)

module(...,package.seeall)

local ffi      = require "ffi"
local C        = ffi.C
local lib      = require("core.lib")
local pci      = require("lib.hardware.pci")
local register = require("lib.hardware.register")
local index_set = require("lib.index_set")
local macaddress = require("lib.macaddress")
local mib = require("lib.ipc.shmem.mib")
local timer = require("core.timer")
local bits, bitset = lib.bits, lib.bitset
local floor = math.floor
local cast = ffi.cast
local band, bor, shl, shr, bswap, bnot =
   bit.band, bit.bor, bit.lshift, bit.rshift, bit.bswap, bit.bnot

local debug = true

ConnectX4 = {}
ConnectX4.__index = ConnectX4

--utils

local function alloc_pages(pages)
   local ptr, phy = memory.dma_alloc(4096 * pages, 4096)
   assert(band(phy, 0xfff) == 0) --the phy address must be 4K-aligned
   return cast('uint32_t*', ptr), phy
end

function getint(addr, ofs)
   local ofs = ofs/4
   assert(ofs == floor(ofs))
   return bswap(addr[ofs])
end

function setint(addr, ofs, val)
   local ofs = ofs/4
   assert(ofs == floor(ofs))
   addr[ofs] = bswap(tonumber(val))
end

local function getbits(val, bit2, bit1)
   local mask = shl(2^(bit2-bit1+1)-1, bit1)
   return shr(band(val, mask), bit1)
end

local function ptrbits(ptr, bit2, bit1)
   local addr = cast('uint64_t', ptr)
   return tonumber(getbits(addr, bit2, bit1))
end

local function setbits1(bit2, bit1, val)
   local mask = shl(2^(bit2-bit1+1)-1, bit1)
   local bits = band(shl(val, bit1), mask)
   return bits
end

local function setbits(...) --bit2, bit1, val, ...
   local endval = 0
   for i = 1, select('#', ...), 3 do
      local bit2, bit1, val = select(i, ...)
      endval = bor(endval, setbits1(bit2, bit1, val or 0))
   end
   return endval
end


--init segment (section 4.3)

local init_seg = {}
init_seg.__index = init_seg

function init_seg:getbits(ofs, bit2, bit1)
   return getbits(getint(self.ptr, ofs), bit2, bit1)
end

function init_seg:setbits(ofs, ...)
   setint(self.ptr, ofs, setbits(...))
end

function init_seg:init(ptr)
   return setmetatable({ptr = cast('uint32_t*', ptr)}, self)
end

function init_seg:fw_rev() --maj, min, subminor
   return
      self:getbits(0, 15, 0),
      self:getbits(0, 31, 16),
      self:getbits(4, 15, 0)
end

function init_seg:cmd_interface_rev()
   return self:getbits(4, 31, 16)
end

function init_seg:cmdq_phy_addr(addr)
   if addr then
      --must write the MSB of the addr first
      self:setbits(0x10, 31, 0, ptrbits(addr, 63, 32))
      --also resets nic_interface and log_cmdq_*
      self:setbits(0x14, 31, 12, ptrbits(addr, 31, 12))
   else
      return cast('void*',
         cast('uint64_t', self:getbits(0x10, 31, 0) * 2^32 +
         cast('uint64_t', self:getbits(0x14, 31, 12)) * 2^12))
   end
end

function init_seg:nic_interface(mode)
   self:setbits(0x14, 9, 8, mode)
end

function init_seg:log_cmdq_size()
   return self:getbits(0x14, 7, 4)
end

function init_seg:log_cmdq_stride()
   return self:getbits(0x14, 3, 0)
end

function init_seg:ring_doorbell(i)
   self:setbits(0x18, i, i, 1)
end

function init_seg:ready(i, val)
   return self:getbits(0x1fc, 31, 31) == 0
end

function init_seg:nic_interface_supported()
   return self:getbits(0x1fc, 26, 24) == 0
end

function init_seg:internal_timer()
   return
      self:getbits(0x1000, 31, 0) * 2^32 +
      self:getbits(0x1004, 31, 0)
end

function init_seg:clear_int()
   self:setbits(0x100c, 0, 0, 1)
end

function init_seg:health_syndrome()
   return self:getbits(0x1010, 31, 24)
end

--command queue (section 7.14.1)

local cmdq = {}
cmdq.__index = cmdq

--init cmds
local QUERY_HCA_CAP      = 0x100
local QUERY_ADAPTER      = 0x101
local INIT_HCA           = 0x102
local TEARDOWN_HCA       = 0x103
local ENABLE_HCA         = 0x104
local DISABLE_HCA        = 0x105
local QUERY_PAGES        = 0x107
local MANAGE_PAGES       = 0x108
local SET_HCA_CAP        = 0x109
local QUERY_ISSI         = 0x10A
local CREATE_EQ          = 0x301
local SET_ISSI           = 0x10B
local SET_DRIVER_VERSION = 0x10D
local ALLOC_TRANSPORT_DOMAIN = 0x816
local CREATE_TIS         = 0x912
local CREATE_CQ          = 0x400
local CREATE_SQ          = 0x904

-- bytewise xor function used for signature calcuation.
local function xor8 (ptr, len)
   local u8 = ffi.cast("uint8_t*", ptr)
   local acc = 0
   for i = 0, len-1 do
      acc = bit.bxor(acc, u8[i])
   end
   return acc
end

local cmdq_entry_t   = ffi.typeof("uint32_t[0x40/4]")
local cmdq_mailbox_t = ffi.typeof("uint32_t[0x240/4]")

-- XXX Check with maximum length of commands that we really use.
local max_mailboxes = 1000
local data_per_mailbox = 0x200 -- Bytes of input/output data in a mailbox

-- Create a command queue with dedicated/reusable DMA memory.
function cmdq:new(init_seg)
   local entry = ffi.cast("uint32_t*", memory.dma_alloc(0x40))
   local inboxes, outboxes = {}, {}
   for i = 0, max_mailboxes-1 do
      -- XXX overpadding.. 0x240 alignment is not accepted?
      inboxes[i]  = ffi.cast("uint32_t*", memory.dma_alloc(0x240, 4096))
      outboxes[i] = ffi.cast("uint32_t*", memory.dma_alloc(0x240, 4096))
   end
   return setmetatable({entry = entry,
                        inboxes = inboxes,
                        outboxes = outboxes,
                        init_seg = init_seg,
                        size = init_seg:log_cmdq_size(),
                        stride = init_seg:log_cmdq_stride()},
      self)
end

-- Reset all data structures to zero values.
-- This is to prevent leakage from one command to the next.
local token = 0xAA
function cmdq:prepare(command, last_input_offset, last_output_offset)
   print("Command: " .. command)
   local input_size  = last_input_offset + 4
   local output_size = last_output_offset + 4

   -- Command entry:

   ffi.fill(self.entry, ffi.sizeof(cmdq_entry_t), 0)
   self:setbits(0x00, 31, 24, 0x7)        -- type
   self:setbits(0x04, 31, 0, input_size)
   self:setbits(0x38, 31, 0, output_size)
   self:setbits(0x3C,
                0, 0, 1, -- ownership = hardware
                31, 24, token)

   -- Mailboxes:

   -- How many mailboxes do we need?
   local ninboxes  = math.ceil((input_size  - 16) / data_per_mailbox)
   local noutboxes = math.ceil((output_size - 16) / data_per_mailbox)
   if ninboxes  > max_mailboxes then error("Input overflow: " ..input_size)  end
   if noutboxes > max_mailboxes then error("Output overflow: "..output_size) end

   if ninboxes > 0 then
      local phy = memory.virtual_to_physical(self.inboxes[0])
      setint(self.entry, 0x08, phy / 2^32)
      setint(self.entry, 0x0C, phy % 2^32)
   end
   if noutboxes > 0 then
      local phy = memory.virtual_to_physical(self.outboxes[0])
      setint(self.entry, 0x30, phy / 2^32)
      setint(self.entry, 0x34, phy % 2^32)
   end

   -- Initialize mailboxes
   for i = 0, max_mailboxes-1 do
      -- Zap old state
      ffi.fill(self.inboxes[i],  ffi.sizeof(cmdq_mailbox_t), 0)
      ffi.fill(self.outboxes[i], ffi.sizeof(cmdq_mailbox_t), 0)
      -- Set mailbox block number
      setint(self.inboxes[i],  0x238, i)
      setint(self.outboxes[i], 0x238, i)
      -- Tokens to match command entry
      setint(self.inboxes[i],  0x23C, setbits(23, 16, token))
      setint(self.outboxes[i], 0x23C, setbits(23, 16, token))
      -- Set 'next' mailbox pointers (when used)
      if i < ninboxes then
         local phy = memory.virtual_to_physical(self.inboxes[i+1])
         setint(self.inboxes[i], 0x230, phy / 2^32)
         setint(self.inboxes[i], 0x234, phy % 2^32)
      end
      if i < noutboxes then
         local phy = memory.virtual_to_physical(self.outboxes[i+1])
         setint(self.outboxes[i], 0x230, phy / 2^32)
         setint(self.outboxes[i], 0x234, phy % 2^32)
      end
   end
   token = (token == 255) and 1 or token+1
end

function cmdq:getbits(ofs, bit2, bit1)
   return getbits(getint(self.entry, ofs), bit2, bit1)
end

function cmdq:setbits(ofs, ...)
   setint(self.entry, ofs, setbits(...))
end

function cmdq:setinbits(ofs, ...) --bit1, bit2, val, ...
   assert(ofs % 4 == 0)
   if ofs <= 16 - 4 then --inline
      self:setbits(0x10 + ofs, ...)
   else --input mailbox
      local mailbox = math.floor((ofs - 16) / data_per_mailbox)
      local offset = (ofs - 16) % data_per_mailbox
      setint(self.inboxes[mailbox], offset, setbits(...))
   end
end

function cmdq:getoutbits(ofs, bit2, bit1)
   if ofs <= 16 - 4 then --inline
      return self:getbits(0x20 + ofs, bit2, bit1)
   else --output mailbox
      local mailbox = math.floor((ofs - 16) / data_per_mailbox)
      local offset  = (ofs - 16) % data_per_mailbox
      local b = getbits(getint(self.outboxes[mailbox], offset), bit2, bit1)
      return b
   end
end

-- "Command delivery status" error codes.
local delivery_errors = {
   [0x00] = 'no errors',
   [0x01] = 'signature error',
   [0x02] = 'token error',
   [0x03] = 'bad block number',
   [0x04] = 'bad output pointer. pointer not aligned to mailbox size',
   [0x05] = 'bad input pointer. pointer not aligned to mailbox size',
   [0x06] = 'internal error',
   [0x07] = 'input len error. input length less than 0x8',
   [0x08] = 'output len error. output length less than 0x8',
   [0x09] = 'reserved not zero',
   [0x10] = 'bad command type',
   -- Note: Suspicious to jump from 0x09 to 0x10 here i.e. skipping 0x0A - 0x0F.
   --       This is consistent with both the PRM and the Linux mlx5_core driver.
}

local function checkz(z)
   if z == 0 then return end
   error('command error: '..(delivery_errors[z] or z))
end

-- Command error code meanings.
-- Note: This information is missing from the PRM. Can compare with Linux mlx5_core.
local command_errors = {
   -- General:
   [0x01] = 'INTERNAL_ERR: internal error',
   [0x02] = 'BAD_OP: Operation/command not supported or opcode modifier not supported',
   [0x03] = 'BAD_PARAM: parameter not supported; parameter out of range; reserved not equal 0',
   [0x04] = 'BAD_SYS_STATE: System was not enabled or bad system state',
   [0x05] = 'BAD_RESOURCE: Attempt to access reserved or unallocated resource, or resource in inappropriate status. for example., not existing CQ when creating QP',
   [0x06] = 'RESOURCE_BUSY: Requested resource is currently executing a command. No change in any resource status or state i.e. command just not executed.',
   [0x08] = 'EXCEED_LIM: Required capability exceeds device limits',
   [0x09] = 'BAD_RES_STATE: Resource is not in the appropriate state or ownership',
   [0x0F] = 'NO_RESOURCES: Command was not executed because lack of resources (for example ICM pages). This is unrecoverable situation from driver point of view',
   [0x50] = 'BAD_INPUT_LEN: Bad command input len',
   [0x51] = 'BAD_OUTPUT_LEN: Bad command output len',
   -- QP/RQ/SQ/TIP:
   [0x10] = 'BAD_RESOURCE_STATE: Attempt to modify a Resource (RQ/SQ/TIP/QPs) which is not in the presumed state',
   -- MAD:
   [0x30] = 'BAD_PKT: Bad management packet (silently discarded)',
   -- CQ:
   [0x40] = 'BAD_SIZE: More outstanding CQEs in CQ than new CQ size',
}

function cmdq:post(last_in_ofs, last_out_ofs,  async)
   if debug then
      local dumpoffset = 0
      print("command INPUT:")
      dumpoffset = hexdump(self.entry, 0, 0x40, dumpoffset)
      local ninboxes  = math.ceil((last_in_ofs + 4 - 16) / data_per_mailbox)
      for i = 0, ninboxes-1 do
         local blocknumber = getint(self.inboxes[i], 0x238, 31, 0)
         local address = memory.virtual_to_physical(self.inboxes[i])
         print("Block "..blocknumber.." @ "..bit.tohex(address, 12)..":")
         dumpoffset = hexdump(self.inboxes[i], 0, ffi.sizeof(cmdq_mailbox_t), dumpoffset)
      end
   end

   self.init_seg:ring_doorbell(0) --post command

   if async then return end

   --poll for command completion
   while self:getbits(0x3C, 0, 0) == 1 do
      if self.init_seg:getbits(0x1010, 31, 24) ~= 0 then
         error("HCA health syndrome: " .. bit.tohex(self.init_seg:getbits(0x1010, 31, 24)))
      end
      C.usleep(10000)
   end

   if debug then
      local dumpoffset = 0
      print("command OUTPUT:")
      dumpoffset = hexdump(self.entry, 0, 0x40, dumpoffset)
      local noutboxes = math.ceil((last_out_ofs + 4 - 16) / data_per_mailbox)
      for i = 0, noutboxes-1 do
         local blocknumber = getint(self.outboxes[i], 0x238, 31, 0)
         local address = memory.virtual_to_physical(self.outboxes[i])
         print("Block "..blocknumber.." @ "..bit.tohex(address, 12)..":")
         dumpoffset = hexdump(self.outboxes[i], 0, ffi.sizeof(cmdq_mailbox_t), dumpoffset)
      end
   end

   local token     = self:getbits(0x3C, 31, 24)
   local signature = self:getbits(0x3C, 23, 16)
   local status    = self:getbits(0x3C,  7,  1)

   checkz(status)
   self:checkstatus()

   return signature, token
end

-- see 12.2 Return Status Summary
function cmdq:checkstatus()
   local status = self:getoutbits(0x00, 31, 24)
   local syndrome = self:getoutbits(0x04, 31, 0)
   if status == 0 then return end
   error(string.format('status: 0x%x (%s), syndrome: 0x%x',
                       status, command_errors[status], syndrome))
end

function cmdq:enable_hca()
   self:prepare("ENABLE_HCA", 0x0C, 0x08)
   self:setinbits(0x00, 31, 16, ENABLE_HCA)
   self:post(0x0C, 0x08)
end

function cmdq:disable_hca()
   self:prepare("DISABLE_HCA", 0x0C, 0x08)
   self:setinbits(0x00, 31, 16, DISABLE_HCA)
   self:post(0x0C, 0x08, true)
end


function cmdq:query_issi()
   self:prepare("QUERY_ISSI", 0x0C, 0x6C)
   self:setinbits(0x00, 31, 16, QUERY_ISSI)
   self:post(0x0C, 0x6C)
   local cur_issi = self:getoutbits(0x08, 15, 0)
   local t = {}
   for i = 639, 0, -1 do
      -- Bit N (0..639) when set means ISSI version N is enabled.
      -- Bits are ordered from highest to lowest.
      local byte = 0x20 + math.floor(i / 8)
      local offset = byte - (byte % 4)
      local bit = 31 - (i % 32)
      if self:getoutbits(offset, bit, bit) == 1 then
         local issi = 639 - i
         t[issi] = true
      end
   end
   return {
      cur_issi = cur_issi,
      sup_issi = t,
   }
end

function cmdq:set_issi(issi)
   self:prepare("SET_ISSI", 0x0C, 0x0C)
   self:setinbits(0x00, 31, 16, SET_ISSI)
   self:setinbits(0x08, 15, 0, issi)
   self:post(0x0C, 0x0C)
end

function cmdq:dump_issi(issi)
   print('  cur_issi            = ', issi.cur_issi)
   print('  sup_issi            = ')
   for i=0,79 do
      if issi.sup_issi[i] then
   print(string.format(
         '     %02d               ', i))
      end
   end
end

local codes = {
   boot = 1,
   init = 2,
   regular = 3,
}
function cmdq:query_pages(which)
   self:prepare("QUERY_PAGES", 0x0C, 0x0C)
   self:setinbits(0x00, 31, 16, QUERY_PAGES)
   self:setinbits(0x04, 15, 0, codes[which])
   self:post(0x0C, 0x0C)
   return self:getoutbits(0x0C, 31, 0)
end

function cmdq:flush_pages()
   self:prepare("MANAGE_PAGES", 0x10, 0x80C)
   self:setinbits(0x00, 31, 16, MANAGE_PAGES)
   self:setinbits(0x04, 15, 0, 2) -- return pages
   self:setinbits(0x0C, 31, 0, 100)
   --[[
   for i=0, num_pages-1 do
      local _, phy = memory.dma_alloc(4096, 4096)
      self:setinbits(0x10 + i*8, 31,  0, ptrbits(phy, 63, 32))
      self:setinbits(0x14 + i*8, 31, 12, ptrbits(phy, 31, 12))
   end
   ]]--
   self:post(0x10, 0x80C)
   return self:getoutbits(0x08, 31, 0)
end

function cmdq:alloc_pages(num_pages)
   self:prepare("MANAGE_PAGES", 0x10 + num_pages*8, 0x0C)
   self:setinbits(0x00, 31, 16, MANAGE_PAGES)
   self:setinbits(0x04, 15, 0, 1) --alloc
   self:setinbits(0x0C, 31, 0, num_pages)
   for i=0, num_pages-1 do
      local _, phy = memory.dma_alloc(4096, 4096)
      self:setinbits(0x10 + i*8, 31,  0, ptrbits(phy, 63, 32))
      self:setinbits(0x14 + i*8, 31, 12, ptrbits(phy, 31, 12))
   end
   self:post(0x10 + num_pages*8, 0x0C)
end

-- Create Event Queue (EQ)
function cmdq:create_eq(uar, numpages)
   self:prepare("CREATE_EQ", 0x10C + numpages*8, 0x0C)
   self:setinbits(0x00, 31, 16, CREATE_EQ)
   -- Setup Event Queue Context:
   --

   -- XXX Had wanted to use log_page_size=0 for 4KB pages
   -- (2^0*4096=4096) but get BAD_INPUT_LEN errors. Have consulted the
   -- hexdump for the Linux mlx5 driver and seen them choose
   -- log_page_size=2 for presumably 16KB pages (2^2*4096=16384) and
   -- mimicking this value resolves the error.
   --
   -- So, it works, but questions:
   -- 1. How come we are choosing a page size? 4KB is used elsewhere.
   -- 2. Are we setting the value correctly or is there some silly bug?
   -- 3. How come log_page_size 2 is okay but 0 is not?
   --    (What is the root cause of the BAD_INPUT_LEN error, really?)
   local status = 0             -- 0 = OK
   local ec = 0                 -- event collapse flag
   local oi = 0                 -- overrun ignore flag
   local st = 0x0               -- (Card did not accept 0x0A)
   local page_offset = 0        -- (must be 0)
   local log_eq_size = 7        -- Log (base 2) of EQ size (in entries)
   local uar_page = 0           -- UAR page 0 for main event queue
   local intr = 0               -- MSI-X table entry (should not be used)
   local log_page_size = 2      -- Log (base 2) of page size in 4KB units
   local consumer_counter = 0   -- Software cursor (init to zero)
   local producer_counter = 0   -- Hardware cursor (init to zero)
   self:setinbits(0x10 + 0x00,
                  31, 28, status,
                  18, 18, ec,
                  17, 17, oi,
                  11, 8, st)
   self:setinbits(0x10 + 0x08, 7,9, page_offset)
   self:setinbits(0x10 + 0x0C, 28, 24, log_eq_size,  23, 0, uar_page)
   self:setinbits(0x10 + 0x14, 7, 9, intr)
   self:setinbits(0x10 + 0x18, 28, 24, log_page_size)
   self:setinbits(0x10 + 0x28, 23, 0, consumer_counter)
   self:setinbits(0x10 + 0x2C, 23, 0, producer_counter)
   -- Set event bitmask
   local events = bits{CE=0x0, LQWE=0x13, CQERR=0x04, IntErr=0x08,
                       PoStCh=0x09, CoInCo=0x0A, PageRequest=0x0B}
   self:setinbits(0x10 + 0x5C, 31, 0, events)
   -- Allocate pages in contiguous physical memory
   local ptr, phy = memory.dma_alloc(4096 * numpages, 4096)
   for i = 0, numpages-1 do
      self:setinbits(0x110 + i*8, 31, 0, ptrbits(phy + i * 4096, 63, 32))
      self:setinbits(0x114 + i*8, 31, 0, ptrbits(phy + i * 4096, 31, 0))
   end
   self:post(0x10C + numpages*8, 0x0C)
   local eqn = self:getoutbits(0x08, 7, 0)
   return eq:new(eqn, ptr, 2^log_eq_size)
end

-- EQ (Event Queue) object

-- Event Queue Entry (EQE)
local eqe_t = ffi.typeof[[
  struct {
    uint16_t event_type;
    uint16_t event_sub_type;
    uint32_t event_data;
    uint16_t pad;
    uint8_t signature;
    uint8_t owner;
  }
]]

uar = {}
uar.__index = uar

local uar_t = ffi.typeof[[
  struct {
    uint8_t pad0[0x20];
    uint64_t cq;
    uint8_t pad1[0x1C];
    uint32_t eq_arm;
    uint8_t pad2[0x04];
    uint32_t eq;
    uint8_t pad3[0x7B8];
    uint32_t db_blueflame_buffer0_even;
    uint32_t db_blueflame_buffer0_odd;
    uint32_t db_blueflame_buffer1_even;
    uint32_t db_blueflame_buffer1_odd;
  }
]]

function uar:new(uar_number, base_ptr)
   local ptr = ffi.cast("uint8_t*", base_ptr) + uar_number * 4096
   return setmetatable({uar_n = uar_number,
                        uar = ffi.cast(uar_t, ptr)})
end

function uar:ring_eq_doorbell(eqn, consumer_index)
   local n = bswap(shl(eqn, 24) + consumer_index)
   
   
end

function uar:ring_cq_doorbell()
end

eq = {}
eq.__index = eq

function eq:new(eqn, pointer, nentries)
   local ring = ffi.cast(ffi.typeof("$*", eqe_t), pointer)
   for i = 0, nentries-1 do
      ring[i].owner = 1
   end
   return setmetatable({eqn = eqn,
                        ring = ring,
                        index = 0,
                        n = nentries},
      self)
end

function eq:poll()
   print("Polling EQ")
   local eqe = self.ring[self.index]
   while eqe.owner == 0 and eqe.event_type ~= 0xFF do
      self.index = self.index + 1
      eqe = self.ring[self.index % self.n]
      self:event(eqe)
   end
   print("done polling EQ")
end

function eq:event()
   print(("Got event %s.%s"):format(eqe.event_type, eqe.event_sub_type))
   error("Event handling not yet implemented")
end

local what_codes = {
   max = 0,
   cur = 1,
}
local which_codes = {
   general = 0,
   offload = 1,
   flow_table = 7,
}
function cmdq:query_hca_cap(what, which)
   self:prepare("QUERY_HCA_CAP", 0x0C, 0x100C - 3000)
   self:setinbits(0x00, 31, 16, QUERY_HCA_CAP)
   self:setinbits(0x04,
      15,  1, assert(which_codes[which]),
       0,  0, assert(what_codes[what]))
   self:post(0x0C, 0x100C - 3000)
   local caps = {}
   if which == 'general' then
      caps.log_max_cq_sz            = self:getoutbits(0x10 + 0x18, 23, 16)
      caps.log_max_cq               = self:getoutbits(0x10 + 0x18,  4,  0)
      caps.log_max_eq_sz            = self:getoutbits(0x10 + 0x1C, 31, 24)
      caps.log_max_mkey             = self:getoutbits(0x10 + 0x1C, 21, 16)
      caps.log_max_eq               = self:getoutbits(0x10 + 0x1C,  3,  0)
      caps.max_indirection          = self:getoutbits(0x10 + 0x20, 31, 24)
      caps.log_max_mrw_sz           = self:getoutbits(0x10 + 0x20, 22, 16)
      caps.log_max_klm_list_size    = self:getoutbits(0x10 + 0x20,  5,  0)
      caps.end_pad                  = self:getoutbits(0x10 + 0x2C, 31, 31)
      caps.start_pad                = self:getoutbits(0x10 + 0x2C, 28, 28)
      caps.cache_line_128byte       = self:getoutbits(0x10 + 0x2C, 27, 27)
      caps.vport_counters           = self:getoutbits(0x10 + 0x30, 30, 30)
      caps.vport_group_manager      = self:getoutbits(0x10 + 0x34, 31, 31)
      caps.nic_flow_table           = self:getoutbits(0x10 + 0x34, 25, 25)
      caps.port_type                = self:getoutbits(0x10 + 0x34,  9,  8)
      caps.num_ports                = self:getoutbits(0x10 + 0x34,  7,  0)
      caps.log_max_msg              = self:getoutbits(0x10 + 0x38, 28, 24)
      caps.max_tc                   = self:getoutbits(0x10 + 0x38, 19, 16)
      caps.cqe_version              = self:getoutbits(0x10 + 0x3C,  3,  0)
      caps.cmdif_checksum           = self:getoutbits(0x10 + 0x40, 15, 14)
      caps.wq_signature             = self:getoutbits(0x10 + 0x40, 11, 11)
      caps.sctr_data_cqe            = self:getoutbits(0x10 + 0x40, 10, 10)
      caps.eth_net_offloads         = self:getoutbits(0x10 + 0x40,  3,  3)
      caps.cq_oi                    = self:getoutbits(0x10 + 0x44, 31, 31)
      caps.cq_resize                = self:getoutbits(0x10 + 0x44, 30, 30)
      caps.cq_moderation            = self:getoutbits(0x10 + 0x44, 29, 29)
      caps.cq_eq_remap              = self:getoutbits(0x10 + 0x44, 25, 25)
      caps.scqe_break_moderation    = self:getoutbits(0x10 + 0x44, 21, 21)
      caps.cq_period_start_from_cqe = self:getoutbits(0x10 + 0x44, 20, 20)
      caps.imaicl                   = self:getoutbits(0x10 + 0x44, 14, 14)
      caps.xrc                      = self:getoutbits(0x10 + 0x44,  3,  3)
      caps.ud                       = self:getoutbits(0x10 + 0x44,  2,  2)
      caps.uc                       = self:getoutbits(0x10 + 0x44,  1,  1)
      caps.rc                       = self:getoutbits(0x10 + 0x44,  0,  0)
      caps.uar_sz                   = self:getoutbits(0x10 + 0x48, 21, 16)
      caps.log_pg_sz                = self:getoutbits(0x10 + 0x48,  7,  0)
      caps.bf                       = self:getoutbits(0x10 + 0x4C, 31, 31)
      caps.driver_version           = self:getoutbits(0x10 + 0x4C, 30, 30)
      caps.pad_tx_eth_packet        = self:getoutbits(0x10 + 0x4C, 29, 29)
      caps.log_bf_reg_size          = self:getoutbits(0x10 + 0x4C, 20, 16)
      caps.log_max_transport_domain = self:getoutbits(0x10 + 0x64, 28, 24)
      caps.log_max_pd               = self:getoutbits(0x10 + 0x64, 20, 16)
      caps.max_flow_counter         = self:getoutbits(0x10 + 0x68, 15,  0)
      caps.log_max_rq               = self:getoutbits(0x10 + 0x6C, 28, 24)
      caps.log_max_sq               = self:getoutbits(0x10 + 0x6C, 20, 16)
      caps.log_max_tir              = self:getoutbits(0x10 + 0x6C, 12,  8)
      caps.log_max_tis              = self:getoutbits(0x10 + 0x6C,  4,  0)
      caps.basic_cyclic_rcv_wqe     = self:getoutbits(0x10 + 0x70, 31, 31)
      caps.log_max_rmp              = self:getoutbits(0x10 + 0x70, 28, 24)
      caps.log_max_rqt              = self:getoutbits(0x10 + 0x70, 20, 16)
      caps.log_max_rqt_size         = self:getoutbits(0x10 + 0x70, 12,  8)
      caps.log_max_tis_per_sq       = self:getoutbits(0x10 + 0x70,  4,  0)
      caps.log_max_stride_sz_rq     = self:getoutbits(0x10 + 0x74, 28, 24)
      caps.log_min_stride_sz_rq     = self:getoutbits(0x10 + 0x74, 20, 16)
      caps.log_max_stride_sz_sq     = self:getoutbits(0x10 + 0x74, 12,  8)
      caps.log_min_stride_sz_sq     = self:getoutbits(0x10 + 0x74,  4,  0)
      caps.log_max_wq_sz            = self:getoutbits(0x10 + 0x78,  4,  0)
      caps.log_max_vlan_list        = self:getoutbits(0x10 + 0x7C, 20, 16)
      caps.log_max_current_mc_list  = self:getoutbits(0x10 + 0x7C, 12,  8)
      caps.log_max_current_uc_list  = self:getoutbits(0x10 + 0x7C,  4,  0)
      caps.log_max_l2_table         = self:getoutbits(0x10 + 0x90, 28, 24)
      caps.log_uar_page_sz          = self:getoutbits(0x10 + 0x90, 15,  0)
      caps.device_frequency_mhz     = self:getoutbits(0x10 + 0x98, 31,  0)
   elseif which_caps == 'offload' then
      --TODO
   elseif which_caps == 'flow_table' then
      --TODO
   end
   return caps
end

function cmdq:set_hca_cap(which, caps)
   self:prepare("SET_HCA_CAP", 0x100C - 3000, 0x0C)
   self:setinbits(0x00, 31, 16, SET_HCA_CAP)
   self:setinbits(0x04, 15,  1, assert(which_codes[which]))
   if which == 'general' then
      self:setinbits(0x10 + 0x18,
         23, 16, caps.log_max_cq_sz,
         4,   0, caps.log_max_cq)
      self:setinbits(0x10 + 0x1C,
         31, 24, caps.log_max_eq_sz,
         21, 16, caps.log_max_mkey,
         3,   0, caps.log_max_eq)
      self:setinbits(0x10 + 0x20,
         31, 24, caps.max_indirection,
         22, 16, caps.log_max_mrw_sz,
         5,   0, caps.log_max_klm_list_size)
      self:setinbits(0x10 + 0x2C,
         31, 31, caps.end_pad,
         28, 28, caps.start_pad,
         27, 27, caps.cache_line_128byte)
      self:setinbits(0x10 + 0x30,
         30, 30, caps.vport_counters)
      self:setinbits(0x10 + 0x34,
         31, 31, caps.vport_group_manager,
         25, 25, caps.nic_flow_table,
          9,  8, caps.port_type,
          7,  0, caps.num_ports)
      self:setinbits(0x10 + 0x38,
         28, 24, caps.log_max_msg,
         19, 16, caps.max_tc)
      self:setinbits(0x10 + 0x3C,
          3,   0, caps.cqe_version)
      self:setinbits(0x10 + 0x40,
         15, 14, caps.cmdif_checksum,
         11, 11, caps.wq_signature,
         10, 10, caps.sctr_data_cqe,
          3,  3, caps.eth_net_offloads)
      self:setinbits(0x10 + 0x44,
         31, 31, caps.cq_oi,
         30, 30, caps.cq_resize,
         29, 29, caps.cq_moderation,
         25, 25, caps.cq_eq_remap,
         21, 21, caps.scqe_break_moderation,
         20, 20, caps.cq_period_start_from_cqe,
         14, 14, caps.imaicl,
          3,  3, caps.xrc,
          2,  2, caps.ud,
          1,  1, caps.uc,
          0,  0, caps.rc)
      self:setinbits(0x10 + 0x48,
         21, 16, caps.uar_sz,
          7,  0, caps.log_pg_sz)
      self:setinbits(0x10 + 0x4C,
         31, 31, caps.bf,
         30, 30, caps.driver_version,
         29, 29, caps.pad_tx_eth_packet,
         20, 16, caps.log_bf_reg_size)
      self:setinbits(0x10 + 0x64,
         28, 24, caps.log_max_transport_domain,
         20, 16, caps.log_max_pd)
      self:setinbits(0x10 + 0x68,
         15,  0, caps.max_flow_counter)
      self:setinbits(0x10 + 0x6C,
         28, 24, caps.log_max_rq,
         20, 16, caps.log_max_sq,
         12,  8, caps.log_max_tir,
          4,  0, caps.log_max_tis)
      self:setinbits(0x10 + 0x70,
         31, 31, caps.basic_cyclic_rcv_wqe,
         28, 24, caps.log_max_rmp,
         20, 16, caps.log_max_rqt,
         12,  8, caps.log_max_rqt_size,
          4,  0, caps.log_max_tis_per_sq)
      self:setinbits(0x10 + 0x74,
         28, 24, caps.log_max_stride_sz_rq,
         20, 16, caps.log_min_stride_sz_rq,
         12,  8, caps.log_max_stride_sz_sq,
          4,  0, caps.log_min_stride_sz_sq)
      self:setinbits(0x10 + 0x78,
          4,  0, caps.log_max_wq_sz)
      self:setinbits(0x10 + 0x7C,
         20, 16, caps.log_max_vlan_list,
         12,  8, caps.log_max_current_mc_list,
          4,  0, caps.log_max_current_uc_list)
      self:setinbits(0x10 + 0x90,
         28, 24, caps.log_max_l2_table,
         15,  0, caps.log_uar_page_sz)
      self:setinbits(0x10 + 0x98,
         31,  0, caps.device_frequency_mhz)
   elseif which == 'offload' then
      self:setinbits(0x10 + 0x00,
         31, 31, caps.csum_cap,
         30, 30, caps.vlan_cap,
         29, 29, caps.lro_cap,
         28, 28, caps.lro_psh_flag,
         27, 27, caps.lro_time_stamp,
         26, 25, caps.lro_max_msg_sz_mode,
         23, 23, caps.self_lb_en_modifiable,
         22, 22, caps.self_lb_mc,
         21, 21, caps.self_lb_uc,
         20, 16, caps.max_lso_cap,
         13, 12, caps.wqe_inline_mode,
         11,  8, caps.rss_ind_tbl_cap)
      self:setinbits(0x10 + 0x08,
         15,  0, caps.lro_min_mss_size)
      for i = 1, 4 do
         self:setinbits(0x10 + 0x30 + (i-1)*4, 31, 0, caps.lro_timer_supported_periods[i])
      end
   elseif which == 'flow_table' then
      --TODO
   end
   self:post(0x100C, 0x0C)
end

-- XXX VPORT commands /may/ not be needed since we are not using SR-IOV.
--     In this case the functions below can be removed.

function cmdq:modify_vport_state(up)
   self:prepare("MODIFY_VPORT_STATE", 0x0c, 0x0c)
   self:setinbits(0x00, 31, 16, 0x751)
   self:setinbits(0x0C,  7,  4, up and 1 or 0)
end

function cmdq:query_vport_state()
   self:prepare("QUERY_VPORT_STATE", 0x0c, 0x0c)
   self:setinbits(0x00, 31, 16, 0x750)
   self:post(0x0C, 0x0C)
   return { admin_state = self:getoutbits(0x0C, 7, 4),
            oper_state  = self:getoutbits(0x0C, 3, 0) }
end

function cmdq:query_vport_counter()
   self:prepare("QUERY_VPORT_COUNTER", 0x1c, 0x20c)
   self:setinbits(0x00, 31, 16, 0x770)
   self:post(0x1C, 0x20c)
   print("vport counters")
   for i = 0x10, 0x200, 4 do
      local n = self:getoutbits(i, 31, 0)
      if n > 0 then print(bit.tohex(i), n) end
   end
   return self:getoutbits(0x84, 31, 0)
end

function cmdq:modify_vport_state(admin_state)
   self:prepare("MODIFY_VPORT_STATE", 0x0c, 0x0c)
   self:setinbits(0x00, 31, 16, 0x751)
   self:setinbits(0x0C, 7, 4, admin_state)
   self:post(0x0C, 0x0C)
end

function cmdq:query_nic_vport_context()
   -- XXX This command can be used to manipulate long lists of allowed
   -- unicast addresses, multicast addresses, and VLANs. For now we
   -- skip that (leave the list length as zero) and access only the
   -- global settings. Is this interaction correct ?
   self:prepare("QUERY_NIC_VPORT_CONTEXT", 0x0c, 0x10+0xFC)
   self:setinbits(0x00, 31, 16, 0x754) -- Command opcode
   self:post(0x0C, 0x10+0xFC)
   local mac_hi = self:getoutbits(0x10+0xF4, 31, 0)
   local mac_lo = self:getoutbits(0x10+0xF8, 31, 0)
   local mac_hex = bit.tohex(mac_hi, 4) .. bit.tohex(mac_lo, 8)
   return { min_wqe_inline_mode = self:getoutbits(0x10+0x00, 26, 24),
            mtu = self:getoutbits(0x10+0x24, 15, 0),
            promisc_uc  = self:getoutbits(0x10+0xf0, 31, 31),
            promisc_mc  = self:getoutbits(0x10+0xf0, 30, 30),
            promisc_all = self:getoutbits(0x10+0xf0, 29, 29),
            permanent_address = mac_hex }
end

-- Allocate Receive Domain

-- Allocate Transport Domain
function cmdq:alloc_transport_domain()
   self:prepare("ALLOC_TRANSPORT_DOMAIN", 0x0c, 0x0c)
   self:setinbits(0x00, 31, 16, ALLOC_TRANSPORT_DOMAIN)
   self:post(0x0C, 0x0C)
   return self:getoutbits(0x08, 23, 0)
end

function cmdq:create_tir(rqn, transport_domain)
   self:prepare("CREATE_TIR", 0x10C, 0x0C)
   self:setinbits(0x00, 31, 16, 0x900)
   -- TIR Context
   self:setinbits(0x20 + 0x04, 31, 28, 0) -- direct
   -- (LRO fields reserved because not used)
   self:setinbits(0x20 + 0x1C, 23, 0, rqn)
   self:setinbits(0x20 + 0x24,
                  31, 28, 0,    -- no hashing
                  23, 0, transport_domain)
   self:post(0x10C, 0x0C)
   return self:getoutbits(0x08, 23, 0)
end
   
-- Create Transmit Interface Send
function cmdq:create_tis(prio, transport_domain)
   self:prepare("CREATE_TIS", 0x20 + 0x9C, 0x0C)
   self:setinbits(0x00, 31, 16, CREATE_TIS)
   self:setinbits(0x20 + 0x00, 19, 16, prio)
   self:setinbits(0x20 + 0x24, 23,  0, transport_domain)
   self:post(0x20 + 0x9C, 0x0C)
   return self:getoutbits(0x08, 23, 0)
end

function cmdq:alloc_uar()
   self:prepare("ALLOC_UAR", 0x0C, 0x0C)
   self:setinbits(0x00, 31, 16, 0x802)
   self:post(0x0C, 0x0C)
   return self:getoutbits(0x08, 23, 0)
end

function cmdq:alloc_pd()
   self:prepare("ALLOC_PD", 0x0C, 0x0C)
   self:setinbits(0x00, 31, 16, 0x800)
   self:post(0x0C, 0x0C)
   return self:getoutbits(0x08, 23, 0)
end

function cmdq:create_cq(entries, uar_page, eq, db_phy)
   local ptr, phy = memory.dma_alloc(entries * 64, 4096)
   -- Fill with ones.
   -- Note: This sets hardware ownership bit for first use.
   --ffi.fill(ptr, entries * 64, 0xFF)
   self:prepare("CREATE_CQ", 0x114, 0x0C)
   self:setinbits(0x00, 31, 16, CREATE_CQ)
   -- PAS (physical address)
   self:setinbits(0x110, 31, 0, ptrbits(phy, 63, 32))
   self:setinbits(0x114, 31, 0, ptrbits(phy, 31, 0))
   -- CQ Context
   self:setinbits(0x10 + 0,
                  31, 28, 0, -- status
                  23, 21, 0, -- cqe_sz (64 bytes)
                  20, 20, 1, -- cc (collapse events)
                  18, 18, 0, -- scqe_break_moderation_en
                  17, 17, 1, -- oi: override ignore
                  16, 15, 0, -- cq_period_mode
                  14, 14, 0, -- cq_compression_en
                  13, 12, 0, -- mini_cqe_res_format
                  11, 8,  0) -- state (reserved on create)
   -- (page_offset is 0)
   self:setinbits(0x10 + 0x0C,
                  28, 24, 10, -- log_cq_size (10 matches mlx5 trace)
                  23, 0, uar_page) -- uar_page
   -- (cq_period and cq_max_count are 0)
   self:setinbits(0x10 + 0x14, 7, 0, eq)
   self:setinbits(0x10 + 0x18, 28, 24, 4) -- log_page_size (4 matches mlx5 trace)
   self:setinbits(0x10 + 0x38, 31, 0, ptrbits(db_phy, 63, 32))
   self:setinbits(0x10 + 0x3C, 31, 0, ptrbits(db_phy, 31, 0))
   self:post(0x114, 0x0C)
   return self:getoutbits(0x08, 23, 0), ptr
end

function cmdq:create_rq(cqn, user_index, uar, pd, db_phy, rqmem)
   local rqphy = memory.virtual_to_physical(rqmem)
   self:prepare("CREATE_RQ", 0x20 + 0x30 + 0xC4, 0x0C)
   self:setinbits(0x00, 31, 16, 0x908) -- CREATE_RQ
   -- Receive Queue
   self:setinbits(0x20 + 0x00,
                  31, 31, 1, -- rlkey
                  28, 28, 1, -- vlan_strip_disable
                  27, 24, 0) -- inlined memory queue
   self:setinbits(0x20 + 0x04, 23, 0, user_index)
   self:setinbits(0x20 + 0x08, 23, 0, cqn)
   -- Work Queue
   self:setinbits(0x20 + 0x30 + 0x00,
                  31, 28, 1, -- wq_type = cyclic
                  27, 27, 0, -- wq_signature = disabled
                  26, 25, 0, -- no padding
                  20, 16, 0, -- page_offset = 0
                  15, 0,  0) -- lwm = 0
   self:setinbits(0x20 + 0x30 + 0x08, 23, 0, pd)
   self:setinbits(0x20 + 0x30 + 0x0C, 23, 0, user_index)
   self:setinbits(0x20 + 0x30 + 0x10, 31, 0, ptrbits(db_phy, 63, 32))
   self:setinbits(0x20 + 0x30 + 0x14, 31, 0, ptrbits(db_phy, 31, 0))
   self:setinbits(0x20 + 0x30 + 0x20,
                  19, 16, 4,     -- log_wq_stride
                  12, 8, 4,      -- page size (XXX one big page?)
                  4, 0, 10)      -- log_wq_size (1024 x 64 byte entries)
   self:setinbits(0x20 + 0x30 + 0xC0, 63, 32, ptrbits(rqphy, 63, 32))
   self:setinbits(0x20 + 0x30 + 0xC4, 31, 0,  ptrbits(rqphy, 31, 0))
   self:post(0x10 + 0x20 + 0x30 + 0xC4, 0x0C)
   return self:getoutbits(0x08, 23, 0), rqmem
end

function cmdq:modify_rq(rqn, curr_state, next_state)
   self:prepare("MODIFY_RQ", 0x20 + 0x30 + 0xC4, 0x0C)
   self:setinbits(0x00, 31, 16, 0x909)
   self:setinbits(0x08,
                  31, 28, curr_state,
                  27,  0, rqn)
   self:setinbits(0x20 + 0x00, 23, 20, next_state)
   self:post(0x20 + 0x30 + 0xC4, 0x0C)
end

function cmdq:modify_sq(sqn, curr_state, next_state)
   self:prepare("MODIFY_SQ", 0x20 + 0x30 + 0xC4, 0x0C)
   self:setinbits(0x00, 31, 16, 0x905)
   self:setinbits(0x08,
                  31, 28, curr_state,
                  23, 0, sqn)
   self:setinbits(0x20 + 0x00,
                  23, 20, next_state)
   self:post(0x20 + 0x30 + 0xC4, 0x0C)
end

function cmdq:create_sq(tis, cqn, user_index, uar, pd, db_phy, sqmem)
   self:prepare("CREATE_SQ", 0x20 + 0x30 + 0xC4, 0x0C)
   self:setinbits(0x00, 31, 16, 0x904) -- CREATE_SQ
   -- Send queue
   self:setinbits(0x20 + 0x00,
                  31, 31, 1,    -- rlkey
                  29, 29, 1,    -- fre
                  28, 28, 1,    -- flush_in_error_en
                  26, 24, 1,    -- min_wqe_inline_mode
                  23, 20, 0)    -- state = RST
   self:setinbits(0x20 + 0x04, 23, 0, user_index)
   self:setinbits(0x20 + 0x08, 23, 0, cqn)
   self:setinbits(0x20 + 0x20, 31, 16, 1) -- tis_lst_sz
   self:setinbits(0x20 + 0x2C, 23, 0, tis)
   -- Work queue
   self:setinbits(0x20 + 0x30 + 0x00,
                  31, 28, 1,     -- wq_type = cyclic
                  27, 27, 0,     -- signature disable
                  26, 25, 0)     -- end_padding_mode (reserved for SQ)
   self:setinbits(0x20 + 0x30 + 0x04,
                  20, 16, 0,     -- page_offset = 0
                  15, 0,  0)     -- limit water mark (reserved for SQ)
   self:setinbits(0x20 + 0x30 + 0x08, 23, 0, pd) -- pd
   self:setinbits(0x20 + 0x30 + 0x0C, 23, 0, uar) -- uar_page (XXX reserved for SQ)
   self:setinbits(0x20 + 0x30 + 0x10, 31, 0, ptrbits(db_phy, 63, 32))
   self:setinbits(0x20 + 0x30 + 0x14, 31, 0, ptrbits(db_phy, 31, 0))
   self:setinbits(0x20 + 0x30 + 0x20,
                  19, 16, 6,     -- log_wq_stride
                  12, 8,  0xF,   -- log_wq_pg_sz = 0xF (max)
                  4,  0,  log_qsize)    -- log_wq_sz
   local wqphy = memory.virtual_to_physical(sqmem)
   self:setinbits(0x20 + 0x30 + 0xC0, 31, 0, ptrbits(wqphy, 63, 32))
   self:setinbits(0x20 + 0x30 + 0xC4, 31, 0, ptrbits(wqphy, 31, 0))
   self:post(0x10 + 0x20 + 0x30 + 0xC4, 0x0C)
   return self:getoutbits(0x08, 23, 0), sqmem
end

function cmdq:query_sq(sqn)
   self:prepare("QUERY_SQ", 0x0c, 0x10 + 0x20 + 0x30 + 0xC4)
   self:setinbits(0x00, 31, 16, 0x907)
   self:setinbits(0x08, 23, 0, sqn)
   self:post(0x0C, 0x10 + 0x20 + 0x30 + 0xC4)
   return {
      --user_index = self:getoutbits(0x20 + 0x04, 23, 0),
      --cqn        = self:getoutbits(0x20 + 0x08, 23, 0),
      hw_counter = self:getoutbits(0x20 + 0x18, 31, 0),
      sw_counter = self:getoutbits(0x20 + 0x1C, 31, 0),
   }
end

local flow_table_types = { receive = 0, send = 1}

function cmdq:create_root_flow_table(table_type)
   assert(flow_table_types[table_type], "bad table type: "..table_type)
   self:prepare("CREATE_FLOW_TABLE", 0x3C, 0x0C)
   self:setinbits(0x00, 31, 16, 0x930)
   self:setinbits(0x10, 31, 24, flow_table_types[table_type])
   -- Flow table context
   self:setinbits(0x18 + 0x00, 27, 24, 0) -- default miss action
   self:setinbits(0x18 + 0x00, 23, 16, 0) -- level=0 (root)
   self:setinbits(0x18 + 0x00, 7, 0,   4) -- log_size (reserve some space)
   self:post(0x3C, 0x0C)
   return self:getoutbits(0x08, 23, 0)
end

function cmdq:set_flow_table_root(table_id, table_type)
   assert(flow_table_types[table_type], "bad table type: "..table_type)
   self:prepare("SET_FLOW_TABLE_ROOT", 0x3C, 0x0C)
   self:setinbits(0x00, 31, 16, 0x92F)
   self:setinbits(0x10, 31, 24, flow_table_types[table_type])
   self:setinbits(0x14, 23, 0, table_id)
   self:post(0x3C, 0x0C)
end

function cmdq:create_flow_group_wildcard(table_id, table_type, start_ix, end_ix)
   assert(flow_table_types[table_type], "bad table type: "..table_type)
   self:prepare("CREATE_FLOW_GROUP", 0x3FC, 0x0C)
   self:setinbits(0x00, 31, 16, 0x933)
   self:setinbits(0x10, 31, 24, flow_table_types[table_type])
   self:setinbits(0x14, 23, 0, table_id)
   self:setinbits(0x1C, 31, 0, start_ix)
   self:setinbits(0x24, 31, 0, end_ix)
   self:setinbits(0x3C,  7, 0, 0) -- match outer headers
   -- XXX Have to match something? Let's try ethertype
   --self:setinbits(0x40 + 0x04, 15, 0, 0xFFFF)
   --self:setinbits(0x40 + 0x10, 31, 24, 0xFF)
   self:post(0x3FC, 0x0C)
end

function cmdq:set_flow_table_entry_simple(table_id, table_type, group_id, flow_index, tir)
   assert(flow_table_types[table_type], "bad table type: "..table_type)
   self:prepare("SET_FLOW_TABLE_ENTRY", 0x40 + 0x300, 0x0C)
   self:setinbits(0x00, 31, 16, 0x936)
   self:setinbits(0x04, 15,  0, 0) -- opmode = new entry
   self:setinbits(0x10, 31, 24, flow_table_types[table_type])
   self:setinbits(0x14, 23,  0, table_id)
   self:setinbits(0x20, 31, 0, flow_index)
   -- Flow context
   self:setinbits(0x40 + 0x04, 31, 0, group_id)
   self:setinbits(0x40 + 0x08, 23, 0, 1) -- flow tag value for CQE
   self:setinbits(0x40 + 0x0C, 15, 0, 4) -- action = FWD_DST
   self:setinbits(0x40 + 0x10, 23, 0, 1) -- destination list size
   -- (Match value blank for wildcard)
   self:setinbits(0x40 + 0x300,
                  31, 24, 2,
                  23,  0, tir)
   self:post(0x40 + 0x300, 0x0C)
end

function cmdq:set_admin_status (portnumber, admin_up)
   self:prepare("ACCESS_REGISTER", 0x1C, 0x0C)
   self:setinbits(0x00, 31, 16, 0x805)
   self:setinbits(0x04, 15,  0, 0) -- write
   self:setinbits(0x08, 15,  0, 0x5006) -- PAOS (Port Admin & Op Status)
   self:setinbits(0x10,
                  23, 16, portnumber, -- local port 0
                  11,  8, admin_up and 1 or 2) -- admin status = up or down
   self:setinbits(0x14, 31, 31, 1) -- admin state update enable
   self:post(0x1C, 0x0C)
end

function cmdq:get_port_status(portnumber)
   self:prepare("ACCESS_REGISTER", 0x10, 0x1C)
   self:setinbits(0x00, 31, 16, 0x805)
   self:setinbits(0x04, 15,  0, 1) -- read
   self:setinbits(0x08, 15,  0, 0x5006) -- PAOS (Port Admin & Op Status)
   self:setinbits(0x10, 23, 16, portnumber)
   self:post(0x0C, 0x1C)
   return {admin_status = self:getoutbits(0x10, 11, 8),
           oper_status  = self:getoutbits(0x10, 3, 0)}
end

function cmdq:get_port_loopback_capability(portnumber)
   self:prepare("ACCESS_REGISTER", 0x10, 0x14)
   self:setinbits(0x00, 31, 16, 0x805)
   self:setinbits(0x04, 15,  0, 1) -- read
   self:setinbits(0x08, 15,  0, 0x5018) -- PPLR (Port Physical Loopback Register)
   self:setinbits(0x10, 23, 16, portnumber)
   self:post(0x10, 0x14)
   return self:getoutbits(0x14, 23, 16)
end

function cmdq:set_port_loopback(portnumber, loopback_mode)
   self:prepare("ACCESS_REGISTER", 0x14, 0x0C)
   self:setinbits(0x00, 31, 16, 0x805)
   self:setinbits(0x04, 15,  0, 0) -- write
   self:setinbits(0x08, 15,  0, 0x5018) -- PPLR (Port Physical Loopback Register)
   self:setinbits(0x10, 23, 16, portnumber)
   self:setinbits(0x14,  7,  0, loopback_mode and 2 or 0) -- local or none
   self:post(0x14, 0x0C)
end

function cmdq:enable_loopback_mode(portnumber, loopback_mode)
   assert(loopback_mode == true)
   --print("Loopback capability: ", self:get_port_loopback_capability(portnumber))
   C.usleep(1e6)
   --print("Port admin status:   ", self:get_port_status(portnumber).admin_status)
   C.usleep(1e6)
   --self:modify_vport_state(false)
   print("Disable port in PAOS ", self:set_admin_status(portnumber, false))
   C.usleep(1e6)
   print("Set loopback mode", self:set_port_loopback(portnumber, true))
   C.usleep(1e6)
   print("Enable port in PAOS", self:set_admin_status(portnumber, true))
   C.usleep(1e6)
   self:modify_vport_state(true)

   if false then
      self:set_admin_status(portnumber, false)
      C.usleep(3e6)
      local cap = self:get_port_loopback_capability(portnumber)
      local stat = self:get_port_status(portnumber)
      print("Port status:")
      for k,v in pairs(stat) do print(k,v) end
      if band(cap, 2) == 0 then
         error("loopback mode not supported, capability mask: " .. cap)
      else
         print("Loopback OK")
      end
      C.usleep(3e6)
      self:set_port_loopback(portnumber, loopback_mode)
      C.usleep(3e6)
      self:set_admin_status(portnumber, true)
      -- ACCESS REGISTER:
      --   PAOS: Disable port
      --   PPLR: Enable loopback
      --   PAOS: Enable port
   end
end

-- Teardown the NIC. mode = 0 (graceful) or 1 (panic)
function cmdq:teardown_hca(mode)
   self:prepare("TEARDOWN_HCA", 0x0c, 0x0c)
   self:setinbits(0x00, 31, 16, 0x103)
   self:setinbits(0x04, 15, 0, mode)
   self:post(0x0C, 0x0C, true)
end

function cmdq:init_hca()
   self:prepare("INIT_HCA", 0x0c, 0x0c)
   self:setinbits(0x00, 31, 16, INIT_HCA)
   self:post(0x0C, 0x0C)
end

-- Create a "memory key" describing the physical address layout.
-- XXX We are always using physical addresses. I am not sure if an
--     mkey is really required or whether the "reserved lkey" value
--     can be used to bypass this whole mechanism. Implementing this
--     command now to see if it helps resolve a surprising error on
--     CREATE_CQ that may or may not be related.
function cmdq:create_mkey(pd)
   self:prepare("CREATE_MKEY", 0x110, 0x0C)
   self:setinbits(0x00, 31, 16, 0x200)
   -- MKey Context
   self:setinbits(0x10 + 0x00,
                  30, 30, 0, -- free
                  15, 15, 1, -- umr_en
                  13, 13, 1, -- rw
                  12, 12, 1, -- rr
                  11, 11, 1, -- lw
                  10, 10, 1, -- lr
                  9, 8, 0)   -- access mode (physical)
   self:setinbits(0x10 + 0x04,
                  31, 8, 0xFFFFFF, -- qpn
                  7, 0, 0)   -- mkey (variant part)
   self:setinbits(0x10 + 0x0C,
                  31, 31, 1, -- length64
                  23, 0, pd)
   -- Start address = 0
   -- (Remaining fields are reserved when using physical addresses)
   self:post(0x110, 0x0C)
   return self:getoutbits(0x08, 23, 0)
end

-- Return reserved lkey.
function cmdq:query_special_contexts()
   self:prepare("QUERY_SPECIAL_CONTEXTS", 0x0C, 0x0C)
   self:setinbits(0x00, 31, 16, 0x203)
   self:post(0x0C, 0x0C)
   return self:getoutbits(0x0C, 31, 0)
end

function init_seg:dump()
   print('fw_rev                  ', self:fw_rev())
   print('cmd_interface_rev       ', self:cmd_interface_rev())
   print('cmdq_phy_addr           ', self:cmdq_phy_addr())
   print('log_cmdq_size           ', self:log_cmdq_size())
   print('log_cmdq_stride         ', self:log_cmdq_stride())
   print('ready                   ', self:ready())
   print('nic_interface_supported ', self:nic_interface_supported())
   print('internal_timer          ', self:internal_timer())
   print('health_syndrome         ', self:health_syndrome())
end

function ConnectX4:new(arg)
   local self = setmetatable({}, self)
   local conf = config.parse_app_arg(arg)
   local pciaddress = pci.qualified(conf.pciaddress)

   -- Perform a hard reset of the device to bring it into a blank state.
   -- (PRM does not suggest this but it is practical for resetting the
   -- firmware from bad states.)
   pci.unbind_device_from_linux(pciaddress)
   --pci.reset_device(pciaddress)
   pci.set_bus_master(pciaddress, true)
   local base, fd = pci.map_pci_memory(pciaddress, 0, true)

   trace("Read the initialization segment")
   local init_seg = init_seg:init(base)

   --allocate and set the command queue which also initializes the nic
   local cmdq = cmdq:new(init_seg)

   --8.2 HCA Driver Start-up

   trace("Write the physical location of the command queues to the init segment.")
   init_seg:cmdq_phy_addr(memory.virtual_to_physical(cmdq.entry))

   --trace("Teardown (graceful mode) to reset NIC")
   --[[
   cmdq:teardown_hca(1)
   cmdq:disable_hca()
   debug = false
   while true do
      local flushed = cmdq:flush_pages()
      print("flushed "..flushed)
      if flushed == 0 then break end
   end
   --C.usleep(10000)
   cmdq:disable_hca()
   --]]

   trace("Wait for the 'initializing' field to clear")
   while not init_seg:ready() do
      C.usleep(1000)
   end

   init_seg:dump()

   cmdq:enable_hca()
   local issi = cmdq:query_issi()
   cmdq:dump_issi(issi)

   --os.exit(0)
   cmdq:set_issi(1)

   -- PRM: Execute QUERY_PAGES to understand the HCA need to boot pages.
   local boot_pages = cmdq:query_pages'boot'
   print("query_pages'boot'       ", boot_pages)
   assert(boot_pages > 0)

   -- PRM: Execute MANAGE_PAGES to provide the HCA with all required
   -- init-pages. This can be done by multiple MANAGE_PAGES commands.
   for i = 1, 10 do
      cmdq:alloc_pages(boot_pages)
   end

   local cur = cmdq:query_hca_cap('cur', 'general')
   local max = cmdq:query_hca_cap('max', 'general')
   print'Capabilities - current and (maximum):'
   for k in pairs(cur) do
      print(("  %-24s = %-3s (%s)"):format(k, cur[k], max[k]))
   end

   cmdq:set_hca_cap('general', cur)

   -- Initialization pages
   local init_pages = cmdq:query_pages('init')
   print("query_pages'init'       ", init_pages)
   assert(init_pages > 0)

   for i = 1, 4 do
      cmdq:alloc_pages(init_pages)
   end

   cmdq:init_hca()

   --os.exit(1)

   --debug = false
   --print("LOOPBACK:", cmdq:get_port_loopback_capability(1))
   --cmdq:enable_loopback_mode(1, true)

   local eq_uar = cmdq:alloc_uar()

   local eq = cmdq:create_eq(eq_uar, 1)
   print("eq               = " .. eq.eqn)

   local vport_ctx = cmdq:query_nic_vport_context()
   for k,v in pairs(vport_ctx) do
      print(k,v)
   end

   local vport_state = cmdq:query_vport_state()
   for k,v in pairs(vport_state) do
      print(k,v)
   end

   local tdomain = cmdq:alloc_transport_domain()
   print("transport domain = " .. tdomain)

   local tis = cmdq:create_tis(0, tdomain)
   print("tis              = " .. tis)

   local uar = cmdq:alloc_uar()
   print("uar              = " .. uar)

   local rlkey = cmdq:query_special_contexts()
   print("reserved lkey    = " .. rlkey)
   local pd = cmdq:alloc_pd()
   print("protection dom.  = " .. pd)
   local mkey = cmdq:create_mkey(pd)
   print("mkey             = " .. mkey)

   eq:poll()

   local db_ptr_cq, db_phy_cq = memory.dma_alloc(16)
   local cq, cqes = cmdq:create_cq(1024, uar, eq.eqn, db_phy_cq)
   print("cq               = " .. cq)
   local db_sq, db_phy_sq = memory.dma_alloc(16)
   -- Allocate rqmem and sqmem contiguously
   local rqmem = memory.dma_alloc(64 * 2 * 1024, 4096)
   --local sqmem = rqmem + 64 * 1024
   local sqmem = memory.dma_alloc(64 * 32 * 1024)
   --local sqmem = memory.dma_alloc(64*1024, 4096)
   --local rqmem = memory.dma_alloc(64*1024, 4096)

   local sq, wq = cmdq:create_sq(tis, cq, 11, uar, pd, db_phy_sq, sqmem, mkey)
   cmdq:modify_sq(sq, 0, 1)
   print("sq               = " .. sq)

   eq:poll()
   local db_ptr_rcq, db_phy_rcq = memory.dma_alloc(16)
   local rcq, rcqes = cmdq:create_cq(1024, uar, eq.eqn, db_phy_rcq)
   eq:poll()
   local rq, rwq = cmdq:create_rq(rcq, 0, uar, pd, db_phy_sq, rqmem)
   print("rq               = " .. rq .. "("..bit.tohex(rq)..")")
   cmdq:modify_rq(rq, 0, 1)

   local tir = cmdq:create_tir(rq, tdomain)

   local rx_table_id = cmdq:create_root_flow_table('receive')
   --local tx_table_id = cmdq:create_root_flow_table('send')
   print("rx_flow_table    = " .. rx_table_id)
   --print("tx_flow_table    = " .. tx_table_id)
   local group_id = cmdq:create_flow_group_wildcard(rx_table_id, 'receive', 0, 0)
   cmdq:set_flow_table_entry_simple(rx_table_id, 'receive', group_id, 0, tir)
   cmdq:set_flow_table_root(rx_table_id, 'receive')

   print("Creating send queue entries")

   -- 64B Work Queue Entry (WQE)
   local wq_db_t = ffi.typeof("struct { uint32_t receive, send; } *")
   local wqe_t = ffi.typeof([[union {
                                uint8_t  u8[64];
                                uint32_t u32[16];
                                uint64_t u64[8];
                             } *]])
   local wqes = ffi.cast(wqe_t, wq)
   -- Create the send WQEs
   --local nsend = 22500
   local nsend = qsize

   local p = packet.allocate()
   p.length = tonumber(os.getenv("SNABB_PKTSIZE"))
   
   ffi.fill(p.data, 6, 0xFF) -- dmac = broadcast
   p.data[11] = 1            -- last octet of smac = packet#
   p.data[12] = 0xff         -- ethertype = 0xFF00
   ffi.fill(p.data+14, p.length-14, 0xFF) -- payload = packet #
   
   ffi.fill(p.data, p.length, 0xff)

   for i = 0, nsend-1 do
      local wqe = wqes[i]
      -- 16B control segment
      wqe.u32[0] = bswap(shl(i, 8) + 0x0A)
      wqe.u32[1] = bswap(shl(sq, 8) + 4)
      if i % 256 == 0 then
         wqe.u32[2] = bswap(shl(2, 2)) -- completion always
      end
      -- 32B ethernet segment (partial inline header)
      local ninline = 16
      wqe.u32[7] = bswap(shl(ninline, 16))         -- 20 byte inline header
      ffi.copy(wqe.u8 + 0x1E, p.data, ninline) -- inline headers
      -- 16B send data segment (DMA pointer)
      wqe.u32[12] = bswap(p.length - ninline)
      wqe.u32[13] = bswap(rlkey)
      local phy = memory.virtual_to_physical(p.data + ninline)
      wqe.u32[14] = bswap(tonumber(phy) / 2^32)
      wqe.u32[15] = bswap(tonumber(phy) % 2^32)
      --[[
      print("size", bswap(wqe.u32[12]))
      print("lkey", bswap(wqe.u32[13]))
      print("high", bit.tohex(bswap(wqe.u32[14])))
      print("low",  bit.tohex(bswap(wqe.u32[15])))
      ]]--
   end

   --local cq_ci = ffi.cast("uint64_t *", ffi.cast("uint8_t *", base) + (uar * 4096) + 0x20)
   --cq_ci[0] = 0

   print("waiting for link up")
   while cmdq:query_vport_state().oper_state ~= 1 do
      C.usleep(1e6)
   end

   eq:poll()

   --print("Creating receive queue entries")

   local rwqe_t = ffi.typeof[[
     struct {
       uint32_t length;
       uint32_t lkey;
       uint32_t address_high;
       uint32_t address_low;
     } *]]

   local nrecv = nsend
   local rwqes = ffi.cast(rwqe_t, rwq)
   local rxpackets = {}
   -- Create receive queue entries
   for i = 0, nrecv-1 do
      local p = packet.allocate()
      local phy = memory.virtual_to_physical(p.data)
      rwqes[i].length = bswap(10000) -- XXX sizeof packet
      rwqes[i].lkey = bswap(rlkey)
      rwqes[i].address_high = bswap(tonumber(phy) / 2^32)
      rwqes[i].address_low  = bswap(tonumber(phy) % 2^32)
      rxpackets[i] = p
   end

   print("Ringing send doorbell")
--   for i = 0, nsend-1 do
      --C.full_memory_barrier()
      --C.usleep(100)
      -- Update doorbell record
   db_sq = ffi.cast(wq_db_t, db_sq)
   db_sq.send = bswap(nsend-1)
   db_sq.receive = bswap(nrecv)
   C.full_memory_barrier()
   -- Ring doorbell in blue flame registers
   local bf0odd  = ffi.cast("uint64_t *", ffi.cast("uint8_t *", base) + (uar * 4096) + 0x800)
   local bf0even = ffi.cast("uint64_t *", ffi.cast("uint8_t *", base) + (uar * 4096) + 0x900)
   local cqe = ffi.cast("uint32_t*", cqes)
   -- Loop refreshing the WQEs
   local index = 0
   local lib = require("core.lib")
   local bf_a, bf_b = bf0even, bf0odd
   bf_a[0] = wqes[nsend-1].u64[0]
   local start = C.get_time_ns()
   for i = 0, 1000000000 do
      local last = shr(bswap(cqe[0x3C/4]), 16) % qsize
      local opcode = ffi.cast("uint8_t*", cqe)[0x3F]
      --if opcode ~= 0 then error("bad opcode: " .. opcode) end
      if last ~= index then
         --print("->", index, last)
         while index ~= last do
            -- Bump WQE index by 16384
            local bump = (256 * qsize / (64*1024))
            wqes[index].u8[1] = wqes[index].u8[1] + bump
            index = ((index + 1) % qsize)
         end
         local ix = (index-1)%qsize
         --print("ringing "..ix)
         bf_a, bf_b = bf_b, bf_a -- swap
         bf_a[0] = wqes[ix].u64[0]
         lib.compiler_barrier()
      end
      lib.compiler_barrier()
   end
   local finish = C.get_time_ns()
   local packets = cmdq:query_vport_counter()
   print(("Processed %fM packets in %f seconds (%f Mpps)"):format(
             packets/1e6,
             tonumber(finish-start)/1e9,
             packets * 1000 / tonumber(finish-start)))
   --[[
   for i = 1, 100000 do
      bf0odd[0]  = wqes[0].u64[0]
      bf0even[0] = wqes[512].u64[0]
   end
   --]]

   C.usleep(0.5e6)

   for name, WQ in pairs({send=cqes, receive=rcqes}) do
      print("Scanning send completion queue: " .. name)
      local cq_wqe = ffi.cast(wqe_t, WQ)
      for i = 0, 0 do
         local opcodes = { [0] = 'requester', [1] = 'responder', [13] = 'requester error', [14] = 'responder error', [15] = 'invalid CQE'}
         local opcode = shr(cq_wqe[i].u8[0x3F], 4)
         local wqe_counter = shr(bswap(cq_wqe[i].u32[0x3C/4]), 16)
         local len = bswap(cq_wqe[i].u32[0x2C/4])
         print(("CQE[%03d]: WQE=%05d len=%d %2d (%s)"):format(i, wqe_counter, len, opcode, opcodes[opcode]))
         if opcode == 2 then
            print("packet dump:")
            print(lib.hexdump(ffi.string(rxpackets[wqe_counter].data, len)))
         end
         if opcode ~= 0 and opcode ~= 2 then
            local syndromes = {
               [0x1] = "Local_Length_Error",
               [0x4] = "Local_Protection_Error",
               [0x5] = "Work_Request_Flushed_Error",
               [0x6] = "Memory_Window_Bind_Error",
               [0x10] = "Bad_Response_Error",
               [0x11] = "Local_Access_Error",
               [0x12] = "Remote_Invalid_Request_Error",
               [0x13] = "Remote_Access_Error",
               [0x14] = "Remote_Operation_Error"
            }
            local syndrome = cq_wqe[i].u8[0x37]
            print(("          syndrome = 0x%x (%s)"):format(syndrome, syndromes[syndrome]))
         end
      end
   end

   eq:poll()

   print("Send CQ")
   hexdump(cqes, 0, 64)
   print("Recv CQ")
   hexdump(rcqes, 0, 64)
   --print(lib.hexdump(ffi.string(cqes, 64)))
   print("CQ doorbell")
   hexdump(db_ptr_cq, 0, 64)
   print("SQ doorbell")
   hexdump(db_sq, 0, 64)
   print("SQ[0]")
   hexdump(wq, 0, 64)

   debug = false
   local qsq = cmdq:query_sq(sq)
   print("query_sq:")
   for k,v in pairs(qsq) do
      print(k,v)
   end

   cmdq:query_vport_counter()
   print("Finished test") io.flush()

   eq:poll()

   os.exit(0)

   --[[
   cmdq:set_hca_cap()
   cmdq:query_pages()
   cmdq:manage_pages()
   cmdq:init_hca()
   cmdq:set_driver_version()
   cmdq:create_eq()
   cmdq:query_vport_state()
   cmdq:modify_vport_context()
   ]]

   function self:stop()
      pci.set_bus_master(pciaddress, false)
      pci.reset_device(pciaddress)
      pci.close_pci_resource(fd, base)
      base, fd = nil
   end

   return self
end

-- Print a hexdump in the same format as the Linux kernel.
-- 
-- Optionally take a 'dumpoffset' giving the logical address where the
-- trace starts (useful when printing multiple related hexdumps i.e.
-- for consistency with the Linux mlx5_core driver format).
function hexdump (pointer, index, bytes,  dumpoffset)
   if true then return end
   local u8 = ffi.cast("uint8_t*", pointer)
   dumpoffset = dumpoffset or 0
   for i = 0, bytes-1 do
      if i % 16 == 0 then
         if i > 0 then io.stdout:write("\n") end
         io.stdout:write(("%03x: "):format(dumpoffset+i))
      elseif i % 4 == 0 then
         io.stdout:write(" ")
      end
      io.stdout:write(bit.tohex(u8[index+i], 2))
   end
   io.stdout:write("\n")
   io.flush()
   return dumpoffset + bytes
end

function trace (...)
   print("TRACE", ...)
end

function selftest()
   io.stdout:setvbuf'no'

   local pcidev = lib.getenv("SNABB_PCI_CONNECTX4_0")
   -- XXX check PCI device type
   if not pcidev then
      print("SNABB_PCI_CONNECTX4_0 not set")
      os.exit(engine.test_skipped_code)
   end

   local device_info = pci.device_info(pcidev)
   local app = ConnectX4:new{pciaddress = pcidev}
   app:stop()
end

