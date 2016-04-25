--go@ git up
--- Device driver for the Mellanox ConnectX-4 series Ethernet controller.

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

ConnectX4 = {}
ConnectX4.__index = ConnectX4

--utils

local function alloc_pages(pages)
   local ptr, phy = memory.dma_alloc(4096 * pages)
   assert(band(phy, 0xfff) == 0) --the phy address must be 4K-aligned
   return ptr, phy
end

function getint(addr, ofs)
   local ofs = ofs/4
   assert(ofs == floor(ofs))
   return bswap(addr[ofs])
end

function setint(addr, ofs, val)
   local ofs = ofs/4
   assert(ofs == floor(ofs))
   addr[ofs] = bswap(val)
end

local function getbits(val, bit2, bit1)
   local mask = shl(2^(bit2-bit1+1)-1, bit1)
   return shr(band(val, mask), bit1)
end

local function ptrbits(ptr, bit2, bit1)
   local addr = cast('uint64_t', ptr)
   return tonumber(getbits(addr, bit2, bit1))
end

local function setbits(bit2, bit1, val)
   local mask = shl(2^(bit2-bit1+1)-1, bit1)
   local bits = band(shl(val, bit1), mask)
   return bits
end

local function getbit(addr, bit)
   local i = math.floor(bit / 32)
   local j = bit % 32
   return getbits(getint(addr, i * 4), j, j)
end

--init segment (section 4.3)

local init_seg = {}
init_seg.__index = init_seg

function init_seg:getbits(ofs, bit2, bit1)
   return getbits(getint(self.ptr, ofs), bit2, bit1)
end

function init_seg:setbits(ofs, bit2, bit1, val)
   setint(self.ptr, ofs, setbits(bit2, bit1, val))
end

function init_seg:init(addr)
   return setmetatable({ptr = cast('uint32_t*', addr)}, self)
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
local SET_ISSI           = 0x10B
local SET_DRIVER_VERSION = 0x10D

function cmdq:new(init_seg)
   local ptr, phy = alloc_pages(1)
   local ib_ptr, ib_phy = alloc_pages(1)
   local ob_ptr, ob_phy = alloc_pages(1)
   return setmetatable({
      ptr = cast('uint32_t*', ptr),
      phy = phy,
      ib_ptr = cast('uint32_t*', ib_ptr),
      ob_ptr = cast('uint32_t*', ob_ptr),
      init_seg = init_seg,
      size = init_seg:log_cmdq_size(),
      stride = init_seg:log_cmdq_stride(),
   }, self)
end

function cmdq:getbits(ofs, bit2, bit1)
   return getbits(getint(self.ptr, ofs), bit2, bit1)
end

function cmdq:setbits(ofs, bit2, bit1, val)
   setint(self.ptr, ofs, setbits(bit2, bit1, val))
end

function cmdq:setinbits(ofs, bit2, bit1, val)
   assert(band(ofs, 3) == 0) --offset must be 4-byte aligned
   if ofs <= 12 then --inline
      self:setbits(0x10 + ofs, bit2, bit1, val)
   else --mailbox
      print(ofs)
      --self:setbits(
   end
end

function cmdq:getoutbits(ofs, bit2, bit1)
   return self:getbits(0x20 + ofs, bit2, bit1)
end

function cmdq:getoutaddr(ofs)
   local ofs = (0x20 + ofs) / 4
   assert(ofs == math.floor(ofs))
   return self.ptr + ofs
end

function cmdq:getbit(ofs, bit)
   return getbit(self:getoutaddr(ofs), bit)
end

local errors = {
   'signature error',
   'token error',
   'bad block number',
   'bad output pointer. pointer not aligned to mailbox size',
   'bad input pointer. pointer not aligned to mailbox size',
   'internal error',
   'input len error. input length less than 0x8',
   'output len error. output length less than 0x8',
   'reserved not zero',
   'bad command type',
}
local function checkz(z)
   if z == 0 then return end
   error('command error: '..(errors[z] or z))
end

function cmdq:post(in_sz, out_sz)
   self:setbits(0x00, 31, 24, 0x7) --type

   self:setbits(0x04, 31, 0, in_sz) --input_length
   self:setbits(0x38, 31, 0, out_sz) --output_length

   self:setbits(0x08, 31, 0, ptrbits(self.ib_addr, 63, 32))
   self:setbits(0x0C, 31, 9, ptrbits(self.ib_addr, 31, 9))

   self:setbits(0x30, 31, 0, ptrbits(self.ob_addr, 63, 32))
   self:setbits(0x34, 31, 9, ptrbits(self.ob_addr, 31, 9))

   self:setbits(0x3C, 0, 0, 1) --set ownership

   self.init_seg:ring_doorbell(0) --post command

   --poll for command completion
   while self:getbits(0x3C, 0, 0) == 1 do
      C.usleep(1000)
   end

   local token     = self:getbits(0x3C, 31, 24)
   local signature = self:getbits(0x3C, 23, 16)
   local status    = self:getbits(0x3C,  7,  1)

   checkz(status)

   return signature, token
end

--see 12.2 Return Status Summary
function cmdq:checkstatus()
   local status = self:getoutbits(0x00, 31, 24)
   local syndrome = self:getoutbits(0x04, 31, 0)
   if status == 0 then return end
   error(string.format('status: %d, syndrome: %d', status, syndrome))
end

function cmdq:enable_hca()
   self:setinbits(0x00, 31, 16, ENABLE_HCA)
   self:post(0x0C + 4, 0x08 + 4)
end

function cmdq:query_issi()
   self:setinbits(0x00, 31, 16, QUERY_ISSI)
   self:post(0x0C + 4, 0x6C + 4)
   self:checkstatus()
   local cur_issi = self:getoutbits(0x08, 15, 0)
   local t = {}
   for i=0,80-1 do
      t[i] = self:getbit(0x20, i) == 1 or nil
   end
   return {
      cur_issi = cur_issi,
      sup_issi = t,
   }
end

function cmdq:set_issi(issi)
   self:setinbits(0x00, 31, 16, SET_ISSI)
   self:setinbits(0x08, 15, 0, issi)
   self:post(0x0C + 4, 0x0C + 4)
   self:checkstatus()
end

function cmdq:dump_issi(issi)
   print('  cur_issi              ', issi.cur_issi)
   print('  sup_issi              ')
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
   self:setinbits(0x00, 31, 16, QUERY_PAGES)
   self:setinbits(0x04, 15, 0, codes[which])
   self:post(0x0C + 4, 0x0C + 4)
   self:checkstatus()
   return self:getoutbits(0x0C, 31, 0)
end

function cmdq:alloc_pages(addr, num_pages)
   self:setinbits(0x00, 31, 16, MANAGE_PAGES)
   self:setinbits(0x04, 15, 0, 1) --alloc
   self:setinbits(0x0C, 31, 0, num_pages)
   local addr = cast('char*', addr)
   for i=0, num_pages-1 do
      self:setbits(0x10 + i*8, 31,  0, ptrbits(addr + 4096*i, 63, 32))
      self:setbits(0x14 + i*8, 31, 12, ptrbits(addr + 4096*i, 31, 12))
   end
   self:post(0x10 + 4 + num_pages*8, 0x0C + 4)
   self:checkstatus()
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

   pci.unbind_device_from_linux(pciaddress)
   pci.set_bus_master(pciaddress, true)
   local base, fd = pci.map_pci_memory(pciaddress, 0)

   local init_seg = init_seg:init(base)

   --allocate and set the command queue which also initializes the nic
   local cmdq = cmdq:new(init_seg)

   --8.2 HCA Driver Start-up

   init_seg:cmdq_phy_addr(cmdq.phy)

   --wait until the nic is ready
   while not init_seg:ready() do
      C.usleep(1000)
   end

   init_seg:dump()

   cmdq:enable_hca()

   local issi = cmdq:query_issi()
   cmdq:dump_issi(issi)
   cmdq:set_issi(0)

   local boot_pages = cmdq:query_pages'boot'
   print("query_pages'boot'       ", boot_pages)
   assert(boot_pages > 0)

   local bp_ptr, bp_phy = memory.dma_alloc(4096 * boot_pages)
   assert(band(bp_phy, 0xfff) == 0) --the phy address must be 4K-aligned
   cmdq:alloc_pages(bp_phy, boot_pages)

   --[[
   cmdq:query_hca_cap()
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
      if not base then return end
      pci.set_bus_master(pciaddress, false)
      pci.close_pci_resource(fd, base)
      base, fd = nil
   end

   return self
end

function selftest()
   local pcidev1 = lib.getenv("SNABB_PCI_CONNECTX40") or lib.getenv("SNABB_PCI0")
   local pcidev2 = lib.getenv("SNABB_PCI_CONNECTX41") or lib.getenv("SNABB_PCI1")
   if not pcidev1
      or pci.device_info(pcidev1).driver ~= 'apps.mellanox.connectx4'
      or not pcidev2
      or pci.device_info(pcidev2).driver ~= 'apps.mellanox.connectx4'
   then
      print("SNABB_PCI_CONNECTX4[0|1]/SNABB_PCI[0|1] not set or not suitable.")
      os.exit(engine.test_skipped_code)
   end

   local device_info_1 = pci.device_info(pcidev1)
   local device_info_2 = pci.device_info(pcidev2)

   local app1 = ConnectX4:new{pciaddress = pcidev1}
   local app2 = ConnectX4:new{pciaddress = pcidev2}

   engine.main({duration = 1, report={showlinks=true, showapps=false}})

   app1:stop()
   app2:stop()
end
