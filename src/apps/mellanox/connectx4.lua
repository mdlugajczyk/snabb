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

--init segment (section 4.3)

local init_seg = {}
init_seg.__index = init_seg

function init_seg:getbits(ofs, bit2, bit1)
	return getbits(getint(self.addr, ofs), bit2, bit1)
end

function init_seg:setbits(ofs, bit2, bit1, val)
	setint(self.addr, ofs, setbits(bit2, bit1, val))
end

function init_seg:init(addr)
   return setmetatable({addr = cast('uint32_t*', addr)}, self)
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

local cmd_queue = {}
cmd_queue.__index = cmd_queue

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

function cmd_queue:new(addr, init_seg)
	return setmetatable({
		addr = ffi.cast('uint32_t*', addr),
		init_seg = init_seg,
		size = init_seg:log_cmdq_size(),
		stride = init_seg:log_cmdq_stride(),
	}, self)
end

function cmd_queue:getbits(ofs, bit2, bit1)
	return getbits(getint(self.addr, ofs), bit2, bit1)
end

function cmd_queue:setbits(ofs, bit2, bit1, val)
	setint(self.addr, ofs, setbits(bit2, bit1, val))
end

function cmd_queue:setinbits(ofs, bit2, bit1, val)
	self:setbits(0x10 + ofs, bit2, bit1, val)
end

function cmd_queue:getoutbits(ofs, bit2, bit1)
	return self:getbits(0x20 + ofs, bit2, bit1)
end

function cmd_queue:post(in_sz, out_sz)
	local imptr = 0
	local omptr = 0

	self:setbits(0x00, 31, 24, 0x7) --type

	self:setbits(0x04, 31, 0, in_sz) --input_length
	self:setbits(0x38, 31, 0, out_sz) --output_length

	self:setbits(0x08, 31, 0, ptrbits(imptr, 63, 32))
	self:setbits(0x0C, 31, 9, ptrbits(imptr, 31, 9))

	self:setbits(0x30, 31, 0, ptrbits(omptr, 63, 32))
	self:setbits(0x34, 31, 9, ptrbits(omptr, 31, 9))

	self:setbits(0x3C, 0, 0, 1) --set ownership

	self.init_seg:ring_doorbell(0) --post command

	--poll for command completion
	while self:getbits(0x3C, 0, 0) == 1 do
		C.usleep(1000)
	end

	local token     = self:getbits(0x3C, 31, 24)
	local signature = self:getbits(0x3C, 23, 16)
	local status    = self:getbits(0x3C,  7,  1)

	return status, signature, token
end

function cmd_queue:enable_hca()
	self:setinbits(0x00, 31, 16, ENABLE_HCA)
	local status = self:post(0x0C + 4, 0x08 + 4)
	print(status)
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
	local cmdq_ptr, cmdq_phy = memory.dma_alloc(4096)
	assert(band(cmdq_phy, 0xfff) == 0) --the phy address must be 4K-aligned
	init_seg:cmdq_phy_addr(cmdq_phy)

	--wait until the nic is ready
	while not init_seg:ready() do
      C.usleep(1000)
	end

	init_seg:dump()

	local cmdq = cmd_queue:new(cmdq_ptr, init_seg)
	cmdq:enable_hca()

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
