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
local cast = ffi.cast
local band, bor, shl, shr, bswap, bnot =
	bit.band, bit.bor, bit.lshift, bit.rshift, bit.bswap, bit.bnot

ConnectX4 = {}
ConnectX4.__index = ConnectX4

--utils

local function getbits(val, bit2, bit1)
	local mask = shl(2^(bit2-bit1+1)-1, bit1)
	return shr(band(val, mask), bit1)
end

local function setbits(bit2, bit1, val)
	local mask = shl(2^(bit2-bit1+1)-1, bit1)
	local bits = band(shl(val, bit1), mask)
	return bswap(bits)
end

--init segment (section 4.3)

local init_seg = {}
init_seg.__index = init_seg

function init_seg:get(ofs)
	local ofs = ofs/4
	assert(ofs == math.floor(ofs))
	return bswap(self.addr[ofs])
end

function init_seg:set(ofs, val)
	local ofs = ofs/4
	assert(ofs == math.floor(ofs))
	self.addr[ofs] = bswap(val)
end

function init_seg:getbits(ofs, bit2, bit1)
	return getbits(self:get(ofs), bit2, bit1)
end

function init_seg:setbits(ofs, bit2, bit1, val)
	self:set(ofs, setbits(bit2, bit1, val))
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
		addr = cast('uint64_t', addr)
		local hi = tonumber(shr(addr, 32))
		local lo = tonumber(band(shr(addr, 12), 0xfffff))
		print(string.format('%x %x', hi, lo))
		self:set(0x10, hi) --must write the MSB of the addr first
		self:setbits(0x14, 31, 12, lo) --also resets nic_interface and log_cmdq_*
	else
		return cast('void*',
			cast('uint64_t', self:get(0x10) * 2^32 +
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
	self:setbits(0x18, 31-i, 31-i, 1)
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

	local cmdq_ptr, cmdq_phy = memory.dma_alloc(4096)
	init_seg:cmdq_phy_addr(cmdq_phy)
	while not init_seg:ready() do
      C.usleep(1000)
	end
	print'ready wohoo!'
	print('cmdq', cmdq_phy)
	init_seg:dump()

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
