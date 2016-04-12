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
local band, bor, shl, shr, bswap = bit.band, bit.bor, bit.lshift, bit.rshift, bit.bswap

ConnectX4 = {}
ConnectX4.__index = ConnectX4

--init segment

local init_seg = {}
init_seg.__index = init_seg

function init_seg:init(addr)
   return setmetatable({addr = cast('uint32_t*', addr)}, self)
end

local function split32(x)
   return shr(x, 16), band(x, 0xffff) --hi, lo
end

function init_seg:fw_rev()
   local min, maj = split32(bswap(self.addr[0]))
   local _, sub = split32(bswap(self.addr[1]))
   return maj, min, sub
end

function init_seg:cmd_interface_rev()
   return (split32(bswap(self.addr[1])))
end

function init_seg:cmdq_phy_addr(addr)
	if addr then
		addr = cast('uint64_t', addr)
		self.addr[4] = bswap(cast('uint32_t', shr(addr, 32)))
		self.addr[5] = bswap(bor(
				band(cast('uint32_t', addr), 0xfffff000),
				band(bswap(self.addr[5]),    0x00000fff)))
	else
		return ffi.cast('void*',
			ffi.cast('uint64_t', bswap(self.addr[4])) * 2^32 +
				ffi.cast('uint64_t', band(bswap(self.addr[5]), 0xfffff000)))
	end
end

function ConnectX4:new(arg)
   local self = setmetatable({}, self)
   local conf = config.parse_app_arg(arg)
   local pciaddress = pci.qualified(conf.pciaddress)

   pci.unbind_device_from_linux(pciaddress)
   pci.set_bus_master(pciaddress, true)
   local base, fd = pci.map_pci_memory(pciaddress, 0)

   local init_seg = init_seg:init(base)

   print(init_seg:fw_rev())
   print(init_seg:cmd_interface_rev())
	print(init_seg:cmdq_phy_addr())

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
