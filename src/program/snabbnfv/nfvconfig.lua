-- Use of this source code is governed by the Apache 2.0 license; see COPYING.

module(...,package.seeall)

local VhostUser = require("apps.vhost.vhost_user").VhostUser
local PcapFilter = require("apps.packet_filter.pcap_filter").PcapFilter
local RateLimiter = require("apps.rate_limiter.rate_limiter").RateLimiter
local nd_light = require("apps.ipv6.nd_light").nd_light
local L2TPv3 = require("apps.keyed_ipv6_tunnel.tunnel").SimpleKeyedTunnel
local AES128gcm = require("apps.ipsec.esp").AES128gcm
local pci = require("lib.hardware.pci")
local ffi = require("ffi")
local C = ffi.C
local lib = require("core.lib")
local connectx4 = require("apps.mellanox.connectx4")

-- Load master configuration for <pciaddr> from <ports>
function load_control (ports, pciaddr)
   local c = config.new()
   local driver = assert(pci.device_info(pciaddr).driver, "Unsupported device: "..pciaddr)
   local queues = {}
   for _,t in ipairs(ports) do
      print("id: " .. t.port_id)
      table.insert(queues, {id = t.port_id, mac = t.mac_address, vlan = t.vlan})
   end
   if driver == 'apps.mellanox.connectx4' then
      config.app(c, "iocontrol",
                 connectx4.ConnectX4, {pciaddress = pciaddr, queues = queues, macvlan = false})
   end
   return c
end

-- Return worker name for <core> argument.
function core_worker (core)
   if type(core) == 'number' then return core
   elseif core               then return 'node-bound'
   else                           return 'node-unbound' end
end

-- Compute worker set for <ports>.
function load_workers (ports)
   local workers = {}
   for _,t in ipairs(ports) do workers[core_worker(t.core)] = true end
   return workers
end

-- Return name of port in <port_config>.
function port_name (port_config)
   return port_config.port_id:gsub("-", "_")
end

-- Load app network for <ports> on <worker> with <pciaddr>, and VhostUser
-- <socket>.
function load_ports (ports, worker, pciaddr, sockpath)
   local c = config.new()
   for _,t in ipairs(ports) do
      if core_worker(t.core) == worker then
         load_port(c, t, pciaddr, sockpath, #ports)
      end
   end
   return c
end

-- Configure port <t> in configuration <c> for <pciaddr> and <sockpath>.
function load_port (c, t, pciaddr, sockpath, nports)
   -- Backwards compatibity / deprecated fields
   for deprecated, current in pairs({tx_police_gbps = "tx_police",
                                     rx_police_gbps = "rx_police"}) do
      if t[deprecated] and not t[current] then
         print("Warning: "..deprecated.." is deprecated, use "..current.." instead.")
         t[current] = t[deprecated]
      end
   end
   -- Backwards compatability end
   local name = port_name(t)
   local Virtio = name.."_Virtio"
   config.app(c, Virtio, VhostUser,
              {socket_path=sockpath:format(t.port_id),
               disable_mrg_rxbuf=t.disable_mrg_rxbuf,
               disable_indirect_desc=t.disable_indirect_desc})
   local VM_rx, VM_tx = Virtio..".rx", Virtio..".tx"
   if t.tx_police then
      local TxLimit = name.."_TxLimit"
      local rate = t.tx_police * 1e9 / 8
      config.app(c, TxLimit, RateLimiter, {rate = rate, bucket_capacity = rate})
      config.link(c, VM_tx.." -> "..TxLimit..".input")
      VM_tx = TxLimit..".output"
   end
   -- If enabled, track allowed connections statefully on a per-port basis.
   -- (The table tracking connection state is named after the port ID.)
   local pf_state_table = t.stateful_filter and name
   if t.ingress_filter then
      local Filter = name.."_Filter_in"
      config.app(c, Filter, PcapFilter, { filter = t.ingress_filter,
                                          state_table = pf_state_table })
      config.link(c, Filter..".tx -> " .. VM_rx)
      VM_rx = Filter..".rx"
   end
   if t.egress_filter then
      local Filter = name..'_Filter_out'
      config.app(c, Filter, PcapFilter, { filter = t.egress_filter,
                                          state_table = pf_state_table })
      config.link(c, VM_tx..' -> '..Filter..'.rx')
      VM_tx = Filter..'.tx'
   end
   if t.tunnel and t.tunnel.type == "L2TPv3" then
      local Tunnel = name.."_Tunnel"
      local conf = {local_address = t.tunnel.local_ip,
                    remote_address = t.tunnel.remote_ip,
                    local_cookie = t.tunnel.local_cookie,
                    remote_cookie = t.tunnel.remote_cookie,
                    local_session = t.tunnel.session}
      config.app(c, Tunnel, L2TPv3, conf)
      -- Setup IPv6 neighbor discovery/solicitation responder.
      -- This will talk to our local gateway.
      local ND = name.."_ND"
      config.app(c, ND, nd_light,
                 {local_mac = t.mac_address,
                  local_ip = t.tunnel.local_ip,
                  next_hop = t.tunnel.next_hop})
      -- VM -> Tunnel -> ND <-> Network
      config.link(c, VM_tx.." -> "..Tunnel..".decapsulated")
      config.link(c, Tunnel..".encapsulated -> "..ND..".north")
      -- Network <-> ND -> Tunnel -> VM
      config.link(c, ND..".north -> "..Tunnel..".encapsulated")
      config.link(c, Tunnel..".decapsulated -> "..VM_rx)
      VM_rx, VM_tx = ND..".south", ND..".south"
   end
   if t.crypto and t.crypto.type == "esp-aes-128-gcm" then
      local Crypto = name.."_Crypto"
      config.app(c, Crypto, AES128gcm,
                 {spi = t.crypto.spi,
                  transmit_key = t.crypto.transmit_key,
                  transmit_salt = t.crypto.transmit_salt,
                  receive_key = t.crypto.receive_key,
                  receive_salt = t.crypto.receive_salt,
                  auditing = t.crypto.auditing})
      config.link(c, VM_tx.." -> "..Crypto..".decapsulated")
      config.link(c, Crypto..".decapsulated -> "..VM_rx)
      VM_rx, VM_tx = Crypto..".encapsulated", Crypto..".encapsulated"
   end
   if t.rx_police then
      local RxLimit = name.."_RxLimit"
      local rate = t.rx_police * 1e9 / 8
      config.app(c, RxLimit, RateLimiter, {rate = rate, bucket_capacity = rate})
      config.link(c, RxLimit..".output -> "..VM_rx)
      VM_rx = RxLimit..".input"
   end
   local NIC = name.."_NIC"
   local driver = assert(pci.device_info(pciaddr).driver, "Unsupported device: "..pciaddr)
   if driver == 'apps.mellanox.connectx4' then
      config.app(c, NIC,
                 connectx4.IO, {pciaddress = pciaddr, queue = t.port_id})
   elseif (driver == 'apps.intel.intel_app' or
           driver == 'apps.solarflare.solarflare') then
      local vmdq = true
      if not t.mac_address then
         assert(nports == 1, "multiple ports defined but promiscuous mode requested for port: "..name)
         vmdq = false
      end
      config.app(c, NIC, require(driver).driver,
                 {pciaddr = pciaddr,
                  vmdq = vmdq,
                  macaddr = t.mac_address,
                  vlan = t.vlan})
   else
      error("Unsupported driver: "..driver)
   end
   config.link(c, NIC..".tx".." -> "..VM_rx)
   config.link(c, VM_tx.." -> "..NIC..".rx")
end

function selftest ()
   print("selftest: lib.nfv.config")
   local pcideva = lib.getenv("SNABB_PCI0")
   if not pcideva then
      print("SNABB_PCI0 not set\nTest skipped")
      os.exit(engine.test_skipped_code)
   end
   engine.log = true
   for i, confpath in ipairs({"program/snabbnfv/test_fixtures/nfvconfig/switch_nic/x",
                              "program/snabbnfv/test_fixtures/nfvconfig/switch_filter/x",
                              "program/snabbnfv/test_fixtures/nfvconfig/switch_qos/x",
                              "program/snabbnfv/test_fixtures/nfvconfig/switch_tunnel/x",
                              "program/snabbnfv/test_fixtures/nfvconfig/scale_up/y",
                              "program/snabbnfv/test_fixtures/nfvconfig/scale_up/x",
                              "program/snabbnfv/test_fixtures/nfvconfig/scale_change/x",
                              "program/snabbnfv/test_fixtures/nfvconfig/scale_change/y"})
   do
      print("testing:", confpath)
      local ports = lib.load_conf(confpath)
      local c = load_control(ports, pcideva)
      for _, port in ipairs(ports) do
         load_port(c, port, pcideva, "/dev/null")
      end
      engine.configure(c)
      engine.main({duration = 0.25})
   end
   local c = load_ports(lib.load_conf("program/snabbnfv/test_fixtures/nfvconfig/test_functions/deprecated.port"),
                        'node-unbound', pcideva, "/dev/null")
   assert(c.apps["Test_TxLimit"])
   assert(c.apps["Test_RxLimit"])
end
