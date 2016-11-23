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

-- Return name of port in <port_config>.
function port_name (port_config)
   return port_config.port_id:gsub("-", "_")
end

-- Load NFV configuration <file> and return controller configuration for
-- <pciaddr> and ports for cores
function load_ports (file, pciaddr)
   local ports = lib.load_conf(file)
   local c = config.new()
   -- Configure NIC queues if applicable
   local driver = assert(pci.device_info(pciaddr).driver, "Unsupported device: "..pciaddr)
   if driver == 'apps.mellanox.connectx4' then
      -- XXX - Configure ConnectX4 controller app here
   end
   -- Organize ports by worker
   local workers = {}
   for _, port in ipairs(ports) do
      local worker = port.core or false
      workers[worker] = workers[worker] or {}
      table.insert(workers[worker], port)
   end
   return c, workers
end

-- Compile app network for <ports>, <pciaddr>, and VhostUser <socket>.
function ports_config (ports, pciaddr, sockpath)
   local c = config.new()
   for i,t in ipairs(ports) do
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
         -- XXX - Configure ConnectX4 queue app here
      elseif (driver == 'apps.intel.intel_app' or
              driver == 'apps.solarflare.solarflare') then
         local vmdq = true
         if not t.mac_address then
            assert(#ports == 1, "multiple ports defined but promiscuous mode requested for port: "..name)
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

   -- Return configuration c.
   return c
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
      local m, workers = load_ports(confpath, pcideva)
      local w = ports_config(workers[false], pcideva, "/dev/null")
      engine.configure(m)
      engine.main({duration = 0.25})
      engine.configure(w)
      engine.main({duration = 0.25})
   end
   local m, workers = load_ports("program/snabbnfv/test_fixtures/nfvconfig/test_functions/deprecated.port", pcideva, "/dev/null")
   local w = ports_config(workers[false], pcideva, "/dev/null")
   assert(w.apps["Test_TxLimit"])
   assert(w.apps["Test_RxLimit"])
end
