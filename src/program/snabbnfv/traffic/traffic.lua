-- Use of this source code is governed by the Apache 2.0 license; see COPYING.

module(..., package.seeall)

local lib = require("core.lib")
local worker = require("core.worker")
local nfvconfig = require("program.snabbnfv.nfvconfig")
local usage = require("program.snabbnfv.traffic.README_inc")
local ffi = require("ffi")
local C = ffi.C
local timer = require("core.timer")
local pci = require("lib.hardware.pci")
local ingress_drop_monitor = require("lib.timers.ingress_drop_monitor")
local counter = require("core.counter")

local long_opts = {
   benchmark     = "B",
   help          = "h",
   ["link-report-interval"] = "k",
   ["load-report-interval"] = "l",
   ["debug-report-interval"] = "D",
   ["busy"] = "b",
   ["long-help"] = "H"
}

function run (args)
   local opt = {}
   local worker_args = {
      linkreportinterval = 0,
      loadreportinterval = 1,
      debugreportinterval = 0
   }
   function opt.B (arg) worker_args.benchpackets = tonumber(arg) end
   function opt.h (arg) print(short_usage()) main.exit(1) end
   function opt.H (arg) print(long_usage())  main.exit(1) end
   function opt.k (arg) worker_args.linkreportinterval = tonumber(arg) end
   function opt.l (arg) worker_args.loadreportinterval = tonumber(arg) end
   function opt.D (arg) worker_args.debugreportinterval = tonumber(arg) end
   function opt.b (arg) worker_args.busywait = true end
   args = lib.dogetopt(args, opt, "hHB:k:l:D:b", long_opts)
   if #args == 3 then
      local pciaddr, confpath, sockpath = unpack(args)
      local ok, info = pcall(pci.device_info, pciaddr)
      if not ok then
         print("Error: device not found " .. pciaddr)
         os.exit(1)
      end
      if not info.driver then
         print("Error: no driver for device " .. pciaddr)
         os.exit(1)
      end
      worker_args.pciaddr = pciaddr
      worker_args.sockpath = sockpath
      if worker_args.benchpackets then
         print("Loading " .. confpath)
         local c, workers = nfvconfig.load_ports(confpath, pciaddr)
         engine.configure(c)
         for core, ports in pairs(workers) do
            worker.start(
               worker_name(core),
               'require("program.snabbnfv.traffic.traffic").nfv_worker('..
                  lib.conf_string(worker_args)..','..
                  lib.conf_string(ports)..','..
                  tostring(core)..')'
            )
         end
         local function workers_done ()
            for _, status in pairs(worker.status()) do
               if status.alive then return false end
            end
         end
         engine.main({done = workers_done})
      else
         local mtime = 0
         local needs_reconfigure = true
         function check_for_reconfigure()
            needs_reconfigure = C.stat_mtime(confpath) ~= mtime
         end
         timer.activate(timer.new("reconf", check_for_reconfigure, 1e9, 'repeating'))
         while true do
            needs_reconfigure = false
            print("Loading " .. confpath)
            mtime = C.stat_mtime(confpath)
            if mtime == 0 then
               print(("WARNING: File '%s' does not exist."):format(confpath))
            end
            local c, workers = nfvconfig.load_ports(confpath, pciaddr)
            engine.configure(c)
            for core, ports in pairs(workers) do
               worker.start(
                  worker_name(core),
                  'require("program.snabbnfv.traffic.traffic").nfv_worker('..
                     lib.conf_string(worker_args)..','..
                     lib.conf_string(ports)..','..
                     tostring(core)..')'
               )
            end
            engine.main({done=function() return needs_reconfigure end})
            for name, status in pairs(worker.status()) do
               if status.alive then worker.stop(name) end
            end
         end
      end
   else
      print("Wrong number of arguments: " .. tonumber(#args))
      print()
      print(short_usage())
      main.exit(1)
   end
end

function short_usage () return (usage:gsub("%s*CONFIG FILE FORMAT:.*", "")) end
function long_usage () return usage end

function worker_name (core)
   if type(core) == 'number' then return "worker("..core..")"
   elseif  core              then return "worker(bound)"
   else                           return "worker(unbound)"    end
end

function nfv_worker (args, ports, core)
   engine.busywait = args.busywait
   if type(core) == 'number' then
      numa.bind_to_cpu(core)
   elseif core then
      numa.bind_to_numa_node(numa.pci_get_numa_node(args.pciaddr))
   end
   local voice = worker_name(core)..": "
   local function say (report_fn)
      return function () io.write(voice); report_fn() end
   end
   if args.loadreportinterval > 0 then
      local t = timer.new("nfvloadreport", say(engine.report_load), args.loadreportinterval*1e9, 'repeating')
      timer.activate(t)
   end
   if args.linkreportinterval > 0 then
      local t = timer.new("nfvlinkreport", say(engine.report_links), args.linkreportinterval*1e9, 'repeating')
      timer.activate(t)
      end
   if args.debugreportinterval > 0 then
      local t = timer.new("nfvdebugreport", say(engine.report_apps), args.debugreportinterval*1e9, 'repeating')
      timer.activate(t)
   end
   if args.benchpackets then
      print(voice.."starting (benchmark mode)")
      bench(args.pciaddr, ports, args.sockpath, args.benchpackets)
   else
      print(voice.."starting")
      traffic(args.pciaddr, ports, args.sockpath)
   end
end

-- Run in real traffic mode.
function traffic (pciaddr, ports, sockpath)
   -- Flush logs every second.
   timer.activate(timer.new("flush", io.flush, 1e9, 'repeating'))
   timer.activate(ingress_drop_monitor.new({action='warn'}):timer())
   engine.configure(nfvconfig.ports_config(ports, pciaddr, sockpath))
   engine.main()
end

-- Run in benchmark mode.
function bench (pciaddr, ports, sockpath, npackets)
   local nic = (nfvconfig.port_name(ports[1])).."_NIC"

   engine.configure(nfvconfig.ports_config(ports, pciaddr, sockpath))

   -- From designs/nfv
   local start, packets, bytes = 0, 0, 0
   local done = function ()
      local input = link.stats(engine.app_table[nic].input.rx)
      if start == 0 and input.rxpackets > 0 then
         -- started receiving, record time and packet count
         packets = input.rxpackets
         bytes = input.rxbytes
         start = C.get_monotonic_time()
         if os.getenv("NFV_PROF") then
            require("jit.p").start(os.getenv("NFV_PROF"), os.getenv("NFV_PROF_FILE"))
         else
            print("No LuaJIT profiling enabled ($NFV_PROF unset).")
         end
         if os.getenv("NFV_DUMP") then
            require("jit.dump").start(os.getenv("NFV_DUMP"), os.getenv("NFV_DUMP_FILE"))
            main.dumping = true
         else
            print("No LuaJIT dump enabled ($NFV_DUMP unset).")
         end
      end
      return input.rxpackets - packets >= npackets
   end

   engine.main({done = done, no_report = true})
   local finish = C.get_monotonic_time()

   local runtime = finish - start
   local breaths = tonumber(counter.read(engine.breaths))
   local input = link.stats(engine.app_table[nic].input.rx)
   packets = input.rxpackets - packets
   bytes = input.rxbytes - bytes
   engine.report()
   print()
   print(("Processed %.1f million packets in %.2f seconds (%d bytes; %.2f Gbps)"):format(packets / 1e6, runtime, bytes, bytes * 8.0 / 1e9 / runtime))
   print(("Made %s breaths: %.2f packets per breath; %.2fus per breath"):format(lib.comma_value(breaths), packets / breaths, runtime / breaths * 1e6))
   print(("Rate(Mpps):\t%.3f"):format(packets / runtime / 1e6))
   require("jit.p").stop()
end

