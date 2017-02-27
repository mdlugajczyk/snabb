-- Use of this source code is governed by the Apache 2.0 license; see COPYING.

module(..., package.seeall)

local lib = require("core.lib")
local worker = require("core.worker")
local counter = require("core.counter")
local nfvconfig = require("program.snabbnfv.nfvconfig")
local usage = require("program.snabbnfv.traffic.README_inc")
local ffi = require("ffi")
local C = ffi.C
local timer = require("core.timer")
local pci = require("lib.hardware.pci")
local ingress_drop_monitor = require("lib.timers.ingress_drop_monitor")
local numa = require("lib.numa")

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
   local param = {
      linkreportinterval = 0,
      loadreportinterval = 1,
      debugreportinterval = 0
   }
   function opt.B (arg) param.benchpackets = tonumber(arg) end
   function opt.h (arg) print(short_usage()) main.exit(1) end
   function opt.H (arg) print(long_usage())  main.exit(1) end
   function opt.k (arg) param.linkreportinterval = tonumber(arg) end
   function opt.l (arg) param.loadreportinterval = tonumber(arg) end
   function opt.D (arg) param.debugreportinterval = tonumber(arg) end
   function opt.b (arg) param.busywait = true end
   args = lib.dogetopt(args, opt, "hHB:k:l:D:b", long_opts)
   if #args == 3 then
      param.pciaddr, param.confpath, param.sockpath = unpack(args)
      local ok, info = pcall(pci.device_info, param.pciaddr)
      if not ok then
         print("Error: device not found " .. param.pciaddr)
         os.exit(1)
      end
      if not info.driver then
         print("Error: no driver for device " .. param.pciaddr)
         os.exit(1)
      end
      if param.benchpackets then
         print("Loading " .. param.confpath)
         local ports = lib.load_conf(param.confpath)
         local c = nfvconfig.load_control(ports, param.pciaddr)
         local workers = nfvconfig.load_workers(ports)
         engine.configure(c)
         for w, _ in pairs(workers) do
            worker.start(
               worker_name(w),
               'require("program.snabbnfv.traffic.traffic").nfv_worker('..
                  lib.conf_string(param)..','..
                  lib.conf_string(w)..')'
            )
         end
         local function workers_done ()
            for _, status in pairs(worker.status()) do
               if status.alive then return false end
            end
            return true
         end
         engine.main({done = workers_done})
      else
         local workers = {}
         local function loader (ports)
            engine.configure(nfvconfig.load_control(ports, param.pciaddr))
            local new_workers = nfvconfig.load_workers(ports)
            -- Stop obsolete workers
            for w in pairs(workers) do
               if not new_workers[w] then
                  print("Stopping: "..worker_name(w))
                  worker.stop(worker_name(w))
               end
            end
            -- Start new workers
            for w in pairs(new_workers) do
               if not workers[w] then
                  print("Starting: "..worker_name(w))
                  worker.start(
                     worker_name(w),
                     'require("program.snabbnfv.traffic.traffic").nfv_worker('..
                        lib.conf_string(param)..','..
                        lib.conf_string(w)..')'
                  )
               end
            end
            -- Remember active worker set
            workers = new_workers
         end
         with_nfvconf(param.confpath, loader, 'verbose')
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

function nfv_worker (param, worker)
   if param.busywait then engine.busywait = true end
   if type(worker) == 'number' then
      numa.bind_to_cpu(worker)
   elseif worker == 'node-bound' then
      numa.bind_to_numa_node(numa.pci_get_numa_node(param.pciaddr))
   end
   if param.loadreportinterval > 0 then
      local t = timer.new("nfvloadreport", say(worker, engine.report_load), param.loadreportinterval*1e9, 'repeating')
      timer.activate(t)
   end
   if param.linkreportinterval > 0 then
      local t = timer.new("nfvlinkreport", say(worker, engine.report_links), param.linkreportinterval*1e9, 'repeating')
      timer.activate(t)
      end
   if param.debugreportinterval > 0 then
      local t = timer.new("nfvdebugreport", say(worker, engine.report_apps), param.debugreportinterval*1e9, 'repeating')
      timer.activate(t)
   end
   if param.benchpackets then
      say(worker, function () print("starting (benchmark mode)") end)()
      bench(param.pciaddr, param.confpath, param.sockpath, worker, param.benchpackets)
   else
      say(worker, function () print("starting") end)()
      traffic(param.pciaddr, param.confpath, param.sockpath, worker)
   end
end

-- Run in real traffic mode.
function traffic (pciaddr, confpath, sockpath, worker)
   -- Flush logs every second.
   timer.activate(timer.new("flush", io.flush, 1e9, 'repeating'))
   timer.activate(ingress_drop_monitor.new({action='warn'}):timer())
   local function loader (ports)
      engine.configure(nfvconfig.load_ports(ports, worker, pciaddr, sockpath))
   end
   with_nfvconf(confpath, loader)
end

-- Run in benchmark mode.
function bench (pciaddr, confpath, sockpath, worker, npackets)
   require("jit.dump").start("+r", "jit.dump")

   local ports = lib.load_conf(confpath)
   local nic = (nfvconfig.port_name(ports[1])).."_NIC"

   engine.configure(nfvconfig.load_ports(ports, worker, pciaddr, sockpath))

   -- From designs/nfv
   local start, packets, bytes = 0, 0, 0
   local done = function ()
      local input = link.stats(engine.app_table[nic].input.rx)
      if start == 0 and input.rxpackets > 0 then
         -- started receiving, record time and packet count
         packets = input.rxpackets
         bytes = input.rxbytes
         start = C.get_monotonic_time()

         require("lib.traceprof.traceprof").start()

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
   require("lib.traceprof.traceprof").stop()
   require("jit.p").stop()
end

function worker_name (worker)
   return "worker("..worker..")"
end

function say (worker, report_fn)
   local voice = worker_name(worker)..": "
   return function () io.write(voice); report_fn() end
end

function with_nfvconf (confpath, loader, verbose)
   local mtime = 0
   local needs_reconfigure = true
   function check_for_reconfigure()
      needs_reconfigure = C.stat_mtime(confpath) ~= mtime
   end
   timer.activate(timer.new("reconf", check_for_reconfigure, 1e9, 'repeating'))
   while true do
      needs_reconfigure = false
      if verbose then print("Loading " .. confpath) end
      mtime = C.stat_mtime(confpath)
      if mtime == 0 and verbose then
         print(("WARNING: File '%s' does not exist."):format(confpath))
      end
      loader(lib.load_conf(confpath))
      engine.main({done=function() return needs_reconfigure end})
   end
end
