-- Use of this source code is governed by the Apache 2.0 license; see COPYING.

module(...,package.seeall)

local packet    = require("core.packet")
local lib       = require("core.lib")
local link      = require("core.link")
local config    = require("core.config")
local timer     = require("core.timer")
local shm       = require("core.shm")
local histogram = require('core.histogram')
local counter   = require("core.counter")
local zone      = require("jit.zone")
local jit       = require("jit")
local ffi       = require("ffi")
local C         = ffi.C
require("core.packet_h")

-- Packet per pull
pull_npackets = math.floor(link.max / 10)

-- Set to true to enable logging
log = false
local use_restart = false

test_skipped_code = 43

-- The set of all active apps and links in the system.
-- Indexed both by name (in a table) and by number (in an array).
app_table,  app_array  = {}, {}
link_table, link_array = {}, {}

configuration = config.new()

-- Counters for statistics.
breaths   = counter.create("engine/breaths.counter")   -- Total breaths taken
frees     = counter.create("engine/frees.counter")     -- Total packets freed
freebits  = counter.create("engine/freebits.counter")  -- Total packet bits freed (for 10GbE)
freebytes = counter.create("engine/freebytes.counter") -- Total packet bytes freed
configs   = counter.create("engine/configs.counter")   -- Total configurations loaded

-- Events

local tl = require("core.timeline")
timeline = tl.new("engine/timeline")

local function define_event(level, message)
   return tl.define(timeline, 'engine', level, message)
end

-- Sleeping

local event_sleep_Hz = define_event('trace', [[sleeping $usec for $Hz
The engine requests that the kernel suspend this process for a period of
microseconds in order to reduce CPU utilization and achieve a fixed
frequency of breaths per second (Hz).]])

local event_sleep_idle = define_event('trace', [[sleeping $usec on idle
The engine requests that the kernel suspend this process for a period
of microseconds in order to reduce CPU utilization because idleness
has been detected (a breath in which no packets were processed.)]])

local event_wakeup = define_event('trace', [[wakeup from sleep
The engine resumes operation after sleeping voluntarily.]])

-- Traffic processing start/stop

local event_traffic_start = define_event('info', [[start traffic processing
The engine starts the traffic processing loop.]])

local event_traffic_stop = define_event('info', [[stop traffic processing
The engine stops the traffic processing loop.]])

-- Breath steps

local event_breath_start = define_event('info', [[start $breath: total $packets $bytes $etherbits
The engine starts an iteration of the packet-processing event loop (a
"breath".)

The total count of packets, bytes, and bits (including layer-1
ethernet overhead) that the engine has processed are included. These
can be used to track the rate of traffic processing.
]])

local event_breath_pulled = define_event('trace', [[pulled input packets
The engine has "pulled" new packets into the event loop for processing.]])

local event_breath_pushed = define_event('trace', [[pushed output packets
The engine has "pushed" packets one step through the processing network.]])

local event_breath_end = define_event('info', [[end $breath: processed $packets $bpp
The engine completes an iteration of the event loop (a "breath.")

'packets' gives the number of packets that were processed during the breath.
'bpp' gives the average number of bytes per packet.

Note: 'packets' is an internal measure of how many packets were
deallocated (freed) during processing. This does not necessarily
correspond directly to ingress or egress packets on a given interface.
]])

local event_breath_initialized = define_event('trace', [[initialized breath at $unixnanos
The engine has completed initialization for the breath: synchronized
the current time and handled any pending error recovery.]])

local event_ran_timers = define_event('trace', [[ran timers
The engine polled its timers and executed any that were expired.]])

local event_commit_counters = define_event('trace', [[commit counters
The engine commits the latest counter values to externally visible
shared memory.]])

-- Table to be populated with per-link events
local events_link_packet = {}

local event_packet_length = define_event('app', [[packet length $bytes]])

-- Breathing regluation to reduce CPU usage when idle by calling usleep(3).
--
-- There are two modes available:
--
--   Hz = <n> means to aim for an exact <n> breaths per second rhythm
--   Hz = false means dynamic adjustment of the breathing interval
--
-- Dynamic adjustment automatically scales the time to sleep between
-- breaths from nothing up to maxsleep (default: 100us). If packets
-- are processed during a breath then the sleep period is halved, and
-- if no packets are processed during a breath then the sleep interval
-- is increased by one microsecond.
--
-- The default is dynamic adjustment which should work well for the
-- majority of cases.

Hz = false
sleep = 0
maxsleep = 100

-- busywait: If true then the engine will poll for new data in a tight
-- loop (100% CPU) instead of sleeping according to the Hz setting.
busywait = false

-- Return current monotonic time in seconds.
-- Can be used to drive timers in apps.
monotonic_now = false
function now ()
   return monotonic_now or C.get_monotonic_time()
end

-- Run app:methodname() in protected mode (pcall). If it throws an
-- error app will be marked as dead and restarted eventually.
function with_restart (app, method)
   local status, result
   if use_restart then
      -- Run fn in protected mode using pcall.
      status, result = pcall(method, app)

      -- If pcall caught an error mark app as "dead" (record time and cause
      -- of death).
      if not status then
         app.dead = { error = result, time = now() }
      end
   else
      status, result = true, method(app)
   end
   return status, result
end

-- Restart dead apps.
function restart_dead_apps ()
   if not use_restart then return end
   local restart_delay = 2 -- seconds
   local actions = { start={}, restart={}, reconfig={}, keep={}, stop={} }
   local restart = false

   -- Collect 'restart' actions for dead apps and log their errors.
   for i = 1, #app_array do
      local app = app_array[i]
      if app.dead and (now() - app.dead.time) >= restart_delay then
         restart = true
         io.stderr:write(("Restarting %s (died at %f: %s)\n")
                         :format(app.appname, app.dead.time, app.dead.error))
         table.insert(actions.restart, app.appname)
      else
         table.insert(actions.keep, app.appname)
      end
   end

   -- Restart dead apps if necessary.
   if restart then apply_config_actions(actions, configuration) end
end

-- Configure the running app network to match new_configuration.
--
-- Successive calls to configure() will migrate from the old to the
-- new app network by making the changes needed.
function configure (new_config)
   local actions = compute_config_actions(configuration, new_config)
   apply_config_actions(actions, new_config)
   configuration = new_config
   counter.add(configs)
end

-- Return the configuration actions needed to migrate from old config to new.
--
-- Here is an example return value for a case where two apps must
-- start, one must stop, and one is kept as it is:
--   { start = {'newapp1', 'newapp2'},
--     stop  = {'deadapp1'},
--     keep  = {'oldapp1'},
--     restart = {},
--     reconfig = {}
--   }
function compute_config_actions (old, new)
   local actions = { start={}, restart={}, reconfig={}, keep={}, stop={} }
   for appname, info in pairs(new.apps) do
      local class, arg = info.class, info.arg
      local action = nil
      if not old.apps[appname]                then action = 'start'
      elseif old.apps[appname].class ~= class then action = 'restart'
      elseif not lib.equal(old.apps[appname].arg, arg)
                                              then action = 'reconfig'
      else                                         action = 'keep'  end
      table.insert(actions[action], appname)
   end
   for appname in pairs(old.apps) do
      if not new.apps[appname] then
         table.insert(actions['stop'], appname)
      end
   end
   return actions
end

-- Update the active app network by applying the necessary actions.
function apply_config_actions (actions, conf)
   -- The purpose of this function is to populate these tables:
   local new_app_table,  new_app_array  = {}, {}
   local new_link_table, new_link_array = {}, {}
   -- Temporary name->index table for use in link renumbering
   local app_name_to_index = {}
   -- Table of functions that execute config actions
   local ops = {}
   function ops.stop (name)
      if app_table[name].stop then
         app_table[name]:stop()
      end
      if app_table[name].shm then
         shm.delete_frame(app_table[name].shm)
      end
   end
   function ops.keep (name)
      new_app_table[name] = app_table[name]
      table.insert(new_app_array, app_table[name])
      app_name_to_index[name] = #new_app_array
   end
   function ops.start (name)
      local class = conf.apps[name].class
      local arg = conf.apps[name].arg
      local app = class:new(arg)
      if type(app) ~= 'table' then
         error(("bad return value from app '%s' start() method: %s"):format(
                  name, tostring(app)))
      end
      local zone = app.zone or getfenv(class.new)._NAME or name
      local pullevent = define_event('trace', ("pulled %s (%s): $inpackets $inbytes $outpackets $outbytes"):format(
                                        name, zone))
      local pushevent = define_event('trace', ("pushed %s (%s): $inpackets $inbytes $outpackets $outbytes"):format(
                                        name, zone))
      app.appname = name
      app.output = {}
      app.input = {}
      new_app_table[name] = app
      table.insert(new_app_array, app)
      app_name_to_index[name] = #new_app_array
      app.zone = zone
      app.pullevent = pullevent
      app.pushevent = pushevent
      if app.shm then
         app.shm.dtime = {counter, C.get_unix_time()}
         app.shm = shm.create_frame("apps/"..name, app.shm)
      end
   end
   function ops.restart (name)
      ops.stop(name)
      ops.start(name)
   end
   function ops.reconfig (name)
      if app_table[name].reconfig then
         local arg = conf.apps[name].arg
         local app = app_table[name]
         app:reconfig(arg)
         new_app_table[name] = app
         table.insert(new_app_array, app)
         app_name_to_index[name] = #new_app_array
      else
         ops.restart(name)
      end
   end
   -- Dispatch actions in a suitable sequence.
   for _, action in ipairs({'stop', 'restart', 'keep', 'reconfig', 'start'}) do
      for _, name in ipairs(actions[action]) do
         if log and action ~= 'keep' then
            io.write("engine: ", action, " app ", name, "\n")
         end
         ops[action](name)
      end
   end
   -- Setup links: create (or reuse) and renumber.
   for linkspec in pairs(conf.links) do
      local fa, fl, ta, tl = config.parse_link(linkspec)
      if not new_app_table[fa] then error("no such app: " .. fa) end
      if not new_app_table[ta] then error("no such app: " .. ta) end
      -- Create or reuse a link and assign/update receiving app index
      local link = link_table[linkspec] or link.new(linkspec)
      events_link_packet[link] = define_event('app', "packet data $a $b $c $d on "..linkspec)
      link.receiving_app = app_name_to_index[ta]
      -- Add link to apps
      new_app_table[fa].output[fl] = link
      table.insert(new_app_table[fa].output, link)
      new_app_table[ta].input[tl] = link
      table.insert(new_app_table[ta].input, link)
      -- Remember link
      new_link_table[linkspec] = link
      table.insert(new_link_array, link)
   end
   -- Free obsolete links.
   for linkspec, r in pairs(link_table) do
      if not new_link_table[linkspec] then link.free(r, linkspec) end
   end
   -- Commit changes.
   app_table, link_table = new_app_table, new_link_table
   app_array, link_array = new_app_array, new_link_array
   -- Trigger link event for each app.
   for _, app in ipairs(app_array) do
      if app.link then app:link() end
   end
end

-- Call this to "run snabb switch".
function main (options)
   event_traffic_start()
   options = options or {}
   local done = options.done
   local no_timers = options.no_timers
   if options.duration then
      assert(not done, "You can not have both 'duration' and 'done'")
      done = lib.timer(options.duration * 1e9)
   end

   local breathe = breathe
   if options.measure_latency or options.measure_latency == nil then
      local latency = histogram.create('engine/latency.histogram', 1e-6, 1e0)
      breathe = latency:wrap_thunk(breathe, now)
   end

   monotonic_now = C.get_monotonic_time()
   repeat
      breathe()
      if not no_timers then timer.run() event_ran_timers() end
      if not busywait then pace_breathing() end
   until done and done()
   counter.commit()
   if not options.no_report then report(options.report) end
   event_traffic_stop()
end

local nextbreath
local lastfrees = 0
local lastfreebits = 0
local lastfreebytes = 0
-- Wait between breaths to keep frequency with Hz.
function pace_breathing ()
   if Hz then
      nextbreath = nextbreath or monotonic_now
      local sleep = tonumber(nextbreath - monotonic_now)
      if sleep > 1e-6 then
         event_sleep_Hz(Hz, math.round(sleep*1e6))
         C.usleep(sleep * 1e6)
         monotonic_now = C.get_monotonic_time()
         event_wakeup()
      end
      nextbreath = math.max(nextbreath + 1/Hz, monotonic_now)
   else
      if lastfrees == counter.read(frees) then
         sleep = math.min(sleep + 1, maxsleep)
         event_sleep_idle(sleep)
         C.usleep(sleep)
         event_wakeup()
      else
         sleep = math.floor(sleep/2)
      end
      lastfrees = counter.read(frees)
      lastfreebytes = counter.read(freebytes)
      lastfreebits = counter.read(freebits)
   end
end

function breathe ()
   local freed_packets0 = counter.read(frees)
   local freed_bytes0 = counter.read(freebytes)
   event_breath_start(counter.read(breaths), freed_packets0, freed_bytes0,
                      counter.read(freebits))
   monotonic_now = C.get_monotonic_time()
   -- Restart: restart dead apps
   restart_dead_apps()
   -- Inhale: pull work into the app network
   event_breath_initialized(C.get_time_ns())
   for i = 1, #app_array do
      local app = app_array[i]
--      if app.pull then
--         zone(app.zone) app:pull() zone()
      if app.pull and not app.dead then
         zone(app.zone)
         if tl.enabled(timeline, 'trace') then
            local inp0, inb0, outp0, outb0 = linkstats(app)
            with_restart(app, app.pull)
            local inp1, inb1, outp1, outb1 = linkstats(app)
            app.pullevent(inp1-inp0, inb1-inb0, outp1-outp0, outb1-outb0)
         else
            with_restart(app, app.pull)
         end
         zone()
      end
   end
   event_breath_pulled()
   -- Exhale: push work out through the app network
   local firstloop = true
   repeat
      local progress = false
      -- For each link that has new data, run the receiving app
      for i = 1, #link_array do
         local l = link_array[i]
         if firstloop or l.has_new_data then
            if l.has_new_data and tl.enabled(timeline, 'app') then
               local p = link.front(l)
               if p ~= nil then
                  event_packet_length(p.length)
                  local u64 = ffi.cast("uint64_t*", p.data)
                  for n = 0, p.length/8, 4 do
                     events_link_packet[l](u64[n], u64[n+1], u64[n+2], u64[n+3])
                  end
               end
            end
            l.has_new_data = false
            local receiver = app_array[l.receiving_app]
            if receiver.push and not receiver.dead then
               zone(receiver.zone)
               if tl.enabled(timeline, 'trace') then
                  local inp0, inb0, outp0, outb0 = linkstats(receiver)
                  with_restart(receiver, receiver.push)
                  local inp1, inb1, outp1, outb1 = linkstats(receiver)
                  receiver.pushevent(inp1-inp0, inb1-inb0, outp1-outp0, outb1-outb0)
               else
                  with_restart(receiver, receiver.push)
               end
               zone()
               progress = true
            end
         end
      end
      event_breath_pushed()
      firstloop = false
   until not progress  -- Stop after no link had new data
   local freed
   local freed_packets = counter.read(frees) - freed_packets0
   local freed_bytes = (counter.read(freebytes) - freed_bytes0)
   local freed_bytes_per_packet = freed_bytes / math.max(tonumber(freed_packets), 1)
   event_breath_end(counter.read(breaths), freed_packets, freed_bytes_per_packet)
   counter.add(breaths)
   -- Commit counters at a reasonable frequency
   if counter.read(breaths) % 100 == 0 then
      counter.commit()
      event_commit_counters()
   end
   -- Sample events with dynamic priorities.
   -- Lower priorities are enabled 1/10th as often as the one above.
   local r = math.random()
   if     r < 0.00001 then tl.priority(timeline, 'packet')
   elseif r < 0.00010 then tl.priority(timeline, 'app')
   elseif r < 0.01000 then tl.priority(timeline, 'trace')
   elseif r < 0.10000 then tl.priority(timeline, 'info')
   else                    tl.priority(timeline, 'warning')
   end
end

function linkstats (app)
   local inp, inb, outp, outb = 0, 0, 0, 0
   for i = 1, #app.input do
      inp  = inp  + tonumber(counter.read(app.input[i].stats.rxpackets))
      inb  = inb  + tonumber(counter.read(app.input[i].stats.rxbytes))
   end
   for i = 1, #app.output do
      outp = outp + tonumber(counter.read(app.output[i].stats.txpackets))
      outb = outb + tonumber(counter.read(app.output[i].stats.txbytes))
   end
   return inp, inb, outp, outb
end

function report (options)
   if not options or options.showload then
      report_load()
   end
   if options and options.showlinks then
      report_links()
   end
   if options and options.showapps then
      report_apps()
   end
end

-- Load reporting prints several metrics:
--   time - period of time that the metrics were collected over
--   fps  - frees per second (how many calls to packet.free())
--   fpb  - frees per breath
--   bpp  - bytes per packet (average packet size)
local lastloadreport = nil
local reportedfrees = nil
local reportedfreebits = nil
local reportedfreebytes = nil
local reportedbreaths = nil
function report_load ()
   local frees = counter.read(frees)
   local freebits = counter.read(freebits)
   local freebytes = counter.read(freebytes)
   local breaths = counter.read(breaths)
   if lastloadreport then
      local interval = now() - lastloadreport
      local newfrees   = tonumber(frees - reportedfrees)
      local newbytes   = tonumber(freebytes - reportedfreebytes)
      local newbits    = tonumber(freebits - reportedfreebits)
      local newbreaths = tonumber(breaths - reportedbreaths)
      local fps = math.floor(newfrees/interval)
      local fbps = math.floor(newbits/interval)
      local fpb = math.floor(newfrees/newbreaths)
      local bpp = math.floor(newbytes/newfrees)
      print(("load: time: %-2.2fs  fps: %-9s fpGbps: %-3.3f fpb: %-3s bpp: %-4s sleep: %-4dus"):format(
         interval,
         lib.comma_value(fps),
         fbps / 1e9,
         lib.comma_value(fpb),
         (bpp ~= bpp) and "-" or tostring(bpp), -- handle NaN
         sleep))
   end
   lastloadreport = now()
   reportedfrees = frees
   reportedfreebits = freebits
   reportedfreebytes = freebytes
   reportedbreaths = breaths
end

function report_links ()
   print("link report:")
   local function loss_rate(drop, sent)
      sent = tonumber(sent)
      if not sent or sent == 0 then return 0 end
      return tonumber(drop) * 100 / (tonumber(drop)+sent)
   end
   local names = {}
   for name in pairs(link_table) do table.insert(names, name) end
   table.sort(names)
   for i, name in ipairs(names) do
      l = link_table[name]
      local txpackets = counter.read(l.stats.txpackets)
      local txdrop = counter.read(l.stats.txdrop)
      print(("%20s sent on %s (loss rate: %d%%)"):format(
            lib.comma_value(txpackets), name, loss_rate(txdrop, txpackets)))
   end
end

function report_apps ()
   print ("apps report:")
   for name, app in pairs(app_table) do
      if app.dead then
         print(name, ("[dead: %s]"):format(app.dead.error))
      elseif app.report then
         print(name)
         if use_restart then
            with_restart(app, app.report)
         else
            -- Restarts are disabled, still we want to not die on
            -- errors during app reports, thus this workaround:
            local status, err = pcall(app.report, app)
            if not status then
               print("Warning: "..name.." threw an error during report: "..err)
            end
         end
      end
   end
end

function selftest ()
   print("selftest: app")
   local App = {}
   function App:new () return setmetatable({}, {__index = App}) end
   local c1 = config.new()
   config.app(c1, "app1", App)
   config.app(c1, "app2", App)
   config.link(c1, "app1.x -> app2.x")
   print("empty -> c1")
   configure(c1)
   assert(#app_array == 2)
   assert(#link_array == 1)
   assert(app_table.app1 and app_table.app2)
   local orig_app1 = app_table.app1
   local orig_app2 = app_table.app2
   local orig_link = link_array[1]
   print("c1 -> c1")
   configure(c1)
   assert(app_table.app1 == orig_app1)
   assert(app_table.app2 == orig_app2)
   local c2 = config.new()
   config.app(c2, "app1", App, "config")
   config.app(c2, "app2", App)
   config.link(c2, "app1.x -> app2.x")
   config.link(c2, "app2.x -> app1.x")
   print("c1 -> c2")
   configure(c2)
   assert(#app_array == 2)
   assert(#link_array == 2)
   assert(app_table.app1 ~= orig_app1) -- should be restarted
   assert(app_table.app2 == orig_app2) -- should be the same
   -- tostring() because == does not work on FFI structs?
   assert(tostring(orig_link) == tostring(link_table['app1.x -> app2.x']))
   print("c2 -> c1")
   configure(c1) -- c2 -> c1
   assert(app_table.app1 ~= orig_app1) -- should be restarted
   assert(app_table.app2 == orig_app2) -- should be the same
   assert(#app_array == 2)
   assert(#link_array == 1)
   print("c1 -> empty")
   configure(config.new())
   assert(#app_array == 0)
   assert(#link_array == 0)
   -- Test app arg validation
   local AppC = {
      config = {
         a = {required=true}, b = {default="foo"}
      }
   }
   local c3 = config.new()
   config.app(c3, "app_valid", AppC, {a="bar"})
   assert(not pcall(config.app, c3, "app_invalid", AppC))
   assert(not pcall(config.app, c3, "app_invalid", AppC, {b="bar"}))
   assert(not pcall(config.app, c3, "app_invalid", AppC, {a="bar", c="foo"}))
-- Test app restarts on failure.
   use_restart = true
   print("c_fail")
   local App1 = {zone="test"}
   function App1:new () return setmetatable({}, {__index = App1}) end
   function App1:pull () error("Pull error.") end
   function App1:push () return true end
   function App1:report () return true end
   local App2 = {zone="test"}
   function App2:new () return setmetatable({}, {__index = App2}) end
   function App2:pull () return true end
   function App2:push () error("Push error.") end
   function App2:report () return true end
   local App3 = {zone="test"}
   function App3:new () return setmetatable({}, {__index = App3}) end
   function App3:pull () return true end
   function App3:push () return true end
   function App3:report () error("Report error.") end
   local c_fail = config.new()
   config.app(c_fail, "app1", App1)
   config.app(c_fail, "app2", App2)
   config.app(c_fail, "app3", App3)
   config.link(c_fail, "app1.x -> app2.x")
   configure(c_fail)
   local orig_app1 = app_table.app1
   local orig_app2 = app_table.app2
   local orig_app3 = app_table.app3
   local orig_link1 = link_array[1]
   local orig_link2 = link_array[2]
   main({duration = 4, report = {showapps = true}})
   assert(app_table.app1 ~= orig_app1) -- should be restarted
   assert(app_table.app2 ~= orig_app2) -- should be restarted
   assert(app_table.app3 == orig_app3) -- should be the same
   main({duration = 4, report = {showapps = true}})
   assert(app_table.app3 ~= orig_app3) -- should be restarted
end
