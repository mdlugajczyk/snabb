-- Binary messages with symmetric encoding/decoding
-- Use of this source code is governed by the Apache 2.0 license; see COPYING.

module(..., package.seeall)

local ffi = require("ffi")

-- Convert between Lua dictionaries and binary messages based on
-- user-defined structure. Print and diff hexdumps to help with
-- debugging obscure things like endian mismatches.

local binmsg = {
}

function new ()
   local m = {
      offset = 0,
      fields = {}
   }
   return setmetatable(m, {__index=binmsg})
end

function binmsg:intBE (name, size)
   table.insert(self.fields, {name = name, type = 'intBE', size=size})
   self.offset = self.offset + size
end

function binmsg:def (name, size, type, options)
   table.insert(self.fields,
                {name = name, size = size, type = type, options = options})
end

function binmsg:decode (data)
   data = ffi.cast("uint8_t*", data)
   local values = {}
   local offset = 0
   for i, field in ipairs(self.fields) do
      local o = field.options or {}
      local v
      if field.type == 'intBE' then
         v = 0
         for i = 1, field.size do
--            print(field.name, i, offset)
            v = bit.lshift(v, 8) + data[offset]
            offset = offset + 1
            print("v", data[offset], bit.tohex(v), v)
         end
      end
      if (o.valid and not o.valid(v)) or (v ~= (o.constant or v)) then
         error(("'%s' illegal: %s (%s)"):format(field.name, v, bit.tohex(v)))
      end
      values[field.name] = v
   end
   return values
end

function decode (field, data, offset)
end

function selftest ()
   local m = new()
   m:def('type', 1, 'intBE')
   m:def('pad0', 3, 'intBE', {constant = 0})
   for k, v in pairs(m:decode({42, 0, 0, 0})) do
      print(k, v)
   end
   --[[
   m:hexconstant("00 00 00")
   m:u32B('input_length', {offset=0x04})
   m:u64B('input_mailbox_pointer',
          {valid = function (v) return v%256 == 0 end,
           offset=0x08})
   m:fixedstring('inline_input',  16, {offset=0x10})
   m:fixedstring('inline_output', 16, {offset=0x20})
   m:u64B('output_mailbox_pointer',
          {valid = function (v) return v^256 == 0 end,
           offset=0x30})
   --]]
end

