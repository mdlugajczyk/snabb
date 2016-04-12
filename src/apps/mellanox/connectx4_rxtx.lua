-- Mellanox ConnectX-4 transmit/receive routines
-- Use of this source code is governed by the Apache 2.0 license; see COPYING.

module(..., package.seeall)

local ffi = require("ffi")

-- Notes from the PRM (Mellanox Programmer's Reference Manual):
--
-- Q: DMA descriptors (WQEs) are padded to 64-bytes. Does this
--    padding consume PCIe bandwidth in practice?
--
-- Q: What are "atomic WQEs"? Mentioned in 7.4.4.1.4 but not defined.
--
-- Address translation:
--
--   Can we simply use physical addresses for DMA and skip the address
--   translation? This would be consistent with our other drivers. (In
--   the future if we want memory protection for DMA then we could
--   evaulate the merits of this NIC feature vs the new CPU IOMMU.)
--
--   "Reserved LKey" in 6.3 is a token that we can put into the
--   transmit/receive descriptors to say that they are using physical
--   addresses. This may be the simplest and most consistent with our
--   other drivers at least to begin with. (It is interesting that the
--   Mellanox NIC can perform address translation, but I am not sure
--   whether we want to do that, and if we did whether we would prefer
--   to use the IOMMU anyway.)
--
-- Q: What is the value of the "Reserved LKey"? Section 6.3 says
--    this is "programmed by the Level-0 privilege agent." In this
--    context it seems like that is Snabb. I have not yet found the
--    details of how to program this value.


-- Work Queue Entry (WQE) - DMA descriptor for transmit and receive.
-- 
-- This definition is intended for two purposes:
--   Send QWE - Send Data Segment - Memory Pointer (7.4.4.1.4)
--   Receive WQE - Receive Data Segment (7.4.4.1.5)
--
-- There are other variations and flags specified in the PRM but to
-- avoid distraction this definition only covers the details we are
-- using. See PRM for full details.

local wqe_t = ffi.typeof[[
  struct {
    uint32_t length;   // 30-bit length
    uint32_t lkey;     // always set to "reserved lkey" value
    uint64_t address;  // physical address (because reserved lkey)
  }
]]

-- Doorbell Record - Cursor in work queues
--
-- The doorbell record is written by Snabb to tell the NIC where the
-- last valid read and write descriptors are. This is a cursor in a
-- ring buffer like the 'read' and 'write' fields of 'struct packet'.
--
-- Snabb "rings the doorbell" after it writes descriptors by updating
-- these values to point to where the next new descriptor will be
-- written (which implies that all the previous descriptors have
-- already been written and should be processed.)

local doorbell_t = ffi.typeof[[
  struct {
    uint16_t read;
    uint16_t pad;
    uint16_t write;
    uint16_t pad;
  }
]]

