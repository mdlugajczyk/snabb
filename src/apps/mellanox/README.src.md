# Mellanox ConnectX-4 Ethernet Controller App

## Interface to hardware

This driver controls the firmware running on the ConnectX-4 card. The
interface is described in detail in the [Mellanox Adapter Programmer's
Reference Manual](). Below is a brief summary of the most important
abstractions provided by the firmware and the specific way that this
driver uses each one.

- Host Channel Adapter (HCA) is the NIC: one PCIe function and one physical port.
    - This driver controls one NIC (HCA) per instance.
- Event Queue (EQ) is a ring buffer where the device can send
  asynchronous messages to the driver.
    - One event queue, `eq`, used to receive *Pages Request Events*
      when the device needs the driver to allocate memory.
- Work Queue (WQ) in three consecutive parts: Doorbell Record (DB),
  Send Queue (SQ), Receive Queue (RQ). The Send Queue is a ring buffer
  of 64-byte transmit descriptors and the Receive Queue is a ring
  buffer of 16-byte receive descriptors. The Doorbell Record contains
  the cursors into the rings.
    - One work queue 
- User Access Region (UAR) is a mechanism for implementing
  [capability-based security](https://en.wikipedia.org/wiki/Capability-based_security)
  policies. A master driver can delegate direct access to certain send
  and receive queues by sharing mappings to selective parts of the
  PCIe register space.
    - This driver uses a single UAR for all send and receive queues.
      The UAR can be shared between multiple processes and safely
      accessed concurrently. (We can also consider providing each
      process with a separate UAR but this does not seem to be
      necessary.)
- Protection Domains and Memory Keys (MKey) define virtual address
  spaces. The NIC supports defining multiple address spaces with their
  own virtual-to-physical address mappings. This makes it possible for
  processes to perform DMA using virtual addresses instead of physical
  addresses (much like with the CPU's IOMMU but implemented by the
  NIC).
    - This driver does not use MKeys or virtual addresses. Instead we use physical addresses directly via the "reserved LKey" feature.
  Interface Send (TIS) objects define send and receive queues.
    - This driver uses one instance of each object.
- Flow Tables, Flow Groups, and Flow Table Entries match packet headers during send and receive and take specific actions: forward to a receive queue (or another flow table), bump a counter, or drop the packet. Flow Groups define which bits of the payload to inspect (e.g. ethernet multicast bit) and Flow Table Entries define what action to take when one specific value is matched (e.g. multicast or unicast).
    - This driver will use Flow Tables for Layer-2 switching. The exact structure of the Flow Table, Flow Group, and Flow Table Entry objects is still to be determined.
- Transport Interface Receive (TIR) is an object where packets are dispatched from a Flow Table. The TIR puts the packets onto a Receive Queue (RQ). The TIR can either have one RQ ("direct dispatching") or multiple represented by a Receive Queue Table (RQT) and selected by a per-packet hash ("indirect dispatching").
    - This driver will use one TIR for each application (i.e. L2 destination selected by a Flow Table) and support hashing incoming packets across an RQT based on source and destination L3 and L4 addresses.
- Transport Interface Send (TIS) and Send Queue (SQ) objects are the counterparts to TIR and RQ. The SQ forwards packets to the TIS for transmission onto the network. The [???] can reference a Flow Table to process packets e.g. to forward them to a TIR for loopback onto the host.
