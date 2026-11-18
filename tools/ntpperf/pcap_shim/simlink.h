// SPDX-License-Identifier: Apache-2.0
//
// simlink.h: the contract between ntpperf's replacement libpcap / time shim
// (tools/ntpperf/pcap_shim/) and the simulation bridge
// (rtl/src/bin/ntpperf_bridge.rs). Keep both sides in sync.
//
// 1. The "network interface" is one end of an AF_UNIX SOCK_SEQPACKET
//    socketpair created by the bridge; its descriptor number is passed in
//    the environment variable NTPPERF_SIM_FD.
//    * client -> bridge: every message of at least 14 bytes is one Ethernet
//      frame without FCS, exactly as written by ntpperf's sender
//      (sendmmsg on pcap_fileno()). The bridge stamps it with the virtual
//      time at which it reads it (see 2: that is the instant it was sent).
//      A shorter message is a control message: SIM_HELLO (below), sent by
//      pcap_activate().
//    * bridge -> client: one message per frame transmitted by the simulated
//      design: struct sim_pkt_hdr followed by the frame (no FCS).
// 2. Virtual time lives in a shared page (a 4096-byte file mapped by both
//    sides, path in NTPPERF_VTIME_SHM), u64 words, layout below. Only the
//    bridge advances time, and only when every registered process of
//    ntpperf is "ready" (waiting for a virtual deadline, or blocked in I/O
//    that nobody can complete without time advancing). See vtime.c.
#ifndef NTPPERF_SIMLINK_H
#define NTPPERF_SIMLINK_H

#include <stdint.h>

#define SIM_MAGIC 0x314d54565050544eull  // "NTPPVTM1" (little endian)

enum {
    SW_MAGIC = 0,      // SIM_MAGIC once the bridge has initialised the page
    SW_VT_NS = 1,      // virtual Unix time, ns (written by the bridge only)
    SW_T0_NS = 2,      // virtual time at start (origin of CLOCK_MONOTONIC)
    SW_STOP = 3,       // the bridge gave up: processes must exit
    SW_INFLIGHT = 4,   // (i64) bytes written into tracked pipes, not yet read
    SW_NSLOTS = 5,     // process slots allocated (fetch_add by the shim)
    SW_EPOCH = 6,      // futex word (low 32 bits): bumped at each time publish
    SW_SEQ = 7,        // futex word (low 32 bits): bumped after every state change
    SW_BRIDGE_PID = 8, // pid of the bridge (liveness check by the shim)
    SW_QUANTUM_NS = 9, // wait quantum of a process with no known deadline
    SW_SLOTS = 16,     // first slot
    SW_SLOT_WORDS = 8,
    SW_MAX_SLOTS = 32,
};

// Words of a slot (SW_SLOTS + i * SW_SLOT_WORDS + k).
enum {
    SL_STATE = 0,     // enum slot_state
    SL_PID = 1,
    SL_DEADLINE = 2,  // virtual Unix ns, valid in ST_TIME_WAIT
    SL_WAITS = 3,     // number of time waits (statistics)
    SL_IOWAITS = 4,   // number of blocking I/O waits (statistics)
};

enum slot_state {
    ST_FREE = 0,
    ST_RUNNING = 1,       // computing: time must not advance
    ST_TIME_WAIT = 2,     // spinning until vt >= deadline
    ST_BLOCKED_READ = 3,  // blocked reading a tracked pipe (ready iff inflight == 0)
    ST_BLOCKED_WRITE = 4, // blocked writing a full tracked pipe (ready)
    ST_EXITED = 5,
};

// Control message client -> bridge, sent once by pcap_activate().
// 12 bytes: shorter than any frame.
struct sim_hello {
    char tag[4];          // "HELO"
    uint16_t tstamp_type; // PCAP_TSTAMP_* requested
    uint16_t precision;   // PCAP_TSTAMP_PRECISION_*
    uint32_t snaplen;
};

// Header of a frame bridge -> client.
struct sim_pkt_hdr {
    uint64_t ts_ns;  // virtual Unix time of the timestamp (see the bridge)
    uint32_t len;    // frame length (no FCS)
    uint32_t flags;  // 0
};

#endif
