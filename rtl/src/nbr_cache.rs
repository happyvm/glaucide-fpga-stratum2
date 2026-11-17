// SPDX-License-Identifier: Apache-2.0
//! IPv6 neighbor cache (task T3.6), domain `DefaultClock`: 4 entries
//! (IPv6 address to MAC), the IPv6 counterpart of the cache of `arp.rs`.
//!
//! # Ports
//!
//! Inputs: `learn` (`Rx<NdLearn>`, from `icmpv6`), `lookup` (`Rx<U<128>>`,
//! the address the NTP client wants the MAC of), `flush` (`In<Bit>`, wire).
//! Output: `rsp` (`Tx<ArpEntry>`, `crate::arp::ArpEntry`).
//! Addresses are `U<128>` with the first wire byte in the most significant
//! byte, MACs as in `eth_rx`.
//!
//! # Behaviour
//!
//! * `learn(ip, mac)` (F5/F6: `icmpv6` now only emits it for an NA that
//!   answers its own solicitation, so no host of the link can create entries or
//!   evict the gateway): always taken (never held back). If `ip` is already in a
//!   valid entry, that entry is updated in place (a MAC change replaces the old
//!   one; no duplicate address ever exists). Otherwise the entry at the
//!   round-robin pointer `rr` is replaced (valid or not) and `rr` advances
//!   (0, 1, 2, 3, 0, ...); the fifth distinct address evicts the first. An
//!   update in place does not move `rr`. The address `::` is stored like any
//!   other (the producer, `icmpv6`, never learns it).
//! * `lookup(ip)`: taken when `rsp` has room; answered on `rsp` in the cycle it
//!   is taken (the channel adds one): `found = 1` and the entry's MAC on a hit,
//!   `found = 0` and `mac = 0` on a miss. Exactly one `rsp` per `lookup`, in
//!   order.
//! * Same-cycle `learn` and `lookup`: both are processed in that cycle; the
//!   lookup sees the cache **as it was before** the learn (registers), so a
//!   lookup concurrent with the learn of its own address still misses; the
//!   client polls, the next lookup hits. There is no priority to arbitrate:
//!   they use separate ports.
//! * `flush` (wire, level): in a cycle where it is high all entries become
//!   invalid and `rr` is cleared; it has priority over a `learn` of the same
//!   cycle (that learn is lost); a lookup of that cycle still sees the old
//!   contents.
//! * **No aging in v1**: an entry lives until it is replaced or flushed;
//!   `flush` is the escape hatch (the client or the top level pulses it, for
//!   instance when the link goes down).
//!
//! # Integration
//!
//! Inputs and outputs on registered channels or FIFOs (never port to port).
use crate::arp::ArpEntry;
use crate::icmpv6::NdLearn;
use txhdl::comp::{mux, Clock, DefaultClock, In, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{lower, with, Trace};

#[derive(Trace, Default)]
pub struct NbrCache {
    pub v0: Reg<Bit>,
    pub v1: Reg<Bit>,
    pub v2: Reg<Bit>,
    pub v3: Reg<Bit>,
    pub ip0: Reg<U<128>>,
    pub ip1: Reg<U<128>>,
    pub ip2: Reg<U<128>>,
    pub ip3: Reg<U<128>>,
    pub mac0: Reg<U<48>>,
    pub mac1: Reg<U<48>>,
    pub mac2: Reg<U<48>>,
    pub mac3: Reg<U<48>>,
    pub rr: Reg<U<2>>,
}

#[lower]
impl Unit<(Rx<NdLearn>, Rx<U<128>>, In<Bit>), (Tx<ArpEntry>,)> for NbrCache {
    async fn run(
        &mut self,
        (learn, lookup, flush): (Rx<NdLearn>, Rx<U<128>>, In<Bit>),
        (rsp,): (Tx<ArpEntry>,),
    ) {
        loop {
            DefaultClock::rising().await;
            let fl = flush.get();
            let v0 = self.v0.get();
            let v1 = self.v1.get();
            let v2 = self.v2.get();
            let v3 = self.v3.get();
            let ip0 = self.ip0.get();
            let ip1 = self.ip1.get();
            let ip2 = self.ip2.get();
            let ip3 = self.ip3.get();
            let mac0 = self.mac0.get();
            let mac1 = self.mac1.get();
            let mac2 = self.mac2.get();
            let mac3 = self.mac3.get();
            let rr = self.rr.get();

            // Learn: update in place, else replace at the round-robin pointer.
            let ln = Bit::from(learn.peek().is_some());
            let l = learn.head();
            let _ = learn.recv_if(ln);
            let lh0 = v0 & Bit::from(ip0 == l.ip);
            let lh1 = v1 & Bit::from(ip1 == l.ip);
            let lh2 = v2 & Bit::from(ip2 == l.ip);
            let lh3 = v3 & Bit::from(ip3 == l.ip);
            let any_hit = lh0 | lh1 | lh2 | lh3;
            let w0 = ln & (lh0 | (!any_hit & Bit::from(rr == 0)));
            let w1 = ln & (lh1 | (!any_hit & Bit::from(rr == 1)));
            let w2 = ln & (lh2 | (!any_hit & Bit::from(rr == 2)));
            let w3 = ln & (lh3 | (!any_hit & Bit::from(rr == 3)));

            // Lookup: answered from the registers of this cycle.
            let lk_v = Bit::from(lookup.peek().is_some());
            let lip = lookup.head();
            let lk_take = lk_v & rsp.ready();
            let _ = lookup.recv_if(lk_take);
            let k0 = v0 & Bit::from(ip0 == lip);
            let k1 = v1 & Bit::from(ip1 == lip);
            let k2 = v2 & Bit::from(ip2 == lip);
            let k3 = v3 & Bit::from(ip3 == lip);
            let lfound = k0 | k1 | k2 | k3;
            let lmac = mux(
                k0,
                mac0,
                mux(k1, mac1, mux(k2, mac2, mux(k3, mac3, U::<48>::from(0u8)))),
            );
            if lk_take.to_bool() {
                rsp.send(ArpEntry { found: lfound, mac: lmac });
            }

            with!(self <= {
                w0 ? { v0: Bit::One, ip0: l.ip, mac0: l.mac },
                w1 ? { v1: Bit::One, ip1: l.ip, mac1: l.mac },
                w2 ? { v2: Bit::One, ip2: l.ip, mac2: l.mac },
                w3 ? { v3: Bit::One, ip3: l.ip, mac3: l.mac },
                ln & !any_hit ? rr: rr + 1,
                fl ? {
                    v0: Bit::Zero,
                    v1: Bit::Zero,
                    v2: Bit::Zero,
                    v3: Bit::Zero,
                    rr: U::<2>::from(0u8),
                },
            });
        }
    }
}
