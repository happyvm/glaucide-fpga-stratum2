// SPDX-License-Identifier: Apache-2.0
//! Two-input request arbiters in front of the IP packet builders (task
//! T1.11), domain `DefaultClock` (`clk_sys`, 100 MHz).
//!
//! `ipv4_tx` has a single (`req`, `payload`) input pair but two producers in
//! the server path: `icmp_echo` (echo replies) and `udp4_tx` (NTP replies).
//! Likewise `ipv6_tx` serves `icmpv6` (echo replies, neighbor advertisements)
//! and `udp6_tx`. [`IpArb4`] and [`IpArb6`] are the 2-input version of the
//! `tx_arb` pattern for (request, payload) pairs, one per request type
//! ([`IpTxReq`], [`Ip6TxReq`]); the logic is identical.
//!
//! # Ports (flat, in this order)
//!
//! Inputs `r0`, `p0`, `r1`, `p1`: two sources, each a request channel and
//! its payload channel (`FrameByte`). Outputs `req`, `payload`: the input
//! pair of the IP builder.
//!
//! # Behaviour
//!
//! * IDLE: when at least one `r_i` holds a request and `req` has room, the
//!   winner is chosen round-robin (the source not granted last wins a tie;
//!   source 1 wins the first tie after reset: `rr` = 0 reads as "source 0
//!   granted last"), its request is popped and sent on `req` in
//!   the same cycle, and the unit enters PAY with that source granted.
//! * PAY: the bytes of the granted payload only are forwarded (`err`
//!   copied), one per cycle at best, when present and `payload` has room.
//!   The cycle after the byte with `last` the unit is in IDLE again. The
//!   other source is never touched during a packet: its request and bytes
//!   wait in their channels.
//! * One idle cycle per packet (the request cycle); bytes at one per cycle.
//! * Fairness: with both sources always pending, they alternate; a pending
//!   source waits for at most one packet of the other.
//!
//! # Contract
//!
//! Each source must send its request before (or in the cycle of) the first
//! byte of its payload, and the payload of an accepted request must follow
//! (at least one byte, ending with `last`): the arbiter waits for it while
//! holding the builder, exactly as `tx_arb` does. `icmp_echo`, `icmpv6`,
//! `udp4_tx` and `udp6_tx` satisfy it (their headers: "`req` is sent before
//! the first payload byte, one `req` is followed by exactly one frame").
//! Output: one request, then exactly the granted source's payload, so the
//! builder's own contract (request before payload, one frame per request)
//! is preserved. `err` is forwarded unchanged (only with `last`).
//!
//! Latency: a request offered to an idle arbiter with `req` ready is on
//! `req` in the same cycle it is popped (one registered channel in front,
//! one behind: two cycles from the producer's send to the builder's view);
//! payload bytes likewise one cycle through the unit.
//!
//! # Integration
//!
//! All four inputs and both outputs are registered channels (never port to
//! port, note "Netlist" of `stream.rs`).
use crate::ipv4_tx::IpTxReq;
use crate::ipv6_tx::Ip6TxReq;
use crate::stream::FrameByte;
use txhdl::comp::{mux, Clock, DefaultClock, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{lower, with, Trace};

const IDLE: u8 = 0;
const PAY: u8 = 1;

/// Arbiter of two IPv4 request sources in front of `ipv4_tx`.
#[derive(Trace, Default)]
pub struct IpArb4 {
    /// IDLE or PAY.
    pub st: Reg<U<1>>,
    /// Granted source (valid in PAY).
    pub gr: Reg<Bit>,
    /// Source granted last (round robin).
    pub rr: Reg<Bit>,
}

#[lower]
impl Unit<(Rx<IpTxReq>, Rx<FrameByte>, Rx<IpTxReq>, Rx<FrameByte>), (Tx<IpTxReq>, Tx<FrameByte>)>
    for IpArb4
{
    async fn run(
        &mut self,
        (r0, p0, r1, p1): (Rx<IpTxReq>, Rx<FrameByte>, Rx<IpTxReq>, Rx<FrameByte>),
        (req, payload): (Tx<IpTxReq>, Tx<FrameByte>),
    ) {
        loop {
            DefaultClock::rising().await;
            let is_idle = Bit::from(self.st.get() == IDLE);
            let is_pay = Bit::from(self.st.get() == PAY);
            let gr = self.gr.get();
            let rr = self.rr.get();
            let q0 = Bit::from(r0.peek().is_some());
            let q1 = Bit::from(r1.peek().is_some());
            // Source 1 wins when it is alone, or both wait and 0 went last.
            let s1 = q1 & (!q0 | !rr);
            let take = is_idle & (q0 | q1) & req.ready();
            let t0 = take & !s1;
            let t1 = take & s1;
            let _ = r0.recv_if(t0);
            let _ = r1.recv_if(t1);
            let h0 = r0.head();
            let h1 = r1.head();
            let hsel = IpTxReq {
                dst_mac: mux(s1, h1.dst_mac, h0.dst_mac),
                dst_ip: mux(s1, h1.dst_ip, h0.dst_ip),
                proto: mux(s1, h1.proto, h0.proto),
                len: mux(s1, h1.len, h0.len),
            };
            if take.to_bool() {
                req.send(hsel);
            }

            let b0 = p0.head();
            let b1 = p1.head();
            let v0 = Bit::from(p0.peek().is_some());
            let v1 = Bit::from(p1.peek().is_some());
            let pv = mux(gr, v1, v0);
            let go = is_pay & pv & payload.ready();
            let _ = p0.recv_if(go & !gr);
            let _ = p1.recv_if(go & gr);
            let o_last = mux(gr, b1.last, b0.last);
            let ob = FrameByte {
                data: mux(gr, b1.data, b0.data),
                last: o_last,
                err: mux(gr, b1.err, b0.err),
            };
            if go.to_bool() {
                payload.send(ob);
            }
            let end = go & o_last;
            with!(self <= {
                take ? { st: U::<1>::from(PAY), gr: s1, rr: s1 },
                end ? st: U::<1>::from(IDLE),
            });
        }
    }
}

/// Arbiter of two IPv6 request sources in front of `ipv6_tx`.
#[derive(Trace, Default)]
pub struct IpArb6 {
    /// IDLE or PAY.
    pub st: Reg<U<1>>,
    /// Granted source (valid in PAY).
    pub gr: Reg<Bit>,
    /// Source granted last (round robin).
    pub rr: Reg<Bit>,
}

#[lower]
impl Unit<(Rx<Ip6TxReq>, Rx<FrameByte>, Rx<Ip6TxReq>, Rx<FrameByte>), (Tx<Ip6TxReq>, Tx<FrameByte>)>
    for IpArb6
{
    async fn run(
        &mut self,
        (r0, p0, r1, p1): (Rx<Ip6TxReq>, Rx<FrameByte>, Rx<Ip6TxReq>, Rx<FrameByte>),
        (req, payload): (Tx<Ip6TxReq>, Tx<FrameByte>),
    ) {
        loop {
            DefaultClock::rising().await;
            let is_idle = Bit::from(self.st.get() == IDLE);
            let is_pay = Bit::from(self.st.get() == PAY);
            let gr = self.gr.get();
            let rr = self.rr.get();
            let q0 = Bit::from(r0.peek().is_some());
            let q1 = Bit::from(r1.peek().is_some());
            let s1 = q1 & (!q0 | !rr);
            let take = is_idle & (q0 | q1) & req.ready();
            let t0 = take & !s1;
            let t1 = take & s1;
            let _ = r0.recv_if(t0);
            let _ = r1.recv_if(t1);
            let h0 = r0.head();
            let h1 = r1.head();
            let hsel = Ip6TxReq {
                dst_mac: mux(s1, h1.dst_mac, h0.dst_mac),
                dst_ip: mux(s1, h1.dst_ip, h0.dst_ip),
                src_ip: mux(s1, h1.src_ip, h0.src_ip),
                next: mux(s1, h1.next, h0.next),
                len: mux(s1, h1.len, h0.len),
                hop: mux(s1, h1.hop, h0.hop),
            };
            if take.to_bool() {
                req.send(hsel);
            }

            let b0 = p0.head();
            let b1 = p1.head();
            let v0 = Bit::from(p0.peek().is_some());
            let v1 = Bit::from(p1.peek().is_some());
            let pv = mux(gr, v1, v0);
            let go = is_pay & pv & payload.ready();
            let _ = p0.recv_if(go & !gr);
            let _ = p1.recv_if(go & gr);
            let o_last = mux(gr, b1.last, b0.last);
            let ob = FrameByte {
                data: mux(gr, b1.data, b0.data),
                last: o_last,
                err: mux(gr, b1.err, b0.err),
            };
            if go.to_bool() {
                payload.send(ob);
            }
            let end = go & o_last;
            with!(self <= {
                take ? { st: U::<1>::from(PAY), gr: s1, rr: s1 },
                end ? st: U::<1>::from(IDLE),
            });
        }
    }
}
