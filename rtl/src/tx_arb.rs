// SPDX-License-Identifier: Apache-2.0
//! Output arbiter in front of `eth_tx` (task T1.7), domain `DefaultClock`.
//!
//! Four sources (ARP, ICMP/ICMPv6, UDP over IPv4, UDP over IPv6), each a pair
//! (`hdr_i: Rx<EthHdr>`, `payload_i: Rx<FrameByte>`), one frame per pair: one
//! `EthHdr`, then a payload ending with `last`. Outputs: `hdr: Tx<EthHdr>` and
//! `payload: Tx<FrameByte>`, the input pair of `eth_tx`.
//!
//! Port list (flat, as `#[lower]` wants): inputs `h0, p0, h1, p1, h2, p2, h3,
//! p3`, outputs `hdr, payload`.
//!
//! # Behaviour
//!
//! * IDLE: when at least one `hdr_i` is pending and `hdr` has room, the
//!   winner is chosen round-robin starting after the last winner (order
//!   `rr+1, rr+2, rr+3, rr`), its `hdr` is taken and sent in the same cycle,
//!   and the unit enters PAY with that source granted.
//! * PAY: bytes of the granted payload only are forwarded (`err` copied),
//!   one per cycle at best, stalling when the source has a gap or `payload`
//!   has no room. After the byte with `last` the unit returns to IDLE. The
//!   other payload channels are never touched during a frame. If the granted
//!   payload has not started, the unit waits for it (no other source is
//!   taken).
//! * Fairness: a source whose `hdr` stays pending is granted after at most
//!   three other frames.
//!
//! One idle cycle per frame (the `hdr` cycle); bytes go out at one per cycle.
//!
//! # Integration
//!
//! All inputs and both outputs must be registered channels or FIFOs (never
//! port to port, see the netlist note of `stream.rs`). A source must only
//! present its `hdr` when its payload will follow (the arbiter waits for it
//! while holding the output: a source that never completes blocks the
//! others).
//!
//! # Formal
//!
//! `check!`/`cover!` in `run`, proved with `tools/formal/prove.sh tx_arb
//! prove`: single grant held until `last`, bytes only from the granted source,
//! `hdr` before payload, output stream contract, bounded starvation (see the
//! `w*` monitors). `assume!` only for the stream contract (`err` only with
//! `last`) of each payload input.
use crate::eth_tx::EthHdr;
use crate::stream::FrameByte;
use txhdl::comp::{mux, Clock, DefaultClock, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{assume, check, cover, lower, with, Trace};

const IDLE: u8 = 0;
const PAY: u8 = 1;

#[derive(Trace, Default)]
pub struct TxArb {
    /// 0 = idle, 1 = forwarding the payload of `gr`.
    pub st: Reg<U<1>>,
    /// Granted source (valid in PAY).
    pub gr: Reg<U<2>>,
    /// Last winner; the next search starts after it.
    pub rr: Reg<U<2>>,
    // Monitors (only read by check!/cover!; pruned in synthesis).
    /// A frame is open: its `hdr` was sent and its `last` not yet.
    pub m_open: Reg<Bit>,
    /// Source whose `hdr` was sent for the open frame.
    pub m_g: Reg<U<2>>,
    /// Frames granted to others while source i had a `hdr` pending.
    pub w0: Reg<U<2>>,
    pub w1: Reg<U<2>>,
    pub w2: Reg<U<2>>,
    pub w3: Reg<U<2>>,
}

#[lower]
impl
    Unit<
        (
            Rx<EthHdr>,
            Rx<FrameByte>,
            Rx<EthHdr>,
            Rx<FrameByte>,
            Rx<EthHdr>,
            Rx<FrameByte>,
            Rx<EthHdr>,
            Rx<FrameByte>,
        ),
        (Tx<EthHdr>, Tx<FrameByte>),
    > for TxArb
{
    async fn run(
        &mut self,
        (h0, p0, h1, p1, h2, p2, h3, p3): (
            Rx<EthHdr>,
            Rx<FrameByte>,
            Rx<EthHdr>,
            Rx<FrameByte>,
            Rx<EthHdr>,
            Rx<FrameByte>,
            Rx<EthHdr>,
            Rx<FrameByte>,
        ),
        (hdr, payload): (Tx<EthHdr>, Tx<FrameByte>),
    ) {
        loop {
            DefaultClock::rising().await;
            let is_idle = Bit::from(self.st.get() == IDLE);
            let is_pay = Bit::from(self.st.get() == PAY);
            let gr = self.gr.get();
            let rr = self.rr.get();

            // Pending headers.
            let q0 = Bit::from(h0.peek().is_some());
            let q1 = Bit::from(h1.peek().is_some());
            let q2 = Bit::from(h2.peek().is_some());
            let q3 = Bit::from(h3.peek().is_some());
            // Payload heads and validity.
            let b0 = p0.head();
            let b1 = p1.head();
            let b2 = p2.head();
            let b3 = p3.head();
            let v0 = Bit::from(p0.peek().is_some());
            let v1 = Bit::from(p1.peek().is_some());
            let v2 = Bit::from(p2.peek().is_some());
            let v3 = Bit::from(p3.peek().is_some());
            // Stream contract of each payload input (stream.rs, rule 2).
            assume!(!v0 | !b0.err | b0.last, "payload: err only together with last (stream contract 2)");
            assume!(!v1 | !b1.err | b1.last, "payload: err only together with last (stream contract 2)");
            assume!(!v2 | !b2.err | b2.last, "payload: err only together with last (stream contract 2)");
            assume!(!v3 | !b3.err | b3.last, "payload: err only together with last (stream contract 2)");

            // Round-robin choice: order rr+1, rr+2, rr+3, rr.
            // `ek_i`: source i wins when rr == k (order rr+1, rr+2, rr+3, rr).
            let e01 = q1;
            let e02 = !q1 & q2;
            let e03 = !q1 & !q2 & q3;
            let e00 = !q1 & !q2 & !q3 & q0;
            let e12 = q2;
            let e13 = !q2 & q3;
            let e10 = !q2 & !q3 & q0;
            let e11 = !q2 & !q3 & !q0 & q1;
            let e23 = q3;
            let e20 = !q3 & q0;
            let e21 = !q3 & !q0 & q1;
            let e22 = !q3 & !q0 & !q1 & q2;
            let e30 = q0;
            let e31 = !q0 & q1;
            let e32 = !q0 & !q1 & q2;
            let e33 = !q0 & !q1 & !q2 & q3;
            let s0 = (Bit::from(rr == 0) & e00) | (Bit::from(rr == 1) & e10) | (Bit::from(rr == 2) & e20) | (Bit::from(rr == 3) & e30);
            let s1 = (Bit::from(rr == 0) & e01) | (Bit::from(rr == 1) & e11) | (Bit::from(rr == 2) & e21) | (Bit::from(rr == 3) & e31);
            let s2 = (Bit::from(rr == 0) & e02) | (Bit::from(rr == 1) & e12) | (Bit::from(rr == 2) & e22) | (Bit::from(rr == 3) & e32);
            let s3 = (Bit::from(rr == 0) & e03) | (Bit::from(rr == 1) & e13) | (Bit::from(rr == 2) & e23) | (Bit::from(rr == 3) & e33);
            let any = q0 | q1 | q2 | q3;

            let hroom = hdr.ready();
            let proom = payload.ready();
            let take_h = is_idle & any & hroom;
            let t0 = take_h & s0;
            let t1 = take_h & s1;
            let t2 = take_h & s2;
            let t3 = take_h & s3;
            let _ = h0.recv_if(t0);
            let _ = h1.recv_if(t1);
            let _ = h2.recv_if(t2);
            let _ = h3.recv_if(t3);
            let winner = mux(s1, U::<2>::from(1u8), mux(s2, U::<2>::from(2u8), mux(s3, U::<2>::from(3u8), U::<2>::from(0u8))));
            let hh0 = h0.head();
            let hh1 = h1.head();
            let hh2 = h2.head();
            let hh3 = h3.head();
            let hsel = EthHdr {
                dst: mux(s1, hh1.dst, mux(s2, hh2.dst, mux(s3, hh3.dst, hh0.dst))),
                ethertype: mux(s1, hh1.ethertype, mux(s2, hh2.ethertype, mux(s3, hh3.ethertype, hh0.ethertype))),
            };
            if take_h.to_bool() {
                hdr.send(hsel);
            }

            // Payload of the granted source.
            let g0 = Bit::from(gr == 0);
            let g1 = Bit::from(gr == 1);
            let g2 = Bit::from(gr == 2);
            let g3 = Bit::from(gr == 3);
            let pv = (g0 & v0) | (g1 & v1) | (g2 & v2) | (g3 & v3);
            let pay_go = is_pay & pv & proom;
            let r_0 = pay_go & g0;
            let r_1 = pay_go & g1;
            let r_2 = pay_go & g2;
            let r_3 = pay_go & g3;
            let _ = p0.recv_if(r_0);
            let _ = p1.recv_if(r_1);
            let _ = p2.recv_if(r_2);
            let _ = p3.recv_if(r_3);
            let o_data = mux(g1, b1.data, mux(g2, b2.data, mux(g3, b3.data, b0.data)));
            let o_last = mux(g1, b1.last, mux(g2, b2.last, mux(g3, b3.last, b0.last)));
            let o_err = mux(g1, b1.err, mux(g2, b2.err, mux(g3, b3.err, b0.err)));
            let ob = FrameByte { data: o_data, last: o_last, err: o_err };
            if pay_go.to_bool() {
                payload.send(ob);
            }
            let end = pay_go & o_last;

            // Invariants and properties.
            let m_open = self.m_open.get();
            let m_g = self.m_g.get();
            let w0 = self.w0.get();
            let w1 = self.w1.get();
            let w2 = self.w2.get();
            let w3 = self.w3.get();
            // Distance of source i from the head of the order (0 = next).
            let dd0 = U::<2>::from(0u8) - rr - U::<2>::from(1u8);
            let dd1 = U::<2>::from(1u8) - rr - U::<2>::from(1u8);
            let dd2 = U::<2>::from(2u8) - rr - U::<2>::from(1u8);
            let dd3 = U::<2>::from(3u8) - rr - U::<2>::from(1u8);
            check!(Bit::from(m_open == is_pay), "a frame is open exactly while forwarding");
            check!(!is_pay | Bit::from(gr == m_g), "the grant is held until last was forwarded");
            check!(!take_h | !m_open, "no hdr is sent inside a frame (no interleaving)");
            check!(!pay_go | m_open, "hdr precedes its payload: payload bytes only inside a frame");
            check!(!(t0 | t1 | t2 | t3) | (t0 ^ t1 ^ t2 ^ t3), "at most one source granted at a time");
            check!(!((t0 & t1) | (t0 & t2) | (t0 & t3) | (t1 & t2) | (t1 & t3) | (t2 & t3)), "at most one hdr taken per cycle");
            check!(!((r_0 & r_1) | (r_0 & r_2) | (r_0 & r_3) | (r_1 & r_2) | (r_1 & r_3) | (r_2 & r_3)), "at most one payload channel read per cycle");
            check!(!r_0 | Bit::from(m_g == 0), "a forwarded payload byte comes from the granted source");
            check!(!r_1 | Bit::from(m_g == 1), "a forwarded payload byte comes from the granted source");
            check!(!r_2 | Bit::from(m_g == 2), "a forwarded payload byte comes from the granted source");
            check!(!r_3 | Bit::from(m_g == 3), "a forwarded payload byte comes from the granted source");
            check!(!pay_go | !o_err | o_last, "output: err only together with last (stream contract 2)");
            check!(!(t0 | t1 | t2 | t3) | (t0 & Bit::from(winner == 0)) | (t1 & Bit::from(winner == 1)) | (t2 & Bit::from(winner == 2)) | (t3 & Bit::from(winner == 3)), "the grant index matches the source taken");
            // Starvation bound: monitor w_i counts frames granted to others
            // while hdr_i is pending; w_i + distance <= 3 (helper), so a
            // pending source is never passed over a fourth time.
            check!(Bit::from(w0.zext::<3>() + dd0.zext::<3>() <= 3), "starvation monitor 0");
            check!(!(take_h & !s0 & q0) | Bit::from(w0 <= 2), "source 0 pending is granted within 3 other grants");
            check!(!(take_h & !s1 & q1) | Bit::from(w1 <= 2), "source 1 pending is granted within 3 other grants");
            check!(!(take_h & !s2 & q2) | Bit::from(w2 <= 2), "source 2 pending is granted within 3 other grants");
            check!(!(take_h & !s3 & q3) | Bit::from(w3 <= 2), "source 3 pending is granted within 3 other grants");
            check!(Bit::from(w1.zext::<3>() + dd1.zext::<3>() <= 3), "starvation monitor 1");
            check!(Bit::from(w2.zext::<3>() + dd2.zext::<3>() <= 3), "starvation monitor 2");
            check!(Bit::from(w3.zext::<3>() + dd3.zext::<3>() <= 3), "starvation monitor 3");
            cover!(t0, "source 0 granted");
            cover!(t1, "source 1 granted");
            cover!(t2, "source 2 granted");
            cover!(t3, "source 3 granted");
            cover!(take_h & ((q0 & q1) | (q0 & q2) | (q0 & q3) | (q1 & q2) | (q1 & q3) | (q2 & q3)), "two sources contend");
            cover!(end & o_err, "a frame is forwarded with err on last");
            cover!(end & Bit::from(m_g == 3), "a frame of source 3 is forwarded completely");

            with!(self <= {
                take_h ? {
                    st: U::<1>::from(PAY),
                    gr: winner,
                    rr: winner,
                    m_open: Bit::One,
                    m_g: winner,
                },
                end ? { st: U::<1>::from(IDLE), m_open: Bit::Zero },
                // Monitors: count frames granted to others while pending.
                !q0 | t0 ? w0: U::<2>::from(0u8),
                take_h & !s0 & q0 ? w0: w0 + 1,
                !q1 | t1 ? w1: U::<2>::from(0u8),
                take_h & !s1 & q1 ? w1: w1 + 1,
                !q2 | t2 ? w2: U::<2>::from(0u8),
                take_h & !s2 & q2 ? w2: w2 + 1,
                !q3 | t3 ? w3: U::<2>::from(0u8),
                take_h & !s3 & q3 ? w3: w3 + 1,
            });
        }
    }
}
