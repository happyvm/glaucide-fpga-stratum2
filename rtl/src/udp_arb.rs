// SPDX-License-Identifier: Apache-2.0
//! Two-input arbiters of the client side of the top (task T1.14), domain
//! `DefaultClock` (`clk_sys`, 100 MHz). Same logic as `ip_arb.rs`
//! (`IpArb4`/`IpArb6`), one copy per request type:
//!
//! * [`UdpArb4`] (`UdpTxReq`) in front of `udp4_tx` and [`UdpArb6`]
//!   (`Udp6TxReq`) in front of `udp6_tx`: source 0 = the server's replies
//!   (`ntp_reply_route`), source 1 = the NTP client's requests;
//! * [`MetaArb`] (`Meta`): the mode-4 outputs of the two `ntp_demux`
//!   (source 0 IPv4, source 1 IPv6) merged into the single reply input of
//!   `ntp_client` (`meta`, `rep`; `Meta.family` tells the families apart).
//!
//! # Ports (flat, in this order)
//!
//! Inputs `r0`, `p0`, `r1`, `p1`: two sources, each a request (or `Meta`)
//! channel and its payload channel (`FrameByte`). Outputs `req`, `payload`.
//!
//! # Behaviour
//!
//! * IDLE: when at least one `r_i` holds a request and `req` has room, the
//!   winner is chosen round-robin (the source not granted last wins a tie;
//!   source 1 wins the first tie after reset), its request is popped and sent
//!   on `req` in the same cycle, and the unit enters PAY with that source
//!   granted.
//! * PAY: the bytes of the granted payload only are forwarded (`data`,
//!   `last`, `err` unchanged), one per cycle at best, when present and
//!   `payload` has room. The cycle after the byte with `last` the unit is in
//!   IDLE again. The other source is never touched during a packet: its
//!   request and bytes wait in their channels. **A request is therefore
//!   never followed by another source's bytes** (no interleaving).
//! * One idle cycle per packet (the request cycle); bytes at one per cycle.
//! * Fairness: with both sources always pending, they alternate; a pending
//!   source waits for at most one packet of the other.
//!
//! # Contract
//!
//! Each source sends its request before (or in the cycle of) the first byte
//! of its payload, and the payload of an accepted request follows (at least
//! one byte, ending with `last`): the arbiter holds the output while it
//! waits for it. `ntp_reply_route`, `ntp_client` (request then 48 bytes) and
//! `ntp_demux` (a `Meta` is sent before the first byte of its datagram, all
//! bytes follow) satisfy it. Output: one request, then exactly the granted
//! source's payload: the contract of `udp4_tx`/`udp6_tx` (request first,
//! one datagram per request) and of `ntp_client`'s `meta`/`rep`.
//!
//! Latency: a request offered to an idle arbiter with `req` ready is on
//! `req` in the cycle it is popped; payload bytes likewise pass the unit in
//! the cycle they are popped. Between registered channels, the arbiter adds
//! one channel stage (one cycle) to a path compared with a direct
//! connection.
//!
//! # Integration
//!
//! All four inputs and both outputs are registered channels (never port to
//! port, note "Netlist" of `stream.rs`).
use crate::stream::{FrameByte, Meta};
use crate::udp4_tx::UdpTxReq;
use crate::udp6_tx::Udp6TxReq;
use txhdl::comp::{mux, Clock, DefaultClock, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{lower, with, Trace};

const IDLE: u8 = 0;
const PAY: u8 = 1;

/// Arbiter of two `UdpTxReq` sources in front of `udp4_tx`.
#[derive(Trace, Default)]
pub struct UdpArb4 {
    /// IDLE or PAY.
    pub st: Reg<U<1>>,
    /// Granted source (valid in PAY).
    pub gr: Reg<Bit>,
    /// Source granted last (round robin).
    pub rr: Reg<Bit>,
}

#[lower]
impl
    Unit<
        (Rx<UdpTxReq>, Rx<FrameByte>, Rx<UdpTxReq>, Rx<FrameByte>),
        (Tx<UdpTxReq>, Tx<FrameByte>),
    > for UdpArb4
{
    async fn run(
        &mut self,
        (r0, p0, r1, p1): (Rx<UdpTxReq>, Rx<FrameByte>, Rx<UdpTxReq>, Rx<FrameByte>),
        (req, payload): (Tx<UdpTxReq>, Tx<FrameByte>),
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
            let hsel = UdpTxReq {
                dst_mac: mux(s1, h1.dst_mac, h0.dst_mac),
                dst_ip: mux(s1, h1.dst_ip, h0.dst_ip),
                sport: mux(s1, h1.sport, h0.sport),
                dport: mux(s1, h1.dport, h0.dport),
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

/// Arbiter of two `Udp6TxReq` sources in front of `udp6_tx`.
#[derive(Trace, Default)]
pub struct UdpArb6 {
    /// IDLE or PAY.
    pub st: Reg<U<1>>,
    /// Granted source (valid in PAY).
    pub gr: Reg<Bit>,
    /// Source granted last (round robin).
    pub rr: Reg<Bit>,
}

#[lower]
impl
    Unit<
        (Rx<Udp6TxReq>, Rx<FrameByte>, Rx<Udp6TxReq>, Rx<FrameByte>),
        (Tx<Udp6TxReq>, Tx<FrameByte>),
    > for UdpArb6
{
    async fn run(
        &mut self,
        (r0, p0, r1, p1): (Rx<Udp6TxReq>, Rx<FrameByte>, Rx<Udp6TxReq>, Rx<FrameByte>),
        (req, payload): (Tx<Udp6TxReq>, Tx<FrameByte>),
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
            let hsel = Udp6TxReq {
                dst_mac: mux(s1, h1.dst_mac, h0.dst_mac),
                dst_ip: mux(s1, h1.dst_ip, h0.dst_ip),
                src_ip: mux(s1, h1.src_ip, h0.src_ip),
                sport: mux(s1, h1.sport, h0.sport),
                dport: mux(s1, h1.dport, h0.dport),
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

/// Merge of the two mode-4 (`Meta`, payload) streams of the demultiplexers
/// into the client's single reply input.
#[derive(Trace, Default)]
pub struct MetaArb {
    /// IDLE or PAY.
    pub st: Reg<U<1>>,
    /// Granted source (valid in PAY).
    pub gr: Reg<Bit>,
    /// Source granted last (round robin).
    pub rr: Reg<Bit>,
}

#[lower]
impl Unit<(Rx<Meta>, Rx<FrameByte>, Rx<Meta>, Rx<FrameByte>), (Tx<Meta>, Tx<FrameByte>)>
    for MetaArb
{
    async fn run(
        &mut self,
        (r0, p0, r1, p1): (Rx<Meta>, Rx<FrameByte>, Rx<Meta>, Rx<FrameByte>),
        (req, payload): (Tx<Meta>, Tx<FrameByte>),
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
            let hsel = Meta {
                family: mux(s1, h1.family, h0.family),
                src: mux(s1, h1.src, h0.src),
                dst: mux(s1, h1.dst, h0.dst),
                sport: mux(s1, h1.sport, h0.sport),
                dport: mux(s1, h1.dport, h0.dport),
                len: mux(s1, h1.len, h0.len),
                eth_src: mux(s1, h1.eth_src, h0.eth_src),
                rx_ts: mux(s1, h1.rx_ts, h0.rx_ts),
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
