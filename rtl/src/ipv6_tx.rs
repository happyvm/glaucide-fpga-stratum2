// SPDX-License-Identifier: Apache-2.0
//! IPv6 packet builder in front of `tx_arb` (task T3.1), domain
//! `DefaultClock` (`clk_sys`, 100 MHz). Mirror of `ipv4_tx`.
//!
//! Ports (flat): `req` ([`Ip6TxReq`]) and `payload` (`FrameByte`) in; `hdr`
//! (`Tx<EthHdr>`) and `out` (`Tx<FrameByte>`) out. `hdr` and `out` are a
//! source pair of `tx_arb` (source 3 for UDP over IPv6, source 1 for ICMPv6)
//! or any other source; `eth_tx` adds the source MAC, padding and FCS.
//! Addresses are `U<128>` with the first wire byte in the most significant
//! byte (`2001:db8::1` = `0x2001_0db8_0000_0000_0000_0000_0000_0001`), MACs
//! `U<48>` likewise.
//!
//! # Behaviour
//!
//! One packet per request. When idle, a `req` is taken in a cycle in which
//! `hdr` has room, and in that same cycle `hdr` is sent with
//! `dst = dst_mac`, `ethertype = 0x86DD`. `out` then carries the IPv6 packet
//! without Ethernet header:
//!
//! | bytes | field |
//! |-------|-------|
//! | 0-3 | `0x6000_0000` (version 6, traffic class 0, flow label 0) |
//! | 4-5 | payload length = `len` |
//! | 6 | next header = `next` |
//! | 7 | hop limit = `hop` (taken from the request; the caller chooses 64 for echo/UDP replies and 255 for NDP) |
//! | 8-23 | source address = `src_ip` (chosen by the caller: the address that was targeted) |
//! | 24-39 | destination address = `dst_ip` |
//!
//! followed by the payload bytes taken from `payload`. There is no header
//! checksum in IPv6; the upper-layer checksum is the caller's job. `len` is
//! the payload length in bytes (IPv6 header excluded), so nothing is
//! buffered: header bytes are sent from registers (one per cycle, when `out`
//! has room), payload bytes are copied one per cycle at best, `last` on the
//! final byte. The request fields are sampled in the cycle the request is
//! taken.
//!
//! # Length mismatch and abort
//!
//! `len` is trusted for the header and checked against the payload stream:
//!
//! * payload `last` arrives with `last & err`: the byte is sent with
//!   `last & err` (the upstream aborted); the consumer discards the frame;
//! * payload `last` arrives before `len` bytes were sent (too short): that
//!   byte is sent with `last & err`;
//! * payload still goes on after `len` bytes (too long): the `len`-th byte
//!   is sent with `last & err` and the rest of that payload, up to its
//!   `last`, is swallowed so the next packet starts clean;
//! * `len == 0` or `len > 1460` (40 + `len` would exceed the 1500-byte
//!   Ethernet payload): the 40 header bytes are sent with `err` on byte 39
//!   (`last`) and the payload stream of the request, which must exist (at
//!   least one byte, stream contract), is swallowed up to its `last`.
//!
//! In every case the frame sent on `out` after a `hdr` ends with exactly one
//! `last`: a `hdr` is never sent without completing its frame, which is the
//! rule `tx_arb` needs. `err` is set only together with `last`. A good
//! packet has `err = 0`.
//!
//! The payload of a request must be offered (not necessarily at once) once
//! the request is accepted; `tx_arb` waits for it while holding its output.
//!
//! # Integration
//!
//! `req`, `payload`, `hdr` and `out` must be registered channels or FIFOs
//! (never port to port, see the netlist note of `stream.rs`). The producer
//! (ICMPv6 / UDP over IPv6 units) sends its `req` before (or in the cycle
//! of) the first payload byte.
use crate::eth_tx::EthHdr;
use crate::stream::FrameByte;
use txhdl::comp::{mux, Clock, DefaultClock, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{lower, with, Trace, Transaction as TransactionDerive, Value as ValueDerive};

/// EtherType of IPv6.
pub const ET_IPV6: u16 = 0x86DD;
/// Largest payload we send (1500 - 40).
pub const MAX_PAYLOAD: u16 = 1460;

/// One packet to build, supplied once per packet.
#[derive(TransactionDerive, ValueDerive, Clone, Copy, Default, Debug)]
pub struct Ip6TxReq {
    /// Ethernet destination; first wire byte in the most significant byte.
    pub dst_mac: U<48>,
    pub dst_ip: U<128>,
    /// Source address of the packet (chosen by the caller).
    pub src_ip: U<128>,
    /// Next header (58 ICMPv6, 17 UDP).
    pub next: U<8>,
    /// Payload length in bytes (IPv6 header excluded).
    pub len: U<16>,
    pub hop: U<8>,
}

const IDLE: u8 = 0;
const HDR: u8 = 1;
const PAY: u8 = 2;
const DRAIN: u8 = 3;

#[derive(Trace, Default)]
pub struct Ipv6Tx {
    pub st: Reg<U<2>>,
    /// Header bytes sent so far (0..=39).
    pub cnt: Reg<U<6>>,
    /// Payload bytes sent so far.
    pub n: Reg<U<16>>,
    /// Payload length of the packet in progress.
    pub len: Reg<U<16>>,
    /// The request is invalid (`len` 0 or too large): the header is aborted.
    pub bad: Reg<Bit>,
    /// Header image: `ha` bytes 0-7, `hs` source, `hd` destination. Shifted
    /// left by 8 together after each byte sent (`hs`'s top byte moves into
    /// `ha`, `hd`'s into `hs`); the byte to send is the top byte of `ha`.
    pub ha: Reg<U<64>>,
    pub hs: Reg<U<128>>,
    pub hd: Reg<U<128>>,
}

#[lower]
impl Unit<(Rx<Ip6TxReq>, Rx<FrameByte>), (Tx<EthHdr>, Tx<FrameByte>)> for Ipv6Tx {
    async fn run(
        &mut self,
        (req, payload): (Rx<Ip6TxReq>, Rx<FrameByte>),
        (hdr, out): (Tx<EthHdr>, Tx<FrameByte>),
    ) {
        loop {
            DefaultClock::rising().await;
            let st = self.st.get();
            let cnt = self.cnt.get();
            let n = self.n.get();
            let len = self.len.get();
            let bad = self.bad.get();
            let ha = self.ha.get();
            let hs = self.hs.get();
            let hd = self.hd.get();
            let is_idle = Bit::from(st == IDLE);
            let is_hdr = Bit::from(st == HDR);
            let is_pay = Bit::from(st == PAY);
            let is_drain = Bit::from(st == DRAIN);
            let room = out.ready();

            // A request is taken together with sending its `hdr`.
            let r = req.head();
            let take_h = is_idle & Bit::from(req.peek().is_some()) & hdr.ready();
            let _ = req.recv_if(take_h);
            if take_h.to_bool() {
                hdr.send(EthHdr {
                    dst: r.dst_mac,
                    ethertype: U::<16>::from(ET_IPV6),
                });
            }
            // Header image from the request: 0x60000000, len, next, hop.
            let image_a = (U::<64>::from(0x6000_0000u32) << 32u32)
                | (r.len.zext::<64>() << 16u32)
                | (r.next.zext::<64>() << 8u32)
                | r.hop.zext::<64>();
            let r_bad = Bit::from(r.len == U::<16>::from(0u16))
                | Bit::from(r.len > U::<16>::from(MAX_PAYLOAD));

            // Header bytes.
            let send_h = is_hdr & room;
            let h_end = Bit::from(cnt == 39);
            let hb = ha.slice::<56, 8>();

            // Payload bytes.
            let pb = payload.head();
            let pv = Bit::from(payload.peek().is_some());
            let go = is_pay & pv & room;
            let _ = payload.recv_if(go | (is_drain & pv));
            let end_len = Bit::from(n + U::<16>::from(1u16) == len);
            let p_last = pb.last | end_len;
            let p_err = (pb.last & pb.err) | (pb.last ^ end_len);

            let data = mux(send_h, hb, pb.data);
            let last = (send_h & h_end & bad) | (go & p_last);
            let err = (send_h & h_end & bad) | (go & p_err);
            if (send_h | go).to_bool() {
                out.send(FrameByte { data, last, err });
            }

            with!(self <= {
                take_h ? {
                    st: U::<2>::from(HDR),
                    cnt: U::<6>::from(0u8),
                    n: U::<16>::from(0u16),
                    len: r.len,
                    bad: r_bad,
                    ha: image_a,
                    hs: r.src_ip,
                    hd: r.dst_ip,
                },
                send_h ? {
                    cnt: cnt + U::<6>::from(1u8),
                    ha: (ha << 8u32) | hs.slice::<120, 8>().zext::<64>(),
                    hs: (hs << 8u32) | hd.slice::<120, 8>().zext::<128>(),
                    hd: hd << 8u32,
                },
                send_h & h_end ? st: mux(bad, U::<2>::from(DRAIN), U::<2>::from(PAY)),
                go ? n: n + U::<16>::from(1u16),
                go & p_last ? st: mux(pb.last, U::<2>::from(IDLE), U::<2>::from(DRAIN)),
                is_drain & pv & pb.last ? st: U::<2>::from(IDLE),
            });
        }
    }
}
