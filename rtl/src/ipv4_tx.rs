// SPDX-License-Identifier: Apache-2.0
//! IPv4 packet builder in front of `tx_arb` (task T1.10), domain
//! `DefaultClock` (`clk_sys`, 100 MHz).
//!
//! Ports (flat): `req` ([`IpTxReq`]), `payload` (`FrameByte`), `local_ip`
//! (`In<U<32>>`, config wire, IPv4 address with its first wire byte in the
//! most significant byte, `192.0.2.1` = `0xC000_0201`) in; `hdr`
//! (`Tx<EthHdr>`) and `out` (`Tx<FrameByte>`) out. `hdr` and `out` are the
//! source-1 pair of `tx_arb` (ICMP) or any other source; `eth_tx` adds the
//! source MAC, padding and FCS.
//!
//! # Behaviour
//!
//! One packet per request. When idle, a `req` is taken in a cycle in which
//! `hdr` has room, and in that same cycle `hdr` is sent with
//! `dst = dst_mac`, `ethertype = 0x0800`. `out` then carries the IPv4 packet
//! without Ethernet header:
//!
//! | bytes | field |
//! |-------|-------|
//! | 0 | `0x45` (version 4, IHL 5) |
//! | 1 | `0x00` (DSCP/ECN) |
//! | 2-3 | total length = 20 + `len` |
//! | 4-5 | identification `0x0000` |
//! | 6-7 | `0x4000` (flags DF, offset 0) |
//! | 8 | TTL 64 |
//! | 9 | `proto` |
//! | 10-11 | header checksum (RFC 791, RFC 1071) |
//! | 12-15 | source = `local_ip` sampled when the request is taken |
//! | 16-19 | destination = `dst_ip` |
//!
//! followed by the payload bytes taken from `payload`. The checksum is the
//! complement of the folded one's complement sum of the ten 16-bit header
//! words with the checksum field at zero (`csum_add`/`csum_fold` of
//! `csum.rs`); it is computed combinationally in the cycle the request is
//! taken and held in a register with the other fields, so the first byte
//! leaves with the whole header known. `len` is the payload length in bytes,
//! so nothing is buffered: header bytes are sent from registers (one per
//! cycle, when `out` has room), payload bytes are copied one per cycle at
//! best, `last` on the final byte.
//!
//! # Length mismatch and abort
//!
//! `len` is trusted for the header and checked against the payload
//! stream:
//!
//! * payload `last` arrives with `last & err`: the byte is sent with `last
//!   & err` (the upstream aborted); the consumer discards the frame;
//! * payload `last` arrives before `len` bytes were sent (too short): that
//!   byte is sent with `last & err`;
//! * payload still goes on after `len` bytes (too long): the `len`-th byte
//!   is sent with `last & err` and the rest of that payload, up to its
//!   `last`, is swallowed (DRAIN) so the next packet starts clean;
//! * `len == 0` or `len > 0xFFEB` (total length would not fit 16 bits): the
//!   20 header bytes are sent with `err` on byte 19 (`last`) and the payload
//!   stream of the request, which must exist (at least one byte, stream
//!   contract), is swallowed up to its `last`.
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
//! (never port to port, see the netlist note of `stream.rs`). `local_ip`
//! must be stable at least while a request is being taken. The producer
//! should send its `req` before (or in the cycle of) the first payload byte;
//! `icmp_echo` does.
use crate::csum::{csum_add, csum_fold};
use crate::eth_tx::EthHdr;
use crate::stream::FrameByte;
use txhdl::comp::{mux, Clock, DefaultClock, In, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{lower, with, Trace, Transaction as TransactionDerive, Value as ValueDerive};

/// EtherType of IPv4.
pub const ET_IPV4: u16 = 0x0800;
/// Time to live of every packet we send.
pub const TTL: u8 = 64;

/// One packet to build, supplied once per packet.
#[derive(TransactionDerive, ValueDerive, Clone, Copy, Default, Debug)]
pub struct IpTxReq {
    /// Ethernet destination; first wire byte in the most significant byte.
    pub dst_mac: U<48>,
    pub dst_ip: U<32>,
    pub proto: U<8>,
    /// Payload length in bytes (IPv4 header excluded).
    pub len: U<16>,
}

/// Header checksum of an IPv4 header with version 4, IHL 5, DSCP 0, identification 0,
/// flags DF, TTL [`TTL`], source `ip`, destination `dst`, `total` = total length:
/// the complement of the folded one's complement sum of the ten 16-bit words with
/// the checksum field at zero (RFC 791, RFC 1071). Pure function, proved against
/// the RFC 1071 verification form by `ipv4_hdr_equiv` (formal).
#[lower]
pub fn ipv4_hdr_cksum(ip: U<32>, dst: U<32>, proto: U<8>, total: U<16>) -> U<16> {
    let w_ver = U::<16>::from(0x4500u16);
    let w_df = U::<16>::from(0x4000u16);
    let w_ttl = U::<16>::from((TTL as u16) << 8) | proto.zext::<16>();
    let s0 = csum_add(U::<32>::from(0u32), w_ver);
    let s1 = csum_add(s0, total);
    let s2 = csum_add(s1, w_df);
    let s3 = csum_add(s2, w_ttl);
    let s4 = csum_add(s3, ip.slice::<16, 16>());
    let s5 = csum_add(s4, ip.slice::<0, 16>());
    let s6 = csum_add(s5, dst.slice::<16, 16>());
    let s7 = csum_add(s6, dst.slice::<0, 16>());
    !csum_fold(s7)
}

const IDLE: u8 = 0;
const HDR: u8 = 1;
const PAY: u8 = 2;
const DRAIN: u8 = 3;

#[derive(Trace, Default)]
pub struct Ipv4Tx {
    pub st: Reg<U<2>>,
    /// Header bytes sent so far (0..=19).
    pub cnt: Reg<U<5>>,
    /// Payload bytes sent so far.
    pub n: Reg<U<16>>,
    /// Payload length of the packet in progress.
    pub len: Reg<U<16>>,
    /// The request is invalid (`len` 0 or too large): the header is aborted.
    pub bad: Reg<Bit>,
    /// Header bytes 0-15 and 16-19 (`ha` byte 0 in bits 127:120), shifted left
    /// by 8 together after each byte sent (`hb`'s top byte moves into `ha`).
    pub ha: Reg<U<128>>,
    pub hb: Reg<U<32>>,
}

#[lower]
impl Unit<(Rx<IpTxReq>, Rx<FrameByte>, In<U<32>>), (Tx<EthHdr>, Tx<FrameByte>)> for Ipv4Tx {
    async fn run(
        &mut self,
        (req, payload, local_ip): (Rx<IpTxReq>, Rx<FrameByte>, In<U<32>>),
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
            let hb_r = self.hb.get();
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
                    ethertype: U::<16>::from(ET_IPV4),
                });
            }
            // Header image and checksum, from the request and `local_ip`.
            let ip = local_ip.get();
            let total = r.len + U::<16>::from(20u16);
            let w_ver = U::<16>::from(0x4500u16);
            let w_df = U::<16>::from(0x4000u16);
            let w_ttl = U::<16>::from((TTL as u16) << 8) | r.proto.zext::<16>();
            let ck = ipv4_hdr_cksum(ip, r.dst_ip, r.proto, total);
            let image_a = (w_ver.zext::<128>() << 112u32)
                | (total.zext::<128>() << 96u32)
                | (w_df.zext::<128>() << 64u32)
                | (w_ttl.zext::<128>() << 48u32)
                | (ck.zext::<128>() << 32u32)
                | ip.zext::<128>();
            let r_bad = Bit::from(r.len == U::<16>::from(0u16))
                | Bit::from(r.len > U::<16>::from(0xFFEBu16));

            // Header bytes.
            let send_h = is_hdr & room;
            let h_end = Bit::from(cnt == 19);
            let hb = ha.slice::<120, 8>();

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
                    cnt: U::<5>::from(0u8),
                    n: U::<16>::from(0u16),
                    len: r.len,
                    bad: r_bad,
                    ha: image_a,
                    hb: r.dst_ip,
                },
                send_h ? {
                    cnt: cnt + U::<5>::from(1u8),
                    ha: (ha << 8u32) | hb_r.slice::<24, 8>().zext::<128>(),
                    hb: hb_r << 8u32,
                },
                send_h & h_end ? st: mux(bad, U::<2>::from(DRAIN), U::<2>::from(PAY)),
                go ? n: n + U::<16>::from(1u16),
                go & p_last ? st: mux(pb.last, U::<2>::from(IDLE), U::<2>::from(DRAIN)),
                is_drain & pv & pb.last ? st: U::<2>::from(IDLE),
            });
        }
    }
}
