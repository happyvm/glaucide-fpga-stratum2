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
use txhdl::{assume, check, cover, lower, with, Trace, Transaction as TransactionDerive, Value as ValueDerive};

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
    // Monitors (check!/cover! only): the header image as built at the request (never shifted).
    pub m_ha: Reg<U<128>>,
    pub m_hb: Reg<U<32>>,
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

            // ---- formal properties (orchestrator; docs/formal.md, row "IPv4", stream part; the header
            // checksum is proved by `ipv4_hdr_equiv`) ----
            assume!(!(go | (is_drain & pv)) | !pb.err | pb.last, "payload: err only together with last (stream contract 2)");
            let mha = self.m_ha.get();
            let mhb = self.m_hb.get();
            let hv = is_hdr & Bit::from(cnt <= 19u8);
            check!(!hv | Bit::from(ha.slice::<120, 8>() == mux(Bit::from(cnt == 0), mha.slice::<120, 8>(), mux(Bit::from(cnt == 1), mha.slice::<112, 8>(), mux(Bit::from(cnt == 2), mha.slice::<104, 8>(), mux(Bit::from(cnt == 3), mha.slice::<96, 8>(), mux(Bit::from(cnt == 4), mha.slice::<88, 8>(), mux(Bit::from(cnt == 5), mha.slice::<80, 8>(), mux(Bit::from(cnt == 6), mha.slice::<72, 8>(), mux(Bit::from(cnt == 7), mha.slice::<64, 8>(), mux(Bit::from(cnt == 8), mha.slice::<56, 8>(), mux(Bit::from(cnt == 9), mha.slice::<48, 8>(), mux(Bit::from(cnt == 10), mha.slice::<40, 8>(), mux(Bit::from(cnt == 11), mha.slice::<32, 8>(), mux(Bit::from(cnt == 12), mha.slice::<24, 8>(), mux(Bit::from(cnt == 13), mha.slice::<16, 8>(), mux(Bit::from(cnt == 14), mha.slice::<8, 8>(), mux(Bit::from(cnt == 15), mha.slice::<0, 8>(), mux(Bit::from(cnt == 16), mhb.slice::<24, 8>(), mux(Bit::from(cnt == 17), mhb.slice::<16, 8>(), mux(Bit::from(cnt == 18), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 19), mhb.slice::<0, 8>(), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 0 is image byte cnt + 0");
            check!(!hv | Bit::from(ha.slice::<112, 8>() == mux(Bit::from(cnt == 0), mha.slice::<112, 8>(), mux(Bit::from(cnt == 1), mha.slice::<104, 8>(), mux(Bit::from(cnt == 2), mha.slice::<96, 8>(), mux(Bit::from(cnt == 3), mha.slice::<88, 8>(), mux(Bit::from(cnt == 4), mha.slice::<80, 8>(), mux(Bit::from(cnt == 5), mha.slice::<72, 8>(), mux(Bit::from(cnt == 6), mha.slice::<64, 8>(), mux(Bit::from(cnt == 7), mha.slice::<56, 8>(), mux(Bit::from(cnt == 8), mha.slice::<48, 8>(), mux(Bit::from(cnt == 9), mha.slice::<40, 8>(), mux(Bit::from(cnt == 10), mha.slice::<32, 8>(), mux(Bit::from(cnt == 11), mha.slice::<24, 8>(), mux(Bit::from(cnt == 12), mha.slice::<16, 8>(), mux(Bit::from(cnt == 13), mha.slice::<8, 8>(), mux(Bit::from(cnt == 14), mha.slice::<0, 8>(), mux(Bit::from(cnt == 15), mhb.slice::<24, 8>(), mux(Bit::from(cnt == 16), mhb.slice::<16, 8>(), mux(Bit::from(cnt == 17), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 18), mhb.slice::<0, 8>(), mux(Bit::from(cnt == 19), U::<8>::from(0u8), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 1 is image byte cnt + 1");
            check!(!hv | Bit::from(ha.slice::<104, 8>() == mux(Bit::from(cnt == 0), mha.slice::<104, 8>(), mux(Bit::from(cnt == 1), mha.slice::<96, 8>(), mux(Bit::from(cnt == 2), mha.slice::<88, 8>(), mux(Bit::from(cnt == 3), mha.slice::<80, 8>(), mux(Bit::from(cnt == 4), mha.slice::<72, 8>(), mux(Bit::from(cnt == 5), mha.slice::<64, 8>(), mux(Bit::from(cnt == 6), mha.slice::<56, 8>(), mux(Bit::from(cnt == 7), mha.slice::<48, 8>(), mux(Bit::from(cnt == 8), mha.slice::<40, 8>(), mux(Bit::from(cnt == 9), mha.slice::<32, 8>(), mux(Bit::from(cnt == 10), mha.slice::<24, 8>(), mux(Bit::from(cnt == 11), mha.slice::<16, 8>(), mux(Bit::from(cnt == 12), mha.slice::<8, 8>(), mux(Bit::from(cnt == 13), mha.slice::<0, 8>(), mux(Bit::from(cnt == 14), mhb.slice::<24, 8>(), mux(Bit::from(cnt == 15), mhb.slice::<16, 8>(), mux(Bit::from(cnt == 16), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 17), mhb.slice::<0, 8>(), mux(Bit::from(cnt == 18), U::<8>::from(0u8), mux(Bit::from(cnt == 19), U::<8>::from(0u8), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 2 is image byte cnt + 2");
            check!(!hv | Bit::from(ha.slice::<96, 8>() == mux(Bit::from(cnt == 0), mha.slice::<96, 8>(), mux(Bit::from(cnt == 1), mha.slice::<88, 8>(), mux(Bit::from(cnt == 2), mha.slice::<80, 8>(), mux(Bit::from(cnt == 3), mha.slice::<72, 8>(), mux(Bit::from(cnt == 4), mha.slice::<64, 8>(), mux(Bit::from(cnt == 5), mha.slice::<56, 8>(), mux(Bit::from(cnt == 6), mha.slice::<48, 8>(), mux(Bit::from(cnt == 7), mha.slice::<40, 8>(), mux(Bit::from(cnt == 8), mha.slice::<32, 8>(), mux(Bit::from(cnt == 9), mha.slice::<24, 8>(), mux(Bit::from(cnt == 10), mha.slice::<16, 8>(), mux(Bit::from(cnt == 11), mha.slice::<8, 8>(), mux(Bit::from(cnt == 12), mha.slice::<0, 8>(), mux(Bit::from(cnt == 13), mhb.slice::<24, 8>(), mux(Bit::from(cnt == 14), mhb.slice::<16, 8>(), mux(Bit::from(cnt == 15), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 16), mhb.slice::<0, 8>(), mux(Bit::from(cnt == 17), U::<8>::from(0u8), mux(Bit::from(cnt == 18), U::<8>::from(0u8), mux(Bit::from(cnt == 19), U::<8>::from(0u8), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 3 is image byte cnt + 3");
            check!(!hv | Bit::from(ha.slice::<88, 8>() == mux(Bit::from(cnt == 0), mha.slice::<88, 8>(), mux(Bit::from(cnt == 1), mha.slice::<80, 8>(), mux(Bit::from(cnt == 2), mha.slice::<72, 8>(), mux(Bit::from(cnt == 3), mha.slice::<64, 8>(), mux(Bit::from(cnt == 4), mha.slice::<56, 8>(), mux(Bit::from(cnt == 5), mha.slice::<48, 8>(), mux(Bit::from(cnt == 6), mha.slice::<40, 8>(), mux(Bit::from(cnt == 7), mha.slice::<32, 8>(), mux(Bit::from(cnt == 8), mha.slice::<24, 8>(), mux(Bit::from(cnt == 9), mha.slice::<16, 8>(), mux(Bit::from(cnt == 10), mha.slice::<8, 8>(), mux(Bit::from(cnt == 11), mha.slice::<0, 8>(), mux(Bit::from(cnt == 12), mhb.slice::<24, 8>(), mux(Bit::from(cnt == 13), mhb.slice::<16, 8>(), mux(Bit::from(cnt == 14), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 15), mhb.slice::<0, 8>(), mux(Bit::from(cnt == 16), U::<8>::from(0u8), mux(Bit::from(cnt == 17), U::<8>::from(0u8), mux(Bit::from(cnt == 18), U::<8>::from(0u8), mux(Bit::from(cnt == 19), U::<8>::from(0u8), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 4 is image byte cnt + 4");
            check!(!hv | Bit::from(ha.slice::<80, 8>() == mux(Bit::from(cnt == 0), mha.slice::<80, 8>(), mux(Bit::from(cnt == 1), mha.slice::<72, 8>(), mux(Bit::from(cnt == 2), mha.slice::<64, 8>(), mux(Bit::from(cnt == 3), mha.slice::<56, 8>(), mux(Bit::from(cnt == 4), mha.slice::<48, 8>(), mux(Bit::from(cnt == 5), mha.slice::<40, 8>(), mux(Bit::from(cnt == 6), mha.slice::<32, 8>(), mux(Bit::from(cnt == 7), mha.slice::<24, 8>(), mux(Bit::from(cnt == 8), mha.slice::<16, 8>(), mux(Bit::from(cnt == 9), mha.slice::<8, 8>(), mux(Bit::from(cnt == 10), mha.slice::<0, 8>(), mux(Bit::from(cnt == 11), mhb.slice::<24, 8>(), mux(Bit::from(cnt == 12), mhb.slice::<16, 8>(), mux(Bit::from(cnt == 13), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 14), mhb.slice::<0, 8>(), mux(Bit::from(cnt == 15), U::<8>::from(0u8), mux(Bit::from(cnt == 16), U::<8>::from(0u8), mux(Bit::from(cnt == 17), U::<8>::from(0u8), mux(Bit::from(cnt == 18), U::<8>::from(0u8), mux(Bit::from(cnt == 19), U::<8>::from(0u8), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 5 is image byte cnt + 5");
            check!(!hv | Bit::from(ha.slice::<72, 8>() == mux(Bit::from(cnt == 0), mha.slice::<72, 8>(), mux(Bit::from(cnt == 1), mha.slice::<64, 8>(), mux(Bit::from(cnt == 2), mha.slice::<56, 8>(), mux(Bit::from(cnt == 3), mha.slice::<48, 8>(), mux(Bit::from(cnt == 4), mha.slice::<40, 8>(), mux(Bit::from(cnt == 5), mha.slice::<32, 8>(), mux(Bit::from(cnt == 6), mha.slice::<24, 8>(), mux(Bit::from(cnt == 7), mha.slice::<16, 8>(), mux(Bit::from(cnt == 8), mha.slice::<8, 8>(), mux(Bit::from(cnt == 9), mha.slice::<0, 8>(), mux(Bit::from(cnt == 10), mhb.slice::<24, 8>(), mux(Bit::from(cnt == 11), mhb.slice::<16, 8>(), mux(Bit::from(cnt == 12), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 13), mhb.slice::<0, 8>(), mux(Bit::from(cnt == 14), U::<8>::from(0u8), mux(Bit::from(cnt == 15), U::<8>::from(0u8), mux(Bit::from(cnt == 16), U::<8>::from(0u8), mux(Bit::from(cnt == 17), U::<8>::from(0u8), mux(Bit::from(cnt == 18), U::<8>::from(0u8), mux(Bit::from(cnt == 19), U::<8>::from(0u8), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 6 is image byte cnt + 6");
            check!(!hv | Bit::from(ha.slice::<64, 8>() == mux(Bit::from(cnt == 0), mha.slice::<64, 8>(), mux(Bit::from(cnt == 1), mha.slice::<56, 8>(), mux(Bit::from(cnt == 2), mha.slice::<48, 8>(), mux(Bit::from(cnt == 3), mha.slice::<40, 8>(), mux(Bit::from(cnt == 4), mha.slice::<32, 8>(), mux(Bit::from(cnt == 5), mha.slice::<24, 8>(), mux(Bit::from(cnt == 6), mha.slice::<16, 8>(), mux(Bit::from(cnt == 7), mha.slice::<8, 8>(), mux(Bit::from(cnt == 8), mha.slice::<0, 8>(), mux(Bit::from(cnt == 9), mhb.slice::<24, 8>(), mux(Bit::from(cnt == 10), mhb.slice::<16, 8>(), mux(Bit::from(cnt == 11), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 12), mhb.slice::<0, 8>(), mux(Bit::from(cnt == 13), U::<8>::from(0u8), mux(Bit::from(cnt == 14), U::<8>::from(0u8), mux(Bit::from(cnt == 15), U::<8>::from(0u8), mux(Bit::from(cnt == 16), U::<8>::from(0u8), mux(Bit::from(cnt == 17), U::<8>::from(0u8), mux(Bit::from(cnt == 18), U::<8>::from(0u8), mux(Bit::from(cnt == 19), U::<8>::from(0u8), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 7 is image byte cnt + 7");
            check!(!hv | Bit::from(ha.slice::<56, 8>() == mux(Bit::from(cnt == 0), mha.slice::<56, 8>(), mux(Bit::from(cnt == 1), mha.slice::<48, 8>(), mux(Bit::from(cnt == 2), mha.slice::<40, 8>(), mux(Bit::from(cnt == 3), mha.slice::<32, 8>(), mux(Bit::from(cnt == 4), mha.slice::<24, 8>(), mux(Bit::from(cnt == 5), mha.slice::<16, 8>(), mux(Bit::from(cnt == 6), mha.slice::<8, 8>(), mux(Bit::from(cnt == 7), mha.slice::<0, 8>(), mux(Bit::from(cnt == 8), mhb.slice::<24, 8>(), mux(Bit::from(cnt == 9), mhb.slice::<16, 8>(), mux(Bit::from(cnt == 10), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 11), mhb.slice::<0, 8>(), mux(Bit::from(cnt == 12), U::<8>::from(0u8), mux(Bit::from(cnt == 13), U::<8>::from(0u8), mux(Bit::from(cnt == 14), U::<8>::from(0u8), mux(Bit::from(cnt == 15), U::<8>::from(0u8), mux(Bit::from(cnt == 16), U::<8>::from(0u8), mux(Bit::from(cnt == 17), U::<8>::from(0u8), mux(Bit::from(cnt == 18), U::<8>::from(0u8), mux(Bit::from(cnt == 19), U::<8>::from(0u8), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 8 is image byte cnt + 8");
            check!(!hv | Bit::from(ha.slice::<48, 8>() == mux(Bit::from(cnt == 0), mha.slice::<48, 8>(), mux(Bit::from(cnt == 1), mha.slice::<40, 8>(), mux(Bit::from(cnt == 2), mha.slice::<32, 8>(), mux(Bit::from(cnt == 3), mha.slice::<24, 8>(), mux(Bit::from(cnt == 4), mha.slice::<16, 8>(), mux(Bit::from(cnt == 5), mha.slice::<8, 8>(), mux(Bit::from(cnt == 6), mha.slice::<0, 8>(), mux(Bit::from(cnt == 7), mhb.slice::<24, 8>(), mux(Bit::from(cnt == 8), mhb.slice::<16, 8>(), mux(Bit::from(cnt == 9), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 10), mhb.slice::<0, 8>(), mux(Bit::from(cnt == 11), U::<8>::from(0u8), mux(Bit::from(cnt == 12), U::<8>::from(0u8), mux(Bit::from(cnt == 13), U::<8>::from(0u8), mux(Bit::from(cnt == 14), U::<8>::from(0u8), mux(Bit::from(cnt == 15), U::<8>::from(0u8), mux(Bit::from(cnt == 16), U::<8>::from(0u8), mux(Bit::from(cnt == 17), U::<8>::from(0u8), mux(Bit::from(cnt == 18), U::<8>::from(0u8), mux(Bit::from(cnt == 19), U::<8>::from(0u8), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 9 is image byte cnt + 9");
            check!(!hv | Bit::from(ha.slice::<40, 8>() == mux(Bit::from(cnt == 0), mha.slice::<40, 8>(), mux(Bit::from(cnt == 1), mha.slice::<32, 8>(), mux(Bit::from(cnt == 2), mha.slice::<24, 8>(), mux(Bit::from(cnt == 3), mha.slice::<16, 8>(), mux(Bit::from(cnt == 4), mha.slice::<8, 8>(), mux(Bit::from(cnt == 5), mha.slice::<0, 8>(), mux(Bit::from(cnt == 6), mhb.slice::<24, 8>(), mux(Bit::from(cnt == 7), mhb.slice::<16, 8>(), mux(Bit::from(cnt == 8), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 9), mhb.slice::<0, 8>(), mux(Bit::from(cnt == 10), U::<8>::from(0u8), mux(Bit::from(cnt == 11), U::<8>::from(0u8), mux(Bit::from(cnt == 12), U::<8>::from(0u8), mux(Bit::from(cnt == 13), U::<8>::from(0u8), mux(Bit::from(cnt == 14), U::<8>::from(0u8), mux(Bit::from(cnt == 15), U::<8>::from(0u8), mux(Bit::from(cnt == 16), U::<8>::from(0u8), mux(Bit::from(cnt == 17), U::<8>::from(0u8), mux(Bit::from(cnt == 18), U::<8>::from(0u8), mux(Bit::from(cnt == 19), U::<8>::from(0u8), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 10 is image byte cnt + 10");
            check!(!hv | Bit::from(ha.slice::<32, 8>() == mux(Bit::from(cnt == 0), mha.slice::<32, 8>(), mux(Bit::from(cnt == 1), mha.slice::<24, 8>(), mux(Bit::from(cnt == 2), mha.slice::<16, 8>(), mux(Bit::from(cnt == 3), mha.slice::<8, 8>(), mux(Bit::from(cnt == 4), mha.slice::<0, 8>(), mux(Bit::from(cnt == 5), mhb.slice::<24, 8>(), mux(Bit::from(cnt == 6), mhb.slice::<16, 8>(), mux(Bit::from(cnt == 7), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 8), mhb.slice::<0, 8>(), mux(Bit::from(cnt == 9), U::<8>::from(0u8), mux(Bit::from(cnt == 10), U::<8>::from(0u8), mux(Bit::from(cnt == 11), U::<8>::from(0u8), mux(Bit::from(cnt == 12), U::<8>::from(0u8), mux(Bit::from(cnt == 13), U::<8>::from(0u8), mux(Bit::from(cnt == 14), U::<8>::from(0u8), mux(Bit::from(cnt == 15), U::<8>::from(0u8), mux(Bit::from(cnt == 16), U::<8>::from(0u8), mux(Bit::from(cnt == 17), U::<8>::from(0u8), mux(Bit::from(cnt == 18), U::<8>::from(0u8), mux(Bit::from(cnt == 19), U::<8>::from(0u8), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 11 is image byte cnt + 11");
            check!(!hv | Bit::from(ha.slice::<24, 8>() == mux(Bit::from(cnt == 0), mha.slice::<24, 8>(), mux(Bit::from(cnt == 1), mha.slice::<16, 8>(), mux(Bit::from(cnt == 2), mha.slice::<8, 8>(), mux(Bit::from(cnt == 3), mha.slice::<0, 8>(), mux(Bit::from(cnt == 4), mhb.slice::<24, 8>(), mux(Bit::from(cnt == 5), mhb.slice::<16, 8>(), mux(Bit::from(cnt == 6), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 7), mhb.slice::<0, 8>(), mux(Bit::from(cnt == 8), U::<8>::from(0u8), mux(Bit::from(cnt == 9), U::<8>::from(0u8), mux(Bit::from(cnt == 10), U::<8>::from(0u8), mux(Bit::from(cnt == 11), U::<8>::from(0u8), mux(Bit::from(cnt == 12), U::<8>::from(0u8), mux(Bit::from(cnt == 13), U::<8>::from(0u8), mux(Bit::from(cnt == 14), U::<8>::from(0u8), mux(Bit::from(cnt == 15), U::<8>::from(0u8), mux(Bit::from(cnt == 16), U::<8>::from(0u8), mux(Bit::from(cnt == 17), U::<8>::from(0u8), mux(Bit::from(cnt == 18), U::<8>::from(0u8), mux(Bit::from(cnt == 19), U::<8>::from(0u8), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 12 is image byte cnt + 12");
            check!(!hv | Bit::from(ha.slice::<16, 8>() == mux(Bit::from(cnt == 0), mha.slice::<16, 8>(), mux(Bit::from(cnt == 1), mha.slice::<8, 8>(), mux(Bit::from(cnt == 2), mha.slice::<0, 8>(), mux(Bit::from(cnt == 3), mhb.slice::<24, 8>(), mux(Bit::from(cnt == 4), mhb.slice::<16, 8>(), mux(Bit::from(cnt == 5), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 6), mhb.slice::<0, 8>(), mux(Bit::from(cnt == 7), U::<8>::from(0u8), mux(Bit::from(cnt == 8), U::<8>::from(0u8), mux(Bit::from(cnt == 9), U::<8>::from(0u8), mux(Bit::from(cnt == 10), U::<8>::from(0u8), mux(Bit::from(cnt == 11), U::<8>::from(0u8), mux(Bit::from(cnt == 12), U::<8>::from(0u8), mux(Bit::from(cnt == 13), U::<8>::from(0u8), mux(Bit::from(cnt == 14), U::<8>::from(0u8), mux(Bit::from(cnt == 15), U::<8>::from(0u8), mux(Bit::from(cnt == 16), U::<8>::from(0u8), mux(Bit::from(cnt == 17), U::<8>::from(0u8), mux(Bit::from(cnt == 18), U::<8>::from(0u8), mux(Bit::from(cnt == 19), U::<8>::from(0u8), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 13 is image byte cnt + 13");
            check!(!hv | Bit::from(ha.slice::<8, 8>() == mux(Bit::from(cnt == 0), mha.slice::<8, 8>(), mux(Bit::from(cnt == 1), mha.slice::<0, 8>(), mux(Bit::from(cnt == 2), mhb.slice::<24, 8>(), mux(Bit::from(cnt == 3), mhb.slice::<16, 8>(), mux(Bit::from(cnt == 4), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 5), mhb.slice::<0, 8>(), mux(Bit::from(cnt == 6), U::<8>::from(0u8), mux(Bit::from(cnt == 7), U::<8>::from(0u8), mux(Bit::from(cnt == 8), U::<8>::from(0u8), mux(Bit::from(cnt == 9), U::<8>::from(0u8), mux(Bit::from(cnt == 10), U::<8>::from(0u8), mux(Bit::from(cnt == 11), U::<8>::from(0u8), mux(Bit::from(cnt == 12), U::<8>::from(0u8), mux(Bit::from(cnt == 13), U::<8>::from(0u8), mux(Bit::from(cnt == 14), U::<8>::from(0u8), mux(Bit::from(cnt == 15), U::<8>::from(0u8), mux(Bit::from(cnt == 16), U::<8>::from(0u8), mux(Bit::from(cnt == 17), U::<8>::from(0u8), mux(Bit::from(cnt == 18), U::<8>::from(0u8), mux(Bit::from(cnt == 19), U::<8>::from(0u8), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 14 is image byte cnt + 14");
            check!(!hv | Bit::from(ha.slice::<0, 8>() == mux(Bit::from(cnt == 0), mha.slice::<0, 8>(), mux(Bit::from(cnt == 1), mhb.slice::<24, 8>(), mux(Bit::from(cnt == 2), mhb.slice::<16, 8>(), mux(Bit::from(cnt == 3), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 4), mhb.slice::<0, 8>(), mux(Bit::from(cnt == 5), U::<8>::from(0u8), mux(Bit::from(cnt == 6), U::<8>::from(0u8), mux(Bit::from(cnt == 7), U::<8>::from(0u8), mux(Bit::from(cnt == 8), U::<8>::from(0u8), mux(Bit::from(cnt == 9), U::<8>::from(0u8), mux(Bit::from(cnt == 10), U::<8>::from(0u8), mux(Bit::from(cnt == 11), U::<8>::from(0u8), mux(Bit::from(cnt == 12), U::<8>::from(0u8), mux(Bit::from(cnt == 13), U::<8>::from(0u8), mux(Bit::from(cnt == 14), U::<8>::from(0u8), mux(Bit::from(cnt == 15), U::<8>::from(0u8), mux(Bit::from(cnt == 16), U::<8>::from(0u8), mux(Bit::from(cnt == 17), U::<8>::from(0u8), mux(Bit::from(cnt == 18), U::<8>::from(0u8), mux(Bit::from(cnt == 19), U::<8>::from(0u8), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 15 is image byte cnt + 15");
            check!(!hv | Bit::from(hb_r.slice::<24, 8>() == mux(Bit::from(cnt == 0), mhb.slice::<24, 8>(), mux(Bit::from(cnt == 1), mhb.slice::<16, 8>(), mux(Bit::from(cnt == 2), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 3), mhb.slice::<0, 8>(), mux(Bit::from(cnt == 4), U::<8>::from(0u8), mux(Bit::from(cnt == 5), U::<8>::from(0u8), mux(Bit::from(cnt == 6), U::<8>::from(0u8), mux(Bit::from(cnt == 7), U::<8>::from(0u8), mux(Bit::from(cnt == 8), U::<8>::from(0u8), mux(Bit::from(cnt == 9), U::<8>::from(0u8), mux(Bit::from(cnt == 10), U::<8>::from(0u8), mux(Bit::from(cnt == 11), U::<8>::from(0u8), mux(Bit::from(cnt == 12), U::<8>::from(0u8), mux(Bit::from(cnt == 13), U::<8>::from(0u8), mux(Bit::from(cnt == 14), U::<8>::from(0u8), mux(Bit::from(cnt == 15), U::<8>::from(0u8), mux(Bit::from(cnt == 16), U::<8>::from(0u8), mux(Bit::from(cnt == 17), U::<8>::from(0u8), mux(Bit::from(cnt == 18), U::<8>::from(0u8), mux(Bit::from(cnt == 19), U::<8>::from(0u8), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 16 is image byte cnt + 16");
            check!(!hv | Bit::from(hb_r.slice::<16, 8>() == mux(Bit::from(cnt == 0), mhb.slice::<16, 8>(), mux(Bit::from(cnt == 1), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 2), mhb.slice::<0, 8>(), mux(Bit::from(cnt == 3), U::<8>::from(0u8), mux(Bit::from(cnt == 4), U::<8>::from(0u8), mux(Bit::from(cnt == 5), U::<8>::from(0u8), mux(Bit::from(cnt == 6), U::<8>::from(0u8), mux(Bit::from(cnt == 7), U::<8>::from(0u8), mux(Bit::from(cnt == 8), U::<8>::from(0u8), mux(Bit::from(cnt == 9), U::<8>::from(0u8), mux(Bit::from(cnt == 10), U::<8>::from(0u8), mux(Bit::from(cnt == 11), U::<8>::from(0u8), mux(Bit::from(cnt == 12), U::<8>::from(0u8), mux(Bit::from(cnt == 13), U::<8>::from(0u8), mux(Bit::from(cnt == 14), U::<8>::from(0u8), mux(Bit::from(cnt == 15), U::<8>::from(0u8), mux(Bit::from(cnt == 16), U::<8>::from(0u8), mux(Bit::from(cnt == 17), U::<8>::from(0u8), mux(Bit::from(cnt == 18), U::<8>::from(0u8), mux(Bit::from(cnt == 19), U::<8>::from(0u8), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 17 is image byte cnt + 17");
            check!(!hv | Bit::from(hb_r.slice::<8, 8>() == mux(Bit::from(cnt == 0), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 1), mhb.slice::<0, 8>(), mux(Bit::from(cnt == 2), U::<8>::from(0u8), mux(Bit::from(cnt == 3), U::<8>::from(0u8), mux(Bit::from(cnt == 4), U::<8>::from(0u8), mux(Bit::from(cnt == 5), U::<8>::from(0u8), mux(Bit::from(cnt == 6), U::<8>::from(0u8), mux(Bit::from(cnt == 7), U::<8>::from(0u8), mux(Bit::from(cnt == 8), U::<8>::from(0u8), mux(Bit::from(cnt == 9), U::<8>::from(0u8), mux(Bit::from(cnt == 10), U::<8>::from(0u8), mux(Bit::from(cnt == 11), U::<8>::from(0u8), mux(Bit::from(cnt == 12), U::<8>::from(0u8), mux(Bit::from(cnt == 13), U::<8>::from(0u8), mux(Bit::from(cnt == 14), U::<8>::from(0u8), mux(Bit::from(cnt == 15), U::<8>::from(0u8), mux(Bit::from(cnt == 16), U::<8>::from(0u8), mux(Bit::from(cnt == 17), U::<8>::from(0u8), mux(Bit::from(cnt == 18), U::<8>::from(0u8), mux(Bit::from(cnt == 19), U::<8>::from(0u8), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 18 is image byte cnt + 18");
            check!(!hv | Bit::from(hb_r.slice::<0, 8>() == mux(Bit::from(cnt == 0), mhb.slice::<0, 8>(), mux(Bit::from(cnt == 1), U::<8>::from(0u8), mux(Bit::from(cnt == 2), U::<8>::from(0u8), mux(Bit::from(cnt == 3), U::<8>::from(0u8), mux(Bit::from(cnt == 4), U::<8>::from(0u8), mux(Bit::from(cnt == 5), U::<8>::from(0u8), mux(Bit::from(cnt == 6), U::<8>::from(0u8), mux(Bit::from(cnt == 7), U::<8>::from(0u8), mux(Bit::from(cnt == 8), U::<8>::from(0u8), mux(Bit::from(cnt == 9), U::<8>::from(0u8), mux(Bit::from(cnt == 10), U::<8>::from(0u8), mux(Bit::from(cnt == 11), U::<8>::from(0u8), mux(Bit::from(cnt == 12), U::<8>::from(0u8), mux(Bit::from(cnt == 13), U::<8>::from(0u8), mux(Bit::from(cnt == 14), U::<8>::from(0u8), mux(Bit::from(cnt == 15), U::<8>::from(0u8), mux(Bit::from(cnt == 16), U::<8>::from(0u8), mux(Bit::from(cnt == 17), U::<8>::from(0u8), mux(Bit::from(cnt == 18), U::<8>::from(0u8), mux(Bit::from(cnt == 19), U::<8>::from(0u8), U::<8>::from(0u8)))))))))))))))))))))), "header shift register byte 19 is image byte cnt + 19");
            check!(!send_h | Bit::from(hb == mux(Bit::from(cnt == 0), mha.slice::<120, 8>(), mux(Bit::from(cnt == 1), mha.slice::<112, 8>(), mux(Bit::from(cnt == 2), mha.slice::<104, 8>(), mux(Bit::from(cnt == 3), mha.slice::<96, 8>(), mux(Bit::from(cnt == 4), mha.slice::<88, 8>(), mux(Bit::from(cnt == 5), mha.slice::<80, 8>(), mux(Bit::from(cnt == 6), mha.slice::<72, 8>(), mux(Bit::from(cnt == 7), mha.slice::<64, 8>(), mux(Bit::from(cnt == 8), mha.slice::<56, 8>(), mux(Bit::from(cnt == 9), mha.slice::<48, 8>(), mux(Bit::from(cnt == 10), mha.slice::<40, 8>(), mux(Bit::from(cnt == 11), mha.slice::<32, 8>(), mux(Bit::from(cnt == 12), mha.slice::<24, 8>(), mux(Bit::from(cnt == 13), mha.slice::<16, 8>(), mux(Bit::from(cnt == 14), mha.slice::<8, 8>(), mux(Bit::from(cnt == 15), mha.slice::<0, 8>(), mux(Bit::from(cnt == 16), mhb.slice::<24, 8>(), mux(Bit::from(cnt == 17), mhb.slice::<16, 8>(), mux(Bit::from(cnt == 18), mhb.slice::<8, 8>(), mux(Bit::from(cnt == 19), mhb.slice::<0, 8>(), U::<8>::from(0u8)))))))))))))))))))))), "header byte cnt sent is byte cnt of the image");
            check!(!take_h | (Bit::from(image_a.slice::<120, 8>() == 0x45u8) & Bit::from(image_a.slice::<112, 8>() == 0u8)), "version 4, IHL 5, DSCP/ECN 0");
            check!(!take_h | Bit::from(image_a.slice::<96, 16>() == total), "total length = 20 + payload length");
            check!(!take_h | (Bit::from(image_a.slice::<80, 16>() == 0u16) & Bit::from(image_a.slice::<64, 16>() == 0x4000u16)), "identification 0, flags DF, fragment offset 0");
            check!(!take_h | (Bit::from(image_a.slice::<56, 8>() == TTL) & Bit::from(image_a.slice::<48, 8>() == r.proto)), "TTL and protocol");
            check!(!take_h | Bit::from(image_a.slice::<32, 16>() == ck), "header checksum");
            check!(!take_h | Bit::from(image_a.slice::<0, 32>() == ip), "source address is local_ip");
            check!(!take_h | is_idle, "a request is taken only between packets");
            check!(!send_h | is_hdr, "header bytes only in the header state");
            check!(!go | is_pay, "payload bytes only in the payload state");
            check!(!is_hdr | Bit::from(cnt <= 19u8), "the header has 20 bytes");
            check!(!(send_h & h_end & bad) | (last & err), "an invalid request ends the 20-byte header with last and err (nothing else is sent)");
            check!(!(send_h & !(h_end & bad)) | !last, "a header byte is not the last unless the request is invalid");
            check!(!bad | Bit::from(len == 0u16) | Bit::from(len > 0xFFEBu16), "invalid means length 0 or above 65515");
            check!(!is_hdr | Bit::from(n == 0u16), "no payload byte before the header is out");
            check!(!((is_hdr | is_pay | is_drain) & !bad) | (Bit::from(len >= 1u16) & Bit::from(len <= 0xFFEBu16)), "a valid request has a length from 1 to 65515");
            check!(!is_pay | (Bit::from(n < len) & !bad), "payload state: bytes remain and the request is valid");
            check!(!(go & last & !err) | Bit::from(n + 1u16 == len), "a packet without err carries exactly the announced length");
            check!(!(go & Bit::from(n + 1u16 == len)) | last, "the announced length ends the packet");
            check!(!(go & err) | last, "err only together with last (stream contract 2)");
            check!(!(send_h & err) | last, "err only together with last (stream contract 2)");
            cover!(take_h, "a request is taken");
            cover!(go & last & !err, "a good packet is completed");
            cover!(go & last & err, "a packet is aborted");
            cover!(send_h & h_end & bad, "an invalid request is aborted in its header");

            with!(self <= {
                take_h ? { m_ha: image_a, m_hb: r.dst_ip },
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
