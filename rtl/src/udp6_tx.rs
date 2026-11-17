// SPDX-License-Identifier: Apache-2.0
//! UDP over IPv6, transmit side (task T3.4), domain `DefaultClock`
//! (`clk_sys`, 100 MHz). Mirror of `udp4_tx`, with the source address given
//! per request (the caller answers from the address that was targeted).
//!
//! Ports (flat): `req` (`Rx<Udp6TxReq>`) and `payload` (`Rx<FrameByte>`) in;
//! `ip_req` (`Tx<Ip6TxReq>`) and `ip_payload` (`Tx<FrameByte>`) out, towards
//! `ipv6_tx` (`req`, `payload`). Addresses are `U<128>`, first wire byte in
//! the most significant byte; `dst_mac` likewise in `U<48>`.
//!
//! # Behaviour
//!
//! The UDP checksum covers the payload, which follows the header, so the
//! unit **buffers the whole payload** (at most [`MAX_PAYLOAD`] = 128 bytes,
//! a 128 x 8 distributed RAM) and only then emits anything. One datagram
//! per request:
//!
//! 1. Idle: a `req` is taken (one per datagram, before its payload; payload
//!    bytes are not taken before). Its fields are registered. Two
//!    accumulators are seeded in that same cycle from the request, in two
//!    independent sums, each a balanced tree of additions (at most 4 deep,
//!    task T5.3; a single `pseudo_v6` chain would be 19 long): `acc` = sum of `src_ip` + `sport` +
//!    `dport`; `acc2` = sum of `dst_ip` + 33, where 33 = next header 17 +
//!    16, the latter being the length (`8 + n`) counted twice (pseudo-header
//!    and UDP header) without its variable part `2 n`.
//! 2. Collect: the payload stream (`FrameByte`, contract of `stream.rs`) is
//!    taken at one byte per cycle, **never stalled**, written to the buffer
//!    and summed (RFC 1071 words, odd length padded with a zero byte) into
//!    `acc` as it arrives. `req.len` is only checked against the bytes
//!    received.
//! 3. When the `last` byte is taken, the unit sends **`ip_req`**
//!    (`dst_mac`, `dst_ip`, `src_ip` as requested, `next = 17`,
//!    `len = 8 + n`, `hop = 64`, `n` = number of payload bytes buffered)
//!    and then the datagram on `ip_payload`: 8 header bytes (sport, dport,
//!    UDP length `8 + n`, checksum, big endian) and the `n` payload bytes,
//!    one byte per cycle when `ip_payload` has room, `last` on the final
//!    byte. `ip_req` is always sent before the first `ip_payload` byte; one
//!    `ip_req` is followed by exactly one frame of `8 + n` bytes: this is
//!    what `ipv6_tx` needs. **Nothing is ever sent before the payload is
//!    complete**: no `ip_req` for a payload that did not end.
//!
//! The checksum is the complement of the folded sum of the RFC 8200
//! section 8.1 pseudo-header (source, destination, 32-bit length `8 + n`,
//! next header 17), UDP header (checksum field zero) and payload. A
//! computed `0x0000` is sent as `0xFFFF` (`udp_csum_final`): over IPv6 the
//! checksum is mandatory, zero is never sent.
//!
//! # Latency
//!
//! Let `t` be the cycle in which the last payload byte is taken (`recv`)
//! and no output stalls (`ip_req` and `ip_payload` ready):
//!
//! * `ip_req` is sent in cycle `t + 1`;
//! * the first byte of `ip_payload` (UDP source port, high byte) is sent in
//!   cycle `t + 2`; **the latency is 2 cycles, independent of the payload
//!   length** `n` (the sum is accumulated while the bytes arrive; only the
//!   final addition of `acc`, `acc2` and `2 n` and the fold are done in
//!   `t + 1`);
//! * header byte `k` (0..=7) in `t + 2 + k`; payload byte `j` (0-based) in
//!   `t + 10 + j`; the last byte in `t + 9 + n`.
//!
//! Each cycle in which the receiver of `ip_req` or `ip_payload` is not ready
//! moves everything after it by one cycle. The cycle a byte is *sent* is the
//! cycle it is written into the registered channel; the wire time adds the
//! downstream pipeline (`ipv6_tx`, `tx_arb`, `eth_tx`, `tx_path`), which is
//! the NTP server's constant. The unit returns to idle in cycle `t + 10 + n`:
//! the next request is taken from then on (the request channel is
//! not popped before).
//!
//! # Abort
//!
//! An aborted datagram is emitted complete in length, but its final byte
//! carries `last & err` (the consumer discards it; `ipv6_tx` forwards
//! `err`, `tx_path` discards the frame). `err` is only ever set with
//! `last`. The causes:
//!
//! * the payload's `last` byte carries `err` (upstream abort);
//! * the payload is longer than [`MAX_PAYLOAD`]: the first 128 bytes are
//!   kept, the rest is swallowed up to `last`, and the datagram sent is
//!   `8 + 128` bytes long, ending with `err`;
//! * the number of payload bytes differs from `req.len` (shorter or longer,
//!   or `req.len = 0`): the datagram sent has the bytes actually received
//!   (`n`, at most 128) and ends with `err`.
//!
//! # Integration
//!
//! `req`, `payload`, `ip_req` and `ip_payload` must be registered channels
//! or FIFOs (never port to port, see the netlist note of `stream.rs`). A
//! producer must send `req` before the payload; the payload stream of a
//! request must exist (at least one byte, stream contract).
use crate::csum::{csum_add, csum_fold, udp_csum_final};
use crate::ipv6_tx::Ip6TxReq;
use crate::stream::FrameByte;
use crate::udp6_rx::words8_into;
use txhdl::comp::{mux, Clock, DefaultClock, Mem, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{assume, check, cover, lower, with, Trace, Transaction as TransactionDerive, Value as ValueDerive};

/// Largest payload buffered, in bytes.
pub const MAX_PAYLOAD: usize = 128;
/// Next header value of UDP.
pub const PROTO_UDP: u8 = 17;
/// Hop limit of the packets sent.
pub const HOP_LIMIT: u8 = 64;

/// One datagram to send, supplied once per datagram, before its payload.
#[derive(TransactionDerive, ValueDerive, Clone, Copy, Default, Debug)]
pub struct Udp6TxReq {
    /// Ethernet destination; first wire byte in the most significant byte.
    pub dst_mac: U<48>,
    /// Destination IPv6 address, first wire byte most significant.
    pub dst_ip: U<128>,
    /// Source IPv6 address (the address that was targeted by the request).
    pub src_ip: U<128>,
    pub sport: U<16>,
    pub dport: U<16>,
    /// Payload length in bytes (UDP header excluded), 1 to 128.
    pub len: U<16>,
}

const IDLE: u8 = 0;
const COLLECT: u8 = 1;
const REQ: u8 = 2;
const OUT: u8 = 3;

#[derive(Trace, Default)]
pub struct Udp6Tx {
    /// Payload buffer.
    #[distributed]
    pub mem: Mem<U<8>, 128, DefaultClock>,
    pub st: Reg<U<2>>,
    /// Payload bytes buffered (0..=128).
    pub n: Reg<U<8>>,
    pub len: Reg<U<16>>,
    pub dmac: Reg<U<48>>,
    pub dip: Reg<U<128>>,
    pub sip: Reg<U<128>>,
    pub sport: Reg<U<16>>,
    pub dport: Reg<U<16>>,
    /// Running sum (not folded): source address and ports first, then payload.
    pub acc: Reg<U<32>>,
    /// Sum of the destination address and the constants (see the header).
    pub acc2: Reg<U<32>>,
    pub hi: Reg<U<8>>,
    /// The datagram ends with `err`.
    pub abort: Reg<Bit>,
    /// Header image sport, dport, length, checksum; byte 0 in bits 63:56,
    /// shifted left by 8 after each output byte.
    pub hd: Reg<U<64>>,
    /// Output bytes sent so far (0..=135).
    pub oc: Reg<U<8>>,
}

#[lower]
impl Unit<(Rx<Udp6TxReq>, Rx<FrameByte>), (Tx<Ip6TxReq>, Tx<FrameByte>)> for Udp6Tx {
    async fn run(
        &mut self,
        (req, payload): (Rx<Udp6TxReq>, Rx<FrameByte>),
        (ip_req, ip_payload): (Tx<Ip6TxReq>, Tx<FrameByte>),
    ) {
        loop {
            DefaultClock::rising().await;
            let st = self.st.get();
            let n = self.n.get();
            let acc = self.acc.get();
            let hi = self.hi.get();
            let oc = self.oc.get();
            let is_idle = Bit::from(st == IDLE);
            let is_coll = Bit::from(st == COLLECT);
            let is_req = Bit::from(st == REQ);
            let is_out = Bit::from(st == OUT);

            // Idle: take a request, seed the two sums.
            let r = req.head();
            let take_r = is_idle & Bit::from(req.peek().is_some());
            let _ = req.recv_if(take_r);
            // Balanced (task T5.3): the same integer as the chain.
            let seed_a = words8_into(csum_add(r.sport.zext::<32>(), r.dport), r.src_ip);
            let seed_b = words8_into(U::<32>::from(PROTO_UDP as u32 + 16), r.dst_ip);

            // Collect: take every byte, buffer up to 128.
            let pv = Bit::from(payload.peek().is_some());
            let b = payload.head();
            let take_p = is_coll & pv;
            let _ = payload.recv_if(take_p);
            let full = Bit::from(n == U::<8>::from(128u8));
            let wr = take_p & !full;
            let odd = n.bit(0);
            let lo = b.data.zext::<16>();
            let word = mux(odd, (hi.zext::<16>() << 8u32) | lo, lo << 8u32);
            let add = odd | b.last;
            let next = mux(add, csum_add(acc, word), acc);
            let mism = Bit::from(self.len.get() != n.zext::<16>() + U::<16>::from(1u16));

            // Request: the checksum is final after the sum of both
            // accumulators and 2 n (the length counted twice), and a fold.
            let tot = n.zext::<16>() + U::<16>::from(8u16);
            let cks = (acc + self.acc2.get()) + (n.zext::<32>() << 1u32);
            let ck = udp_csum_final(!csum_fold(cks));
            let send_q = is_req & ip_req.ready();
            if send_q.to_bool() {
                ip_req.send(Ip6TxReq {
                    dst_mac: self.dmac.get(),
                    dst_ip: self.dip.get(),
                    src_ip: self.sip.get(),
                    next: U::<8>::from(PROTO_UDP),
                    len: tot,
                    hop: U::<8>::from(HOP_LIMIT),
                });
            }

            // Output: 8 header bytes, then the buffer.
            let send_o = is_out & ip_payload.ready();
            let in_hdr = Bit::from(oc < U::<8>::from(8u8));
            let pb = self.mem.read((oc - U::<8>::from(8u8)).slice::<0, 7>());
            let o_last = Bit::from(oc == n + U::<8>::from(7u8));
            if send_o.to_bool() {
                ip_payload.send(FrameByte {
                    data: mux(in_hdr, self.hd.get().slice::<56, 8>(), pb),
                    last: o_last,
                    err: o_last & self.abort.get(),
                });
            }

            // ---- formal properties (orchestrator; docs/formal.md, rows "UDP" and "Flux d'octets") ----
            assume!(!(take_p) | !b.err | b.last, "payload: err only together with last (stream contract 2)");
            let abort = self.abort.get();
            let rest = (is_req | is_out);
            check!(Bit::from(n <= 128u8), "at most 128 payload bytes are buffered");
            check!(!rest | Bit::from(n >= 1u8), "a datagram has at least one payload byte");
            check!(!send_q | is_req, "nothing is sent before the payload is complete (request only after the last byte)");
            check!(!send_o | is_out, "datagram bytes only after the request was sent");
            check!(!(send_q & send_o), "request and datagram bytes never share a cycle");
            check!(!(rest & !abort) | Bit::from(self.len.get() == n.zext::<16>()), "a datagram without abort has the length the request announced");
            check!(!send_q | Bit::from(tot == n.zext::<16>() + 8u16), "the IP request announces 8 + n bytes");
            check!(!is_out | Bit::from(oc <= n + 7u8), "the datagram has 8 + n bytes");
            check!(!(send_o & o_last) | Bit::from(oc == n + 7u8), "last on byte 8 + n");
            check!(!(send_o & Bit::from(oc == n + 7u8)) | o_last, "byte 8 + n carries last");
            cover!(send_q, "a request is sent");
            cover!(send_o & o_last & !abort, "a good datagram is completed");
            cover!(send_o & o_last & abort, "an aborted datagram is completed");

            with!(self <= {
                take_r ? {
                    st: U::<2>::from(COLLECT),
                    n: U::<8>::from(0u8),
                    len: r.len,
                    dmac: r.dst_mac,
                    dip: r.dst_ip,
                    sip: r.src_ip,
                    sport: r.sport,
                    dport: r.dport,
                    acc: seed_a,
                    acc2: seed_b,
                    abort: Bit::Zero,
                },
                take_p ? hi: b.data,
                wr ? {
                    mem.at(n.slice::<0, 7>()): b.data,
                    n: n + U::<8>::from(1u8),
                    acc: next,
                },
                take_p & full ? abort: Bit::One,
                take_p & b.last ? {
                    st: U::<2>::from(REQ),
                    abort: self.abort.get() | full | b.err | mism,
                },
                send_q ? {
                    st: U::<2>::from(OUT),
                    oc: U::<8>::from(0u8),
                    hd: (self.sport.get().zext::<64>() << 48u32)
                        | (self.dport.get().zext::<64>() << 32u32)
                        | (tot.zext::<64>() << 16u32)
                        | ck.zext::<64>(),
                },
                send_o ? {
                    oc: oc + U::<8>::from(1u8),
                    hd: self.hd.get() << 8u32,
                },
                send_o & o_last ? st: U::<2>::from(IDLE),
            });
        }
    }
}
