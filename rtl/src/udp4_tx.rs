// SPDX-License-Identifier: Apache-2.0
//! UDP over IPv4, transmit side (task T2.2), domain `DefaultClock`
//! (`clk_sys`, 100 MHz).
//!
//! Ports (flat): `req` (`Rx<UdpTxReq>`), `payload` (`Rx<FrameByte>`) and
//! `local_ip` (`In<U<32>>`, config wire, first wire byte most significant:
//! `192.0.2.1` = `0xC000_0201`) in; `ip_req` (`Tx<IpTxReq>`) and `ip_payload`
//! (`Tx<FrameByte>`) out, towards `ipv4_tx` (`req`, `payload`).
//!
//! # Behaviour
//!
//! The UDP checksum covers the payload, which follows the header, so the unit
//! **buffers the whole payload** (at most [`MAX_PAYLOAD`] = 128 bytes, a
//! 128 x 8 distributed RAM) and only then emits anything. One datagram per
//! request:
//!
//! 1. Idle: a `req` is taken (one per datagram, before its payload; payload
//!    bytes are not taken before). Its fields are registered, and the checksum
//!    accumulator is seeded with the pseudo-header (`pseudo_v4` of `csum.rs`:
//!    `local_ip` as source, `dst_ip`, protocol 17) and the two ports. `local_ip`
//!    is sampled in the cycle the request is taken and must be stable then.
//! 2. Collect: the payload stream (`FrameByte`, contract of `stream.rs`) is
//!    taken at one byte per cycle, **never stalled**, written to the buffer
//!    and summed (RFC 1071 words, odd length padded with a zero byte) as it
//!    arrives. `req.len` is only checked against the bytes received.
//! 3. When the `last` byte is taken, the unit sends **`ip_req`** (`dst_mac`,
//!    `dst_ip`, `proto = 17`, `len = 8 + n`, `n` = number of payload bytes
//!    buffered) and then the datagram on `ip_payload`: 8 header bytes (sport,
//!    dport, UDP length `8 + n`, checksum, big endian) and the `n` payload
//!    bytes, one byte per cycle when `ip_payload` has room, `last` on the
//!    final byte. `ip_req` is always sent before the first `ip_payload` byte,
//!    one `ip_req` is followed by exactly one frame of `8 + n` bytes: this is
//!    what `ipv4_tx` needs. **Nothing is ever sent before the payload is
//!    complete**: no `ip_req` for a payload that did not end.
//!
//! The checksum is the complement of the folded sum of pseudo-header, UDP
//! header (checksum field zero) and payload; a computed `0x0000` is sent as
//! `0xFFFF` (`udp_csum_final`, RFC 768).
//!
//! # Latency
//!
//! Let `t` be the cycle in which the last payload byte is taken (`recv`) and
//! no output stalls (`ip_req` and `ip_payload` ready):
//!
//! * `ip_req` is sent in cycle `t + 1`;
//! * the first byte of `ip_payload` (UDP source port, high byte) is sent in
//!   cycle `t + 2`; **the latency is 2 cycles, independent of the payload
//!   length** `n` (the sum is accumulated while the bytes arrive; only its
//!   final fold, one addition, is done in `t + 1`);
//! * header byte `k` (0..=7) in `t + 2 + k`; payload byte `j` (0-based) in
//!   `t + 10 + j`; the last byte in `t + 9 + n`.
//!
//! Each cycle in which the receiver of `ip_req` or `ip_payload` is not ready
//! moves everything after it by one cycle. The cycle a byte is *sent* is the
//! cycle it is written into the registered channel; the wire time adds the
//! downstream pipeline (`ipv4_tx`, `tx_arb`, `eth_tx`, `tx_path`), which is
//! the NTP server's constant. The unit then returns to idle in cycle
//! `t + 10 + n`: the next request is taken from then on.
//!
//! # Abort
//!
//! An aborted datagram is emitted complete in length, but its final byte
//! carries `last & err` (the consumer discards it; `ipv4_tx` forwards `err`,
//! `tx_path` discards the frame). `err` is only ever set with `last`. The
//! causes:
//!
//! * the payload's `last` byte carries `err` (upstream abort);
//! * the payload is longer than [`MAX_PAYLOAD`]: the first 128 bytes are kept,
//!   the rest is swallowed up to `last`, and the datagram sent is `8 + 128`
//!   bytes long, ending with `err`;
//! * the number of payload bytes differs from `req.len` (shorter or longer, or
//!   `req.len = 0`): the datagram sent has the bytes actually received
//!   (`n`, at most 128) and ends with `err`.
//!
//! # Integration
//!
//! `req`, `payload`, `ip_req` and `ip_payload` must be registered channels or
//! FIFOs (never port to port, see the netlist note of `stream.rs`). A producer
//! must send `req` before the payload; the payload stream of a request must
//! exist (at least one byte, stream contract).
use crate::csum::{csum_add, csum_fold, pseudo_v4, udp_csum_final};
use crate::ipv4_tx::IpTxReq;
use crate::stream::FrameByte;
use txhdl::comp::{mux, Clock, DefaultClock, In, Mem, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{lower, with, Trace, Transaction as TransactionDerive, Value as ValueDerive};

/// Largest payload buffered, in bytes.
pub const MAX_PAYLOAD: usize = 128;
/// IP protocol number of UDP.
pub const PROTO_UDP: u8 = 17;

/// One datagram to send, supplied once per datagram, before its payload.
#[derive(TransactionDerive, ValueDerive, Clone, Copy, Default, Debug)]
pub struct UdpTxReq {
    /// Ethernet destination; first wire byte in the most significant byte.
    pub dst_mac: U<48>,
    /// Destination IPv4 address, first wire byte most significant.
    pub dst_ip: U<32>,
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
pub struct Udp4Tx {
    /// Payload buffer.
    #[distributed]
    pub mem: Mem<U<8>, 128, DefaultClock>,
    pub st: Reg<U<2>>,
    /// Payload bytes buffered (0..=128).
    pub n: Reg<U<8>>,
    pub len: Reg<U<16>>,
    pub dmac: Reg<U<48>>,
    pub dip: Reg<U<32>>,
    pub sport: Reg<U<16>>,
    pub dport: Reg<U<16>>,
    /// Running sum (not folded): pseudo-header and ports first, then payload.
    pub acc: Reg<U<32>>,
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
impl Unit<(Rx<UdpTxReq>, Rx<FrameByte>, In<U<32>>), (Tx<IpTxReq>, Tx<FrameByte>)> for Udp4Tx {
    async fn run(
        &mut self,
        (req, payload, local_ip): (Rx<UdpTxReq>, Rx<FrameByte>, In<U<32>>),
        (ip_req, ip_payload): (Tx<IpTxReq>, Tx<FrameByte>),
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

            // Idle: take a request, seed the sum.
            let r = req.head();
            let take_r = is_idle & Bit::from(req.peek().is_some());
            let _ = req.recv_if(take_r);
            // Two balanced halves (task T5.3): the same integer as the chain
            // pseudo-header + sport + dport, no partial sum wraps.
            let seed = pseudo_v4(
                local_ip.get(),
                r.dst_ip,
                U::<8>::from(PROTO_UDP),
                U::<16>::from(16u16),
            ) + csum_add(r.sport.zext::<32>(), r.dport);

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

            // Request: the checksum is final after one more addition (the
            // length appears twice, 2 * n; the seed holds 16 for it).
            let tot = n.zext::<16>() + U::<16>::from(8u16);
            let cks = csum_add(acc, n.zext::<16>() << 1u32);
            let ck = udp_csum_final(!csum_fold(cks));
            let send_q = is_req & ip_req.ready();
            if send_q.to_bool() {
                ip_req.send(IpTxReq {
                    dst_mac: self.dmac.get(),
                    dst_ip: self.dip.get(),
                    proto: U::<8>::from(PROTO_UDP),
                    len: tot,
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

            with!(self <= {
                take_r ? {
                    st: U::<2>::from(COLLECT),
                    n: U::<8>::from(0u8),
                    len: r.len,
                    dmac: r.dst_mac,
                    dip: r.dst_ip,
                    sport: r.sport,
                    dport: r.dport,
                    acc: seed,
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
