// SPDX-License-Identifier: Apache-2.0
//! ICMPv4 echo responder (task T1.10), domain `DefaultClock` (`clk_sys`,
//! 100 MHz).
//!
//! Ports (flat): `meta` (`Rx<IpMeta>`) and `payload` (`Rx<FrameByte>`) are
//! the ICMP outputs of `ipv4_rx` (`icmp_meta`, `icmp`); `req`
//! (`Tx<IpTxReq>`) and `out_payload` (`Tx<FrameByte>`) feed `ipv4_tx`
//! (`req`, `payload`).
//!
//! # Behaviour (rules J0-J2 of `tools/golden/responder.py`)
//!
//! The ICMP message (IPv4 payload of `ipv4_rx`, no padding) is read one byte
//! per cycle; `meta` is popped with byte 0. On byte 1 (type, code known) the
//! unit decides:
//!
//! * type 8, code 0 (echo request), at least 8 bytes in total (`meta.len >=
//!   8`) and the message continues after byte 1: it is an echo to answer.
//!   One [`IpTxReq`] is sent with `dst_mac = meta.eth_src`, `dst_ip =
//!   meta.src`, `proto = 1`, `len = meta.len` (so the reply has the same
//!   length), then the reply is streamed on `out_payload`;
//! * anything else (other type or code, shorter than 8, message ending at
//!   byte 1): **nothing is sent at all**, no `req` and no payload byte; the
//!   rest of the message is swallowed.
//!
//! No `req` is ever sent before byte 1 is taken, and none for an unanswered
//! message, so `ipv4_tx` never gets a `hdr` it cannot complete.
//!
//! The reply is byte for byte the request except: type 0 (byte 0), and the
//! checksum (bytes 2-3), which is adjusted incrementally (RFC 1624): the
//! type word changes from `0x0800` to `0x0000`, so the new checksum is the old
//! one plus `0x0800` with end-around carry (`csum_fold(csum_add(c, 0x0800))`,
//! in `1..=0xFFFF`). Identifier, sequence number and data are copied. The
//! first four bytes are held until byte 3 is in (the carry of the low byte can
//! change the high byte), so the reply starts four bytes after the request
//! (no buffering of the packet beyond that), and from byte 4 on a byte goes
//! out in the cycle it is taken.
//!
//! # Abort by late `err`
//!
//! The request's ICMP checksum (RFC 1071 sum of the whole message, checksum
//! field included, folds to `0xFFFF`) is accumulated as the bytes stream
//! through. The verdict is only known at the **last** byte, when the reply
//! has already been sent. The reply is cut-through: if the request checksum
//! was bad, the final byte of the reply is sent with `last & err`
//! (abort). `ipv4_tx` forwards that `err`, and `tx_path` discards frames
//! ending with `err`, so a bad reply never reaches the wire. `err` of the
//! input `last` is copied to the output `last` as well (OR). `err` is set
//! only together with `last`.
//!
//! Input ending early (a truncated packet, `last` before byte 4 after the
//! `req` was sent) cannot be completed: the unit sends the four first reply
//! bytes with `last & err` on the fourth, so the reply always ends with
//! exactly one `last`. `ipv4_tx` sees a length mismatch and aborts the frame
//! as well.
//!
//! # Checksum corner case
//!
//! One's complement has two zeros. The incremental checksum equals the one
//! the Python model computes from scratch (`tools/golden/icmp.py`), except
//! possibly when the reply's data sum is exactly `0xFFFF` with a nonzero
//! data: the model writes `0x0000`, the incremental form `0xFFFF`. Both
//! verify; this unit chooses `0xFFFF` (the only valid choice when the reply
//! words are all zero, which the stream cannot tell from this case).
//!
//! # Integration
//!
//! Inputs and outputs must be registered channels or FIFOs (never port to
//! port). `req` is sent before the first `out_payload` byte, and one `req`
//! is followed by exactly one frame on `out_payload`, which `ipv4_tx` needs.
//! Throughput is one byte per cycle at best; the reply never starts before
//! `req` is accepted, so a full `req` channel stalls the input.
use crate::csum::{csum_add, csum_fold};
use crate::ipv4_rx::IpMeta;
use crate::ipv4_tx::IpTxReq;
use crate::stream::FrameByte;
use txhdl::comp::{mux, Clock, DefaultClock, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{lower, with, Trace};

pub const TYPE_ECHO_REQUEST: u8 = 8;
pub const PROTO_ICMP: u8 = 1;

const GATHER: u8 = 0;
const OUT: u8 = 1;
const PASS: u8 = 2;
const DROP: u8 = 3;

#[derive(Trace, Default)]
pub struct IcmpEcho {
    pub st: Reg<U<2>>,
    /// Bytes of the message taken so far.
    pub pos: Reg<U<16>>,
    /// Running sum of the request (not folded) and high byte in progress.
    pub acc: Reg<U<32>>,
    pub hi: Reg<U<8>>,
    pub mlen: Reg<U<16>>,
    pub msrc: Reg<U<32>>,
    pub esrc: Reg<U<48>>,
    /// First four reply bytes, byte 0 in bits 31:24, shifted as they go out.
    pub oreg: Reg<U<32>>,
    pub oc: Reg<U<2>>,
    /// The input ended before byte 4: the first four bytes end with `err`.
    pub abort: Reg<Bit>,
}

#[lower]
impl Unit<(Rx<IpMeta>, Rx<FrameByte>), (Tx<IpTxReq>, Tx<FrameByte>)> for IcmpEcho {
    async fn run(
        &mut self,
        (meta, payload): (Rx<IpMeta>, Rx<FrameByte>),
        (req, out_payload): (Tx<IpTxReq>, Tx<FrameByte>),
    ) {
        loop {
            DefaultClock::rising().await;
            let st = self.st.get();
            let pos = self.pos.get();
            let acc = self.acc.get();
            let hi = self.hi.get();
            let mlen = self.mlen.get();
            let oreg = self.oreg.get();
            let oc = self.oc.get();
            let abort = self.abort.get();
            let is_g = Bit::from(st == GATHER);
            let is_out = Bit::from(st == OUT);
            let is_pass = Bit::from(st == PASS);
            let is_drop = Bit::from(st == DROP);
            let p0 = is_g & Bit::from(pos == 0);
            let p1 = is_g & Bit::from(pos == 1);
            let p2 = is_g & Bit::from(pos == 2);
            let p3 = is_g & Bit::from(pos == 3);

            let have = Bit::from(payload.peek().is_some());
            let b = payload.head();
            let last = b.last;
            let m = meta.head();
            let mv = Bit::from(meta.peek().is_some());

            // Decision on byte 1: echo request, code 0, at least 8 bytes.
            let accept = Bit::from(hi == U::<8>::from(TYPE_ECHO_REQUEST))
                & Bit::from(b.data == U::<8>::from(0u8))
                & !last
                & Bit::from(mlen >= 8);
            let acc_go = p1 & accept;
            let take = have
                & !is_out
                & (!p0 | mv)
                & (!acc_go | req.ready())
                & (!is_pass | out_payload.ready());
            let _ = payload.recv_if(take);
            let _ = meta.recv_if(take & p0);
            if (take & acc_go).to_bool() {
                req.send(IpTxReq {
                    dst_mac: self.esrc.get(),
                    dst_ip: self.msrc.get(),
                    proto: U::<8>::from(PROTO_ICMP),
                    len: mlen,
                });
            }

            // Running sum of the request, RFC 1071 as in `csum.rs`.
            let odd = pos.bit(0);
            let lo = b.data.zext::<16>();
            let word = mux(odd, (hi.zext::<16>() << 8u32) | lo, lo << 8u32);
            let add = odd | last;
            let next = mux(add, csum_add(acc, word), acc);
            let sum_ok = Bit::from(csum_fold(next) == U::<16>::from(0xFFFFu16));

            // New checksum from the old one (RFC 1624), known at byte 3.
            let ck_old = (hi.zext::<16>() << 8u32) | lo;
            let ck_new = csum_fold(csum_add(ck_old.zext::<32>(), U::<16>::from(0x0800u16)));

            // First four reply bytes: type 0, code 0, new checksum.
            let o_last = is_out & Bit::from(oc == 3) & abort;
            let send_o = is_out & out_payload.ready();
            let send_p = take & is_pass;
            let data = mux(is_out, oreg.slice::<24, 8>(), b.data);
            let last_o = o_last | (is_pass & last);
            let err_o = o_last | (is_pass & (b.err | (last & !sum_ok)));
            if (send_o | send_p).to_bool() {
                out_payload.send(FrameByte {
                    data,
                    last: last_o,
                    err: err_o,
                });
            }

            let to_out = take & ((p2 & last) | p3);
            let end_in = take & last & !(is_g & (Bit::from(pos == 2) | Bit::from(pos == 3)));
            let end_out = send_o & Bit::from(oc == 3) & abort;
            let rst = end_in | end_out;
            with!(self <= {
                take ? {
                    pos: pos + U::<16>::from(1u16),
                    hi: b.data,
                },
                take & (is_g | is_pass) ? acc: next,
                take & p0 ? {
                    esrc: m.eth_src,
                    msrc: m.src,
                    mlen: m.len,
                },
                take & p1 & !accept & !last ? st: U::<2>::from(DROP),
                to_out ? {
                    st: U::<2>::from(OUT),
                    oreg: ck_new.zext::<32>(),
                    oc: U::<2>::from(0u8),
                    abort: last,
                },
                send_o ? {
                    oreg: oreg << 8u32,
                    oc: oc + U::<2>::from(1u8),
                },
                send_o & Bit::from(oc == 3) ? st: U::<2>::from(PASS),
                rst ? {
                    st: U::<2>::from(GATHER),
                    pos: U::<16>::from(0u16),
                    acc: U::<32>::from(0u32),
                    oc: U::<2>::from(0u8),
                    abort: Bit::Zero,
                },
            });
        }
    }
}
