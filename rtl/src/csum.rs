// SPDX-License-Identifier: Apache-2.0
//! Internet checksum, RFC 1071 (task T3.2), shared by IPv4 and ICMP
//! (seed 0), UDP over IPv4 and IPv6 and ICMPv6 (pseudo-header seeds of
//! RFC 768 and RFC 8200 section 8.1). Reference: `tools/golden/csum.py`.
//!
//! # Arithmetic
//!
//! A one's complement sum of big-endian 16-bit words, kept as a plain
//! 32-bit sum while a stream is read, and folded only when it is read out:
//!
//! * [`csum_add`] adds one word to a running sum (no end-around carry yet);
//! * [`csum_fold`] folds the carries twice, `(x & 0xFFFF) + (x >> 16)`, and
//!   returns the one's complement **sum**, not complemented. The result is
//!   0 only when `x` is 0; otherwise it is in `1..=0xFFFF`, the same value
//!   as the end-around-carry sum of RFC 1071 section 1, so 0xFFFF is kept
//!   apart from 0x0000 where a receiver needs it (see [`CsumOut::ok`]).
//!
//! # Stream
//!
//! [`Csum`] consumes a frame byte stream (T0.1 contract). Bytes pair up
//! into big-endian words: the byte at an even position is the high byte,
//! the one at an odd position the low byte. A stream of odd length ends
//! with a word whose low byte is zero (RFC 1071 section 4.1). The seed
//! input is sampled with the first byte of each stream: it is the caller's
//! pseudo-header sum ([`pseudo_v4`], [`pseudo_v6`]), or 0 for an IPv4 header
//! or an ICMP message. It is folded on sampling ([`csum_fold`]), so the
//! accumulator bound below does not depend on the seed, and this keeps the
//! value and the zero/0xFFFF distinction of the total sum.
//!
//! The seed is read in the cycle the first byte is taken, not when the
//! producer sends it: the channel holds two bytes, so a producer that starts
//! the next stream early must not change the seed until the previous first
//! byte has been taken. The caller holds the seed for the whole stream,
//! which is the simple rule.
//!
//! When the stream's `last` byte is taken, one [`CsumOut`] is sent:
//!
//! * `sum` = complement of the folded total: the value to write in a header
//!   (for a checksum field computed over data whose field is zero);
//! * `ok` = 1 when the folded total is 0xFFFF: the receiver's test, for data
//!   whose checksum field was included.
//!
//! The unit takes a byte only when its output channel has room (`take`
//! below), so the `send` is never refused. The output is written through
//! the channel's register: its `valid` is not combinational from the input.
//! The `err` of the last byte is ignored: the caller discards the frame.
//!
//! # Limits
//!
//! The stream length the unit accepts is 1518 bytes: the stream contract of
//! `stream.rs` (clause 3), which the simulation and the proof both enforce
//! with `assume!`. Within it the accumulator never wraps, and the proof
//! covers that. The 32-bit accumulator would also hold a 64 KiB stream: with
//! the folded seed, the sum is at most `0xFFFF * (1 + 32768) = 2147516415 <
//! 2^32`. A longer stream is outside the contract; it would need a wider
//! byte counter and a new proof.
//!
//! [`udp_csum_final`] is the caller's rule for UDP over IPv4 (RFC 768): a
//! computed checksum of 0x0000 is sent as 0xFFFF, since 0x0000 means "no
//! checksum". The rule for UDP over IPv6 (RFC 8200 section 8.1) is that 0
//! is never sent: the caller applies the same mapping there.
//!
//! # Formal
//!
//! `check!`/`cover!` in [`Csum::run`] state the checksum row of
//! `docs/formal.md` (proved with `tools/formal/prove.sh csum prove`). The
//! only `assume!`s are the two stream contract clauses: `err` only with
//! `last`, and at most 1518 bytes.
use txhdl::comp::{mux, Clock, DefaultClock, In, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{
    assume, check, cover, lower, Trace, Transaction as TransactionDerive, Value as ValueDerive,
};

use crate::stream::FrameByte;

/// Adds one 16-bit word to a 32-bit running sum. No end-around carry: a
/// sum of up to 65536 words stays below 2^32 (see the module documentation).
#[lower]
pub fn csum_add(acc: U<32>, word: U<16>) -> U<32> {
    acc + word.zext::<32>()
}

/// Folds the carries of a 32-bit sum twice and returns the one's complement
/// sum (not complemented). The first fold leaves at most 0x1FFFE, the second
/// at most 0xFFFF; the second fold is what makes the result fit 16 bits.
#[lower]
pub fn csum_fold(acc: U<32>) -> U<16> {
    let x1 = (acc & U::<32>::from(0xFFFFu32)) + (acc >> 16u32);
    let x2 = (x1 & U::<32>::from(0xFFFFu32)) + (x1 >> 16u32);
    x2.slice::<0, 16>()
}

/// Sum of the eight 16-bit words of a 128-bit address, as a balanced tree
/// of additions (depth 3, task T5.3): the same integer as a chain, since
/// no partial sum can wrap (at most `8 * 0xFFFF`).
#[lower]
pub fn sum8(x: U<128>) -> U<32> {
    let b0 = csum_add(x.slice::<0, 16>().zext::<32>(), x.slice::<16, 16>());
    let b1 = csum_add(x.slice::<32, 16>().zext::<32>(), x.slice::<48, 16>());
    let b2 = csum_add(x.slice::<64, 16>().zext::<32>(), x.slice::<80, 16>());
    let b3 = csum_add(x.slice::<96, 16>().zext::<32>(), x.slice::<112, 16>());
    (b0 + b1) + (b2 + b3)
}

/// Unfolded sum of the RFC 768 pseudo-header for IPv4: source, destination
/// (IPv4 in the low 32 bits of the arguments' words), zero and protocol as
/// one word, upper-layer length. Result in `0..=6 * 0xFFFF`. A balanced
/// tree of additions (depth 3, task T5.3): no partial sum wraps, so it is
/// the plain integer sum of the six words.
#[lower]
pub fn pseudo_v4(src: U<32>, dst: U<32>, proto: U<8>, len: U<16>) -> U<32> {
    let a = csum_add(src.slice::<0, 16>().zext::<32>(), src.slice::<16, 16>());
    let b = csum_add(dst.slice::<0, 16>().zext::<32>(), dst.slice::<16, 16>());
    let c = csum_add(proto.zext::<32>(), len);
    (a + b) + c
}

/// Unfolded sum of the RFC 8200 section 8.1 pseudo-header for IPv6: source,
/// destination, 32-bit upper-layer length, three zero bytes and the next
/// header (one word `0x00NN`). Result in `0..=19 * 0xFFFF`. A balanced tree
/// of additions (depth 5 instead of a chain of 19, task T5.3): no partial
/// sum wraps, so it is the plain integer sum of the nineteen words.
#[lower]
pub fn pseudo_v6(src: U<128>, dst: U<128>, next: U<8>, len: U<32>) -> U<32> {
    let t = csum_add(
        csum_add(next.zext::<32>(), len.slice::<16, 16>()),
        len.slice::<0, 16>(),
    );
    (sum8(src) + sum8(dst)) + t
}

/// UDP over IPv4 (RFC 768): a computed checksum of 0x0000 is sent as 0xFFFF.
/// The argument is the complemented sum, i.e. the value [`CsumOut::sum`].
#[lower]
pub fn udp_csum_final(sum: U<16>) -> U<16> {
    mux(sum == U::<16>::from(0u16), U::<16>::from(0xFFFFu16), sum)
}

/// The checksum of one stream, sent when its last byte is taken.
#[derive(TransactionDerive, ValueDerive, Clone, Copy, Default, Debug)]
pub struct CsumOut {
    /// One's complement of the folded sum: the value for a header field.
    pub sum: U<16>,
    /// 1 when the folded sum is 0xFFFF (receiver check, field included).
    pub ok: Bit,
}

/// RFC 1071 checksum unit; see the module documentation.
#[derive(Trace)]
pub struct Csum {
    /// Running sum of the stream's bytes so far, 32 bits, carries not folded.
    /// Zero when no byte of a stream has been taken.
    pub acc: Reg<U<32>>,
    /// High byte of the word in progress.
    pub hi: Reg<U<8>>,
    /// Bytes taken in the current stream.
    pub nb: Reg<U<16>>,
}

impl Default for Csum {
    fn default() -> Self {
        Csum {
            acc: Reg::default(),
            hi: Reg::default(),
            nb: Reg::default(),
        }
    }
}

#[lower]
impl Unit<(Rx<FrameByte>, In<U<32>>), Tx<CsumOut>> for Csum {
    async fn run(&mut self, (bytes, seed): (Rx<FrameByte>, In<U<32>>), out: Tx<CsumOut>) {
        loop {
            DefaultClock::rising().await;
            // Invariants, emitted in every cycle (the induction needs them
            // in cycles where no byte is taken as well).
            let nb = self.nb.get();
            let acc = self.acc.get();
            let at_start = Bit::from(nb == U::<16>::from(0u16));
            check!(
                !at_start | (acc == U::<32>::from(0u32)),
                "the accumulator is back to its initial state after each last"
            );
            check!(
                nb <= U::<16>::from(1517u16),
                "a stream has at most 1518 bytes (stream contract 3)"
            );
            // acc <= 0xFFFF * (1 + nb / 2): the folded seed counts as one word,
            // and each complete word is at most 0xFFFF.
            let words = (nb >> 1u32).zext::<40>() + U::<40>::from(1u32);
            check!(
                acc.zext::<40>() <= words * U::<40>::from(0xFFFFu32),
                "the sum is bounded by 0xFFFF per word, the folded seed counting as one"
            );

            let take = out.ready();
            let r = bytes.recv_if(take);
            let got = Bit::from(r.is_some());
            let b = r.unwrap_or_default();
            assume!(
                !got | !b.err | b.last,
                "err only together with last (stream contract 2)"
            );
            assume!(
                !got | (nb < U::<16>::from(1517u16)) | b.last,
                "a stream has at most 1518 bytes (stream contract 3)"
            );

            // The byte at an odd position completes a word with the high byte
            // kept from the previous one; a last byte at an even position is
            // an odd-length stream, padded with a zero low byte.
            let odd = nb.bit(0);
            let lo = b.data.zext::<16>();
            let word = mux(odd, (self.hi.get().zext::<16>() << 8u32) | lo, lo << 8u32);
            let add = odd | b.last;
            let base = mux(at_start, csum_fold(seed.get()).zext::<32>(), acc);
            let next = mux(add, csum_add(base, word), base);
            check!(
                !(got & add) | (next.zext::<33>() == base.zext::<33>() + word.zext::<33>()),
                "no wrap: a word is added to the sum without overflow"
            );

            // The output: the fold, checked against a second, separately
            // written fold of the same accumulator.
            let folded = csum_fold(next);
            let sum = !folded;
            let ok = Bit::from(folded == U::<16>::from(0xFFFFu16));
            let r1 = (next & U::<32>::from(0xFFFFu32)) + (next >> 16u32);
            let r2 = (r1 & U::<32>::from(0xFFFFu32)) + (r1 >> 16u32);
            check!(
                folded.zext::<32>() == r2,
                "the folded sum is the two-step carry fold of the accumulator"
            );
            let z_sum = Bit::from(next == U::<32>::from(0u32));
            let z_fold = Bit::from(folded == U::<16>::from(0u16));
            check!(!z_sum | z_fold, "a sum of zero folds to zero");
            check!(
                z_sum | !z_fold,
                "a sum that is not zero does not fold to zero"
            );
            check!(
                ok == Bit::from(sum == U::<16>::from(0u16)),
                "ok is set exactly when the complement is zero, i.e. the sum is 0xFFFF"
            );

            cover!(got & b.last & ok, "a stream whose sum is 0xFFFF is ok");
            cover!(
                got & b.last & !ok,
                "a stream whose sum is not 0xFFFF is not ok"
            );
            cover!(
                got & b.last & !odd,
                "an odd length stream is padded with a zero byte"
            );
            cover!(
                got & Bit::from(next > U::<32>::from(0xFFFFu32)),
                "the sum carries past 16 bits"
            );

            if got.to_bool() {
                self.hi.set(b.data);
                if b.last.to_bool() {
                    out.send(CsumOut { sum, ok });
                    self.acc.set(U::<32>::from(0u32));
                    self.nb.set(U::<16>::from(0u16));
                } else {
                    self.acc.set(next);
                    self.nb.set(nb + U::<16>::from(1u16));
                }
            }
        }
    }
}
