// SPDX-License-Identifier: Apache-2.0
//! MII transmitter (task T1.2): frame bytes in, 4-bit MII nibbles out, in
//! the `ClkTx` domain (25 MHz, 100 Mb/s). IEEE 802.3 clause 22 / 3.2.
//!
//! # Interface
//!
//! * `frames: Rx<FrameByte, ClkTx>`: the COMPLETE frame (padded to 60 bytes,
//!   FCS included, see `stream.rs` item 4), `last` on the final byte. This
//!   unit adds only preamble, SFD and the inter-frame gap.
//! * `txd: U<4>`, `tx_en`: the MII lines. Both are registered (one flop
//!   each, driven straight from a register: no logic before the pad).
//! * `sfd`: single-cycle strobe, high in **the same cycle in which the SFD's
//!   second nibble 0xD is on `txd`** (so `sfd`, `txd = 0xD` and `tx_en` are
//!   seen together at the same `ClkTx` edge). Timestamp strobe for T1.4.
//! * `underrun`: single-cycle pulse, high in the first cycle where `tx_en`
//!   is low after an early end (underrun or truncation, see below).
//!
//! # Wire format
//!
//! Low nibble of every byte first. For each frame: 14 nibbles `0x5`
//! (7 bytes of preamble), the SFD byte `0xD5` (nibbles `0x5`, `0xD`), then
//! the frame bytes, then `tx_en` low. `tx_en` is high for exactly
//! `16 + 2 * len` cycles for a frame of `len` bytes.
//!
//! # Handshake
//!
//! `frames.ready` is high only (a) when idle with the inter-frame gap over:
//! the first byte of a frame is latched there and held through the
//! preamble (ready is low during the gap and the preamble, so no byte is
//! lost and a queued frame simply waits), and (b) in the cycle where the
//! high nibble of a byte is on the wire and the frame goes on: the next
//! byte must be there, since the low nibble of the next byte follows
//! immediately. In every other cycle ready is low.
//!
//! # Inter-frame gap
//!
//! `tx_en` is low for at least 24 `ClkTx` cycles (96 bit times, 802.3
//! 4.4.2.2) between the end of a frame and the next preamble. The counter
//! starts at the first cycle with `tx_en` low and is also loaded at reset,
//! so the first frame after reset is delayed as well.
//!
//! # Early end: underrun, over-long frame
//!
//! `tx_en` never drops inside a frame on its own: a frame is sent without
//! gaps. If the producer has no byte when one is needed (underrun), or the
//! frame would exceed 1518 bytes ([`MAX_WIRE`], truncation), the frame is
//! ended at once: `tx_en` goes low right after the nibble on the wire (the
//! PHY then sees a short frame, with a bad FCS) and `underrun` pulses for
//! one cycle. The rest of that frame is then **discarded** (state `DRAIN`:
//! ready high, bytes dropped up to and including the one with `last`), so
//! that its tail is not taken for a new frame. The gap counts from the end
//! of `tx_en`, the drain runs in parallel; the next frame is accepted
//! after both are done.
//!
//! # Aborted frame (`err` on `last`)
//!
//! The upstream aborted the frame (stream.rs item 2) but earlier bytes are
//! already on the wire. Choice: the last byte is sent as its **low nibble
//! only** and `tx_en` drops right after it, so the frame ends on an odd
//! nibble count: a receiver sees an alignment error and an invalid FCS and
//! discards it. No `underrun` pulse (not an underrun); the frame ends like
//! a normal one otherwise (gap, next frame).
//!
//! # Formal
//!
//! The `check!`/`cover!` statements below are the MII TX row of
//! `docs/formal.md`. They use a few "ghost" registers (`g`, `p*`, `seen`)
//! that only feed those statements: they have no fanout in the real design
//! and synthesis removes them. No `assume!`: the unit is safe for any
//! input stream.
use crate::clocks::ClkTx;
use crate::stream::FrameByte;
use txhdl::comp::{Clock, Out, Reg, Rx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{check, cover, lower, with, Trace};

/// Longest frame on the wire, FCS included: 802.3 maxFrameSize.
pub const MAX_WIRE: usize = 1518;
/// `tx_en`-low cycles needed between two frames: 96 bit times / 4 bits.
pub const IFG_CYCLES: usize = 24;

// States (a `U<3>`; `case!` on an enum is avoided so that checks can use
// plain comparisons).
const IDLE: u8 = 0; // tx_en low, waiting for gap end and a frame
const PRE: u8 = 1; // preamble / SFD, then the first low nibble
const LO: u8 = 2; // the low nibble of `byte` is on txd
const HI: u8 = 3; // the high nibble of `byte` is on txd
const DRAIN: u8 = 4; // tx_en low, discarding the rest of an ended frame

#[derive(Trace)]
/// Generic in its clock for the pin-level tests (real 25 MHz ratio);
/// [`MiiTx`] is the design's instance.
pub struct MiiTxG<C: Clock> {
    pub state: Reg<U<3>, C>,
    /// IDLE/DRAIN: cycles of gap still to wait. PRE: nibbles sent so far.
    pub cnt: Reg<U<5>, C>,
    /// Bytes taken in the current frame.
    pub len: Reg<U<11>, C>,
    pub byte: Reg<U<8>, C>,
    pub last: Reg<Bit, C>,
    pub err: Reg<Bit, C>,
    pub o_txd: Reg<U<4>, C>,
    pub en: Reg<Bit, C>,
    pub o_sfd: Reg<Bit, C>,
    pub und: Reg<Bit, C>,
    // Ghosts, formal only.
    pub g: Reg<U<5>, C>,
    pub pst: Reg<U<3>, C>,
    pub pen: Reg<Bit, C>,
    pub plast: Reg<Bit, C>,
    pub perr: Reg<Bit, C>,
    pub psfd: Reg<Bit, C>,
    pub seen: Reg<Bit, C>,
}

/// The design's instance, at the board's `eth_tx_clk`.
pub type MiiTx = MiiTxG<ClkTx>;

impl<C: Clock> Default for MiiTxG<C> {
    fn default() -> Self {
        MiiTxG {
            state: Reg::new(IDLE),
            // A full gap after reset.
            cnt: Reg::new((IFG_CYCLES - 1) as u8),
            len: Reg::new(0),
            byte: Reg::new(0),
            last: Reg::new(Bit::Zero),
            err: Reg::new(Bit::Zero),
            o_txd: Reg::new(0),
            en: Reg::new(Bit::Zero),
            o_sfd: Reg::new(Bit::Zero),
            und: Reg::new(Bit::Zero),
            g: Reg::new(0),
            pst: Reg::new(IDLE),
            pen: Reg::new(Bit::Zero),
            plast: Reg::new(Bit::Zero),
            perr: Reg::new(Bit::Zero),
            psfd: Reg::new(Bit::Zero),
            seen: Reg::new(Bit::Zero),
        }
    }
}

#[lower]
impl<C: Clock>
    Unit<
        Rx<FrameByte, C>,
        (
            Out<U<4>, C>,
            Out<Bit, C>,
            Out<Bit, C>,
            Out<Bit, C>,
        ),
    > for MiiTxG<C>
{
    async fn run(
        &mut self,
        frames: Rx<FrameByte, C>,
        (txd, tx_en, sfd, underrun): (
            Out<U<4>, C>,
            Out<Bit, C>,
            Out<Bit, C>,
            Out<Bit, C>,
        ),
    ) {
        loop {
            C::rising().await;
            let st = self.state.get();
            let cnt = self.cnt.get();
            let len = self.len.get();
            let byte = self.byte.get();
            let blast = self.last.get();
            let berr = self.err.get();
            let is_idle = Bit::from(st == IDLE);
            let is_pre = Bit::from(st == PRE);
            let is_lo = Bit::from(st == LO);
            let is_hi = Bit::from(st == HI);
            let is_drain = Bit::from(st == DRAIN);
            let gap_over = Bit::from(cnt == 0);
            let short = Bit::from(len < MAX_WIRE as u16);

            // Where a byte is wanted (ready) and whether one came.
            let want_first = is_idle & gap_over;
            let want_next = is_hi & !blast & short;
            let take_c = want_first | want_next | is_drain;
            let offered = frames.head();
            let got = take_c & frames.peek().is_some();
            let _ = frames.recv_if(take_c);

            // Cases of the cycle (mutually exclusive).
            let start = want_first & got;
            let pre_run = is_pre & Bit::from(cnt < 16);
            let pre_sfd = is_pre & Bit::from(cnt == 15);
            let pre_lo = is_pre & Bit::from(cnt == 16);
            let end_err = is_lo & blast & berr;
            let end_ok = is_hi & blast;
            let ended = end_err | end_ok;
            let cont = want_next & got;
            // Early end: a byte is needed but missing, or the frame is too long.
            let abort = is_hi & !blast & !got;
            let drained = is_drain & got & offered.last;

            with!(self <= {
                Bit::One ? {
                    o_sfd: Bit::Zero,
                    und: Bit::Zero,
                },
                (is_idle | is_drain) & !gap_over ? cnt: cnt - 1,
                start ? {
                    state: PRE,
                    cnt: 1,
                    len: 1,
                    byte: offered.data,
                    last: offered.last,
                    err: offered.err,
                    o_txd: 5,
                    en: Bit::One,
                },
                pre_run ? {
                    cnt: cnt + 1,
                    o_txd: 5,
                },
                pre_sfd ? {
                    o_txd: 0xD,
                    o_sfd: Bit::One,
                },
                pre_lo ? {
                    state: LO,
                    o_txd: byte.slice::<0, 4>(),
                },
                is_lo & !end_err ? {
                    state: HI,
                    o_txd: byte.slice::<4, 4>(),
                },
                cont ? {
                    state: LO,
                    len: len + 1,
                    byte: offered.data,
                    last: offered.last,
                    err: offered.err,
                    o_txd: offered.data.slice::<0, 4>(),
                },
                ended ? {
                    state: IDLE,
                    cnt: (IFG_CYCLES - 1) as u8,
                    o_txd: 0,
                    en: Bit::Zero,
                },
                abort ? {
                    state: DRAIN,
                    cnt: (IFG_CYCLES - 1) as u8,
                    o_txd: 0,
                    en: Bit::Zero,
                    und: Bit::One,
                },
                drained ? state: IDLE,
                // Ghosts.
                Bit::One ? {
                    pst: st,
                    pen: self.en.get(),
                    plast: blast,
                    perr: berr,
                    psfd: self.o_sfd.get(),
                },
                self.en.get() ? g: 0,
                !self.en.get() & Bit::from(self.g.get() < 24) ? g: self.g.get() + 1,
                ended | abort ? seen: Bit::One,
            });

            // --- Formal properties (docs/formal.md, "MII TX") ---
            let en = self.en.get();
            let txdv = self.o_txd.get();
            let sfdv = self.o_sfd.get();
            let in_frame = is_pre | is_lo | is_hi;
            // tx_en is high exactly while a frame is being sent.
            if in_frame.to_bool() {
                check!(en.to_bool(), "802.3 3.2: tx_en high throughout a frame");
            } else {
                check!(!en.to_bool(), "802.3 3.2: tx_en low outside a frame");
            }
            // Preamble: 14 nibbles 0x5 then SFD nibbles 0x5, 0xD.
            if pre_run.to_bool() {
                check!(cnt >= 1, "preamble counts from one nibble");
                check!(
                    txdv == 5,
                    "802.3 3.2.1: preamble nibble is 0x5, SFD low nibble 0x5"
                );
            }
            if pre_lo.to_bool() {
                check!(txdv == 0xD, "802.3 3.2.1: SFD second nibble is 0xD");
                check!(sfdv.to_bool(), "sfd strobe with the 0xD nibble");
            }
            check!(
                sfdv == pre_lo,
                "sfd is high exactly while the SFD 0xD nibble is sent"
            );
            if sfdv.to_bool() {
                check!(!self.psfd.get().to_bool(), "sfd is one cycle wide");
            }
            // Nibbles out equal bytes in (low nibble first).
            if is_lo.to_bool() {
                check!(
                    txdv == byte.slice::<0, 4>(),
                    "802.3 3.2: low nibble of the byte first"
                );
            }
            if is_hi.to_bool() {
                check!(
                    txdv == byte.slice::<4, 4>(),
                    "802.3 3.2: high nibble of the byte second"
                );
            }
            // Gap of at least 24 cycles before a preamble (96 bit times).
            if start.to_bool() {
                check!(
                    self.g.get() >= 23,
                    "802.3 4.4.2.2: tx_en low at least 96 bit times between frames"
                );
            }
            if (is_idle | is_drain).to_bool() {
                check!(
                    self.g.get().zext::<6>() + cnt.zext::<6>() >= 23,
                    "gap counter: cycles waited plus cycles to wait reach 24"
                );
            }
            if is_pre.to_bool() {
                check!(cnt >= 1 && cnt <= 16, "preamble nibble counter range");
            }
            check!(
                len <= MAX_WIRE as u16,
                "no frame longer than 1518 bytes goes out"
            );
            // tx_en falls only at the end of the last byte, after the low
            // nibble of an aborted last byte, or on the early-end path.
            if (self.pen.get() & !en).to_bool() {
                check!(
                    ((Bit::from(self.pst.get() == LO) & self.plast.get() & self.perr.get())
                        | (Bit::from(self.pst.get() == HI) & (self.plast.get() | self.und.get())))
                    .to_bool(),
                    "tx_en drops inside a frame only on the documented end, err or underrun paths"
                );
            }
            if self.und.get().to_bool() {
                check!(
                    (self.pen.get() & !en).to_bool(),
                    "underrun pulses where tx_en has just dropped"
                );
            }

            cover!(end_ok, "a normal frame is sent to its last byte");
            cover!(start & self.seen.get(), "back-to-back frames");
            cover!(abort, "underrun");
            // Truncation (1518 bytes, ~3000 cycles) is out of reach of a bounded
            // cover search; it is exercised by the Rust test instead.
            cover!(end_err, "aborted frame (err on last)");
            cover!(drained, "rest of an ended frame discarded");
            cover!(sfdv, "the SFD strobe fires");

            txd.set(self.o_txd.get());
            tx_en.set(self.en.get());
            sfd.set(self.o_sfd.get());
            underrun.set(self.und.get());
        }
    }
}
