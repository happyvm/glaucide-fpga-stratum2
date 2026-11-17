// SPDX-License-Identifier: Apache-2.0
//! PHY initialisation (task T1.8): reset of the DP83848J, MDIO management
//! of its registers, and a link-status poll. IEEE 802.3 clause 22 (MDIO,
//! MDC at 1 MHz) and the DP83848J datasheet for the reset and ID values.
//!
//! # Interface
//!
//! * `phy_addr: In<U<5>>`: the PHY's MDIO address. The Arty A7-100 straps
//!   the DP83848J to address **1** (PHYAD0 pulled up, the others down);
//!   the top level ties it to 1. Sampled at each frame, so a wrong value
//!   gives a PHY that never answers (see the `id_ok` note below).
//! * `eth_rstn`: PHY reset, active low. Low for `T::RST` cycles from power
//!   up, then released.
//! * `mdc`, `mdio_o`, `mdio_oe`, `mdio_i`: the management bus. `mdio_o` is
//!   the level, `mdio_oe` high while this unit drives it. The tri-state
//!   buffer and the pull-up are in the top level: `mdio_i` is the pad read
//!   back. `mdio_oe` is low outside a frame, so the line is released.
//! * `link_up`: link status from BMSR bit 2 (last poll).
//! * `id_ok`: PHYIDR1 (reg 2) = `0x2000` and the upper 12 bits of PHYIDR2
//!   (reg 3) = `0x5C9`. The low four bits of PHYIDR2 are the silicon
//!   revision and are ignored. Set once the ID matches, cleared on a
//!   mismatch. A PHY that does not answer reads `0xFFFF` (pull-up): id_ok
//!   stays low.
//! * `ready`: high once `id_ok` is set and the BMCR write is done. Never
//!   high without `id_ok`.
//!
//! # Sequence
//!
//! 1. `eth_rstn` low for `T::RST` cycles (10 ms on the board), then high.
//! 2. `T::POST` cycles (200 ms on the board; the DP83848J needs about
//!    167 ms after reset before MDIO access), then the first frame.
//! 3. Read PHYIDR1, read PHYIDR2. On mismatch: wait `T::RETRY` cycles and
//!    read them again (the PHY may still be starting).
//! 4. Write BMCR (reg 0) = `0x1200`: auto-negotiation enable and restart.
//! 5. Poll BMSR (reg 1) every `T::POLL` cycles. BMSR bit 2 is latched low:
//!    a read that gives 0 is followed at once by a second read, which is
//!    the current status, and that one is taken as `link_up`.
//!
//! # MDC and frames
//!
//! `tick` is a free-running divider: `T::DIV` clock cycles per MDC cycle,
//! so `T::DIV = 100` is 1 MHz from 100 MHz. MDC is low in the first half
//! and high in the second, and it is a flop (`mdc_q`), so it does not
//! glitch. A frame is 64 bits, one per MDC cycle, sent from the
//! start of the low half: 32 ones of preamble, `01` start, the operation
//! (`10` read, `01` write), 5 bits of PHY address, 5 of register, 2 bits
//! of turnaround (`10` for a write, released for a read), 16 bits of data,
//! high bit first. A read releases `mdio_oe` from the first turnaround bit
//! and samples the PHY near the end of each low half (a whole MDC cycle
//! after the PHY changed its bit).
//!
//! # Time constants
//!
//! The delays are a trait ([`PhyTiming`]) so that tests and the formal
//! proof run the same logic with shorter constants. [`Board`] is the real
//! design, [`Sim`] keeps the 1 MHz MDC and shrinks the delays for the
//! testbench, [`Formal`] also shrinks the divider so the proof's induction
//! and cover depths stay reachable.
//!
//! # Formal
//!
//! The `check!`/`cover!` statements below follow the method of
//! `docs/formal.md`: guarantees on the registers, no ghost state, no
//! `assume!`. `mdio_i` and `phy_addr` are free, so the proof covers every
//! possible PHY behaviour. The proof runs on [`Formal`]; the covers are
//! reached in the simulation tests (`rtl/tests/phy_init.rs`) and in the
//! bounded search described in the commit message.
use std::marker::PhantomData;
use txhdl::comp::{mux, Clock, DefaultClock, In, Out, Reg, Unit};
use txhdl::types::{Bit, U};
use txhdl::{check, cover, lower, with, Trace};

// Phases of the sequencer.
const P_RST: u8 = 0; // eth_rstn low, counting `T::RST`
const P_POST: u8 = 1; // released, counting `T::POST`
const P_SET: u8 = 2; // waiting for a bit boundary to launch a frame
const P_XFER: u8 = 3; // a frame is on the line
const P_PAUSE: u8 = 4; // counting the delay before the next frame

// Steps of the initialisation sequence.
const S_ID1: u8 = 0; // read PHYIDR1
const S_ID2: u8 = 1; // read PHYIDR2
const S_BMCR: u8 = 2; // write BMCR = 0x1200
const S_BMSR: u8 = 3; // poll BMSR
const S_BMSR2: u8 = 4; // second BMSR read after a 0

/// The delays and the MDC divider, in clock cycles of `clk_sys`.
pub trait PhyTiming: Default {
    /// Clock cycles per MDC cycle (even; 100 is 1 MHz from 100 MHz).
    const DIV: u64;
    /// Cycles `eth_rstn` is held low.
    const RST: u64;
    /// Cycles from the release of `eth_rstn` to the first frame.
    const POST: u64;
    /// Cycles before the ID is read again after a mismatch.
    const RETRY: u64;
    /// Cycles between two polls of BMSR.
    const POLL: u64;
    /// Half of an MDC cycle: MDC is low for this many cycles.
    const HALF: u64 = Self::DIV / 2;
    /// The longer of the two pauses, which bounds the pause counter.
    const PAUSE_MAX: u64 = if Self::RETRY > Self::POLL {
        Self::RETRY
    } else {
        Self::POLL
    };
}

/// The real board: 1 MHz MDC, 10 ms reset, 200 ms wait, 100 ms polls.
#[derive(Default)]
pub struct Board;
impl PhyTiming for Board {
    const DIV: u64 = 100;
    const RST: u64 = 1_000_000;
    const POST: u64 = 20_000_000;
    const RETRY: u64 = 10_000_000;
    const POLL: u64 = 10_000_000;
}

/// The testbench: the real 1 MHz MDC, with the delays cut to a few hundred
/// cycles so that a run is a few 10^4 cycles.
#[derive(Default)]
pub struct Sim;
impl PhyTiming for Sim {
    const DIV: u64 = 100;
    const RST: u64 = 50;
    const POST: u64 = 200;
    const RETRY: u64 = 300;
    const POLL: u64 = 150;
}

/// The proof: the same logic with a divider of 4 and delays of 2 cycles.
#[derive(Default)]
pub struct Formal;
impl PhyTiming for Formal {
    const DIV: u64 = 4;
    const RST: u64 = 2;
    const POST: u64 = 2;
    const RETRY: u64 = 2;
    const POLL: u64 = 2;
}

/// The 64-bit frame of a transaction, its first bit on top. Preamble
/// (32 ones), `01`, the operation, the PHY address, the register, the
/// turnaround (`10` for a write; the read's is left to the PHY), and the
/// data (the write's word; zero for a read, which the PHY answers).
#[lower]
fn frame_word(step: U<3>, phy: U<5>) -> U<64> {
    let is_w = Bit::from(step == S_BMCR);
    let is_id2 = Bit::from(step == S_ID2);
    let is_bmsr = Bit::from(step == S_BMSR) | Bit::from(step == S_BMSR2);
    let regad = mux(
        is_w,
        U::<5>::from(0u8),
        mux(
            is_bmsr,
            U::<5>::from(1u8),
            mux(is_id2, U::<5>::from(3u8), U::<5>::from(2u8)),
        ),
    );
    // Start `01` and operation: `0101` a write (`01`), `0110` a read (`10`).
    let op = mux(is_w, U::<64>::from(0x5u8), U::<64>::from(0x6u8));
    // Turnaround `10` and the word 0x1200 (BMCR) for the write.
    let tail = mux(is_w, U::<64>::from(0x2_1200u32), U::<64>::from(0u8));
    (U::<64>::from(0xffff_ffffu32) << 32)
        | (op << 28)
        | (phy.zext::<64>() << 23)
        | (regad.zext::<64>() << 18)
        | tail
}

#[derive(Trace)]
pub struct PhyInit<T: PhyTiming> {
    /// Free-running MDC divider, 0 to `T::DIV - 1`.
    pub tick: Reg<U<8>>,
    /// MDC as driven: high in the second half of each cycle.
    pub mdc_q: Reg<Bit>,
    /// Sequencer phase (`P_*`).
    pub phase: Reg<U<3>>,
    /// Cycles into the current reset, post-reset or pause phase.
    pub tmr: Reg<U<25>>,
    /// Transaction of the initialisation sequence (`S_*`).
    pub step: Reg<U<3>>,
    /// The frame; the bit on the line is the top one.
    pub sh: Reg<U<64>>,
    /// Index of the bit on the line, 0 to 63.
    pub bitn: Reg<U<6>>,
    /// The frame is a write.
    pub wr: Reg<Bit>,
    /// `mdio_oe` as driven: high exactly during a frame.
    pub oe_q: Reg<Bit>,
    /// The last 16 bits the PHY drove, the last one at the bottom.
    pub rdv: Reg<U<16>>,
    /// PHYIDR1 matched (step 0), waiting for PHYIDR2.
    pub id1ok: Reg<Bit>,
    /// The ID matched.
    pub idok: Reg<Bit>,
    /// The initialisation is done.
    pub ready: Reg<Bit>,
    /// BMSR bit 2 of the last poll.
    pub link: Reg<Bit>,
    /// `eth_rstn` as driven.
    pub rst_q: Reg<Bit>,
    _t: PhantomData<T>,
}

impl<T: PhyTiming> Default for PhyInit<T> {
    fn default() -> Self {
        PhyInit {
            tick: Reg::new(0),
            mdc_q: Reg::new(Bit::Zero),
            phase: Reg::new(P_RST),
            tmr: Reg::new(0),
            step: Reg::new(S_ID1),
            sh: Reg::new(0),
            bitn: Reg::new(0),
            wr: Reg::new(Bit::Zero),
            oe_q: Reg::new(Bit::Zero),
            rdv: Reg::new(0),
            id1ok: Reg::new(Bit::Zero),
            idok: Reg::new(Bit::Zero),
            ready: Reg::new(Bit::Zero),
            link: Reg::new(Bit::Zero),
            rst_q: Reg::new(Bit::Zero),
            _t: PhantomData,
        }
    }
}

#[lower]
impl<T: PhyTiming>
    Unit<
        (In<U<5>>, In<Bit>),
        (
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
        ),
    > for PhyInit<T>
{
    async fn run(
        &mut self,
        (phy_addr, mdio_i): (In<U<5>>, In<Bit>),
        (pin_rstn, pin_mdc, pin_mdo, pin_oe, pin_link, pin_idok, pin_ready): (
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
            Out<Bit>,
        ),
    ) {
        loop {
            DefaultClock::rising().await;
            let tick = self.tick.get();
            let phase = self.phase.get();
            let tmr = self.tmr.get();
            let step = self.step.get();
            let sh = self.sh.get();
            let bitn = self.bitn.get();
            let wr = self.wr.get();
            let rdv = self.rdv.get();
            let id1ok = self.id1ok.get();
            let idok = self.idok.get();
            let ready = self.ready.get();
            let oe = self.oe_q.get();

            // Phases and steps (mutually exclusive).
            let in_rst = Bit::from(phase == P_RST);
            let in_post = Bit::from(phase == P_POST);
            let in_set = Bit::from(phase == P_SET);
            let in_xfer = Bit::from(phase == P_XFER);
            let in_pause = Bit::from(phase == P_PAUSE);
            let s0 = Bit::from(step == S_ID1);
            let s1 = Bit::from(step == S_ID2);
            let s2 = Bit::from(step == S_BMCR);
            let s3 = Bit::from(step == S_BMSR);
            let s4 = Bit::from(step == S_BMSR2);

            // Divider and bit boundaries.
            let div_end = Bit::from(tick == T::DIV - 1);
            let last_bit = Bit::from(bitn == 63);
            let rst_last = in_rst & Bit::from(tmr == T::RST - 1);
            let post_last = in_post & Bit::from(tmr == T::POST - 1);
            let pause_last = in_pause
                & ((s0 & Bit::from(tmr == T::RETRY - 1)) | (!s0 & Bit::from(tmr == T::POLL - 1)));
            let set_go = in_set & div_end;
            let xfer_end = in_xfer & div_end & last_bit;
            let xfer_bit = in_xfer & div_end & !last_bit;
            // The PHY's bit is sampled at the end of its low half.
            let sample = in_xfer & Bit::from(tick == T::HALF - 1) & Bit::from(bitn >= 48);
            // Read turnaround and data: the master lets go of MDIO.
            let rd_turn = in_xfer & !wr & Bit::from(bitn >= 46);

            // What a finished frame tells us.
            let is_w = Bit::from(step == S_BMCR);
            let word = frame_word(step, phy_addr.get());
            let id1_match = Bit::from(rdv == 0x2000);
            let id2_match = Bit::from(rdv.slice::<4, 12>() == 0x5c9);
            let s1_match = id1ok & id2_match;
            let link_b = rdv.bit(2);

            // --- Formal properties (docs/formal.md, "PHY init") ---
            check!(tick < T::DIV, "the MDC divider stays inside its period");
            check!(
                self.mdc_q.get() == Bit::from(tick >= T::HALF),
                "802.3 22.2.2: MDC is low in the first half of its cycle, high in the second"
            );
            check!(
                self.rst_q.get() == Bit::from(phase != P_RST),
                "eth_rstn is low exactly during the reset phase"
            );
            check!(
                oe == (in_xfer & (wr | Bit::from(bitn < 46))),
                "802.3 22.2.4.3: MDIO is driven exactly for the bits of a frame before a read's turnaround"
            );
            if in_rst.to_bool() {
                check!(tmr < T::RST, "the reset counter stays within its period");
            }
            if in_post.to_bool() {
                check!(
                    tmr < T::POST,
                    "the post-reset counter stays within its period"
                );
            }
            if in_pause.to_bool() {
                check!(
                    tmr < T::PAUSE_MAX,
                    "the pause counter stays within its period"
                );
            }
            check!(step <= S_BMSR2, "the step is one of the known transactions");
            if rd_turn.to_bool() {
                check!(
                    !oe.to_bool(),
                    "802.3 22.2.4.3: a read releases MDIO for the turnaround and the data"
                );
            }
            if ready.to_bool() {
                check!(idok.to_bool(), "ready implies id_ok");
            }
            if Bit::from(step >= S_BMCR).to_bool() {
                check!(
                    idok.to_bool(),
                    "the BMCR write and the polls run only after the ID matched"
                );
            }
            cover!(idok.to_bool(), "id_ok is reached");
            cover!(ready.to_bool(), "ready is reached");
            cover!(self.link.get(), "link_up is reached");
            cover!(xfer_end & s1 & !s1_match, "an ID mismatch is retried");

            with!(self <= {
                // The divider runs all the time and MDC follows it.
                Bit::One ? {
                    tick: mux(div_end, U::<8>::from(0u8), tick + 1),
                    mdc_q: Bit::from(tick >= T::HALF - 1)
                        & Bit::from(tick < T::DIV - 1),
                    tmr: 0,
                },
                // Counters run only in their own phase.
                (in_rst & !rst_last) | (in_post & !post_last) | (in_pause & !pause_last) ? {
                    tmr: tmr + 1,
                },
                rst_last ? { phase: P_POST, rst_q: Bit::One },
                post_last ? { phase: P_SET },
                set_go ? {
                    phase: P_XFER,
                    sh: word,
                    bitn: 0,
                    wr: is_w,
                    oe_q: Bit::One,
                },
                xfer_bit ? {
                    bitn: bitn + 1,
                    sh: sh << 1,
                    oe_q: wr | Bit::from(bitn < 45),
                },
                sample ? { rdv: (rdv << 1) | mdio_i.get().zext::<16>() },
                xfer_end ? { oe_q: Bit::Zero },
                xfer_end & s0 ? { phase: P_SET, step: S_ID2, id1ok: id1_match },
                xfer_end & s1 & s1_match ? {
                    phase: P_SET,
                    step: S_BMCR,
                    idok: Bit::One,
                },
                xfer_end & s1 & !s1_match ? {
                    phase: P_PAUSE,
                    step: S_ID1,
                    idok: Bit::Zero,
                    ready: Bit::Zero,
                },
                xfer_end & s2 ? {
                    phase: P_PAUSE,
                    step: S_BMSR,
                    ready: Bit::One,
                },
                xfer_end & s3 & link_b ? {
                    phase: P_PAUSE,
                    step: S_BMSR,
                    link: Bit::One,
                },
                xfer_end & s3 & !link_b ? {
                    phase: P_SET,
                    step: S_BMSR2,
                    link: Bit::Zero,
                },
                xfer_end & s4 ? {
                    phase: P_PAUSE,
                    step: S_BMSR,
                    link: link_b,
                },
                pause_last ? { phase: P_SET },
            });

            pin_rstn.set(self.rst_q.get());
            pin_mdc.set(self.mdc_q.get());
            pin_mdo.set(self.sh.get().bit(63));
            pin_oe.set(self.oe_q.get());
            pin_link.set(self.link.get());
            pin_idok.set(self.idok.get());
            pin_ready.set(self.ready.get());
        }
    }
}
