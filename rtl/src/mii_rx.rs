// SPDX-License-Identifier: Apache-2.0
//! MII receiver (task T1.1): PHY nibbles to a frame byte stream, all in the
//! `ClkRx` domain (25 MHz `eth_rx_clk`).
//!
//! Ports: `rxd` (4 bits, low nibble first), `rx_dv`, `rx_er` in; `frames`
//! (`Tx<FrameByte>`, raw frame, FCS included) and `sfd` out.
//!
//! # Behaviour (IEEE 802.3 clause 22)
//!
//! * Hunting (idle): while `rx_dv` is low nothing happens. With `rx_dv` high,
//!   nibbles `0x5` are preamble and are skipped (any number, including none);
//!   the first `0xD` is the SFD. Any other value first, or `rx_dv` falling
//!   before the SFD, aborts silently: after another value the unit waits for
//!   `rx_dv` to fall before it looks for a new frame.
//! * `sfd`: registered, one cycle wide. The SFD nibble is on `rxd` at clock
//!   edge k; `sfd` is high from edge k to edge k+1 (the cycle in which the
//!   first data nibble is on the wire). Fixed latency of one `clk_rx` cycle.
//! * After the SFD, nibbles are paired low nibble first into bytes. The last
//!   byte is known only when `rx_dv` falls, so one byte is always held back:
//!   byte n is sent at the edge where byte n+1 completes (2 cycles after
//!   its own last nibble), and the final byte at the first edge with
//!   `rx_dv` low (1 cycle after its last nibble). Output: the whole frame
//!   after the SFD, FCS included, `last` on the final byte. A frame without
//!   a single complete byte produces nothing (a stream frame has at least 1
//!   byte).
//! * `err` on `last` if `rx_er` was high on any cycle with `rx_dv` high after
//!   the SFD, or an odd nibble was left at the end, or on overrun or too
//!   long (below). `err` is 0 on every other byte.
//! * Minimum frame length and the FCS are not checked here (T1.12).
//!
//! # Overrun and length limit
//!
//! The wire cannot be stalled. A byte that must be sent while
//! `frames.ready()` is low cannot wait for the next byte, so:
//! * mid-frame (the held byte must go out as a non-last byte): the frame is
//!   cut here. The held byte is kept and sent as `last` with `err` at the
//!   first edge `ready` is high again (the bytes already sent stay sent);
//!   the rest of the frame is dropped until `rx_dv` falls. A new frame whose
//!   SFD arrives while that byte is still pending is dropped too.
//! * at the end of the frame (the held byte is the final one): it is simply
//!   kept and sent as `last` (with the normal `err`) when `ready` is high.
//!   No data is lost, but it counts as pending as above.
//! * a frame that would exceed [`MAX_RX_FRAME`] bytes (1518) is cut after
//!   byte 1518, which is sent as `last` with `err`; the rest is dropped.
//!
//! Never an `err` on a non-last byte, never a byte after `last`.
//!
//! # Formal
//!
//! `check!`/`assume!`/`cover!` statements in `run` state the MII RX row of
//! `docs/formal.md` and the frame-stream contract; they are proved with
//! `tools/formal/prove.sh mii_rx prove`. No `assume!` is needed: the three
//! wire inputs and `ready` are unconstrained.
use crate::clocks::ClkRx;
use crate::stream::FrameByte;
use txhdl::comp::{Clock, In, Out, Reg, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{check, cover, lower, with, Trace};

/// Longest frame accepted, FCS included.
pub const MAX_RX_FRAME: usize = 1518;

/// Generic in its clock for the pin-level tests (real 25 MHz ratio);
/// [`MiiRx`] is the design's instance.
#[derive(Trace)]
pub struct MiiRxG<C: Clock> {
    /// Frame body in progress (after the SFD).
    pub body: Reg<Bit, C>,
    /// Waiting for `rx_dv` to fall (frame aborted or dropped).
    pub skip: Reg<Bit, C>,
    /// The low nibble of the byte in progress is in `lo`.
    pub half: Reg<Bit, C>,
    pub lo: Reg<U<4>, C>,
    /// `rx_er` seen in this frame.
    pub er_seen: Reg<Bit, C>,
    /// A byte is held back in `hold`.
    pub have: Reg<Bit, C>,
    pub hold: Reg<U<8>, C>,
    /// Bytes of the current frame completed so far (sent + held).
    pub cnt: Reg<U<11>, C>,
    /// The held byte must be sent as `last` when the channel has room.
    pub pend: Reg<Bit, C>,
    pub pend_err: Reg<Bit, C>,
    pub sfd_r: Reg<Bit, C>,
    // Monitors (only read by check!/cover!; pruned in synthesis).
    /// Bytes sent so far in the current frame.
    pub m_n: Reg<U<11>, C>,
    /// An SFD was seen for the frame being sent.
    pub m_in: Reg<Bit, C>,
    /// `sfd` was high in the previous cycle.
    pub m_sfd: Reg<Bit, C>,
    /// `body` in the previous cycle.
    pub m_body: Reg<Bit, C>,
}

/// The design's instance, at the board's `eth_rx_clk`.
pub type MiiRx = MiiRxG<ClkRx>;

impl<C: Clock> Default for MiiRxG<C> {
    fn default() -> Self {
        MiiRxG {
            body: Default::default(),
            skip: Default::default(),
            half: Default::default(),
            lo: Default::default(),
            er_seen: Default::default(),
            have: Default::default(),
            hold: Default::default(),
            cnt: Default::default(),
            pend: Default::default(),
            pend_err: Default::default(),
            sfd_r: Default::default(),
            m_n: Default::default(),
            m_in: Default::default(),
            m_sfd: Default::default(),
            m_body: Default::default(),
        }
    }
}

#[lower]
impl<C: Clock>
    Unit<
        (In<U<4>, C>, In<Bit, C>, In<Bit, C>),
        (Tx<FrameByte, C>, Out<Bit, C>),
    > for MiiRxG<C>
{
    async fn run(
        &mut self,
        (rxd, rx_dv, rx_er): (In<U<4>, C>, In<Bit, C>, In<Bit, C>),
        (frames, sfd): (Tx<FrameByte, C>, Out<Bit, C>),
    ) {
        loop {
            C::rising().await;
            let d = rxd.get();
            let dv = rx_dv.get();
            let er = rx_er.get();
            let body = self.body.get();
            let skip = self.skip.get();
            let half = self.half.get();
            let have = self.have.get();
            let pend = self.pend.get();
            let cnt = self.cnt.get();
            let er_seen = self.er_seen.get();
            let room = frames.ready();
            let hunt = !body & !skip;

            // Start of frame.
            let is_sfd = Bit::from(d == 0xd);
            let is_pre = Bit::from(d == 0x5);
            let sfd_now = hunt & dv & is_sfd & !pend;
            let bad_now = hunt & dv & !is_pre & !sfd_now;

            // Body.
            let nib = body & dv;
            let complete = nib & half;
            let send_prev = complete & have;
            let toolong = send_prev & Bit::from(cnt == 1518);
            let stop = send_prev & (toolong | !room);
            let fin = body & !dv;
            let tail = fin & have;
            let flush = pend & room;

            // One send site: held byte, as non-last, final or cut.
            let go = (send_prev & room) | (tail & room) | flush;
            let last = toolong | fin | flush;
            let errb = toolong
                | (fin & (er_seen | half))
                | (flush & self.pend_err.get());
            let sent_last = go & last;
            let hold_out = self.hold.get();

            check!(!pend | (have & !body), "a pending byte is held and not in a body");
            check!(!half | body, "a half byte only inside a body");
            check!(!body | !skip, "body and skip are exclusive");
            check!(body | pend | !have, "a held byte belongs to a body or is pending");
            check!(!(body | pend) | self.m_in.get(), "a body or a pending byte follows an SFD");
            check!(cnt <= 1518, "a frame has at most 1518 bytes (IEEE 802.3 4.4.2.3)");
            check!(
                self.m_n.get() + have.zext::<11>() == cnt,
                "bytes counted are the bytes sent plus the held one"
            );
            check!(have | (cnt == 0), "no held byte means no byte of the frame yet");
            check!(have | (self.m_n.get() == 0), "no held byte means nothing sent yet");
            check!(
                !self.sfd_r.get() | (body & !have),
                "the SFD pulse is followed by a fresh body"
            );
            check!(
                !self.sfd_r.get() | !self.m_body.get(),
                "an SFD pulse follows a cycle outside any body"
            );
            check!(
                !self.m_sfd.get() | self.m_body.get(),
                "the cycle after an SFD pulse follows a body"
            );
            check!(
                !self.sfd_r.get() | !self.m_sfd.get(),
                "the sfd pulse is one cycle wide"
            );
            if go.to_bool() {
                check!(self.m_in.get(), "no byte is sent before an SFD");
                check!(
                    self.m_n.get() + 1 <= 1518,
                    "a frame has at most 1518 bytes (IEEE 802.3 4.4.2.3)"
                );
                check!(!errb | last, "err only together with last (stream contract 2)");
                check!(!last | have, "last follows at least one byte (stream contract 1)");
                check!(
                    !(fin & (er_seen | half)) | errb,
                    "rx_er or a leftover nibble marks the frame bad (stream contract 4)"
                );
            }
            cover!(sent_last & !errb, "a frame is accepted and sent");
            cover!(
                sent_last & errb & er_seen & !toolong & !flush,
                "a frame with rx_er is sent in error"
            );
            cover!(stop & !room, "an overrun cuts a frame");
            // The 1518-byte cut needs ~3000 cycles to reach: covered by the
            // simulation test `too_long_frame_is_cut_at_1518`, not here.
            cover!(sfd_now, "an SFD is found");
            cover!(bad_now, "a corrupted SFD drops the frame");

            if go.to_bool() {
                frames.send(FrameByte {
                    data: hold_out,
                    last,
                    err: errb,
                });
            }

            with!(self <= {
                Bit::One ? sfd_r: sfd_now,
                Bit::One ? m_sfd: self.sfd_r.get(),
                Bit::One ? m_body: body,
                sfd_now ? { body: Bit::One, m_in: Bit::One },
                bad_now ? skip: Bit::One,
                skip & !dv ? skip: Bit::Zero,
                // Nibbles.
                nib & !half ? lo: d,
                nib & !stop ? half: !half,
                nib & er ? er_seen: Bit::One,
                complete & !stop ? {
                    have: Bit::One,
                    hold: d.concat::<4, 8>(self.lo.get()),
                    cnt: cnt + 1,
                },
                send_prev & room & !toolong ? m_n: self.m_n.get() + 1,
                // End of the body: normal end, or cut.
                stop | fin ? {
                    body: Bit::Zero,
                    half: Bit::Zero,
                    er_seen: Bit::Zero,
                },
                stop ? skip: Bit::One,
                (stop & !room) | (tail & !room) ? {
                    pend: Bit::One,
                    pend_err: stop | (er_seen | half),
                },
                // A byte sent as last ends the frame.
                sent_last ? {
                    have: Bit::Zero,
                    cnt: U::<11>::from(0u8),
                    m_n: U::<11>::from(0u8),
                    m_in: Bit::Zero,
                    pend: Bit::Zero,
                },
                fin & !have ? m_in: Bit::Zero,
            });
            sfd.set(self.sfd_r.get());
        }
    }
}
