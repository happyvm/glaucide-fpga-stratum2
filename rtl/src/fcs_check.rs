// SPDX-License-Identifier: Apache-2.0
//! FCS check and strip (task T1.12), `DefaultClock` domain (the crossing
//! from `ClkRx` is T1.4's job).
//!
//! Ports: `raw` (`Rx<FrameByte>`, raw frame, FCS included, as `mii_rx`
//! emits it), `pick` (formal witness wire) and `stamps_in` (`Rx<Ts>`, the
//! `stamps` of `rx_path`) in; `frames` (`Tx<FrameByte>`, same frame without
//! its 4 FCS bytes) and `stamps_out` (`Tx<Ts>`) out.
//!
//! # Receive timestamp (T1.13)
//!
//! One stamp is popped from `stamps_in` per raw frame, when its first byte
//! is taken, and held in `ts`. `rx_path` delivers a frame's stamp no later
//! than its first byte, so in the normal case this never stalls; if the
//! stamp is late the first byte simply waits (input stalls) until it
//! arrives, and if it never comes the unit waits forever. The stamp is
//! forwarded on `stamps_out` in the cycle the first byte of the output
//! frame is emitted, if and only if the frame emits anything (raw length
//! >= 5); a runt frame drops its stamp with it. That cycle also needs
//! `stamps_out.ready()`: otherwise the input stalls. Order is preserved:
//! one stamp out per output frame, in frame order.
//!
//! # Behaviour
//!
//! * The stream is delayed by 4 bytes (a shift register `d0..d3`), so the
//!   last data byte leaves when the last FCS byte arrives. The CRC register
//!   runs over **all** bytes, FCS included, with `crc32_step`, from
//!   [`INIT`]. At the input `last` the FCS is good iff the register equals
//!   [`RESIDUE`] (pre-complement, see `crc32.rs`).
//! * Output: every byte but the final 4, in order, `last` on the final data
//!   byte, `err` = input `err` OR bad FCS. `err` is never set on a non-last
//!   byte. When the input `err` is set the CRC is irrelevant: the frame is
//!   already flagged and the output `last` byte carries `err`.
//! * A raw frame of fewer than 5 bytes leaves nothing after dropping the
//!   FCS: it emits **nothing** (not even an `err`). A raw frame of exactly 5
//!   bytes emits 1 byte. Output length is always input length minus 4, or 0.
//! * After each `last` the delay line and the CRC are reset: back-to-back
//!   frames are independent.
//!
//! # What the unit needs on each side
//!
//! * Input: the T0.1 contract, in particular at most 1518 raw bytes per
//!   frame (stated as `assume!`). A byte is taken when the delay line is
//!   still filling or when `frames` has room (no output buffer), so the
//!   unit stalls its input (`ready` low)
//!   while the output is stalled, so the producer must be able to wait:
//!   `mii_rx` must be fed through a FIFO that absorbs a whole frame, which
//!   T1.4's CDC provides. The unit is never wired port to port to another
//!   module: join it through a registered channel or FIFO.
//! * Output: any consumer; `ready` may stay low as long as it likes.
//!
//! # Formal
//!
//! `check!`/`cover!` in `run` state the CRC/FCS row of `docs/formal.md`;
//! they are proved with `tools/formal/prove.sh fcs_check prove`. The only
//! `assume!` is the frame length limit of the stream contract. Monitors
//! (`m_*`) only feed `check!`/`cover!` and are pruned in synthesis.
use crate::crc32::{crc32_step, INIT, RESIDUE};
use crate::stream::{FrameByte, Ts};
use txhdl::comp::{mux, Clock, DefaultClock, In, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{assume, check, cover, lower, with, Trace};

#[derive(Trace)]
pub struct FcsCheck {
    /// Delay line, `d0` newest, `d3` oldest.
    pub d0: Reg<U<8>>,
    pub d1: Reg<U<8>>,
    pub d2: Reg<U<8>>,
    pub d3: Reg<U<8>>,
    /// Bytes in the delay line, saturating at 4.
    pub fill: Reg<U<3>>,
    /// CRC register over the bytes of the frame so far (pre-complement).
    pub crc: Reg<U<32>>,
    /// Stamp of the frame in progress (popped with its first byte).
    pub ts: Reg<Ts>,
    // Monitors.
    /// Raw bytes taken in the current frame.
    pub m_in: Reg<U<11>>,
    /// Bytes emitted in the current frame.
    pub m_out: Reg<U<11>>,
    /// First byte of the current frame.
    pub m_first: Reg<U<8>>,
    // Symbolic-position witness (formal only, see `pick` below).
    /// The chosen position `p`, frozen after the first cycle.
    pub g_p: Reg<U<11>>,
    /// `g_p` has been loaded.
    pub g_set: Reg<Bit>,
    /// The raw byte at position `p` of the current frame.
    pub g_cap: Reg<U<8>>,
}

impl Default for FcsCheck {
    fn default() -> Self {
        FcsCheck {
            d0: Reg::default(),
            d1: Reg::default(),
            d2: Reg::default(),
            d3: Reg::default(),
            fill: Reg::default(),
            crc: Reg::new(INIT),
            ts: Reg::default(),
            m_in: Reg::default(),
            m_out: Reg::default(),
            m_first: Reg::default(),
            g_p: Reg::default(),
            g_set: Reg::default(),
            g_cap: Reg::default(),
        }
    }
}

#[lower]
impl Unit<(Rx<FrameByte>, In<U<11>>, Rx<Ts>), (Tx<FrameByte>, Tx<Ts>)> for FcsCheck {
    async fn run(
        &mut self,
        (raw, pick, stamps_in): (Rx<FrameByte>, In<U<11>>, Rx<Ts>),
        (frames, stamps_out): (Tx<FrameByte>, Tx<Ts>),
    ) {
        loop {
            DefaultClock::rising().await;
            // Invariants, emitted in every cycle (the k-induction needs
            // them in states where no byte is taken too).
            let fill = self.fill.get();
            let m_in = self.m_in.get();
            let m_out = self.m_out.get();
            let full = Bit::from(fill == 4);
            check!(fill <= 4, "the delay line holds at most 4 bytes");
            check!(
                full | (fill.zext::<11>() == m_in),
                "fewer than 4 bytes taken: the delay line holds them all"
            );
            check!(
                !full | (m_in >= 4),
                "a full delay line means at least 4 bytes taken"
            );
            check!(
                !full | (m_out + 4 == m_in),
                "output length equals input length minus 4"
            );
            check!(
                full | (m_out == 0),
                "nothing is emitted before 5 bytes are taken"
            );
            check!(m_in <= 1518, "a frame has at most 1518 raw bytes");
            check!(
                (m_in != 0) | (self.crc.get() == U::<32>::from(INIT)),
                "the CRC register is at INIT at the start of each frame"
            );
            check!(
                (fill != 1) | (self.d0.get() == self.m_first.get()),
                "delay line: the first byte of the frame sits in d0"
            );
            check!(
                (fill != 2) | (self.d1.get() == self.m_first.get()),
                "delay line: the first byte of the frame sits in d1"
            );
            check!(
                (fill != 3) | (self.d2.get() == self.m_first.get()),
                "delay line: the first byte of the frame sits in d2"
            );
            check!(
                !full | (m_out != 0) | (self.d3.get() == self.m_first.get()),
                "delay line: the first byte of the frame sits in d3 until emitted"
            );

            // Symbolic position (formal witness, written by the orchestrator,
            // task F1.2): `pick` is a free input; the first value it has is
            // frozen into `g_p`, so a proof about `g_p` holds for EVERY
            // position of a frame. In the real design `pick` is tied to 0 and
            // all of this is pruned. Claim: the byte emitted at output index
            // `p` is the byte taken at input index `p` (the delay line only
            // delays, it never reorders, drops or alters a data byte).
            let gset = self.g_set.get();
            let p = mux(gset, self.g_p.get(), pick.get());
            let cap = self.g_cap.get();
            let m_in12 = m_in.zext::<12>();
            let p12 = p.zext::<12>();
            check!(
                (m_in12 != p12 + 1) | (self.d0.get() == cap),
                "delay line: the byte taken at position p sits in d0 right after"
            );
            check!(
                (m_in12 != p12 + 2) | (self.d1.get() == cap),
                "delay line: the byte taken at position p sits in d1 one byte later"
            );
            check!(
                (m_in12 != p12 + 3) | (self.d2.get() == cap),
                "delay line: the byte taken at position p sits in d2 two bytes later"
            );
            check!(
                (m_in12 != p12 + 4) | (self.d3.get() == cap),
                "delay line: the byte taken at position p sits in d3 three bytes later"
            );

            // A byte is taken when it can go on: always while the delay
            // line fills (nothing is emitted), else when `frames` has room.
            // The first byte of a frame also needs its stamp; the first
            // output byte also needs room on `stamps_out`.
            let first = Bit::from(m_in == 0);
            let st_ok = !first | Bit::from(stamps_in.peek().is_some());
            let first_out = full & Bit::from(m_out == 0);
            let take = (!full | frames.ready()) & st_ok & (!first_out | stamps_out.ready());
            let r = raw.recv_if(take);
            let got = Bit::from(r.is_some());
            let new_ts = stamps_in.recv_if(got & first).unwrap_or_default();
            let b = r.unwrap_or_default();
            assume!(
                !got | (m_in < 1518) | b.last,
                "a frame has at most 1518 bytes (stream contract 3)"
            );
            let next = crc32_step(self.crc.get(), b.data);
            let good = Bit::from(next == U::<32>::from(RESIDUE));
            let bad = b.err | !good;
            let oerr = b.last & bad;
            let emit = got & full;
            check!(
                !oerr | b.last,
                "err only together with last (stream contract 2)"
            );
            check!(
                !(got & b.last & !full) | (m_in < 4),
                "a frame under 5 bytes emits nothing"
            );
            check!(
                !(emit & b.last) | (m_out + 1 + 4 == m_in + 1),
                "the emitted frame is 4 bytes shorter than the raw frame"
            );
            cover!(emit & b.last & !bad, "a good frame is passed");
            cover!(emit & b.last & !b.err & !good, "a bad FCS flags the frame");
            cover!(emit & b.last & b.err, "an input err flags the frame");
            cover!(got & b.last & !full, "a runt frame emits nothing");

            check!(
                !(emit & (m_out.zext::<12>() == p12)) | (self.d3.get() == cap),
                "the byte emitted at output position p is the byte taken at input position p"
            );
            cover!(emit & (m_out.zext::<12>() == p12) & (p != 0), "position p is emitted (p not 0)");
            if (emit & first_out).to_bool() {
                stamps_out.send(self.ts.get());
            }
            if emit.to_bool() {
                frames.send(FrameByte {
                    data: self.d3.get(),
                    last: b.last,
                    err: oerr,
                });
            }
            with!(self <= {
                got ? {
                    d0: b.data,
                    d1: self.d0.get(),
                    d2: self.d1.get(),
                    d3: self.d2.get(),
                    crc: next,
                    m_in: m_in + 1,
                },
                got & !full ? fill: fill + 1,
                got & (m_in == 0) ? {
                    m_first: b.data,
                    ts: new_ts,
                },
                emit ? m_out: m_out + 1,
                !gset ? g_p: pick.get(),
                !gset ? g_set: Bit::One,
                got & (m_in12 == p12) ? g_cap: b.data,
                got & b.last ? {
                    crc: INIT,
                    fill: U::<3>::from(0u8),
                    m_in: U::<11>::from(0u8),
                    m_out: U::<11>::from(0u8),
                },
            });
        }
    }
}
