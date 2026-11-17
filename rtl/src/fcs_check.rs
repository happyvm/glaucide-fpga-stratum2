// SPDX-License-Identifier: Apache-2.0
//! FCS check and strip (task T1.12), `DefaultClock` domain (the crossing
//! from `ClkRx` is T1.4's job).
//!
//! Ports: `raw` (`Rx<FrameByte>`, raw frame, FCS included, as `mii_rx`
//! emits it) in; `frames` (`Tx<FrameByte>`, same frame without its 4 FCS
//! bytes) out.
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
use crate::stream::FrameByte;
use txhdl::comp::{Clock, DefaultClock, Reg, Rx, Tx, Unit};
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
    // Monitors.
    /// Raw bytes taken in the current frame.
    pub m_in: Reg<U<11>>,
    /// Bytes emitted in the current frame.
    pub m_out: Reg<U<11>>,
    /// First byte of the current frame.
    pub m_first: Reg<U<8>>,
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
            m_in: Reg::default(),
            m_out: Reg::default(),
            m_first: Reg::default(),
        }
    }
}

#[lower]
impl Unit<Rx<FrameByte>, Tx<FrameByte>> for FcsCheck {
    async fn run(&mut self, raw: Rx<FrameByte>, frames: Tx<FrameByte>) {
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

            // A byte is taken when it can go on: always while the delay
            // line fills (nothing is emitted), else when `frames` has room.
            let take = !full | frames.ready();
            let r = raw.recv_if(take);
            let got = Bit::from(r.is_some());
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
                got & (m_in == 0) ? m_first: b.data,
                emit ? m_out: m_out + 1,
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
