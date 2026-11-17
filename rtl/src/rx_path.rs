// SPDX-License-Identifier: Apache-2.0
//! Receive path crossing (task T1.4a): `mii_rx` frames from `ClkRx` to
//! `clk_sys`, and one receive timestamp per frame, taken at the SFD.
//!
//! Ports of [`RxPath`] ([`RxPathMii`] in the design):
//! * `ClkRx` side: `raw` (`Rx<FrameByte>`, the raw frames of `mii_rx`, FCS
//!   included; always taken, `ready` is never low while `valid` is high)
//!   and `sfd` (`In<Bit>`, `mii_rx`'s one-cycle SFD pulse).
//! * `clk_sys` side: `time` (`In<U<96>>`, the `NtpClock` accumulator; the
//!   stamp uses `time >> 32`), `frames` (`Tx<FrameByte>`, the same raw
//!   frames, FCS included) and `stamps` (`Tx<Ts>`, one per frame).
//!
//! # Structure
//!
//! Five parts, every crossing a `ChanCdc` (Gray pointers, two flops):
//!
//! ```text
//!  ClkRx                                     |  clk_sys
//!  raw, sfd -> RxIngress -- words ---> DataCdc (2048) --> RxEgress -> frames
//!                        -- events --> EventCdc (4) ----> (stamp,   -> stamps
//!                        <- credits -- CreditCdc (4) <---  pairing)  <- time
//! ```
//!
//! [`RxIngress`] and [`RxEgress`] are single-clock units; they carry the
//! formal properties (below).
//!
//! # Frames, overflow
//!
//! Every input byte is taken in the cycle it is offered (the wire cannot
//! wait). The data FIFO holds 2048 words, more than a whole 1518-byte
//! frame. A frame goes into it only if its first byte finds room in the
//! FIFO, a slot in the event channel and a stamp credit (below); otherwise
//! the whole frame is dropped, nothing of it is written. If the FIFO is
//! full in the middle of a frame (the `clk_sys` consumer has stalled for
//! about 2048 bytes), the frame is cut: the bytes already written stay,
//! the rest of the input frame is dropped, and a terminator word
//! `{data: 0, last: 1, err: 1}` is written as soon as there is room, so
//! the frame ends with `err` (stream contract 2: the consumer discards
//! it). A new frame whose SFD comes while the terminator is still owed is
//! dropped whole. Data is never altered without `err` on the frame's last
//! byte. An SFD while an output frame is still open (which `mii_rx` never
//! does) closes it the same way.
//!
//! Output, stream contract of `stream.rs`: a frame is at least one byte,
//! at most 1518 (the input's limit, the terminator replacing a dropped
//! byte), `err` only with `last`.
//!
//! # Timestamp
//!
//! The SFD pulse crosses as an event on its own small `ChanCdc` (its Gray
//! write pointer is the toggle synchroniser of the event; four slots, so
//! events closer than the synchroniser latency are not merged, and its
//! read side is always ready, so an event never waits). [`RxEgress`] reads
//! `time` in the cycle the event comes out and keeps
//! `(time >> 32) - RX_STAMP_BIAS`.
//!
//! Latency, edge by edge (netlist; the Rust simulation is the same):
//! * `clk_rx` edge `k`: `mii_rx` samples the SFD nibble (the reference
//!   instant of the stamp) and raises its `sfd` register;
//! * `k + 1`: [`RxIngress`] sees `sfd`, offers the SFD event;
//! * `k + 2`: the event `ChanCdc` writes it and moves its Gray pointer;
//! * first `clk_sys` edge strictly after that, `e1` (`phi` in (0, 1]
//!   `clk_sys` cycle later): first synchroniser flop; `e1 + 1`: second;
//!   `e1 + 2`: popped into the channel to [`RxEgress`]; `e1 + 3`: `time`
//!   captured.
//!
//! Total `2 T_rx + (2 + phi + 1) T_sys` = 80 ns + 30 ns + `phi` x 10 ns,
//! from 110 to 120 ns. [`RX_STAMP_BIAS`] removes the mean, 115 ns
//! ([`RX_STAMP_LATENCY_PS`], 494 units of 2^-32 s), so that the stamp
//! estimates the time at which the SFD nibble was sampled on the MII.
//!
//! Uncertainty budget (fixed, documented; the NTP precision field relies
//! on it):
//! * asynchronous crossing: the phase `phi` of the two clocks gives an
//!   error in (-5, +5] ns, uniform; a metastable first flop can resolve
//!   one `clk_sys` cycle late only when the pointer changes inside its
//!   aperture, which keeps the error within **±1 `clk_sys` cycle = ±10
//!   ns**. Measured in simulation (`rtl/tests/rx_path.rs`): -4.66 to +5.12
//!   ns over six phases at a 40/10 ns ratio, -3.49 to +6.05 ns over all
//!   phases with a 40.5 ns `clk_rx` (the extra ns is 2 x 0.5 ns of
//!   off-nominal `clk_rx`, see below);
//! * MII granularity: the SFD is located to one `clk_rx` cycle, **40 ns**;
//! * `clk_rx` frequency: the 2 `clk_rx` cycles of the latency are 80 ns at
//!   the nominal 25 MHz; the PHY's clock is within ±100 ppm: ±8 ps;
//! * `time` resolution: 10 ns steps of `NtpClock`, inside the ±10 ns above;
//!   2^-32 s rounding of the bias: 0.02 ns.
//!
//! Total: |error| <= 10 + 40 = 50 ns < 2^-24 s (59.6 ns), hence precision
//! -24. Not included: the PHY's own receive latency (DP83848J, constant,
//! to be measured on the board and added to the bias later).
//!
//! # Pairing
//!
//! Invariant: the n-th stamp delivered on `stamps` belongs to the n-th
//! frame delivered on `frames`, and is offered no later than its first
//! byte. Mechanism:
//! * two kinds of events go through the event channel, in order: `SFD`
//!   when the SFD is seen, `COMMIT` when the frame's first byte is written
//!   into the data FIFO. A `COMMIT` is sent only for the frame of the last
//!   `SFD` event (no other `SFD` event in between) and in the same cycle
//!   as the frame's first word;
//! * [`RxEgress`] stamps every `SFD` event into `latest`, and pushes
//!   `latest` into its stamp queue on `COMMIT`. An SFD whose frame never
//!   writes a byte (0 complete bytes in `mii_rx`, or dropped here) has no
//!   `COMMIT`: its stamp is overwritten by the next SFD (orphan
//!   discarded). The stamp queue therefore holds exactly the stamps of the
//!   frames in the data FIFO, in their order;
//! * a frame at the head of the FIFO first takes the oldest stamp (it
//!   waits if the `COMMIT` has not arrived yet), sends it on `stamps`, then
//!   its bytes on `frames`;
//! * credits: the stamp queue has [`STAMP_DEPTH`] slots. [`RxIngress`]
//!   holds a credit per free slot, spends one per `COMMIT` and gets one
//!   back through the credit channel each time a stamp leaves the queue.
//!   A frame with no credit is dropped whole, so the queue never
//!   overflows: at most 16 frames wait for a stalled consumer, the next
//!   ones are dropped until it resumes.
//!
//! Throughput: one byte per 2 `clk_rx` cycles in, up to one per `clk_sys`
//! cycle out plus one cycle per frame for its stamp; frames back to back
//! at the minimum gap (96 bit times + preamble) pass without loss when the
//! consumer is ready.
//!
//! # Netlist
//!
//! Join `raw`, `frames` and `stamps` to their neighbours through
//! registered channels (note in `stream.rs`). One `rst` serves both
//! domains: it must be released synchronously to each clock (the top's
//! reset synchronisers, T1.11). The data FIFO is read asynchronously by
//! `ChanCdc` (distributed RAM, 2048 x 10 bits, not block RAM).
//!
//! # Formal
//!
//! `check!`/`assume!`/`cover!` in [`RxIngress`] and [`RxEgress`] are
//! proved per unit (`tools/formal/prove.sh rx_ingress prove`, `rx_egress`):
//! state invariants, overflow and terminator, `err` only with `last`,
//! frame length, commit only after its own SFD event, credits within the
//! depth, pairing counters (stamps sent = frames started + the one waiting
//! for its first byte), stamp queue pointers, stamp = `time >> 32` minus
//! the bias. The two-clock whole is not proved: the `ChanCdc`s (in-order,
//! lossless transfer) and the link between the two units' properties
//! (each `assume!` of [`RxEgress`] about events is a `check!` of the same
//! text in [`RxIngress`], carried by the event channel) are covered by the
//! simulation tests, where every `check!`/`assume!` is also evaluated each
//! cycle, as is the timestamp latency.
use crate::clocks::ClkRx;
use crate::stream::{FrameByte, Ts};
use txhdl::comp::{chan, join2, mux, Clock, DefaultClock, In, Mem, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{assume, check, cover, lower, with, Trace};
use txhdl_parts::cdc::ChanCdc;

/// Stamps that can wait in `clk_sys` for their frame (credits).
pub const STAMP_DEPTH: usize = 16;
/// Nominal `clk_rx` period (100 Mb/s MII), picoseconds.
pub const CLK_RX_PS: u64 = 40_000;
/// `clk_sys` period, picoseconds.
pub const CLK_SYS_PS: u64 = 10_000;
/// Nominal latency from the edge of `clk_rx` that samples the SFD nibble
/// to the edge of `clk_sys` that captures `time`: 2 `clk_rx` cycles, then
/// 3.5 `clk_sys` cycles on average (see the module documentation).
pub const RX_STAMP_LATENCY_PS: u64 = 2 * CLK_RX_PS + 7 * CLK_SYS_PS / 2;
/// [`RX_STAMP_LATENCY_PS`] in units of 2^-32 s, rounded: 494 (115 ns).
pub const RX_STAMP_BIAS: u64 =
    ((RX_STAMP_LATENCY_PS as u128 * (1u128 << 32) + 500_000_000_000) / 1_000_000_000_000) as u64;

/// Event on the `ClkRx` -> `clk_sys` event channel: an SFD was seen.
pub const EV_SFD: u8 = 0;
/// Event: the frame of the last SFD has its first byte in the data FIFO.
pub const EV_COMMIT: u8 = 1;

// ---------------------------------------------------------------------
// ClkRx side
// ---------------------------------------------------------------------

/// `ClkRx` half: overflow handling, SFD and commit events, credits.
#[derive(Trace)]
pub struct RxIngress<R: Clock> {
    /// SFD event sent, waiting for the frame's first byte.
    pub armed: Reg<Bit, R>,
    /// Frame committed, bytes forwarded to the data FIFO.
    pub run: Reg<Bit, R>,
    /// Frame dropped, bytes discarded until `last` or the next SFD.
    pub skip: Reg<Bit, R>,
    /// A cut frame still owes its `last & err` terminator word.
    pub pend: Reg<Bit, R>,
    /// Free stamp slots in `clk_sys`.
    pub cred: Reg<U<5>, R>,
    // Monitors (only read by check!/cover!; pruned in synthesis).
    /// Input bytes of the current frame (since its SFD or the last `last`).
    pub m_in: Reg<U<11>, R>,
    /// Words written to the data FIFO for the open output frame.
    pub m_n: Reg<U<11>, R>,
    /// The current frame (that of the last `sfd` pulse) has had its SFD
    /// event sent and not yet its commit.
    pub m_sfd: Reg<Bit, R>,
}

impl<R: Clock> Default for RxIngress<R> {
    fn default() -> Self {
        RxIngress {
            armed: Reg::default(),
            run: Reg::default(),
            skip: Reg::default(),
            pend: Reg::default(),
            cred: Reg::new(U::<5>::from(STAMP_DEPTH as u8)),
            m_in: Reg::default(),
            m_n: Reg::default(),
            m_sfd: Reg::default(),
        }
    }
}

#[lower]
impl<R: Clock> Unit<(Rx<FrameByte, R>, In<Bit, R>, Rx<U<1>, R>), (Tx<FrameByte, R>, Tx<U<1>, R>)>
    for RxIngress<R>
{
    async fn run(
        &mut self,
        (raw, sfd, tokens): (Rx<FrameByte, R>, In<Bit, R>, Rx<U<1>, R>),
        (words, events): (Tx<FrameByte, R>, Tx<U<1>, R>),
    ) {
        loop {
            R::rising().await;
            let armed = self.armed.get();
            let run = self.run.get();
            let skip = self.skip.get();
            let pend = self.pend.get();
            let cred = self.cred.get();
            let m_in = self.m_in.get();
            let m_n = self.m_n.get();
            let m_sfd = self.m_sfd.get();
            let wroom = words.ready();
            let eroom = events.ready();

            check!(!(armed & run), "armed and run are exclusive");
            check!(!(armed & skip), "armed and skip are exclusive");
            check!(!(run & skip), "run and skip are exclusive");
            check!(
                !pend | !(run | armed),
                "a pending terminator blocks the next frame"
            );
            check!(
                (run | pend) == Bit::from(m_n != 0),
                "an output frame is open exactly while running or owing its terminator"
            );
            check!(
                !run | (m_n == m_in),
                "a running frame forwards every input byte"
            );
            check!(m_n <= 1517, "an open output frame has at most 1517 words");
            check!(!armed | (m_in == 0), "an armed frame has no byte yet");
            check!(!armed | m_sfd, "an armed frame has its own SFD event sent");
            check!(
                cred <= STAMP_DEPTH as u8,
                "credits never exceed the stamp queue depth"
            );

            // Input: always taken, the wire cannot wait.
            let r = raw.recv_if(Bit::One);
            let got = Bit::from(r.is_some());
            let b = r.unwrap_or_default();
            assume!(
                !got | (m_in < 1517) | b.last,
                "a frame has at most 1518 bytes (IEEE 802.3 4.4.2.3)"
            );
            assume!(
                !got | !b.err | b.last,
                "err only together with last (stream contract 2)"
            );
            let s = sfd.get();
            let (tok_b, _tok) = tokens.take();
            let tok = Bit::from(tok_b);
            assume!(
                !tok | (cred != STAMP_DEPTH as u8),
                "a credit token returns a stamp slot taken by a commit"
            );

            // The byte, for the frame in progress.
            let first = got & armed;
            let commit = first & wroom & eroom & (cred != 0);
            let mid = got & run;
            let mid_ok = mid & wroom;
            let cut = mid & !wroom;
            let term = pend & wroom;
            let write = commit | mid_ok | term;
            let wlast = mux(term, Bit::One, b.last);
            let werr = mux(term, Bit::One, b.err);
            let open_after = (run & !(mid_ok & b.last) & !cut) | (commit & !b.last);
            // An SFD while an output frame is open (never from `mii_rx`):
            // the open frame is closed by a terminator.
            let abort = s & open_after;
            let pend_n = (pend & !term) | cut | abort;
            // The SFD of a new frame: its event goes out now, or the frame
            // is dropped whole.
            let sfd_ev = s & eroom & !commit & !pend_n;

            if write.to_bool() {
                check!(
                    !werr | wlast,
                    "err only together with last (stream contract 2)"
                );
                check!(
                    m_n + 1 <= 1518,
                    "a frame has at most 1518 bytes (IEEE 802.3 4.4.2.3)"
                );
                check!(
                    (m_n != 0) | commit,
                    "an output frame starts only with a commit event"
                );
                words.send(FrameByte {
                    data: mux(term, U::<8>::from(0u8), b.data),
                    last: wlast,
                    err: werr,
                });
            }
            if commit.to_bool() {
                check!(m_n == 0, "a commit starts a new output frame");
                check!(m_sfd, "a commit follows its own SFD event");
                check!(
                    cred != 0,
                    "credit flow: a commit always has a free stamp slot"
                );
            }
            if (commit | sfd_ev).to_bool() {
                events.send(mux(commit, U::<1>::from(EV_COMMIT), U::<1>::from(EV_SFD)));
            }
            cover!(commit & !b.last, "a frame is committed");
            cover!(commit & b.last, "a one-byte frame is committed");
            cover!(mid_ok & b.last & !b.err, "a frame is forwarded whole");
            cover!(cut, "an overflow cuts a frame");
            cover!(term, "a cut frame is closed by its terminator");
            cover!(s & armed, "an SFD without bytes leaves an orphan stamp");
            cover!(first & !commit, "a frame is dropped at its first byte");
            cover!(s & !sfd_ev, "a frame is dropped at its SFD");
            cover!(abort, "an SFD inside an open frame closes it");
            cover!(tok, "a credit comes back");

            let armed_n = mux(s, sfd_ev, armed & !got);
            let run_n = !s & ((run & !(got & (b.last | !wroom))) | (commit & !b.last));
            let skip_n = mux(
                s,
                !sfd_ev,
                (skip & !(got & b.last)) | (first & !commit & !b.last) | (cut & !b.last),
            );
            let cred_c = mux(commit, cred - 1, cred);
            let cred_n = mux(tok, cred_c + 1, cred_c);
            let m_in_n = mux(
                s | (got & b.last),
                U::<11>::from(0u8),
                mux(got, m_in + 1, m_in),
            );
            let m_n_n = mux(write & wlast, U::<11>::from(0u8), mux(write, m_n + 1, m_n));
            let m_sfd_n = mux(sfd_ev, Bit::One, mux(s | commit, Bit::Zero, m_sfd));
            with!(self <= {
                armed: armed_n,
                run: run_n,
                skip: skip_n,
                pend: pend_n,
                cred: cred_n,
                m_in: m_in_n,
                m_n: m_n_n,
                m_sfd: m_sfd_n,
            });
        }
    }
}

// ---------------------------------------------------------------------
// clk_sys side
// ---------------------------------------------------------------------

/// `clk_sys` half: stamp capture, stamp queue, pairing, credit return.
#[derive(Trace)]
pub struct RxEgress<S: Clock> {
    /// Stamp of the last SFD event, waiting for its commit.
    pub latest: Reg<U<64>, S>,
    pub lvalid: Reg<Bit, S>,
    /// Stamps of committed frames, oldest at `qh`.
    pub q: Mem<U<64>, STAMP_DEPTH, S>,
    pub qh: Reg<U<5>, S>,
    pub qt: Reg<U<5>, S>,
    /// The stamp of the frame at the head has been sent: bytes may pass.
    pub infr: Reg<Bit, S>,
    /// Credit tokens still to send back.
    pub owed: Reg<U<5>, S>,
    // Monitors.
    /// Stamps sent and frames started, modulo 16.
    pub m_st: Reg<U<4>, S>,
    pub m_fs: Reg<U<4>, S>,
    /// Bytes sent in the current output frame.
    pub m_n: Reg<U<11>, S>,
    /// `time >> 32` at the last SFD event.
    pub m_raw: Reg<U<64>, S>,
}

impl<S: Clock> Default for RxEgress<S> {
    fn default() -> Self {
        RxEgress {
            latest: Reg::default(),
            lvalid: Reg::default(),
            q: Mem::default(),
            qh: Reg::default(),
            qt: Reg::default(),
            infr: Reg::default(),
            owed: Reg::default(),
            m_st: Reg::default(),
            m_fs: Reg::default(),
            m_n: Reg::default(),
            m_raw: Reg::default(),
        }
    }
}

#[lower]
impl<S: Clock>
    Unit<(Rx<FrameByte, S>, Rx<U<1>, S>, In<U<96>, S>), (Tx<FrameByte, S>, Tx<Ts, S>, Tx<U<1>, S>)>
    for RxEgress<S>
{
    async fn run(
        &mut self,
        (words, events, time): (Rx<FrameByte, S>, Rx<U<1>, S>, In<U<96>, S>),
        (frames, stamps, tokens): (Tx<FrameByte, S>, Tx<Ts, S>, Tx<U<1>, S>),
    ) {
        loop {
            S::rising().await;
            let qh = self.qh.get();
            let qt = self.qt.get();
            let owed = self.owed.get();
            let infr = self.infr.get();
            let lvalid = self.lvalid.get();
            let m_st = self.m_st.get();
            let m_fs = self.m_fs.get();
            let m_n = self.m_n.get();
            let count = qt - qh;
            let empty = Bit::from(count == 0);
            let held = count.zext::<6>() + owed.zext::<6>();
            let fresh = infr & Bit::from(m_n == 0);

            check!(
                count <= STAMP_DEPTH as u8,
                "the stamp queue holds at most its depth"
            );
            check!(
                held <= STAMP_DEPTH as u8,
                "queued stamps plus owed credits never exceed the depth"
            );
            check!(
                m_st - m_fs == fresh.zext::<4>(),
                "pairing: the n-th stamp sent belongs to the n-th frame started"
            );
            check!(infr | (m_n == 0), "no byte passes before its frame's stamp");
            check!(m_n <= 1517, "an open output frame has at most 1517 bytes");
            check!(
                !lvalid | (self.latest.get() + RX_STAMP_BIAS == self.m_raw.get()),
                "the stamp is time >> 32 minus the documented bias"
            );

            // Events: always taken, so the capture instant is fixed.
            let (ge, ev) = events.take();
            let is_sfd = Bit::from(ge) & (ev == EV_SFD);
            let is_commit = Bit::from(ge) & (ev == EV_COMMIT);
            assume!(!is_commit | lvalid, "a commit follows its own SFD event");
            assume!(
                !is_commit | (held < STAMP_DEPTH as u8),
                "credit flow: a commit always has a free stamp slot"
            );
            let raw = time.get().slice::<32, 64>();
            let stamp = raw - RX_STAMP_BIAS;

            // Pairing: a frame at the head first takes the oldest stamp.
            let has = Bit::from(words.peek().is_some());
            let give = has & !infr & !empty & stamps.ready();
            let pass = infr & frames.ready();
            let w = words.recv_if(pass);
            let got = Bit::from(w.is_some());
            let b = w.unwrap_or_default();
            assume!(
                !got | !b.err | b.last,
                "err only together with last (stream contract 2)"
            );
            assume!(
                !got | (m_n < 1517) | b.last,
                "a frame has at most 1518 bytes (IEEE 802.3 4.4.2.3)"
            );
            if give.to_bool() {
                stamps.send(self.q.read(qh.slice::<0, 4>()));
            }
            if got.to_bool() {
                check!(
                    !b.err | b.last,
                    "err only together with last (stream contract 2)"
                );
                check!(
                    m_n + 1 <= 1518,
                    "a frame has at most 1518 bytes (IEEE 802.3 4.4.2.3)"
                );
                frames.send(b);
            }
            let tsend = Bit::from(owed != 0) & tokens.ready();
            if tsend.to_bool() {
                tokens.send(U::<1>::from(1u8));
            }
            cover!(give, "a stamp is delivered");
            cover!(got & b.last & !b.err, "a frame is delivered");
            cover!(has & !infr & empty, "a frame waits for its stamp");
            cover!(is_sfd & lvalid, "an orphan stamp is overwritten");
            cover!(is_commit & (count == 15), "the stamp queue fills");
            cover!(tsend, "a credit is returned");

            let push = is_commit;
            let owed_n = mux(give, owed + 1, owed);
            with!(self <= {
                is_sfd ? { latest: stamp, m_raw: raw, lvalid: Bit::One },
                is_commit ? lvalid: Bit::Zero,
                push ? q.at(qt.slice::<0, 4>()): self.latest.get(),
                push ? qt: qt + 1,
                give ? { qh: qh + 1, infr: Bit::One, m_st: m_st + 1 },
                got & (m_n == 0) ? m_fs: m_fs + 1,
                got ? m_n: m_n + 1,
                got & b.last ? { infr: Bit::Zero, m_n: U::<11>::from(0u8) },
                Bit::One ? owed: mux(tsend, owed_n - 1, owed_n),
            });
        }
    }
}

// ---------------------------------------------------------------------
// The whole path
// ---------------------------------------------------------------------

/// Data FIFO: 2048 words, more than a 1518-byte frame.
pub type DataCdc<R, S> = ChanCdc![
    Data = FrameByte,
    ADDR = 11,
    DEPTH = 2048,
    PTRS = 12,
    WriteClock = R,
    ReadClock = S,
];
/// SFD / commit events, `ClkRx` to `clk_sys`.
pub type EventCdc<R, S> = ChanCdc![
    Data = U<1>,
    ADDR = 2,
    DEPTH = 4,
    PTRS = 3,
    WriteClock = R,
    ReadClock = S,
];
/// Credit tokens, `clk_sys` back to `ClkRx`.
pub type CreditCdc<S, R> = ChanCdc![
    Data = U<1>,
    ADDR = 2,
    DEPTH = 4,
    PTRS = 3,
    WriteClock = S,
    ReadClock = R,
];

/// The receive path, generic in its two clocks for the tests;
/// [`RxPathMii`] is the design's instance.
#[derive(Trace)]
pub struct RxPath<R: Clock, S: Clock> {
    pub ingress: RxIngress<R>,
    pub data: DataCdc<R, S>,
    pub ev: EventCdc<R, S>,
    pub cred: CreditCdc<S, R>,
    pub egress: RxEgress<S>,
}

impl<R: Clock, S: Clock> Default for RxPath<R, S> {
    fn default() -> Self {
        RxPath {
            ingress: RxIngress::default(),
            data: Default::default(),
            ev: Default::default(),
            cred: Default::default(),
            egress: RxEgress::default(),
        }
    }
}

/// `ClkRx` to `clk_sys`, as in the design.
pub type RxPathMii = RxPath<ClkRx, DefaultClock>;

#[lower]
impl<R: Clock, S: Clock>
    Unit<(Rx<FrameByte, R>, In<Bit, R>, In<U<96>, S>), (Tx<FrameByte, S>, Tx<Ts, S>)>
    for RxPath<R, S>
{
    async fn run(
        &mut self,
        (raw, sfd, time): (Rx<FrameByte, R>, In<Bit, R>, In<U<96>, S>),
        (frames, stamps): (Tx<FrameByte, S>, Tx<Ts, S>),
    ) {
        let (w_tx, w_rx) = chan::<FrameByte, R>();
        let (d_tx, d_rx) = chan::<FrameByte, S>();
        let (e_tx, e_rx) = chan::<U<1>, R>();
        let (f_tx, f_rx) = chan::<U<1>, S>();
        let (k_tx, k_rx) = chan::<U<1>, S>();
        let (c_tx, c_rx) = chan::<U<1>, R>();
        join2(
            join2(
                self.ingress.run((raw, sfd, c_rx), (w_tx, e_tx)),
                join2(self.data.run(w_rx, d_tx), self.ev.run(e_rx, f_tx)),
            ),
            join2(
                self.cred.run(k_rx, c_tx),
                self.egress.run((d_rx, f_rx, time), (frames, stamps, k_tx)),
            ),
        )
        .await;
    }
}

/// Verilog of the single-clock units that carry the formal properties
/// (`tools/formal/prove.sh <unit>`), each instantiated on `DefaultClock`.
pub fn formal_verilog(unit: &str) -> String {
    match unit {
        "rx_ingress" => RxIngress::<DefaultClock>::verilog("rx_ingress"),
        "rx_egress" => RxEgress::<DefaultClock>::verilog("rx_egress"),
        other => panic!("no formal unit {other:?} in rx_path"),
    }
}
