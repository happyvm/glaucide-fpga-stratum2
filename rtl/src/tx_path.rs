// SPDX-License-Identifier: Apache-2.0
//! Transmit path between `eth_tx` and `mii_tx` (task T1.4b): store and
//! forward from `clk_sys` to `ClkTx`, and the transmit timestamp.
//!
//! # Ports
//!
//! `clk_sys` side (`DefaultClock`):
//! * `frames: Rx<FrameByte>`: complete frames from `eth_tx` (padded, FCS
//!   included), possibly with gaps between bytes; `last & err` = aborted.
//! * `time: In<U<96>>`: the `NtpClock` accumulator (32.64), NTP time is
//!   `time >> 32`.
//! * `stamps: Tx<Ts>`: one transmit timestamp per frame put on the wire.
//!
//! `ClkTx` side (the type parameter `C`, `ClkTx` in the design):
//! * `tx_frames: Tx<FrameByte, C>`: to `mii_tx`'s `frames` (through a
//!   registered channel, see the Netlist note of `stream.rs`), gap free.
//! * `sfd`, `underrun: In<Bit, C>`: the strobes of `mii_tx`.
//!
//! # Store and forward
//!
//! A memory of `DEPTH` bytes (4096 in the design: two maximum frames of
//! 1518 bytes and more) is written on `clk_sys` and read on `ClkTx`.
//! Pointers are `PTRS = ADDR + 1` bits wide (one lap bit).
//!
//! * `wptr`: next write address. `cptr`: committed pointer, the end of the
//!   last frame whose `last` came without `err`. A byte is written at
//!   `wptr`; a good `last` moves `cptr` to just after it (commit); a `last`
//!   with `err` is not written and moves `wptr` back to `cptr` (the aborted
//!   frame is forgotten and later overwritten). Bytes in `[cptr, wptr)`
//!   are an incomplete frame and are never readable by the `ClkTx` side.
//! * Back-pressure: `frames` is not accepted while the buffer is full,
//!   counted against the read pointer as seen on `clk_sys` (Gray code, two
//!   flops), so unread data is never overwritten. A frame longer than
//!   `DEPTH` would wait forever; `eth_tx` never makes one (1518 at most).
//! * Crossing of the frame-complete information: `cptr` jumps by a whole
//!   frame at a time, so it is not Gray-safe; it crosses by a req/ack
//!   handshake instead. `cx` holds a copy of `cptr` that only changes when
//!   no handshake is in flight; `hreq` toggles with it, crosses in two
//!   flops (`hq1`, `hq2`), and on the toggle the `ClkTx` side loads `cpt`
//!   from `cx` (stable for at least one `ClkTx` cycle by then: the usual
//!   multi-bit handshake; `cx -> cpt` is a false path for the timing
//!   analysis, T5.3). The ack (`hq3`) returns in two flops. `cpt` is
//!   therefore always a frame boundary, at or behind `cptr`.
//! * The read pointer `rptr` crosses back as Gray code (`rgray`, two
//!   flops, `rg1`/`rg2`): it moves by one per `ClkTx` cycle at most, the
//!   textbook case. Its binary form is registered once more (`rb`, one
//!   `clk_sys` cycle; task T5.3, so the Gray decode is not on the path of
//!   the input handshake): the writer sees free space one cycle later,
//!   which only makes `full` more conservative.
//! * Release: the `ClkTx` side starts a frame only when `rptr != cpt`, so
//!   every byte of it, up to the `last` stored in the memory, is already
//!   there: once started, a frame goes out one byte per cycle when
//!   `tx_frames.ready()` with no gap (`mii_tx` takes one byte every two
//!   `ClkTx` cycles; the registered channel in between holds two).
//!
//! # Transmit timestamp
//!
//! A frame is *in flight* (`unres`) from its first byte released until it
//! is resolved:
//!
//! * the first `sfd` strobe of `mii_tx` after the release belongs to it:
//!   it toggles `t_sfd`, which crosses in two flops plus an edge flop; at
//!   the edge, `clk_sys` captures `cap = (time >> 32) - TX_STAMP_BIAS`;
//! * *ok*: the SFD was seen, the frame's `last` was released, and
//!   [`RESOLVE_WAIT`] `ClkTx` cycles passed without `underrun` (enough for
//!   any truncation pulse of this frame to show): `t_ok` toggles; on its
//!   edge `clk_sys` queues `cap` as the frame's stamp and sends it on
//!   `stamps`;
//! * *bad*: `underrun` pulsed (frame truncated on the wire): `t_bad`
//!   toggles and **no stamp** is produced for that frame (the capture is
//!   dropped). With the gap-free release an underrun only happens for a
//!   frame longer than 1518 bytes (`mii_tx` truncation), which `eth_tx`
//!   never makes.
//!
//! `clk_sys` acknowledges each resolution with the `ack` toggle (for *ok*,
//! once the stamp has been accepted by `stamps`). The next frame is not
//! released before the ack is back (`ak1`, `ak2`), so at most one frame is in
//! flight, stamps come in frame order, one per frame that went out
//! complete, and a stalled `stamps` consumer back-pressures the
//! transmitter instead of losing a stamp. Toggle order: `t_ok` moves at
//! least `RESOLVE_WAIT` `ClkTx` cycles after `t_sfd`; with `ClkTx` slower
//! than `clk_sys` the SFD edge is never seen after the ok edge (same cycle
//! is handled).
//!
//! ## Stamp accuracy
//!
//! Reference instant: the `ClkTx` edge where `mii_tx` puts the SFD's 0xD
//! nibble on `txd` (`sfd` rises). `t_sfd` toggles one `ClkTx` period later
//! (40 ns); the first synchroniser flop samples it at the next `clk_sys`
//! edge (0 to 10 ns later, one more period if it goes metastable and
//! resolves to the old value); the second flop and the edge flop add one
//! period of `clk_sys` before `cap` loads, and `time` is the accumulator
//! of that cycle. Nominal latency, which [`TX_STAMP_BIAS`] subtracts:
//! 40 + 10 + 10 = 60 ns (`ClkTx` period + two `clk_sys` periods).
//! Residual error of the stamp against the reference instant: within
//! **±1 `clk_sys` cycle (±10 ns)** (-10..0 ns from the sampling phase,
//! up to +10 ns with a metastable first flop). Not included: the
//! DP83848J's MII-to-wire transmit latency (fixed, to be measured on the
//! board, T5.4) and the MII granularity itself (the SFD can only start on
//! a 40 ns `ClkTx` edge). The bias assumes `ClkTx` = 25 MHz.
//!
//! # Latency model (for T2.3, Transmit Timestamp T3)
//!
//! From the `clk_sys` edge where the frame's `last` is accepted on
//! `frames` (call it `s0`) to the SFD strobe of `mii_tx`, for a frame that
//! finds the transmitter idle (nothing in flight, inter-frame gap over),
//! with `Ttx` = 40 ns:
//!
//! | step | time | kind |
//! |---|---|---|
//! | commit, then `cx`/`hreq` at the next `clk_sys` edge | 10 ns | constant |
//! | `hreq` reaches the first flop `hq1` (first `ClkTx` edge after) | 0..40 ns | jitter (phase); +40 ns if `hq1` goes metastable |
//! | `hq2`, `hq3` (`cpt` loads), first byte released, registered channel, `mii_tx` latches it | 4 `ClkTx` = 160 ns | constant |
//! | preamble and SFD nibbles until the strobe (15 nibbles) | 15 `ClkTx` = 600 ns | constant |
//!
//! Total: `s0` + (770, 810] ns, 810..850 ns with a metastable first
//! flop; the test `latency_of_an_isolated_frame` measures 783..820 ns
//! from the cycle the last byte is *offered* (one `clk_sys` cycle before it
//! is accepted) over several phases, for 64- and 1518-byte frames alike
//! (the length does not matter: the whole frame is buffered before
//! release). The span from the SFD strobe to the first bit on the wire is
//! the PHY's (constant, to be measured, T5.4).
//!
//! Not constant, and not bounded by this unit:
//! * a commit handshake already in flight (a frame committed shortly
//!   before) delays the release by up to one handshake round trip
//!   (about 3 `ClkTx` + 3 `clk_sys` cycles, ~150 ns);
//! * a frame that does not find the transmitter idle waits for the frames
//!   ahead of it (`16 + 2 * len` `ClkTx` cycles each on the wire, plus the
//!   24-cycle gap of `mii_tx`) and for the previous frame's stamp to be
//!   taken on `stamps`.
//!
//! What T2.3 can rely on: (1) the stamp on `stamps` is the SFD instant on
//! the MII to within ±10 ns (PHY latency excluded) and arrives after the
//! frame went out complete; (2) for a frame alone in the transmitter, the
//! SFD comes 0.77 to 0.85 µs after its last byte is accepted; queueing
//! behind other frames adds whole frame times. A Transmit Timestamp
//! written into the frame must therefore be predicted with a margin that
//! covers the queue ahead (the server knows it if it is the only source),
//! or the stamp must be used afterwards (interleaved mode/follow-up); this
//! unit does not choose.
//!
//! # Formal
//!
//! The unit has two clocks. `tools/formal/prove.sh tx_path prove` proves
//! the `check!`s on the generated Verilog by k-induction with the
//! solver's single global clock, i.e. with `ClkTx` and `clk_sys` ticking
//! together, synchronisers included (the induction closes at depth 14).
//! Proved that way, with no `assume!` (any input stream, any `sfd` and
//! `underrun`, any `ready`):
//! * occupancy and pointer order: `rptr <= cpt <= cx <= cptr <= wptr <=
//!   rptr + DEPTH` (modulo the lap), the writer's view lagging the reader,
//!   no write onto an unread byte;
//! * release: a byte leaves only from `[rptr, cpt)`, the committed
//!   region, and the byte before every commit point (`cptr`, `cx`, `cpt`)
//!   is a frame's `last`, so a frame is released only after its `last`
//!   was received, and an aborted frame (never committed) never;
//! * no gap: inside a released frame its next byte is always there;
//! * stamps: released = ok + bad + in flight; stamps sent + queued +
//!   in the synchroniser = ok frames; one resolution in flight at a time;
//!   handshake and toggle synchronisers change one stage at a time.
//!
//! `prove.sh tx_path cover 60` reaches every `cover!`. Simulation only
//! (the Rust tests, which also run every `check!` at every edge): all
//! relative phases and ratios of the two clocks (4:1 with four phases,
//! 37 ns and 23 ns against 10 ns, 1:1 against the real `mii_tx`), the
//! stamp error bound and the latency figures. Metastability is covered by
//! neither. The formal unit is the same design with a 16-byte memory
//! ([`TxPathFormal`]): no property depends on the depth. The Gray decoder
//! on the `clk_sys` side handles pointers of up to 16 bits. Ghost
//! registers (`g_*`) feed only the checks and are pruned by synthesis.
//!
//! # Resources
//!
//! The memory is distributed RAM (`#[distributed]`), read asynchronously
//! on `ClkTx` at the `rptr` register, like `ChanCdc`'s: 4096 x 9 bits is
//! on the order of 1000 to 1500 LUTs of the 63 400 of the XC7A100T. The
//! formal checks read it at three more addresses (`cptr - 1`, `cx - 1`,
//! `cpt - 1`); those reads only feed `assert`s under `ifdef FORMAL` and
//! are removed by synthesis. A block-RAM variant would need a registered
//! read with a one-byte prefetch and ghost copies of the `last` bits for
//! the proof; not done here.
use crate::clocks::ClkTx;
use crate::stream::{FrameByte, Ts};
use txhdl::comp::{join2, mux, Clock, DefaultClock, In, Mem, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{check, cover, lower, with, Trace};

/// Nominal latency from the SFD strobe to the capture, in ns (see the
/// module documentation).
pub const TX_STAMP_BIAS_NS: u64 = 60;
/// [`TX_STAMP_BIAS_NS`] in NTP units (2^-32 s), rounded.
pub const TX_STAMP_BIAS: u64 = ((TX_STAMP_BIAS_NS << 32) + 500_000_000) / 1_000_000_000;
/// `ClkTx` cycles after the SFD and the release of `last` before a frame
/// is declared sent (an `underrun` pulse of the frame comes earlier).
pub const RESOLVE_WAIT: u8 = 8;

/// The transmit path, generic over the memory size (`ADDR`, `DEPTH =
/// 1 << ADDR`, `PTRS = ADDR + 1`, all stated because const expressions
/// need nightly) and the two clocks, `S` (`clk_sys`) and `C` (`ClkTx`),
/// so that the tests can choose their periods and phases. The design uses
/// [`TxPath`].
#[derive(Trace)]
pub struct TxPathG<const ADDR: usize, const DEPTH: usize, const PTRS: usize, S: Clock, C: Clock> {
    /// `{data[7:0], last}`: `last` is bit 0. Distributed RAM: one write
    /// port (`clk_sys`), one asynchronous read port (`ClkTx`, at `rptr`);
    /// the other reads only feed the formal checks and are pruned.
    #[distributed]
    pub mem: Mem<U<9>, DEPTH, S>,
    // ---- clk_sys ----
    pub wptr: Reg<U<PTRS>, S>,
    pub cptr: Reg<U<PTRS>, S>,
    /// Copy of `cptr` offered to the ClkTx side, stable during a handshake.
    pub cx: Reg<U<PTRS>, S>,
    pub hreq: Reg<Bit, S>,
    #[async_reg]
    pub ha1: Reg<Bit, S>,
    #[async_reg]
    pub ha2: Reg<Bit, S>,
    #[async_reg]
    pub rg1: Reg<U<PTRS>, S>,
    #[async_reg]
    pub rg2: Reg<U<PTRS>, S>,
    /// `rg2` back to binary, registered.
    pub rb: Reg<U<PTRS>, S>,
    #[async_reg]
    pub sf1: Reg<Bit, S>,
    #[async_reg]
    pub sf2: Reg<Bit, S>,
    pub sf3: Reg<Bit, S>,
    #[async_reg]
    pub ok1: Reg<Bit, S>,
    #[async_reg]
    pub ok2: Reg<Bit, S>,
    pub ok3: Reg<Bit, S>,
    #[async_reg]
    pub bd1: Reg<Bit, S>,
    #[async_reg]
    pub bd2: Reg<Bit, S>,
    pub bd3: Reg<Bit, S>,
    pub cap: Reg<U<64>, S>,
    pub stamp: Reg<U<64>, S>,
    pub pend: Reg<Bit, S>,
    pub ack: Reg<Bit, S>,
    // ---- ClkTx ----
    pub rptr: Reg<U<PTRS>, C>,
    pub rgray: Reg<U<PTRS>, C>,
    pub cpt: Reg<U<PTRS>, C>,
    #[async_reg]
    pub hq1: Reg<Bit, C>,
    #[async_reg]
    pub hq2: Reg<Bit, C>,
    pub hq3: Reg<Bit, C>,
    #[async_reg]
    pub ak1: Reg<Bit, C>,
    #[async_reg]
    pub ak2: Reg<Bit, C>,
    /// Inside a frame: its first byte released, its last not yet.
    pub busy: Reg<Bit, C>,
    /// A released frame not yet resolved (ok or bad).
    pub unres: Reg<Bit, C>,
    pub sfd_seen: Reg<Bit, C>,
    pub sent_last: Reg<Bit, C>,
    pub win: Reg<U<4>, C>,
    pub t_sfd: Reg<Bit, C>,
    pub t_ok: Reg<Bit, C>,
    pub t_bad: Reg<Bit, C>,
    // ---- ghosts (checks only) ----
    /// Frames released (first byte sent), ok, bad: ClkTx.
    pub g_rel: Reg<U<8>, C>,
    pub g_ok: Reg<U<8>, C>,
    pub g_bad: Reg<U<8>, C>,
    /// Stamps sent: clk_sys.
    pub g_st: Reg<U<8>, S>,
    /// Bytes accepted in the current (uncommitted) frame: clk_sys.
    pub g_open: Reg<U<PTRS>, S>,
}

/// The design's transmit path: 4096 bytes, `ClkTx`.
pub type TxPath = TxPathG<12, 4096, 13, DefaultClock, ClkTx>;
/// The same logic with a 16-byte memory, for the formal proof.
pub type TxPathFormal = TxPathG<4, 16, 5, DefaultClock, ClkTx>;

impl<const A: usize, const D: usize, const P: usize, S: Clock, C: Clock> Default
    for TxPathG<A, D, P, S, C>
{
    fn default() -> Self {
        TxPathG {
            mem: Mem::default(),
            wptr: Reg::default(),
            cptr: Reg::default(),
            cx: Reg::default(),
            hreq: Reg::default(),
            ha1: Reg::default(),
            ha2: Reg::default(),
            rg1: Reg::default(),
            rg2: Reg::default(),
            rb: Reg::default(),
            sf1: Reg::default(),
            sf2: Reg::default(),
            sf3: Reg::default(),
            ok1: Reg::default(),
            ok2: Reg::default(),
            ok3: Reg::default(),
            bd1: Reg::default(),
            bd2: Reg::default(),
            bd3: Reg::default(),
            cap: Reg::default(),
            stamp: Reg::default(),
            pend: Reg::default(),
            ack: Reg::default(),
            rptr: Reg::default(),
            rgray: Reg::default(),
            cpt: Reg::default(),
            hq1: Reg::default(),
            hq2: Reg::default(),
            hq3: Reg::default(),
            ak1: Reg::default(),
            ak2: Reg::default(),
            busy: Reg::default(),
            unres: Reg::default(),
            sfd_seen: Reg::default(),
            sent_last: Reg::default(),
            win: Reg::default(),
            t_sfd: Reg::default(),
            t_ok: Reg::default(),
            t_bad: Reg::default(),
            g_rel: Reg::default(),
            g_ok: Reg::default(),
            g_bad: Reg::default(),
            g_st: Reg::default(),
            g_open: Reg::default(),
        }
    }
}

#[lower]
impl<const A: usize, const D: usize, const P: usize, S: Clock, C: Clock>
    Unit<(Rx<FrameByte, S>, In<U<96>, S>, In<Bit, C>, In<Bit, C>), (Tx<Ts, S>, Tx<FrameByte, C>)>
    for TxPathG<A, D, P, S, C>
{
    async fn run(
        &mut self,
        (frames, time, sfd, underrun): (Rx<FrameByte, S>, In<U<96>, S>, In<Bit, C>, In<Bit, C>),
        (stamps, tx_frames): (Tx<Ts, S>, Tx<FrameByte, C>),
    ) {
        join2(
            // ---------------- clk_sys ----------------
            async {
                loop {
                    S::rising().await;
                    let wptr = self.wptr.get();
                    let cptr = self.cptr.get();
                    let cx = self.cx.get();
                    let hreq = self.hreq.get();
                    let g = self.rg2.get();
                    // The read pointer as seen here, back to binary (`rb`
                    // registers it; `rs` is the registered value).
                    let rs_n = g
                        ^ (g >> 1u32)
                        ^ (g >> 2u32)
                        ^ (g >> 3u32)
                        ^ (g >> 4u32)
                        ^ (g >> 5u32)
                        ^ (g >> 6u32)
                        ^ (g >> 7u32)
                        ^ (g >> 8u32)
                        ^ (g >> 9u32)
                        ^ (g >> 10u32)
                        ^ (g >> 11u32)
                        ^ (g >> 12u32)
                        ^ (g >> 13u32)
                        ^ (g >> 14u32)
                        ^ (g >> 15u32);
                    let rs = self.rb.get();
                    let used = wptr - rs;
                    let full = Bit::from(used == D);

                    // Input: an aborted last is taken even when full (it
                    // writes nothing).
                    let b = frames.head();
                    let bv = Bit::from(frames.peek().is_some());
                    let abort_in = b.last & b.err;
                    let take = bv & (!full | abort_in);
                    let _ = frames.recv_if(!full | abort_in);
                    let wr = take & !abort_in;
                    let commit = wr & b.last;
                    let rewind = take & abort_in;
                    let wptr1 = wptr + 1;

                    // Handshake of the committed pointer.
                    let hs_idle = Bit::from(self.ha2.get() == hreq);
                    let offer = hs_idle & Bit::from(cx != cptr);

                    // Timestamp events.
                    let sfd_e = self.sf2.get() ^ self.sf3.get();
                    let ok_e = self.ok2.get() ^ self.ok3.get();
                    let bad_e = self.bd2.get() ^ self.bd3.get();
                    let now = time.get().slice::<32, 64>() - TX_STAMP_BIAS;
                    let capv = mux(sfd_e, now, self.cap.get());
                    let pend = self.pend.get();
                    let send = pend & stamps.ready();
                    if send.to_bool() {
                        stamps.send(self.stamp.get());
                    }

                    // ---- properties (clk_sys) ----
                    let rp = self.rptr.get();
                    let open = self.g_open.get();
                    let m_cp = self.mem.read((cptr - 1).slice::<0, A>());
                    let m_cx = self.mem.read((cx - 1).slice::<0, A>());
                    check!(used <= D, "occupancy seen by the writer never exceeds the memory");
                    check!(rp - rs <= used, "the synchronised read pointer lags the real one");
                    check!(wptr - rp <= D, "never more unread bytes than the memory holds");
                    check!(wptr - cptr == open, "bytes past the commit point are exactly the open frame");
                    check!(cptr - rp <= wptr - rp, "the commit point is between read and write pointers");
                    if wr.to_bool() {
                        check!(wptr - rp < D, "a write never lands on an unread byte");
                    }
                    if Bit::from(cptr != rp).to_bool() {
                        check!(
                            m_cp.bit(0).to_bool(),
                            "the byte before the commit point is a frame's last"
                        );
                    }
                    if Bit::from(cx != rp).to_bool() {
                        check!(
                            m_cx.bit(0).to_bool(),
                            "the offered commit point ends a frame"
                        );
                    }
                    check!(cx - rp <= cptr - rp, "the offered commit point is at or behind the commit point");
                    // Event chains: one transition at most in flight.
                    let par = self.t_ok.get() ^ self.t_bad.get();
                    let ok_fl = self.t_ok.get() ^ self.ok3.get();
                    let bad_fl = self.t_bad.get() ^ self.bd3.get();
                    check!(
                        (self.t_ok.get() ^ self.ok1.get()).zext::<2>()
                            + (self.ok1.get() ^ self.ok2.get()).zext::<2>()
                            + (self.ok2.get() ^ self.ok3.get()).zext::<2>()
                            <= 1,
                        "ok toggle: one change at most in the synchroniser"
                    );
                    check!(
                        (self.t_bad.get() ^ self.bd1.get()).zext::<2>()
                            + (self.bd1.get() ^ self.bd2.get()).zext::<2>()
                            + (self.bd2.get() ^ self.bd3.get()).zext::<2>()
                            <= 1,
                        "bad toggle: one change at most in the synchroniser"
                    );
                    check!(
                        (self.t_sfd.get() ^ self.sf1.get()).zext::<2>()
                            + (self.sf1.get() ^ self.sf2.get()).zext::<2>()
                            + (self.sf2.get() ^ self.sf3.get()).zext::<2>()
                            <= 1,
                        "sfd toggle: one change at most in the synchroniser"
                    );
                    check!(
                        (self.ack.get() ^ self.ak1.get()).zext::<2>()
                            + (self.ak1.get() ^ self.ak2.get()).zext::<2>()
                            <= 1,
                        "ack toggle: one change at most in the synchroniser"
                    );
                    check!(
                        ok_fl.zext::<2>() + bad_fl.zext::<2>() + pend.zext::<2>() <= 1,
                        "one resolved frame at most awaits its acknowledgement"
                    );
                    check!(
                        (par ^ self.ack.get()) == (ok_fl | bad_fl | pend),
                        "every resolution is acknowledged once"
                    );
                    check!(
                        self.g_st.get() + pend.zext::<8>() + ok_fl.zext::<8>() == self.g_ok.get(),
                        "one stamp per frame that went out complete, none for the others"
                    );
                    cover!(full & bv, "back-pressure: the buffer is full");
                    cover!(rewind & Bit::from(open != 0), "an aborted frame is dropped");
                    cover!(send, "a stamp is delivered");
                    cover!(send & Bit::from(self.g_st.get() == 1), "two stamps are delivered");

                    with!(self <= {
                        wr ? mem.at(wptr.slice::<0, A>()): (b.data.zext::<9>() << 1u32) | b.last.zext::<9>(),
                        wr ? { wptr: wptr1, g_open: open + 1 },
                        commit ? { cptr: wptr1, g_open: 0 },
                        rewind ? { wptr: cptr, g_open: 0 },
                        offer ? { cx: cptr, hreq: !hreq },
                        Bit::One ? {
                            ha1: self.hq3.get(),
                            ha2: self.ha1.get(),
                            rg1: self.rgray.get(),
                            rg2: self.rg1.get(),
                            rb: rs_n,
                            sf1: self.t_sfd.get(),
                            sf2: self.sf1.get(),
                            sf3: self.sf2.get(),
                            ok1: self.t_ok.get(),
                            ok2: self.ok1.get(),
                            ok3: self.ok2.get(),
                            bd1: self.t_bad.get(),
                            bd2: self.bd1.get(),
                            bd3: self.bd2.get(),
                        },
                        sfd_e ? cap: now,
                        ok_e ? { pend: Bit::One, stamp: capv },
                        send ? { pend: Bit::Zero, ack: !self.ack.get(), g_st: self.g_st.get() + 1 },
                        bad_e ? ack: !self.ack.get(),
                    });
                }
            },
            // ---------------- ClkTx ----------------
            async {
                loop {
                    C::rising().await;
                    let rp = self.rptr.get();
                    let cpt = self.cpt.get();
                    let busy = self.busy.get();
                    let unres = self.unres.get();
                    let seen = self.sfd_seen.get();
                    let slast = self.sent_last.get();
                    let win = self.win.get();
                    let par = self.t_ok.get() ^ self.t_bad.get();

                    // Committed pointer handshake.
                    let hq_e = self.hq2.get() ^ self.hq3.get();

                    // Release.
                    let word = self.mem.read(rp.slice::<0, A>());
                    let hdata = word.slice::<1, 8>();
                    let hlast = word.bit(0);
                    let have = Bit::from(rp != cpt);
                    let free = !busy & !unres & Bit::from(self.ak2.get() == par);
                    let can = have & (busy | free);
                    let snd = can & tx_frames.ready();
                    if snd.to_bool() {
                        tx_frames.send(FrameByte { data: hdata, last: hlast, err: Bit::Zero });
                    }
                    let start = snd & !busy;
                    let rp1 = rp + 1;

                    // Resolution.
                    let s = sfd.get();
                    let u = underrun.get();
                    let take_sfd = s & unres & !seen;
                    let bad = u & unres;
                    let armed = unres & seen & slast;
                    let ok = armed & Bit::from(win == RESOLVE_WAIT) & !bad;

                    // ---- properties (ClkTx) ----
                    let cptr = self.cptr.get();
                    let cx = self.cx.get();
                    let wptr = self.wptr.get();
                    let m_cpt = self.mem.read((cpt - 1).slice::<0, A>());
                    check!(!busy | have, "no gap inside a released frame: its next byte is there");
                    check!(cpt - rp <= cx - rp, "the released region ends at or behind the offered commit point");
                    check!(cpt - rp <= cptr - rp, "only committed bytes are released");
                    check!(cptr - rp <= wptr - rp, "the commit point is behind the write pointer");
                    check!(wptr - rp <= D, "the read pointer never passes the writer");
                    if Bit::from(cpt != rp).to_bool() {
                        check!(
                            m_cpt.bit(0).to_bool(),
                            "the released region ends with a frame's last"
                        );
                    }
                    if snd.to_bool() {
                        check!(rp != cptr, "a byte is released only after its frame's last was received");
                        check!(!start | !unres, "one frame in flight at a time");
                    }
                    check!(
                        self.rgray.get() == (rp ^ (rp >> 1u32)),
                        "the crossing read pointer is the Gray code of the read pointer"
                    );
                    check!(
                        (self.hreq.get() ^ self.hq1.get()).zext::<2>()
                            + (self.hq1.get() ^ self.hq2.get()).zext::<2>()
                            + (self.hq2.get() ^ self.hq3.get()).zext::<2>()
                            + (self.hq3.get() ^ self.ha1.get()).zext::<2>()
                            + (self.ha1.get() ^ self.ha2.get()).zext::<2>()
                            <= 1,
                        "commit handshake: one change at most around the loop"
                    );
                    if Bit::from(self.hq3.get() == self.hreq.get()).to_bool() {
                        check!(cpt == cx, "a completed handshake leaves the offered pointer loaded");
                    }
                    check!(
                        self.g_rel.get() == self.g_ok.get() + self.g_bad.get() + unres.zext::<8>(),
                        "every released frame is resolved once, ok or bad"
                    );
                    check!(!unres | (par == self.ack.get()), "a frame in flight waits on no earlier acknowledgement");
                    check!(win <= RESOLVE_WAIT, "resolution wait counter range");
                    check!(!slast | !busy, "the last byte ends the frame");
                    if unres.to_bool() {
                        check!(
                            (self.t_ok.get() == self.ok3.get()) && (self.t_bad.get() == self.bd3.get()),
                            "nothing of an earlier frame is in flight while one is unresolved"
                        );
                    }
                    cover!(start, "a frame is released");
                    cover!(snd & hlast & Bit::from(self.g_rel.get() == 2), "two frames go out");
                    cover!(ok, "a frame is resolved as sent");
                    cover!(bad, "a frame is resolved as truncated (underrun)");
                    cover!(snd & busy & hlast, "a frame of two bytes or more is released complete");

                    with!(self <= {
                        hq_e ? cpt: self.cx.get(),
                        Bit::One ? {
                            hq1: self.hreq.get(),
                            hq2: self.hq1.get(),
                            hq3: self.hq2.get(),
                            ak1: self.ack.get(),
                            ak2: self.ak1.get(),
                        },
                        snd ? { rptr: rp1, rgray: rp1 ^ (rp1 >> 1u32) },
                        snd & hlast ? { busy: Bit::Zero, sent_last: Bit::One },
                        start ? {
                            busy: !hlast,
                            unres: Bit::One,
                            sfd_seen: Bit::Zero,
                            sent_last: hlast,
                            win: 0,
                            g_rel: self.g_rel.get() + 1,
                        },
                        take_sfd ? { sfd_seen: Bit::One, t_sfd: !self.t_sfd.get() },
                        armed & Bit::from(win < RESOLVE_WAIT) ? win: win + 1,
                        ok ? { unres: Bit::Zero, t_ok: !self.t_ok.get(), g_ok: self.g_ok.get() + 1 },
                        bad ? { unres: Bit::Zero, t_bad: !self.t_bad.get(), g_bad: self.g_bad.get() + 1 },
                    });
                }
            },
        )
        .await;
    }
}
