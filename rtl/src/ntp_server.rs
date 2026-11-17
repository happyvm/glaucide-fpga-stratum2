// SPDX-License-Identifier: Apache-2.0
//! NTP server reply builder (tasks T2.3, T2.4, T2.7, and the shared v4/v6
//! front of T3.6), domain `DefaultClock` (`clk_sys`, 100 MHz).
//!
//! One unit, [`NtpServer`], turns client requests (the UDP payload of a
//! datagram to port 123, IPv4 or IPv6) into 48-byte server replies
//! (RFC 5905 §7.3, §8, §9.2 `fast_xmit`), applies the policy of T2.4
//! (which requests get an answer, per-source rate limit, Kiss-o'-Death
//! `RATE`) and writes the precision field of T2.7. The byte-exact
//! specification is `tools/golden/responder.py` (rules N1-N10).
//!
//! # Ports (flat, in this order)
//!
//! Inputs:
//!
//! | port | type | meaning |
//! |---|---|---|
//! | `meta4`, `req4` | `Rx<Meta>`, `Rx<FrameByte>` | IPv4 requests: `udp4_rx`'s `ntp_meta`, `ntp` |
//! | `meta6`, `req6` | `Rx<Meta>`, `Rx<FrameByte>` | IPv6 requests: same contract from the IPv6 UDP layer |
//! | `time` | `In<U<96>>` | clock accumulator (`DisciplinedClock`), NTP time = `time >> 32` |
//! | `synced` | `In<Bit>` | the clock is synchronised to an upstream |
//! | `stratum` | `In<U<8>>` | stratum to announce when synchronised (2) |
//! | `ref_id` | `In<U<32>>` | Reference ID in IPv4 replies (first wire byte most significant) |
//! | `ref_id6` | `In<U<32>>` | Reference ID in IPv6 replies (rule N8 of the model, see below) |
//! | `ref_ts` | `In<U<64>>` | Reference Timestamp (32.32): last update of the clock |
//! | `root_delay` | `In<U<32>>` | Root Delay, NTP short format 16.16 |
//! | `root_disp` | `In<U<32>>` | Root Dispersion, 16.16 |
//! | `precision` | `In<U<8>>` | Precision, signed log2 seconds, two's complement (default −24 = `0xE8`, [`DEFAULT_PRECISION`], T2.7) |
//! | `tx_latency` | `In<U<64>>` | constant K added to the sampled time for T3, units of 2^-32 s (see « Transmit timestamp ») |
//! | `rate_limit` | `In<U<16>>` | replies per second allowed per source, 0 = unlimited |
//! | `kod_enable` | `In<Bit>` | over the limit: 0 = silence, 1 = a 48-byte KoD `RATE` |
//!
//! Outputs: `reply_meta: Tx<NtpReplyMeta>` (one per reply, sent before its
//! first byte) and `reply: Tx<FrameByte>` (exactly 48 bytes per reply,
//! `last` on the 48th).
//!
//! Every wire input (`synced` .. `kod_enable`) is sampled in the cycle
//! named below for it; they may change at any time otherwise.
//!
//! # Input contract
//!
//! Each request source is a pair (`Meta`, payload) with the contract of
//! `stream.rs`: one `Meta` per datagram, then the UDP payload bytes, `last`
//! on the final one, `err` with `last` = the datagram is bad (UDP checksum,
//! FCS...). Used fields of `Meta`: `src` (client address, IPv4 in the low
//! 32 bits), `dst` (the address the request was sent to), `sport` (client
//! port), `eth_src` (client MAC), `rx_ts` (receive timestamp of the frame,
//! NTP 32.32, threaded in-band by T1.13). `family`, `dport` and `len` are
//! not used: the family is the channel the request came on (`meta4`/`req4`
//! = IPv4, `meta6`/`req6` = IPv6), the destination port is 123 by
//! construction of the UDP layer, and the length is counted on the stream.
//! A source must deliver the payload of each `Meta` it sent (at least one
//! byte); the unit waits for it.
//!
//! # Behaviour, one request at a time
//!
//! States: IDLE → RECV → (META → OUT) → IDLE.
//!
//! 1. **IDLE**: if `meta4` or `meta6` has a `Meta`, one is popped (round
//!    robin: when both are pending, the family *not* served last wins; when
//!    only one is pending it is taken). Its fields are registered, together
//!    with the rate-limiter index of the source (see « Rate limiter »). No
//!    payload byte is taken in this cycle.
//! 2. **RECV**: the payload of the chosen source is taken, one byte per
//!    cycle when present, never stalled (the other source is not touched).
//!    Parsed on the fly: byte 0 (LI/VN/Mode), byte 2 (Poll), bytes 40-47
//!    (the client's Transmit Timestamp, kept as Origin). Bytes after the
//!    48th (extension fields, MAC) are taken and ignored. In the cycle the
//!    byte with `last` is taken (**cycle L**), the unit decides:
//!    * the request is *valid* iff (N1) it has at least 48 bytes, (N2)
//!      Mode = 3, (N3) VN = 3 or 4 (the model's set: VN 0, 1, 2, 5, 6, 7 are
//!      dropped), and its `last` came without `err`;
//!    * a valid request goes through the rate limiter (below): *allowed* or
//!      *limited*;
//!    * valid and allowed: a normal reply; valid, limited and
//!      `kod_enable = 1`: a KoD reply; anything else: **nothing is emitted
//!      at all** (no `reply_meta`, no byte) and the unit returns to IDLE.
//!
//!    Nothing is ever emitted before the request's `last` is known, so a
//!    request ending with `err` simply produces nothing (`reply` never
//!    carries `err`: the reply stream's `err` is constant 0).
//!    In cycle L the unit samples `synced`, `stratum`, `ref_id`/`ref_id6`,
//!    `ref_ts`, `root_delay`, `root_disp`, `precision`, `rate_limit`,
//!    `kod_enable` and the seconds of `time` (for the rate limiter), and
//!    builds bytes 0-39 of the reply into registers.
//! 3. **META**: `reply_meta` is sent as soon as it has room:
//!    `family` = the source channel (0 IPv4, 1 IPv6), `dst_ip` = `Meta.src`
//!    (the client), `src_ip` = `Meta.dst` (we answer from the address the
//!    request was sent to), `dst_mac` = `Meta.eth_src`, `sport` = 123,
//!    `dport` = `Meta.sport`, `len` = 48.
//! 4. **OUT**: the 48 reply bytes go out on `reply`, one per cycle when it
//!    has room, `last` on byte 47, `err` = 0. After byte 47 is sent the unit
//!    is in IDLE the next cycle and takes the next request.
//!
//! # Reply fields (normal reply)
//!
//! Let `sync = synced & (stratum != 0) & (stratum < 16)` (sampled in L): an
//! out-of-range stratum input is treated as "not synchronised", so a
//! server reply never carries stratum 0 (that would read as a KoD) nor
//! LI = 0 with stratum 16.
//!
//! | bytes | field | value |
//! |---|---|---|
//! | 0 | LI, VN, Mode | LI = 0 if `sync` else 3 (N4); VN = request's VN (3 or 4, = min(VN, 4), N5); Mode = 4 |
//! | 1 | Stratum | `stratum` if `sync` else 16 (N6) |
//! | 2 | Poll | request's byte 2 (N7) |
//! | 3 | Precision | `precision` input (T2.7) |
//! | 4-7 | Root Delay | `root_delay` |
//! | 8-11 | Root Dispersion | `root_disp` |
//! | 12-15 | Reference ID | `ref_id` for an IPv4 request, `ref_id6` for IPv6 (N8; kept when not synchronised, as the model) |
//! | 16-23 | Reference Timestamp | `ref_ts` |
//! | 24-31 | Origin Timestamp | request bytes 40-47, verbatim (N9) |
//! | 32-39 | Receive Timestamp | `Meta.rx_ts` |
//! | 40-47 | Transmit Timestamp | T3, below |
//!
//! Multi-byte fields are big endian (first wire byte = most significant).
//! The reply is exactly 48 bytes (N10), never longer than the request
//! (which is at least 48): no amplification.
//!
//! # Transmit timestamp (T3)
//!
//! Let **S** be the cycle in which reply byte 40 is sent on `reply`. The
//! unit keeps a pipeline register `t3p <= (time >> 32) + tx_latency`
//! (modulo 2^64) loaded every cycle, so in cycle S `t3p` holds the value
//! computed from `time` and `tx_latency` **of cycle S − 1**. Then
//!
//! `T3 = max(t3p, rx_ts)` (unsigned 64-bit compare),
//!
//! so **T3 ≥ Receive Timestamp holds unconditionally** (unsigned). Without
//! the clamp it would hold only under the assumption that the clock is
//! monotone between the frame's receive stamp and S (no backward step by
//! the servo, no wrap of the 64-bit NTP time, i.e. no era change, in
//! between) and `tx_latency` ≥ 0; under that assumption the clamp never
//! acts. Sampling at byte 40 (and not at L) means every stall of `reply`
//! before byte 40 moves T3 with it; stalls after it, and queueing
//! downstream, are not covered (see below).
//!
//! ## `tx_latency` (K)
//!
//! K is the time from the sampling of `time` (cycle S − 1) to the instant
//! the reply's SFD is put on the MII, for a reply that finds the transmit
//! path idle. It is the sum of
//!
//! 1. `clk_sys` cycles from S − 1 to the cycle `tx_path` accepts the last
//!    byte (FCS) of the frame from `eth_tx` (call it s0): 7 more reply
//!    bytes, `udp4_tx`'s store and forward (2 cycles of latency, then the
//!    8 + 48 datagram bytes), `ipv4_tx`'s 20 header bytes, `tx_arb`'s
//!    header cycle, `eth_tx`'s 14 header bytes and 4 FCS bytes, one byte per
//!    cycle, plus the registered channels in between. Measured by the
//!    chain test (`rtl/tests/ntp_server.rs`, `chain_ntp4_basic`):
//!    [`TX_CYCLES_V4`] = 108 cycles for IPv4 (90-byte frame + 4 FCS);
//!    the IPv6 frame has a 40-byte IP header instead of 20,
//!    [`TX_CYCLES_V6`] = 128 cycles (`chain_latency_v6`, through `ipv6_tx`
//!    with `udp4_tx` standing in for the UDP layer, whose latency structure
//!    T3.4 is expected to mirror; the testbench's extra relay stage is
//!    subtracted);
//! 2. `tx_path`'s latency from s0 to the SFD strobe of `mii_tx` (the
//!    `ClkTx` edge where the SFD's 0xD nibble goes out): 770 to 810 ns for
//!    a frame that finds the transmitter idle (its header, « Latency
//!    model »), taken at the middle, [`TX_PATH_LATENCY_PS`] = 790 ns.
//!    This includes the 7 preamble bytes and the SFD (15 nibbles).
//!
//! Defaults: [`TX_LATENCY_V4`] = 108 × 10 ns + 790 ns = 1.870 µs = 8032
//! units of 2^-32 s, and [`TX_LATENCY_V6`] = 128 × 10 ns + 790 ns = 2.070 µs
//! = 8891 units (rounded to nearest; the uncertainty is ±20 ns from
//! `tx_path`'s window, ±1 `clk_sys` cycle of channel phase). The
//! reference instant (SFD 0xD nibble on the MII) is the same one the
//! receive stamp of `rx_path` estimates, so the PHY latencies (DP83848J,
//! fixed, to be measured on the board, T5.4) are excluded from both; that
//! is why K is an input wire: calibrate on the board. Not covered by K:
//! the ±20 ns phase jitter of the `clk_sys`/`ClkTx` handshake in
//! `tx_path`; a frame queued behind another one in `tx_arb`/`eth_tx`/
//! `tx_path` (whole frame times, e.g. at line rate behind the previous
//! reply), or a stall of `reply` after byte 40: T3 is then early by that
//! amount. A single K serves both families only approximately (the IPv6
//! frame is 20 bytes longer, 200 ns); the integration may feed K per
//! family from the family of the reply in progress, or accept the error.
//!
//! # Kiss-o'-Death `RATE` (RFC 8633, RFC 5905 §7.4)
//!
//! Sent instead of silence only when `kod_enable` = 1 and the request is
//! valid but rate limited. Same 48-byte layout and same `reply_meta`, with:
//! LI = 3, Stratum = 0, Reference ID = ASCII "RATE" (`0x52415445`),
//! Reference Timestamp = 0, Origin = request's Transmit (N9 as usual),
//! **Receive = Transmit = the request's Transmit** (as ntpd does: a KoD
//! carries no time). VN, Mode 4, Poll, Precision, Root Delay and Root
//! Dispersion as in a normal reply. A KoD is never larger than the request.
//! Every limited request gets one KoD when enabled (the KoD itself is not
//! further limited): the reply volume equals the request volume at most.
//!
//! # Rate limiter (T2.4/T2.5)
//!
//! A table of [`RL_ENTRIES`] = 64 words in distributed RAM, indexed by a
//! 6-bit hash of the source: the XOR fold of a 64-bit key into 6 bits
//! ([`src_hash`]), key = the IPv4 address (zero extended) for IPv4, the
//! upper 64 bits (the /64 prefix) for IPv6 (an IPv6 host normally owns a
//! whole /64, so limiting per address would be no limit). The index is
//! computed and registered in IDLE when the `Meta` is popped.
//!
//! Each word is `{sec: U<16>, cnt: U<16>}`: the low 16 bits of the NTP
//! seconds of the window it counts, and the replies counted in it. In
//! cycle L, for a *valid* request only (invalid ones do not touch the
//! table), with `now` = bits 64..79 of `time` (low 16 bits of the NTP
//! seconds) and `lim` = `rate_limit`:
//!
//! * `c` = `cnt` if `sec == now`, else 0 (a new one-second window);
//! * allowed iff `lim == 0` (unlimited) or `c < lim`;
//! * the word becomes `{now, c + 1}` if allowed and `lim != 0`, else
//!   `{now, c}` (a limited request, or an unlimited one, does not count).
//!
//! So a source gets at most `lim` normal replies per NTP second (fixed
//! windows aligned on whole seconds: up to `2 lim` in a sliding second
//! that straddles a boundary). Hash collisions merge the counters of the
//! sources that share a word: a source can only be limited *earlier*
//! because of another one, never get more than `lim` replies per window
//! from its own word, by construction (every allowed reply increments the
//! word it was counted on). Across all sources, at most `64 lim` normal
//! replies per second leave the unit. A word whose window is exactly
//! 65536 s old is taken for the current one (only early limiting again).
//! The table starts at zero (bitstream initial value) and has no reset.
//!
//! # Throughput
//!
//! Per request, without stalls: 1 cycle (IDLE, `Meta`) + `n` cycles (the
//! `n` payload bytes, n ≥ 48) + 1 cycle (META) + 48 cycles (OUT) =
//! **`n + 50` cycles, 98 for a plain 48-byte request**, against ~917
//! cycles between two requests at the IPv4 line rate (109 000/s). The
//! test `throughput_back_to_back` measures it. A request is not taken
//! while a reply is still being handed to `reply`: the input stalls
//! (`udp4_rx` and the layers below buffer).
//!
//! # Integration
//!
//! All channels must be registered channels or FIFOs (never port to port,
//! see the netlist note of `stream.rs`). `reply_meta` is sent before the
//! first `reply` byte; one `reply_meta` is followed by exactly one 48-byte
//! reply. Adapters from [`NtpReplyMeta`] to the UDP transmit requests
//! (`udp4_tx`'s `UdpTxReq`, the future `udp6_tx`) are the integration's.
//! `meta4`/`meta6` are popped independently of the payload channels (in
//! IDLE); the producer may send a `Meta` before or with its first byte.
use crate::stream::{FrameByte, Meta};
use txhdl::comp::{mux, Clock, DefaultClock, In, Mem, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{lower, with, Trace, Transaction as TransactionDerive, Value as ValueDerive};

/// UDP port of NTP.
pub const NTP_PORT: u16 = 123;
/// Length of every reply (and minimum length of a request), bytes.
pub const NTP_LEN: u16 = 48;
/// Default precision field: −24 (≈ 60 ns, `docs/references.md`), T2.7.
pub const DEFAULT_PRECISION: u8 = 0xE8;
/// Reference ID of a Kiss-o'-Death `RATE` packet: ASCII "RATE".
pub const KOD_RATE: u32 = 0x5241_5445;
/// Words of the rate-limiter table.
pub const RL_ENTRIES: usize = 64;

/// `clk_sys` cycles from the sampling of `time` (cycle S − 1) to the cycle
/// `tx_path` accepts the last FCS byte, IPv4 reply on an idle path
/// (measured by `chain_ntp4_basic`).
pub const TX_CYCLES_V4: u64 = 108;
/// Same for IPv6: 20 more header bytes (measured by `chain_latency_v6`).
pub const TX_CYCLES_V6: u64 = TX_CYCLES_V4 + 20;
/// `tx_path` latency from the last byte accepted to the SFD strobe,
/// middle of its documented 770..810 ns, in picoseconds.
pub const TX_PATH_LATENCY_PS: u64 = 790_000;
/// Period of `clk_sys`, picoseconds.
pub const CLK_SYS_PS: u64 = 10_000;

/// Picoseconds to NTP 2^-32 s units, rounded to nearest.
pub const fn ps_to_ntp(ps: u64) -> u64 {
    ((ps as u128 * (1u128 << 32) + 500_000_000_000) / 1_000_000_000_000) as u64
}

/// Default `tx_latency` for IPv4 replies, 2^-32 s units.
pub const TX_LATENCY_V4: u64 = ps_to_ntp(TX_CYCLES_V4 * CLK_SYS_PS + TX_PATH_LATENCY_PS);
/// Default `tx_latency` for IPv6 replies, 2^-32 s units.
pub const TX_LATENCY_V6: u64 = ps_to_ntp(TX_CYCLES_V6 * CLK_SYS_PS + TX_PATH_LATENCY_PS);

/// Where and how a reply goes; one per reply, before its 48 bytes.
#[derive(TransactionDerive, ValueDerive, Clone, Copy, Default, Debug)]
pub struct NtpReplyMeta {
    /// 0 = IPv4, 1 = IPv6 (the channel the request came on).
    pub family: Bit,
    /// The client's address (the request's source); IPv4 in the low 32 bits.
    pub dst_ip: U<128>,
    /// The address the request was sent to: answer from it.
    pub src_ip: U<128>,
    /// The client's MAC (the request frame's source MAC).
    pub dst_mac: U<48>,
    /// 123.
    pub sport: U<16>,
    /// The client's source port.
    pub dport: U<16>,
    /// UDP payload length: 48.
    pub len: U<16>,
}

/// Rate-limiter index of a 64-bit source key: XOR fold into 6 bits.
#[lower]
pub fn src_hash(k: U<64>) -> U<6> {
    let a = k.slice::<0, 6>() ^ k.slice::<6, 6>() ^ k.slice::<12, 6>() ^ k.slice::<18, 6>();
    let b = k.slice::<24, 6>() ^ k.slice::<30, 6>() ^ k.slice::<36, 6>() ^ k.slice::<42, 6>();
    let c = k.slice::<48, 6>() ^ k.slice::<54, 6>() ^ k.slice::<60, 4>().zext::<6>();
    a ^ b ^ c
}

const IDLE: u8 = 0;
const RECV: u8 = 1;
const META: u8 = 2;
const OUT: u8 = 3;

#[derive(Trace, Default)]
pub struct NtpServer {
    /// IDLE, RECV, META, OUT.
    pub st: Reg<U<2>>,
    /// The request in progress came on `meta6`/`req6`.
    pub sel6: Reg<Bit>,
    /// The last request taken was IPv6 (round robin).
    pub rr: Reg<Bit>,
    /// Payload bytes taken, saturating at 48.
    pub pos: Reg<U<6>>,
    /// Byte 0 says Mode 3 and VN 3 or 4.
    pub ok0: Reg<Bit>,
    pub vn: Reg<U<3>>,
    pub poll: Reg<U<8>>,
    /// Request bytes 40-47 (client Transmit), shifted in; the reply's Origin.
    pub org: Reg<U<64>>,
    pub m_src: Reg<U<128>>,
    pub m_dst: Reg<U<128>>,
    pub m_sport: Reg<U<16>>,
    pub m_esrc: Reg<U<48>>,
    pub m_rxts: Reg<U<64>>,
    /// Rate-limiter index of the source.
    pub hix: Reg<U<6>>,
    /// The reply in progress is a KoD.
    pub kod: Reg<Bit>,
    /// Reply image: `ra` bytes 0-15, `rb` 16-31, `rc` 32-39; byte k is the
    /// top byte of `ra` when it is sent, the three shift left by 8 together.
    pub ra: Reg<U<128>>,
    pub rb: Reg<U<128>>,
    pub rc: Reg<U<64>>,
    /// Reply bytes sent (0..=47).
    pub k: Reg<U<6>>,
    /// `(time >> 32) + tx_latency` of the previous cycle.
    pub t3p: Reg<U<64>>,
    /// Rate limiter: `{sec[31:16], cnt[15:0]}` per hash index.
    #[distributed]
    pub rl: Mem<U<32>, 64, DefaultClock>,
}

#[lower]
impl
    Unit<
        (
            Rx<Meta>,
            Rx<FrameByte>,
            Rx<Meta>,
            Rx<FrameByte>,
            In<U<96>>,
            In<Bit>,
            In<U<8>>,
            In<U<32>>,
            In<U<32>>,
            In<U<64>>,
            In<U<32>>,
            In<U<32>>,
            In<U<8>>,
            In<U<64>>,
            In<U<16>>,
            In<Bit>,
        ),
        (Tx<NtpReplyMeta>, Tx<FrameByte>),
    > for NtpServer
{
    async fn run(
        &mut self,
        (
            meta4,
            req4,
            meta6,
            req6,
            time,
            synced,
            stratum,
            ref_id,
            ref_id6,
            ref_ts,
            root_delay,
            root_disp,
            precision,
            tx_latency,
            rate_limit,
            kod_enable,
        ): (
            Rx<Meta>,
            Rx<FrameByte>,
            Rx<Meta>,
            Rx<FrameByte>,
            In<U<96>>,
            In<Bit>,
            In<U<8>>,
            In<U<32>>,
            In<U<32>>,
            In<U<64>>,
            In<U<32>>,
            In<U<32>>,
            In<U<8>>,
            In<U<64>>,
            In<U<16>>,
            In<Bit>,
        ),
        (reply_meta, reply): (Tx<NtpReplyMeta>, Tx<FrameByte>),
    ) {
        loop {
            DefaultClock::rising().await;
            let st = self.st.get();
            let sel6 = self.sel6.get();
            let pos = self.pos.get();
            let org = self.org.get();
            let k = self.k.get();
            let ra = self.ra.get();
            let rb = self.rb.get();
            let rc = self.rc.get();
            let kod = self.kod.get();
            let is_idle = Bit::from(st == IDLE);
            let is_recv = Bit::from(st == RECV);
            let is_meta = Bit::from(st == META);
            let is_out = Bit::from(st == OUT);
            let now = time.get();

            // ---------------------------------------------------- IDLE
            let v4 = Bit::from(meta4.peek().is_some());
            let v6 = Bit::from(meta6.peek().is_some());
            let pick6 = v6 & (!v4 | !self.rr.get());
            let take_m = is_idle & (v4 | v6);
            let _ = meta4.recv_if(take_m & !pick6);
            let _ = meta6.recv_if(take_m & pick6);
            let m4 = meta4.head();
            let m6 = meta6.head();
            let n_src = mux(pick6, m6.src, m4.src);
            let n_dst = mux(pick6, m6.dst, m4.dst);
            let n_sport = mux(pick6, m6.sport, m4.sport);
            let n_esrc = mux(pick6, m6.eth_src, m4.eth_src);
            let n_rxts = mux(pick6, m6.rx_ts, m4.rx_ts);
            let key = mux(
                pick6,
                n_src.slice::<64, 64>(),
                n_src.slice::<0, 32>().zext::<64>(),
            );
            let n_hix = src_hash(key);

            // ---------------------------------------------------- RECV
            let p4 = Bit::from(req4.peek().is_some());
            let p6 = Bit::from(req6.peek().is_some());
            let b4 = req4.head();
            let b6 = req6.head();
            let have = mux(sel6, p6, p4);
            let data = mux(sel6, b6.data, b4.data);
            let last = mux(sel6, b6.last, b4.last);
            let err = mux(sel6, b6.err, b4.err);
            let take = is_recv & have;
            let _ = req4.recv_if(take & !sel6);
            let _ = req6.recv_if(take & sel6);

            let mode = data.slice::<0, 3>();
            let vn = data.slice::<3, 3>();
            let hdr_ok = Bit::from(mode == 3) & (Bit::from(vn == 3) | Bit::from(vn == 4));
            let in_ts = Bit::from(pos >= 40) & Bit::from(pos < 48);
            let org_sh = (org << 8u32) | data.zext::<64>();
            // Origin complete in cycle L even when byte 47 is the last one.
            let org_now = mux(Bit::from(pos == 47), org_sh, org);
            let done = take & last;
            let valid = self.ok0.get() & Bit::from(pos >= 47) & !err;

            // Rate limiter (cycle L).
            let sec = now.slice::<64, 16>();
            let word = self.rl.read(self.hix.get());
            let same = Bit::from(word.slice::<16, 16>() == sec);
            let cnt = mux(same, word.slice::<0, 16>(), U::<16>::from(0u16));
            let lim = rate_limit.get();
            let unl = Bit::from(lim == U::<16>::from(0u16));
            let allowed = unl | Bit::from(cnt < lim);
            let cnt_n = mux(allowed & !unl, cnt + U::<16>::from(1u16), cnt);
            let word_n = (sec.zext::<32>() << 16u32) | cnt_n.zext::<32>();
            let go_n = valid & allowed;
            let go_k = valid & !allowed & kod_enable.get();
            let go = go_n | go_k;

            // Reply image (cycle L).
            let s_in = stratum.get();
            let sync = synced.get()
                & Bit::from(s_in != U::<8>::from(0u8))
                & Bit::from(s_in < U::<8>::from(16u8));
            let alarm = go_k | !sync;
            let li = mux(alarm, U::<2>::from(3u8), U::<2>::from(0u8));
            let b0 =
                (li.zext::<8>() << 6u32) | (self.vn.get().zext::<8>() << 3u32) | U::<8>::from(4u8);
            let s_out = mux(go_k, U::<8>::from(0u8), mux(sync, s_in, U::<8>::from(16u8)));
            let rid = mux(
                go_k,
                U::<32>::from(KOD_RATE),
                mux(sel6, ref_id6.get(), ref_id.get()),
            );
            let n_ra = (b0.zext::<128>() << 120u32)
                | (s_out.zext::<128>() << 112u32)
                | (self.poll.get().zext::<128>() << 104u32)
                | (precision.get().zext::<128>() << 96u32)
                | (root_delay.get().zext::<128>() << 64u32)
                | (root_disp.get().zext::<128>() << 32u32)
                | rid.zext::<128>();
            let n_rts = mux(go_k, U::<64>::from(0u64), ref_ts.get());
            let n_rb = (n_rts.zext::<128>() << 64u32) | org_now.zext::<128>();
            let n_rc = mux(go_k, org_now, self.m_rxts.get());

            // ---------------------------------------------------- META
            let send_m = is_meta & reply_meta.ready();
            if send_m.to_bool() {
                reply_meta.send(NtpReplyMeta {
                    family: sel6,
                    dst_ip: self.m_src.get(),
                    src_ip: self.m_dst.get(),
                    dst_mac: self.m_esrc.get(),
                    sport: U::<16>::from(NTP_PORT),
                    dport: self.m_sport.get(),
                    len: U::<16>::from(NTP_LEN),
                });
            }

            // ---------------------------------------------------- OUT
            let rx = self.m_rxts.get();
            let t3p = self.t3p.get();
            let t3c = mux(Bit::from(t3p < rx), rx, t3p);
            let t3 = mux(kod, org, t3c);
            let at40 = Bit::from(k == 40);
            let o_last = Bit::from(k == 47);
            let send_o = is_out & reply.ready();
            if send_o.to_bool() {
                reply.send(FrameByte {
                    data: mux(at40, t3.slice::<56, 8>(), ra.slice::<120, 8>()),
                    last: o_last,
                    err: Bit::Zero,
                });
            }
            let ra_sh = (ra << 8u32) | rb.slice::<120, 8>().zext::<128>();
            let ra_n = mux(at40, t3.zext::<128>() << 72u32, ra_sh);
            let rb_sh = (rb << 8u32) | rc.slice::<56, 8>().zext::<128>();

            with!(self <= {
                t3p: now.slice::<32, 64>() + tx_latency.get(),
                take_m ? {
                    st: U::<2>::from(RECV),
                    sel6: pick6,
                    rr: pick6,
                    pos: U::<6>::from(0u8),
                    ok0: Bit::Zero,
                    m_src: n_src,
                    m_dst: n_dst,
                    m_sport: n_sport,
                    m_esrc: n_esrc,
                    m_rxts: n_rxts,
                    hix: n_hix,
                },
                take & Bit::from(pos < 48) ? pos: pos + U::<6>::from(1u8),
                take & Bit::from(pos == 0) ? {
                    ok0: hdr_ok,
                    vn: vn,
                },
                take & Bit::from(pos == 2) ? poll: data,
                take & in_ts ? org: org_sh,
                done & valid ? rl.at(self.hix.get()): word_n,
                done ? {
                    st: mux(go, U::<2>::from(META), U::<2>::from(IDLE)),
                    kod: go_k,
                    ra: n_ra,
                    rb: n_rb,
                    rc: n_rc,
                },
                send_m ? {
                    st: U::<2>::from(OUT),
                    k: U::<6>::from(0u8),
                },
                send_o ? {
                    k: k + U::<6>::from(1u8),
                    ra: ra_n,
                    rb: rb_sh,
                    rc: rc << 8u32,
                },
                send_o & o_last ? st: U::<2>::from(IDLE),
            });
        }
    }
}
