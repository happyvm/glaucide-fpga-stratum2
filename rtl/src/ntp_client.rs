// SPDX-License-Identifier: Apache-2.0
//! NTP client (task T4.2), domain `DefaultClock` (`clk_sys`, 100 MHz).
//!
//! One unit, [`NtpClient`], polls up to [`PEERS`] = 4 statically configured
//! upstream servers (IPv4 or IPv6, `config.rs` slots `up0..up3`) with
//! 48-byte mode-3 requests, matches their mode-4 replies, and turns each
//! accepted reply into one [`Sample`] for the clock filter (T4.3), or a
//! `miss` when a poll got no usable answer. Bit-exact specification: the
//! Python model `tools/golden/ntpclient/client.py` (`build_request`,
//! `make_sample`, `next_hop4`, `next_hop6`, `src6`), replayed against
//! `tools/golden/ntpclient/vectors/client.txt` by `rtl/tests/ntp_client.rs`.
//!
//! # Ports (flat, in this order)
//!
//! Inputs. Every wire is sampled in the cycle it is used (named below) and
//! may change at any time; in the design they are `config` constants,
//! except `time`, `poll_log2` (the servo's poll exponent) and the latencies.
//!
//! | port | type | meaning |
//! |---|---|---|
//! | `up<i>_addr`, `up<i>_v6`, `up<i>_en` (i = 0..3, 12 wires, in the order `up0_addr, up0_v6, up0_en, up1_addr, ...`) | `In<U<128>>`, `In<Bit>`, `In<Bit>` | slot i: address (IPv4 in the low 32 bits, first wire byte most significant), family (1 = IPv6), in use |
//! | `local_ip4`, `netmask`, `gateway` | `In<U<32>>` | our IPv4 address, netmask, IPv4 gateway |
//! | `gateway6` | `In<U<128>>` | IPv6 gateway (normally a link-local address) |
//! | `local_ll`, `local_gua` | `In<U<128>>` | our link-local and global IPv6 addresses |
//! | `gua_valid` | `In<Bit>` | `local_gua` is in use |
//! | `local_mac` | `In<U<48>>` | not used (kept for the integration's port list) |
//! | `poll_log2` | `In<U<8>>` | regular poll exponent (unsigned), clamped to 4..=17 |
//! | `precision` | `In<U<8>>` | our precision, signed log2 s, two's complement (`PRECISION_LOG2`) |
//! | `tx_latency4`, `tx_latency6` | `In<U<64>>` | K of the IPv4 / IPv6 transmit path, 2^-32 s (`ntp_server::TX_LATENCY_V4/V6`) |
//! | `time` | `In<U<96>>` | clock accumulator; NTP time (32.32) = `time >> 32`, NTP seconds = bits 64..95 |
//! | `meta`, `rep` | `Rx<Meta>`, `Rx<FrameByte>` | mode-4 replies from `ntp_demux`: one `Meta` then the UDP payload, `err` on `last` = bad datagram |
//! | `arp_rsp`, `nd_rsp` | `Rx<ArpEntry>` | answers to `arp_lookup` / `nd_lookup` (one per lookup) |
//!
//! Outputs:
//!
//! | port | type | meaning |
//! |---|---|---|
//! | `arp_lookup`, `arp_resolve` | `Tx<U<32>>` | IPv4 next hop: cache lookup / resolution request (`arp.rs`) |
//! | `nd_lookup`, `nd_resolve` | `Tx<U<128>>` | IPv6 next hop, same shape (neighbor cache, T3.6) |
//! | `req4`, `pay4` | `Tx<UdpTxReq>`, `Tx<FrameByte>` | IPv4 request to `udp4_tx`: the `UdpTxReq`, then 48 payload bytes |
//! | `req6`, `pay6` | `Tx<Udp6TxReq>`, `Tx<FrameByte>` | IPv6 request to `udp6_tx`, same |
//! | `samples` | `Tx<Sample>` | one per accepted reply (to the clock filter) |
//! | `miss` | `Tx<U<2>>` | slot index of a poll counted as unanswered |
//!
//! All channels must be registered channels or FIFOs (netlist note of
//! `stream.rs`). The neighbor units must answer every lookup (exactly one
//! `ArpEntry` per `arp_lookup` / `nd_lookup`, as `arp.rs` does): `LWT`
//! waits for it without a timeout. `samples`, `miss`, `req*` and `pay*`
//! may stall at any time (the unit waits, nothing is lost).
//!
//! # Structure
//!
//! A single sequencer (`st`, 10 states) serves the four slots one event at
//! a time. Two slot indices: `sp`, the scheduler pointer (the slot `SCAN`
//! examines), and `cs`, the slot of the event in progress (reply or request
//! path). In every cycle the slot wires and the per-slot state are read,
//! and written, at `ix` = `sp` in `SCAN`, `cs` in every other state.
//! Per-slot state lives in three 4-word distributed RAMs read
//! asynchronously at `ix`: `m_t1` (T1 of the outstanding request), `m_tm`
//! (`last`: tick of the last poll start, or of the arming (see
//! Scheduler), bits 95..64; `tsent`: NTP second at which T1 was sampled,
//! 63..32; `rt`: tick of the last failed lookup, 31..0, a tick being 2^-6
//! s, i.e. `time` bits 58..89) and `m_fl` (flags: bit 0 `out` = a request
//! is outstanding, bit 1 `pend` = a poll started and waits for its MAC, bit
//! 2 `sent` = a request was sent since the slot was enabled, bits 5..3
//! `burst` = polls started since enable (saturates at 4), bits 9..6
//! `tries` = failed lookups of the current poll, bit 10 `arm` = the slot's
//! start time `last` was taken, meaningful while `burst` = 0). All memories start at zero (no reset). At most one write per
//! memory per cycle.
//!
//! States: `SCAN` (idle: takes a reply or examines slot `sp`), reply path
//! `RX` → `CHK` → `C2` → `SMP`, request path `LKP` → `LWT` → `REQ` →
//! `PAY`, and `MISS`. Each event runs to completion; nothing else is
//! taken meanwhile (replies wait in `meta`/`rep`). Without output stalls
//! an event takes at most 1 + n + 4 cycles for a reply of n bytes, 52 for
//! a request (plus the lookup's round trip), against polls every >= 2 s.
//!
//! # Scheduler (state `SCAN`)
//!
//! Let `now_s` = `time` bits 64..95 (NTP seconds, wrapping; used by the reply
//! deadline), `now_t` = bits 58..89 (ticks of 2^-6 s, wrapping, used by the
//! poll schedule). Each cycle in `SCAN` slot `sp` is
//! examined; its **action** is the first that applies of:
//!
//! 1. `up<sp>_en` = 0 and its flags are not all 0: they are cleared
//!    (`out`, `pend`, `sent`, `burst`, `tries`, `arm` = 0; an outstanding
//!    request is forgotten, **no miss**); re-enabling restarts the iburst.
//! 2. `out` and `now_s - tsent >= DEADLINE_S` (2, unsigned 32-bit wrapping
//!    difference): `out` := 0, `cs` := `sp`, and `MISS` sends `miss(cs)`.
//!    The deadline is thus 1 to 2 s after T1 (whole-second granularity). A
//!    backward clock step makes the difference huge: the request expires at
//!    once (a miss); a forward step likewise.
//! 3. **Arming**: `up<sp>_en` = 1, `burst` = 0 and `arm` = 0 (the slot was
//!    just enabled, or the unit just left reset): `last` := `now_t` (this
//!    instant is the slot's `t0`), `arm` := 1. Nothing is sent yet.
//! 4. not `out`, not `pend` and **due**, where due means: `burst` = 0 (then
//!    `arm` = 1 and) `now_t - last >= 16 * sp` ticks, i.e. the slot's own
//!    **stagger** `sp * 2^-2 s` has elapsed since its arming; or `burst` >= 1
//!    and `now_t - last >= I * 64` ticks with `I` = 2 s ([`IBURST_GAP_S`])
//!    while `burst < 4`, else `I = 2^clamp(poll_log2, 4, 17)` s (sampled
//!    then). The poll starts: `last` := `now_t`, `pend` := 1, `tries` := 0,
//!    `arm` := 0, `burst` := min(burst + 1, 4).
//!
//!    **Staggered schedule.** All slots of a freshly enabled (or reset)
//!    unit are armed within a few cycles of one another, at `t0` (to a few
//!    cycles). Slot `i` then starts its iburst at `t0 + i * 2^-2 s` (a
//!    quarter second per slot; to the tick granularity: the start is
//!    between `i * 2^-2 - 2^-6` and `i * 2^-2` seconds after the arming),
//!    then at `+ 2`, `+ 4`, `+ 6` s ([`IBURST_COUNT`] = 4, **iburst**), then
//!    every `2^poll` s, each measured from the previous poll start of the
//!    same slot (not from the request), once that poll is finished (`out` =
//!    0 and `pend` = 0), so the per-slot phase offsets persist in the
//!    regular schedule. Two slots are therefore never due at the same
//!    instant (their polls differ by at least `2^-2 - 2^-6` s, much more
//!    than a frame time of the transmit path, ~9 us at 100 Mbit/s), which
//!    keeps T1 (predicted for an IDLE transmit path, like the server's T3)
//!    accurate: no request is ever sent right behind another. The Python
//!    model is `client.schedule(poll_log2, n, slot)`. A poll due while the
//!    previous one of the same slot is still open waits for it (deadline or
//!    reply) and starts at the next examination (a delay, which also moves
//!    that slot's later polls).
//!
//!    **One request at a time.** The sequencer runs an event to completion
//!    before taking another: a request (`LKP` ... `PAY`) is started only in
//!    `SCAN`, i.e. after the previous request's 48 payload bytes were all
//!    handed to `pay4` / `pay6` (`last` accepted). Two requests of different
//!    slots are never interleaved, and when the channels do not stall the
//!    stagger leaves the transmit path idle between them.
//! 5. `pend` and (`tries` = 0 or `now_t - rt >= RETRY_TICKS` (2)):
//!    `cs` := `sp`, go to `LKP`.
//! 6. none (including a disabled slot with clear flags): no action.
//!
//! `sp` advances (`sp + 1` mod 4) in each `SCAN` cycle where slot `sp` has
//! no action, and stays otherwise (after actions 1, 3 and 4 the slot is
//! examined again in the next `SCAN` cycle). A waiting reply `Meta` is
//! taken in a `SCAN` cycle when slot `sp` has no action, or when it has one
//! and `pri` = 1; `pri` toggles in every `SCAN` cycle, so under a flood of
//! replies actions and replies alternate (the scheduler is never starved);
//! a cycle that takes a reply performs no action.

//! # Request path
//!
//! * **Next hop** (combinational from the slot wires, sampled in `LKP` and
//!   again in `LWT`): IPv4: `addr` if `(addr & netmask) == (local_ip4 &
//!   netmask)` (on link) else `gateway`. IPv6: `addr` if it is in
//!   `fe80::/10` or (`gua_valid` and its upper 64 bits equal `local_gua`'s)
//!   else `gateway6`.
//! * `LKP`: sends `arp_lookup(next hop)` (IPv4) or `nd_lookup` (IPv6) when
//!   the channel has room, then `LWT`.
//! * `LWT`: waits for the answer on `arp_rsp` / `nd_rsp` (one per lookup;
//!   the other channel is not read). `found`: the MAC is kept, `REQ`.
//!   Not found: `tries` += 1, `rt` := `now_t`, and `arp_resolve` /
//!   `nd_resolve` (next hop) is sent if that channel has room (else it is
//!   skipped: a resolution is already queued). If `tries` reaches
//!   `RES_TRIES` (8) the poll is abandoned (`pend` := 0, `tries` := 0) and
//!   counts as a **miss only if `sent` = 1** (a request was sent to this
//!   slot since it was enabled: a resolution failure before any request
//!   is not a miss); else `SCAN`, and the scheduler retries (action 4) when
//!   `now_t - rt >= 2`, i.e. 15.6 to 31.3 ms later. So at most 8 lookups
//!   and 8 resolves per poll, within about 0.22 s; a poll abandoned before
//!   any request was sent leaves the slot to its next poll start.
//! * `REQ`: sends `req4` = `UdpTxReq { dst_mac, dst_ip = addr[31:0], sport
//!   = 123, dport = 123, len = 48 }` or `req6` = `Udp6TxReq { dst_mac,
//!   dst_ip = addr, src_ip, 123, 123, 48 }` with `src_ip` = `local_ll` if
//!   `addr` is in `fe80::/10`, else `local_gua` if `gua_valid`, else
//!   `local_ll`; then `PAY`. The request goes out before any payload byte.
//! * `PAY`: the 48 bytes on `pay4` / `pay6`, one per cycle when the channel
//!   has room, `last` on byte 47, `err` = 0: byte 0 = 0x23 (LI 0, VN 4,
//!   mode 3), byte 2 = `poll_log2` (raw, sampled when byte 2 is sent), byte
//!   3 = `precision` (sampled when byte 3 is sent), bytes 40..47 = T1 big
//!   endian, every other byte 0 (Python `build_request`).
//!
//! ## Transmit timestamp T1 (same rule as the server's T3)
//!
//! A pipeline register `t1p <= (time >> 32) + (v6 ? tx_latency6 :
//! tx_latency4)` (modulo 2^64, family of slot `cs`) is loaded every cycle.
//! Let **S** be the cycle request byte 40 is sent: `T1 = t1p` in S, i.e.
//! computed from `time` and the latency of cycle S - 1, exactly as
//! `ntp_server`'s T3 (its « Transmit timestamp »): the transmit chain from
//! here (`udp4_tx`/`udp6_tx`, `ipv4_tx`/`ipv6_tx`, `tx_arb`, `eth_tx`,
//! `tx_path`) is the server's, so the same constants apply (the
//! integration may need to add the cycles of an extra channel stage). In S
//! `m_t1[cs]` := T1 and `tsent` := `now_s`; when byte 47 is sent, `out` :=
//! 1, `pend` := 0, `sent` := 1, and the unit returns to `SCAN`.
//!
//! # Reply path
//!
//! * `SCAN`: a `Meta` is taken (`recv`, see « Scheduler » for when). The
//!   slot is matched in that cycle:
//!   the lowest `i` with `up<i>_en`, `up<i>_v6 == Meta.family`, the address
//!   equal (all 128 bits for IPv6, the low 32 for IPv4) to `Meta.src`, and
//!   `Meta.sport == 123`. `hit` and `cs` := i (3 when none) are registered with
//!   `Meta.rx_ts` (T4).
//! * `RX`: the payload is taken one byte per cycle when present, never
//!   stalled, up to and including `last`. Kept: bytes 0..15 (header), 24..31
//!   (Origin), 32..39 (Receive, T2), 40..47 (Transmit, T3); later bytes are
//!   ignored. When `last` is taken: dropped (back to `SCAN`, nothing sent)
//!   if `err`, fewer than 48 bytes or not `hit`; else `CHK`.
//! * `CHK` (slot `cs`): the reply is **authentic** iff `out` = 1, mode = 4,
//!   VN = 3 or 4, and Origin == `m_t1[cs]` (anti-spoofing). Not authentic:
//!   dropped, the request stays outstanding (a bogus reply never causes a
//!   miss nor closes the request). Authentic: `out` := 0 (so a second reply
//!   to the same request is dropped as not outstanding), and if stratum = 0
//!   (KoD), stratum >= 16 or LI = 3 (alarm): `MISS` (the poll is counted as
//!   a miss at once, no sample); else the first arithmetic stage, `C2`.
//! * `C2`, `SMP`: arithmetic (below); `SMP` sends the `Sample` when
//!   `samples` has room and returns to `SCAN`.
//! * `MISS`: sends `miss(cs)` when `miss` has room, returns to `SCAN`.
//!
//! Each authentic request therefore yields exactly one of: one `Sample`,
//! one `miss` (unusable reply or deadline, while enabled), or nothing (slot
//! disabled meanwhile).
//!
//! # Arithmetic (Python `arith`, RFC 5905 §8)
//!
//! `w(x)` is `x mod 2^64` read as a signed 64-bit integer; `T4` = `Meta.rx_ts`.
//!
//! * `CHK` registers `a = T2 - T1`, `b = T3 - T4`, `d1 = T4 - T1`, `d2 = T3
//!   - T2` (64-bit wrapping).
//! * `C2`: `offset = (sext66(a) + sext66(b)) >> 1` (arithmetic: the 66-bit
//!   sum's bits 64..1, rounding toward minus infinity; always fits);
//!   `delay = sext66(d1) - sext66(d2)`, 0 when negative (bit 65), else its
//!   low 64 bits (always fits); `iv = d1` if `d1[63] = 0` else 0; `prod =
//!   iv * PHI_Q32` (96 bits, < 2^80), registered as its two 32 x 16 partial
//!   products `plo` = `iv[31:0] * PHI_Q32` and `phi` = `iv[63:32] * PHI_Q32`
//!   (task T5.3: two DSP slices deep instead of four), so that
//!   `(prod + 2^32 - 1) >> 32 = phi + ((plo + 2^32 - 1) >> 32)`.
//! * `SMP`: `disp = pf(peer precision) + pf(precision) + ((prod + 2^32 - 1)
//!   >> 32)` with `pf(p)` = 2^p in 32.32 = `1 << (32 + p)` for `p` in
//!   -32..=-1, `1 << 32` for `p >= 0`, `1` for `p < -32` ([`prec_pow`], the
//!   convention of `sync_state::prec_floor`). Below 2^34 + 2^48: no wrap.
//!   `precision` is sampled in `SMP`.
//! * `Sample` = `{peer: cs, offset, delay, disp, t4: T4, stratum: byte 1,
//!   leap: byte 0 bits 7..6, ref_id: bytes 12..15, root_delay: bytes 4..7,
//!   root_disp: bytes 8..11}` of the reply.
//!
//! # Deviations from RFC 5905 (v1 decisions)
//!
//! No poll adaptation from the reply's poll, no KoD `RATE`/`DENY` handling
//! beyond counting the poll as missed, no duplicate test on T3 (the single
//! outstanding Origin makes a replay useless), no check of T2/T3 being
//! nonzero, a fixed 1-2 s reply deadline instead of "until the next poll",
//! and the reply is taken only from source port 123.
use crate::arp::ArpEntry;
use crate::ntp_types::{Sample, PEERS, PHI_Q32};
use crate::stream::{FrameByte, Meta};
use crate::udp4_tx::UdpTxReq;
use crate::udp6_tx::Udp6TxReq;
use txhdl::comp::{mux, Clock, DefaultClock, In, Mem, Out, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{lower, with, Trace};

/// UDP port of NTP (our source port and the server's).
pub const NTP_PORT: u16 = 123;
/// Request length, bytes.
pub const REQ_LEN: u16 = 48;
/// Requests of the initial burst, and their spacing in seconds.
pub const IBURST_COUNT: u8 = 4;
pub const IBURST_GAP_S: u32 = 2;
/// A request expires when `now_s - tsent >= DEADLINE_S` (1 to 2 s after T1).
pub const DEADLINE_S: u32 = 2;
/// Clamp of `poll_log2`.
pub const POLL_MIN: u8 = 4;
pub const POLL_MAX: u8 = 17;
/// Failed lookups per poll before it is abandoned.
pub const RES_TRIES: u8 = 8;
/// Ticks (2^-6 s) between two lookups of one poll.
pub const RETRY_TICKS: u32 = 2;
/// Byte 0 of a request: LI 0, VN 4, mode 3.
pub const REQ_B0: u8 = 0x23;
/// Kiss codes (RFC 5905 §7.4) in the Reference ID of a stratum-0 reply.
pub const KOD_RATE: u32 = 0x5241_5445; // "RATE"
pub const KOD_DENY: u32 = 0x4445_4E59; // "DENY"
pub const KOD_RSTR: u32 = 0x5253_5452; // "RSTR"
/// Largest poll exponent a RATE back-off reaches (2^10 s).
pub const KOD_POLL_MAX: u8 = 10;

const _: () = assert!(PEERS == 4);

const S_SCAN: u8 = 0;
const S_RX: u8 = 1;
const S_CHK: u8 = 2;
const S_C2: u8 = 3;
const S_SMP: u8 = 4;
const S_MISS: u8 = 5;
const S_LKP: u8 = 6;
const S_LWT: u8 = 7;
const S_REQ: u8 = 8;
const S_PAY: u8 = 9;

/// `2^p` in 32.32 for the signed log2 `p` (8-bit two's complement):
/// `1 << (32 + p)` for `p` in -32..=-1, `1 << 32` for `p >= 0`, `1` below -32.
#[lower]
pub fn prec_pow(pr: U<8>) -> U<64> {
    let sh = pr.slice::<0, 5>();
    let x0 = U::<64>::from(1u32);
    let x1 = mux(sh.bit(0), x0 << 1u32, x0);
    let x2 = mux(sh.bit(1), x1 << 2u32, x1);
    let x3 = mux(sh.bit(2), x2 << 4u32, x2);
    let x4 = mux(sh.bit(3), x3 << 8u32, x3);
    let x5 = mux(sh.bit(4), x4 << 16u32, x4);
    let nonneg = !pr.bit(7);
    let tiny = pr.bit(7) & Bit::from(pr < U::<8>::from(0xE0u8));
    mux(
        nonneg,
        U::<64>::from(1u64 << 32),
        mux(tiny, U::<64>::from(1u32), x5),
    )
}

/// `clamp(p, 4, 17)`.
#[lower]
pub fn poll_clamp(p: U<8>) -> U<8> {
    let lo = Bit::from(p < U::<8>::from(POLL_MIN));
    let hi = Bit::from(p > U::<8>::from(POLL_MAX));
    mux(
        lo,
        U::<8>::from(POLL_MIN),
        mux(hi, U::<8>::from(POLL_MAX), p),
    )
}

/// `2^clamp(p, 4, 17)` seconds, as a 32-bit count.
#[lower]
pub fn poll_secs(p: U<8>) -> U<32> {
    let c = poll_clamp(p);
    let sh = c.slice::<0, 5>();
    let x0 = U::<32>::from(1u32);
    let x1 = mux(sh.bit(0), x0 << 1u32, x0);
    let x2 = mux(sh.bit(1), x1 << 2u32, x1);
    let x3 = mux(sh.bit(2), x2 << 4u32, x2);
    let x4 = mux(sh.bit(3), x3 << 8u32, x3);
    mux(sh.bit(4), x4 << 16u32, x4)
}

/// `2^(e + 6) - 1` on 16 bits, for a RATE back-off exponent `e` in
/// 5..=10 (the tick bits below the back-off interval `2^e` s); `0xFFFF`
/// for any other `e` (only 5..=10 occur).
#[lower]
pub fn kod_zmask(e: U<4>) -> U<16> {
    mux(
        Bit::from(e == 5),
        U::<16>::from(0x07FFu16),
        mux(
            Bit::from(e == 6),
            U::<16>::from(0x0FFFu16),
            mux(
                Bit::from(e == 7),
                U::<16>::from(0x1FFFu16),
                mux(
                    Bit::from(e == 8),
                    U::<16>::from(0x3FFFu16),
                    mux(Bit::from(e == 9), U::<16>::from(0x7FFFu16), U::<16>::from(0xFFFFu16)),
                ),
            ),
        ),
    )
}

/// `a` is in `fe80::/10`.
#[lower]
pub fn is_ll6(a: U<128>) -> Bit {
    Bit::from(a.slice::<118, 10>() == U::<10>::from(0x3FAu32))
}

#[derive(Trace, Default)]
pub struct NtpClient {
    pub st: Reg<U<4>>,
    /// Slot of the event in progress (reply or request path).
    pub cs: Reg<U<2>>,
    /// Scheduler pointer: the slot `SCAN` examines.
    pub sp: Reg<U<2>>,
    /// Alternates replies and scheduler actions in `SCAN`.
    pub pri: Reg<Bit>,
    // Reply.
    pub hit: Reg<Bit>,
    pub rxts: Reg<U<64>>,
    /// Payload bytes taken, saturating at 48.
    pub pos: Reg<U<6>>,
    /// Bytes 0..15, byte 0 in bits 127..120 once complete.
    pub hdr: Reg<U<128>>,
    pub org: Reg<U<64>>,
    pub t2: Reg<U<64>>,
    pub t3: Reg<U<64>>,
    // Arithmetic.
    pub ra: Reg<U<64>>,
    pub rb: Reg<U<64>>,
    pub rd1: Reg<U<64>>,
    pub rd2: Reg<U<64>>,
    pub plo: Reg<U<48>>,
    pub phi: Reg<U<48>>,
    // Request.
    pub mac: Reg<U<48>>,
    /// Payload bytes sent (0..=47).
    pub k: Reg<U<6>>,
    /// T1 bytes 41..47, shifted left after each.
    pub t1r: Reg<U<64>>,
    /// `(time >> 32) + latency` of the previous cycle.
    pub t1p: Reg<U<64>>,
    // Per-slot KoD state (security finding F9), slot i.
    /// RATE back-off exponent (0: none, else 5..=KOD_POLL_MAX).
    pub kx0: Reg<U<4>>,
    pub kx1: Reg<U<4>>,
    pub kx2: Reg<U<4>>,
    pub kx3: Reg<U<4>>,
    /// Bits of `since[15:0]` that do not count for the back-off interval:
    /// `2^(kx + 6) - 1` (0 when `kx` = 0).
    pub kz0: Reg<U<16>>,
    pub kz1: Reg<U<16>>,
    pub kz2: Reg<U<16>>,
    pub kz3: Reg<U<16>>,
    /// DENY / RSTR received: no more requests (`dead`).
    pub dd0: Reg<Bit>,
    pub dd1: Reg<Bit>,
    pub dd2: Reg<Bit>,
    pub dd3: Reg<Bit>,
    // Per-slot state.
    #[distributed]
    pub m_t1: Mem<U<64>, 4, DefaultClock>,
    /// Fraction (low 32 bits) of the Origin nonce sent: the Transmit
    /// Timestamp of the request is `{m_t1[63:32], m_org}` (F3).
    #[distributed]
    pub m_org: Mem<U<32>, 4, DefaultClock>,
    /// `{last[95:64] (tick), tsent[63:32] (second), rt[31:0] (tick)}`.
    #[distributed]
    pub m_tm: Mem<U<96>, 4, DefaultClock>,
    /// `{arm[10], tries[9:6], burst[5:3], sent[2], pend[1], out[0]}`.
    #[distributed]
    pub m_fl: Mem<U<11>, 4, DefaultClock>,
}

#[lower]
impl
    Unit<
        (
            In<U<128>>,
            In<Bit>,
            In<Bit>,
            In<U<128>>,
            In<Bit>,
            In<Bit>,
            In<U<128>>,
            In<Bit>,
            In<Bit>,
            In<U<128>>,
            In<Bit>,
            In<Bit>,
            In<U<32>>,
            In<U<32>>,
            In<U<32>>,
            In<U<128>>,
            In<U<128>>,
            In<U<128>>,
            In<Bit>,
            In<U<48>>,
            In<U<8>>,
            In<U<8>>,
            In<U<64>>,
            In<U<64>>,
            In<U<96>>,
            In<U<64>>,
            Rx<Meta>,
            Rx<FrameByte>,
            Rx<ArpEntry>,
            Rx<ArpEntry>,
        ),
        (
            Tx<U<32>>,
            Tx<U<32>>,
            Tx<U<128>>,
            Tx<U<128>>,
            Tx<UdpTxReq>,
            Tx<FrameByte>,
            Tx<Udp6TxReq>,
            Tx<FrameByte>,
            Tx<Sample>,
            Tx<U<2>>,
            Out<U<64>>,
        ),
    > for NtpClient
{
    async fn run(
        &mut self,
        (
            up0_addr,
            up0_v6,
            up0_en,
            up1_addr,
            up1_v6,
            up1_en,
            up2_addr,
            up2_v6,
            up2_en,
            up3_addr,
            up3_v6,
            up3_en,
            local_ip4,
            netmask,
            gateway,
            gateway6,
            local_ll,
            local_gua,
            gua_valid,
            local_mac,
            poll_log2,
            precision,
            tx_latency4,
            tx_latency6,
            time,
            rnd,
            meta,
            rep,
            arp_rsp,
            nd_rsp,
        ): (
            In<U<128>>,
            In<Bit>,
            In<Bit>,
            In<U<128>>,
            In<Bit>,
            In<Bit>,
            In<U<128>>,
            In<Bit>,
            In<Bit>,
            In<U<128>>,
            In<Bit>,
            In<Bit>,
            In<U<32>>,
            In<U<32>>,
            In<U<32>>,
            In<U<128>>,
            In<U<128>>,
            In<U<128>>,
            In<Bit>,
            In<U<48>>,
            In<U<8>>,
            In<U<8>>,
            In<U<64>>,
            In<U<64>>,
            In<U<96>>,
            In<U<64>>,
            Rx<Meta>,
            Rx<FrameByte>,
            Rx<ArpEntry>,
            Rx<ArpEntry>,
        ),
        (
            arp_lookup,
            arp_resolve,
            nd_lookup,
            nd_resolve,
            req4,
            pay4,
            req6,
            pay6,
            samples,
            miss,
            rx_stamp,
        ): (
            Tx<U<32>>,
            Tx<U<32>>,
            Tx<U<128>>,
            Tx<U<128>>,
            Tx<UdpTxReq>,
            Tx<FrameByte>,
            Tx<Udp6TxReq>,
            Tx<FrameByte>,
            Tx<Sample>,
            Tx<U<2>>,
            Out<U<64>>,
        ),
    ) {
        loop {
            DefaultClock::rising().await;
            let _ = local_mac.get();
            rx_stamp.set(self.rxts.get());
            let st = self.st.get();
            let cs = self.cs.get();
            let pos = self.pos.get();
            let hdr = self.hdr.get();
            let k = self.k.get();
            let now = time.get();
            let now_s = now.slice::<64, 32>();
            let now_t = now.slice::<58, 32>();
            let is_scan = Bit::from(st == S_SCAN);
            let is_rx = Bit::from(st == S_RX);
            let is_chk = Bit::from(st == S_CHK);
            let is_c2 = Bit::from(st == S_C2);
            let is_smp = Bit::from(st == S_SMP);
            let is_miss = Bit::from(st == S_MISS);
            let is_lkp = Bit::from(st == S_LKP);
            let is_lwt = Bit::from(st == S_LWT);
            let is_req = Bit::from(st == S_REQ);
            let is_pay = Bit::from(st == S_PAY);

            // ------------------------------------------- slot cs wires
            let a0 = up0_addr.get();
            let a1 = up1_addr.get();
            let a2 = up2_addr.get();
            let a3 = up3_addr.get();
            let v0 = up0_v6.get();
            let v1 = up1_v6.get();
            let v2 = up2_v6.get();
            let v3 = up3_v6.get();
            let e0 = up0_en.get();
            let e1 = up1_en.get();
            let e2 = up2_en.get();
            let e3 = up3_en.get();
            // Slot whose wires and state are read / written this cycle.
            let ix = mux(is_scan, self.sp.get(), cs);
            let c0 = ix.bit(0);
            let c1 = ix.bit(1);
            let s_addr = mux(c1, mux(c0, a3, a2), mux(c0, a1, a0));
            let s_v6 = mux(c1, mux(c0, v3, v2), mux(c0, v1, v0));
            let s_en = mux(c1, mux(c0, e3, e2), mux(c0, e1, e0));
            // Per-slot KoD state of slot ix.
            let kx_s = mux(
                c1,
                mux(c0, self.kx3.get(), self.kx2.get()),
                mux(c0, self.kx1.get(), self.kx0.get()),
            );
            let kz_s = mux(
                c1,
                mux(c0, self.kz3.get(), self.kz2.get()),
                mux(c0, self.kz1.get(), self.kz0.get()),
            );
            let dd_s = mux(
                c1,
                mux(c0, self.dd3.get(), self.dd2.get()),
                mux(c0, self.dd1.get(), self.dd0.get()),
            );
            let ix0 = Bit::from(ix == 0);
            let ix1 = Bit::from(ix == 1);
            let ix2 = Bit::from(ix == 2);
            let ix3 = Bit::from(ix == 3);

            let fl = self.m_fl.read(ix);
            let tm = self.m_tm.read(ix);
            let t1s = self.m_t1.read(ix);
            let orgs = self.m_org.read(ix);
            let f_out = fl.bit(0);
            let f_pend = fl.bit(1);
            let f_sent = fl.bit(2);
            let f_burst = fl.slice::<3, 3>();
            let f_tries = fl.slice::<6, 4>();
            let m_last = tm.slice::<64, 32>();
            let f_arm = fl.bit(10);
            let m_tsent = tm.slice::<32, 32>();
            let m_rt = tm.slice::<0, 32>();

            // ------------------------------------------- next hop
            let ip4 = s_addr.slice::<0, 32>();
            let msk = netmask.get();
            let on4 = Bit::from((ip4 & msk) == (local_ip4.get() & msk));
            let nh4 = mux(on4, ip4, gateway.get());
            let gua = local_gua.get();
            let gv = gua_valid.get();
            let ll = is_ll6(s_addr);
            let same64 = Bit::from(s_addr.slice::<64, 64>() == gua.slice::<64, 64>());
            let on6 = ll | (gv & same64);
            let nh6 = mux(on6, s_addr, gateway6.get());
            let src6 = mux(ll | !gv, local_ll.get(), gua);

            // ------------------------------------------- scheduler (SCAN)
            let dl_due = Bit::from((now_s - m_tsent) >= U::<32>::from(DEADLINE_S));
            let in_burst = Bit::from(f_burst < U::<3>::from(IBURST_COUNT));
            let ival = mux(
                in_burst,
                U::<32>::from(IBURST_GAP_S),
                poll_secs(poll_log2.get()),
            );
            // Ticks since `last` (the arming or the previous poll start).
            let since = now_t - m_last;
            // First poll: `ix * 16` ticks (ix * 2^-2 s) after the arming.
            let stagger = ix.zext::<32>() << 4u32;
            // RATE back-off (F9): also `since >= 2^(kx + 6)` ticks, i.e. a
            // set bit of `since` outside the zero mask `kz` (bits 16..31 are
            // never in it); with kx = 0 (kz = 0) implied by the test above.
            let due_k = Bit::from(since.slice::<16, 16>() != U::<16>::from(0u8))
                | Bit::from(
                    (since.slice::<0, 16>() & (kz_s ^ U::<16>::from(0xFFFFu16)))
                        != U::<16>::from(0u8),
                );
            let due = mux(
                Bit::from(f_burst == U::<3>::from(0u8)),
                f_arm & Bit::from(since >= stagger),
                Bit::from(since >= (ival << 6u32)) & due_k,
            );
            let rt_ok = Bit::from(f_tries == U::<4>::from(0u8))
                | Bit::from((now_t - m_rt) >= U::<32>::from(RETRY_TICKS));
            let sc_dis = !s_en & Bit::from(fl != U::<11>::from(0u8));
            let sc_arm = s_en & !f_out & !f_pend & Bit::from(f_burst == U::<3>::from(0u8)) & !f_arm;
            let sc_dl = s_en & f_out & dl_due;
            let sc_fire = s_en & !f_out & !f_pend & due;
            let sc_lkp = s_en & !f_out & f_pend & rt_ok;
            let sc_act = sc_dis | sc_dl | sc_arm | sc_fire | sc_lkp;

            let have_m = Bit::from(meta.peek().is_some());
            let take_m = is_scan & have_m & (!sc_act | self.pri.get());
            let _ = meta.recv_if(take_m);
            let mh = meta.head();
            let do_sc = is_scan & !take_m;

            // Slot match of the Meta (lowest index).
            let fam = mh.family;
            let src = mh.src;
            let p_ok = Bit::from(mh.sport == U::<16>::from(NTP_PORT));
            let mt0 = e0
                & Bit::from(v0 == fam)
                & mux(
                    v0,
                    Bit::from(a0 == src),
                    Bit::from(a0.slice::<0, 32>() == src.slice::<0, 32>()),
                );
            let mt1 = e1
                & Bit::from(v1 == fam)
                & mux(
                    v1,
                    Bit::from(a1 == src),
                    Bit::from(a1.slice::<0, 32>() == src.slice::<0, 32>()),
                );
            let mt2 = e2
                & Bit::from(v2 == fam)
                & mux(
                    v2,
                    Bit::from(a2 == src),
                    Bit::from(a2.slice::<0, 32>() == src.slice::<0, 32>()),
                );
            let mt3 = e3
                & Bit::from(v3 == fam)
                & mux(
                    v3,
                    Bit::from(a3 == src),
                    Bit::from(a3.slice::<0, 32>() == src.slice::<0, 32>()),
                );
            let m_hit = p_ok & (mt0 | mt1 | mt2 | mt3);
            let m_idx = mux(
                mt0,
                U::<2>::from(0u8),
                mux(
                    mt1,
                    U::<2>::from(1u8),
                    mux(mt2, U::<2>::from(2u8), U::<2>::from(3u8)),
                ),
            );

            // ------------------------------------------- RX
            let have_b = Bit::from(rep.peek().is_some());
            let take_b = is_rx & have_b;
            let _ = rep.recv_if(take_b);
            let b = rep.head();
            let data = b.data;
            let rx_done = take_b & b.last;
            let rx_ok = !b.err & Bit::from(pos >= 47) & self.hit.get();
            let d64 = data.zext::<64>();

            // ------------------------------------------- CHK
            let r_b0 = hdr.slice::<120, 8>();
            let r_mode = r_b0.slice::<0, 3>();
            let r_vn = r_b0.slice::<3, 3>();
            let r_li = r_b0.slice::<6, 2>();
            let r_st = hdr.slice::<112, 8>();
            let r_prec = hdr.slice::<96, 8>();
            // Origin must be the nonce sent: T1's seconds, `m_org` fraction (F3).
            let r_org = self.org.get();
            let auth = f_out
                & Bit::from(r_mode == 4)
                & (Bit::from(r_vn == 3) | Bit::from(r_vn == 4))
                & Bit::from(r_org.slice::<32, 32>() == t1s.slice::<32, 32>())
                & Bit::from(r_org.slice::<0, 32>() == orgs);
            let unusable = Bit::from(r_st == U::<8>::from(0u8))
                | Bit::from(r_st >= U::<8>::from(16u8))
                | Bit::from(r_li == 3);
            // Kiss-o'-Death (F9): RATE backs off, DENY / RSTR kill the slot.
            let r_rid = hdr.slice::<0, 32>();
            let kod = Bit::from(r_st == U::<8>::from(0u8));
            let k_rate = kod & Bit::from(r_rid == U::<32>::from(KOD_RATE));
            let k_deny = kod
                & (Bit::from(r_rid == U::<32>::from(KOD_DENY))
                    | Bit::from(r_rid == U::<32>::from(KOD_RSTR)));
            let w_auth_c = is_chk & auth;
            let w_rate = w_auth_c & k_rate;
            let w_deny = w_auth_c & k_deny;
            let w_kod = w_rate | w_deny;
            let pcl = poll_clamp(poll_log2.get());
            let kxw = kx_s.zext::<8>();
            let ecur = mux(kxw > pcl, kxw, pcl);
            let bo = w_rate & Bit::from(ecur < U::<8>::from(KOD_POLL_MAX));
            let enew = (ecur + U::<8>::from(1u8)).slice::<0, 4>();
            let kz_new = kod_zmask(enew);
            let rxts = self.rxts.get();
            let t2 = self.t2.get();
            let t3 = self.t3.get();
            let c_a = t2 - t1s;
            let c_b = t3 - rxts;
            let c_d1 = rxts - t1s;
            let c_d2 = t3 - t2;

            // ------------------------------------------- C2
            let ra = self.ra.get();
            let rb = self.rb.get();
            let rd1 = self.rd1.get();
            let rd2 = self.rd2.get();
            let sum = ra.sext::<66>() + rb.sext::<66>();
            let off = sum.slice::<1, 64>();
            let dd = rd1.sext::<66>() - rd2.sext::<66>();
            let dly = mux(dd.bit(65), U::<64>::from(0u8), dd.slice::<0, 64>());
            let iv = mux(rd1.bit(63), U::<64>::from(0u8), rd1);
            let p_lo = iv.slice::<0, 32>().zext::<48>() * U::<48>::from(PHI_Q32);
            let p_hi = iv.slice::<32, 32>().zext::<48>() * U::<48>::from(PHI_Q32);

            // ------------------------------------------- SMP
            // (prod + 2^32 - 1) >> 32 from the two partial products (exact:
            // the high one has 32 zero low bits once shifted).
            let lo_up = ((self.plo.get().zext::<49>() + U::<49>::from(0xFFFF_FFFFu32)) >> 32u32).zext::<64>();
            let phi = self.phi.get().zext::<64>() + lo_up;
            let disp = (prec_pow(r_prec) + prec_pow(precision.get())) + phi;
            let send_s = is_smp & samples.ready();
            if send_s.to_bool() {
                samples.send(Sample {
                    peer: cs,
                    offset: ra,
                    delay: rb,
                    disp,
                    t4: rxts,
                    stratum: r_st,
                    leap: r_li,
                    ref_id: hdr.slice::<0, 32>(),
                    root_delay: hdr.slice::<64, 32>(),
                    root_disp: hdr.slice::<32, 32>(),
                });
            }

            // ------------------------------------------- MISS
            let send_x = is_miss & miss.ready();
            if send_x.to_bool() {
                miss.send(cs);
            }

            // ------------------------------------------- LKP / LWT
            let send_l4 = is_lkp & !s_v6 & arp_lookup.ready();
            let send_l6 = is_lkp & s_v6 & nd_lookup.ready();
            if send_l4.to_bool() {
                arp_lookup.send(nh4);
            }
            if send_l6.to_bool() {
                nd_lookup.send(nh6);
            }
            let have_r4 = Bit::from(arp_rsp.peek().is_some());
            let have_r6 = Bit::from(nd_rsp.peek().is_some());
            let take_r4 = is_lwt & !s_v6 & have_r4;
            let take_r6 = is_lwt & s_v6 & have_r6;
            let _ = arp_rsp.recv_if(take_r4);
            let _ = nd_rsp.recv_if(take_r6);
            let r4 = arp_rsp.head();
            let r6 = nd_rsp.head();
            let got = take_r4 | take_r6;
            let found = mux(s_v6, r6.found, r4.found);
            let rmac = mux(s_v6, r6.mac, r4.mac);
            let fail = got & !found;
            let tries_n = f_tries + U::<4>::from(1u8);
            let give_up = Bit::from(tries_n >= U::<4>::from(RES_TRIES));
            let send_v4 = fail & !s_v6 & arp_resolve.ready();
            let send_v6 = fail & s_v6 & nd_resolve.ready();
            if send_v4.to_bool() {
                arp_resolve.send(nh4);
            }
            if send_v6.to_bool() {
                nd_resolve.send(nh6);
            }

            // ------------------------------------------- REQ / PAY
            let mac = self.mac.get();
            let send_q4 = is_req & !s_v6 & req4.ready();
            let send_q6 = is_req & s_v6 & req6.ready();
            if send_q4.to_bool() {
                req4.send(UdpTxReq {
                    dst_mac: mac,
                    dst_ip: ip4,
                    sport: U::<16>::from(NTP_PORT),
                    dport: U::<16>::from(NTP_PORT),
                    len: U::<16>::from(REQ_LEN),
                });
            }
            if send_q6.to_bool() {
                req6.send(Udp6TxReq {
                    dst_mac: mac,
                    dst_ip: s_addr,
                    src_ip: src6,
                    sport: U::<16>::from(NTP_PORT),
                    dport: U::<16>::from(NTP_PORT),
                    len: U::<16>::from(REQ_LEN),
                });
            }
            let t1p = self.t1p.get();
            let t1r = self.t1r.get();
            // The Transmit Timestamp sent (Origin nonce, F3): T1's seconds,
            // T1's fraction XOR 32 random bits sampled in cycle S.
            let nonce_lo = t1p.slice::<0, 32>() ^ rnd.get().slice::<0, 32>();
            let nonce = t1p.slice::<32, 32>().concat::<32, 64>(nonce_lo);
            let at40 = Bit::from(k == 40);
            let ts_b = mux(at40, t1p.slice::<56, 8>(), t1r.slice::<56, 8>());
            let pb = mux(
                Bit::from(k >= 40),
                ts_b,
                mux(
                    Bit::from(k == 0),
                    U::<8>::from(REQ_B0),
                    mux(
                        Bit::from(k == 2),
                        poll_log2.get(),
                        mux(Bit::from(k == 3), precision.get(), U::<8>::from(0u8)),
                    ),
                ),
            );
            let p_last = Bit::from(k == 47);
            let send_p4 = is_pay & !s_v6 & pay4.ready();
            let send_p6 = is_pay & s_v6 & pay6.ready();
            let fb = FrameByte {
                data: pb,
                last: p_last,
                err: Bit::Zero,
            };
            if send_p4.to_bool() {
                pay4.send(fb);
            }
            if send_p6.to_bool() {
                pay6.send(fb);
            }
            let send_p = send_p4 | send_p6;

            // ------------------------------------------- per-slot writes
            let burst_n = mux(in_burst, f_burst + U::<3>::from(1u8), f_burst);
            // fire: pend 1, tries 0, burst + 1, arm 0
            // (a dead slot: pend stays 0, the poll is a miss at once)
            let fl_fire = (burst_n.zext::<11>() << 3u32)
                | (fl & U::<11>::from(0b00000000101u16))
                | mux(dd_s, U::<11>::from(0u8), U::<11>::from(0b10u8));
            // deadline / authentic reply: out 0
            let fl_noout = fl & U::<11>::from(0b11111111110u16);
            // authentic RATE / DENY / RSTR: out 0 and the iburst ends (burst 4)
            let fl_kod = (fl_noout & U::<11>::from(0b11111000111u16)) | U::<11>::from(0b100000u8);
            // lookup failed: tries + 1, or abandon (pend 0, tries 0)
            let fl_fail = mux(
                give_up,
                fl & U::<11>::from(0b00000111101u16),
                (fl & U::<11>::from(0b00000111111u16)) | (tries_n.zext::<11>() << 6u32),
            );
            // request sent: out 1, pend 0, sent 1, tries 0
            let fl_sent = (fl & U::<11>::from(0b00000111000u16)) | U::<11>::from(0b101u8);
            let w_dis = do_sc & sc_dis;
            let w_arm = do_sc & sc_arm;
            let w_dl = do_sc & !sc_dis & sc_dl;
            let w_fire = do_sc & !sc_dis & !sc_dl & !sc_arm & sc_fire;
            let w_auth = is_chk & auth;
            let w_sent = send_p & p_last;
            let fl_we = w_dis | w_dl | w_arm | w_fire | w_auth | fail | w_sent;
            let fl_wd = mux(
                w_dis,
                U::<11>::from(0u8),
                mux(
                    w_fire,
                    fl_fire,
                    mux(
                        w_arm,
                        fl | U::<11>::from(0b10000000000u16),
                        mux(
                            fail,
                            fl_fail,
                            mux(w_sent, fl_sent, mux(w_kod, fl_kod, fl_noout)),
                        ),
                    ),
                ),
            );
            let w_t40 = send_p & at40;
            let tm_we = w_arm | w_fire | fail | w_t40;
            let tm_fire = (now_t.zext::<96>() << 64u32) | tm.slice::<0, 64>().zext::<96>();
            let tm_fail = (tm.slice::<32, 64>().zext::<96>() << 32u32) | now_t.zext::<96>();
            let tm_t40 =
                (m_last.zext::<96>() << 64u32) | (now_s.zext::<96>() << 32u32) | m_rt.zext::<96>();
            let tm_wd = mux(w_arm | w_fire, tm_fire, mux(fail, tm_fail, tm_t40));

            // ------------------------------------------- next state
            let sc_next = mux(
                sc_dis | (sc_fire & !dd_s),
                U::<4>::from(S_SCAN),
                mux(
                    sc_dl | sc_fire,
                    U::<4>::from(S_MISS),
                    mux(sc_lkp, U::<4>::from(S_LKP), U::<4>::from(S_SCAN)),
                ),
            );
            let lwt_next = mux(
                found,
                U::<4>::from(S_REQ),
                mux(give_up & f_sent, U::<4>::from(S_MISS), U::<4>::from(S_SCAN)),
            );

            with!(self <= {
                t1p: now.slice::<32, 64>() + mux(s_v6, tx_latency6.get(), tx_latency4.get()),
                is_scan ? pri: !self.pri.get(),
                take_m ? {
                    st: U::<4>::from(S_RX),
                    cs: m_idx,
                    hit: m_hit,
                    rxts: mh.rx_ts,
                    pos: U::<6>::from(0u8),
                },
                do_sc ? {
                    st: sc_next,
                    cs: ix,
                },
                is_scan & !sc_act ? sp: ix + U::<2>::from(1u8),
                take_b & Bit::from(pos < 48) ? pos: pos + U::<6>::from(1u8),
                take_b & Bit::from(pos < 16) ? hdr: (hdr << 8u32) | data.zext::<128>(),
                take_b & Bit::from(pos >= 24) & Bit::from(pos < 32) ? org: (self.org.get() << 8u32) | d64,
                take_b & Bit::from(pos >= 32) & Bit::from(pos < 40) ? t2: (t2 << 8u32) | d64,
                take_b & Bit::from(pos >= 40) & Bit::from(pos < 48) ? t3: (t3 << 8u32) | d64,
                rx_done ? st: mux(rx_ok, U::<4>::from(S_CHK), U::<4>::from(S_SCAN)),
                is_chk ? {
                    st: mux(auth, mux(unusable, U::<4>::from(S_MISS), U::<4>::from(S_C2)), U::<4>::from(S_SCAN)),
                    ra: c_a,
                    rb: c_b,
                    rd1: c_d1,
                    rd2: c_d2,
                },
                is_c2 ? {
                    st: U::<4>::from(S_SMP),
                    ra: off,
                    rb: dly,
                    plo: p_lo,
                    phi: p_hi,
                },
                send_s | send_x ? st: U::<4>::from(S_SCAN),
                send_l4 | send_l6 ? st: U::<4>::from(S_LWT),
                got ? {
                    st: lwt_next,
                    mac: rmac,
                },
                send_q4 | send_q6 ? {
                    st: U::<4>::from(S_PAY),
                    k: U::<6>::from(0u8),
                },
                send_p ? k: k + U::<6>::from(1u8),
                w_t40 ? t1r: nonce << 8u32,
                send_p & Bit::from(k > 40) ? t1r: t1r << 8u32,
                w_sent ? st: U::<4>::from(S_SCAN),
                fl_we ? m_fl.at(ix): fl_wd,
                tm_we ? m_tm.at(ix): tm_wd,
                w_t40 ? m_t1.at(ix): t1p,
                w_t40 ? m_org.at(ix): nonce_lo,
                // Per-slot KoD state: cleared when the slot is disabled.
                w_dis & ix0 ? { kx0: U::<4>::from(0u8), kz0: U::<16>::from(0u8), dd0: Bit::Zero },
                w_dis & ix1 ? { kx1: U::<4>::from(0u8), kz1: U::<16>::from(0u8), dd1: Bit::Zero },
                w_dis & ix2 ? { kx2: U::<4>::from(0u8), kz2: U::<16>::from(0u8), dd2: Bit::Zero },
                w_dis & ix3 ? { kx3: U::<4>::from(0u8), kz3: U::<16>::from(0u8), dd3: Bit::Zero },
                bo & ix0 ? { kx0: enew, kz0: kz_new },
                bo & ix1 ? { kx1: enew, kz1: kz_new },
                bo & ix2 ? { kx2: enew, kz2: kz_new },
                bo & ix3 ? { kx3: enew, kz3: kz_new },
                w_deny & ix0 ? dd0: Bit::One,
                w_deny & ix1 ? dd1: Bit::One,
                w_deny & ix2 ? dd2: Bit::One,
                w_deny & ix3 ? dd3: Bit::One,
            });
        }
    }
}
