// SPDX-License-Identifier: Apache-2.0
//! Shared types of the NTP client side (clock filter, selection, servo, sync
//! state). Written once by the orchestrator so the units developed in parallel
//! agree on their interfaces. Do not change a field without telling the
//! orchestrator (add a note in docs/questions.md).
//!
//! # Fixed-point conventions
//! * Times and durations are 64-bit **32.32** fixed point seconds (`Ts` of
//!   stream.rs: 32 integer bits of seconds, 32 fraction bits).
//! * **Offsets are signed**: two's complement in a `U<64>` (range about
//!   ±2^31 s, resolution 2^-32 s = 0.23 ns).
//! * Delays, dispersions, jitters and root distances are unsigned `U<64>`
//!   (32.32), except `root_delay` / `root_disp`, which are the NTP short
//!   format **16.16** (`U<32>`), as on the wire (RFC 5905 §6).
//! * Up to [`PEERS`] upstream servers, indexed `0..PEERS` (`U<2>`).
use txhdl::types::{Bit, U};
use txhdl::{Transaction as TransactionDerive, Value as ValueDerive};

/// Number of upstream peers (matches `ConfigSet::UPSTREAM` in config.rs).
pub const PEERS: usize = 4;
/// Maximum clock frequency tolerance PHI, 15 ppm (RFC 5905 §7.3), in
/// 2^-32 s per second: 15e-6 * 2^32 = 64424.5, rounded to 64425.
pub const PHI_Q32: u64 = 64_425;
/// Stratum above which a source is unusable (RFC 5905 MAXSTRAT).
pub const MAXSTRAT: u8 = 16;

/// One measurement against one peer: the result of one request/reply
/// exchange (RFC 5905 §8: T1 origin, T2 receive, T3 transmit, T4 arrival).
#[derive(TransactionDerive, ValueDerive, Clone, Copy, Default, Debug)]
pub struct Sample {
    pub peer: U<2>,
    /// theta = ((T2 - T1) + (T3 - T4)) / 2, signed 32.32.
    pub offset: U<64>,
    /// delta = (T4 - T1) - (T3 - T2), unsigned 32.32.
    pub delay: U<64>,
    /// Dispersion of the sample: the peer's precision plus our own precision
    /// plus PHI times the measurement interval (computed by the client).
    pub disp: U<64>,
    /// Our clock when the reply arrived (T4), used to age the sample.
    pub t4: U<64>,
    /// Peer's stratum, leap indicator (0..=3, 3 = unsynchronised), reference
    /// id, root delay and root dispersion (16.16), poll exponent, copied from
    /// the reply.
    pub stratum: U<8>,
    pub leap: U<2>,
    pub ref_id: U<32>,
    pub root_delay: U<32>,
    pub root_disp: U<32>,
}

/// The filtered state of one peer, after the clock filter (RFC 5905 §10).
#[derive(TransactionDerive, ValueDerive, Clone, Copy, Default, Debug)]
pub struct PeerStat {
    pub peer: U<2>,
    /// At least one sample is in the filter and the peer is usable
    /// (reach != 0, stratum 1..15, not leap 3).
    pub valid: Bit,
    /// Offset and delay of the minimum-delay sample of the filter.
    pub offset: U<64>,
    pub delay: U<64>,
    /// Filter dispersion and jitter (RMS of offset differences, RFC 5905 §10).
    pub disp: U<64>,
    pub jitter: U<64>,
    /// Reachability register: last 8 polls, 1 = answered.
    pub reach: U<8>,
    /// Copied from the peer's last reply.
    pub stratum: U<8>,
    pub leap: U<2>,
    pub ref_id: U<32>,
    pub root_delay: U<32>,
    pub root_disp: U<32>,
    /// Time of the last accepted sample (our clock), 32.32.
    pub t4: U<64>,
}

/// The system peer chosen by selection and clustering (RFC 5905 §11.2),
/// with the combined offset the servo must remove.
#[derive(TransactionDerive, ValueDerive, Clone, Copy, Default, Debug)]
pub struct Selected {
    /// A system peer exists (at least one survivor, majority intersection).
    pub valid: Bit,
    pub peer: U<2>,
    /// Combined offset (signed 32.32) and jitter (system jitter).
    pub offset: U<64>,
    pub jitter: U<64>,
    /// Root distance of the system peer (32.32).
    pub root_dist: U<64>,
    /// Of the system peer: stratum, leap, ref id, root delay/dispersion
    /// (16.16), delay (32.32) and filter dispersion (32.32), and the time
    /// (our clock) of its last sample.
    pub stratum: U<8>,
    pub leap: U<2>,
    pub ref_id: U<32>,
    pub root_delay: U<32>,
    pub root_disp: U<32>,
    pub delay: U<64>,
    pub disp: U<64>,
    pub t4: U<64>,
}
