// SPDX-License-Identifier: Apache-2.0
//! Free-running NTP clock: a 96-bit accumulator, 32.64 fixed point: 32.32 NTP time in the top
//! 64 bits (acc >> 32) plus 32 sub-LSB bits so a 10 ns tick (100 MHz) has no rounding
//! drift. `INCR` is 2^64 / 100e6; the servo will later replace it by a
//! register trimmed in ppb.
use txhdl::comp::{Clock, DefaultClock, Out, Reg, Unit};
use txhdl::types::{Bit, U};
use txhdl::{lower, when, Trace};

pub const CLK_HZ: u128 = 100_000_000;
/// 2^64 / CLK_HZ: the per-cycle increment of the accumulator.
pub const INCR: u128 = (1u128 << 64) / CLK_HZ;

#[derive(Default, Trace)]
pub struct NtpClock {
    pub acc: Reg<U<96>>,
}

#[lower]
impl Unit for NtpClock {
    async fn run(&mut self, _inputs: (), time: Out<U<96>>) {
        loop {
            DefaultClock::rising().await;
            let on = Bit::One;
            when!(on => self { acc: self.acc + INCR });
            time.set(self.acc);
        }
    }
}

/// NTP 32.32 seconds from the accumulator.
pub fn ntp_time(acc: u128) -> u64 {
    (acc >> 32) as u64
}
