// SPDX-License-Identifier: Apache-2.0
//! Ethernet CRC-32 (IEEE 802.3), eight bits per cycle (task T1.3).
//!
//! Convention, the one the wire uses: the polynomial is 0xEDB88320, the
//! reflected form of 0x04C11DB7, so every byte enters least significant
//! bit first and the register shifts right.
//!
//! * The register starts at [`INIT`] = 0xFFFFFFFF before a frame.
//! * Each byte of the frame goes through [`crc32_step`] (one byte per
//!   cycle; the eight bit steps are combinational).
//! * The FCS is the complement of the register after the last payload
//!   byte, sent least significant byte first.
//! * Residue: after the payload and its FCS, the register holds
//!   [`RESIDUE`] = 0xDEBB20E3, before any complement. That is the same
//!   constant as 0xC704DD7B read most significant bit first (its bit
//!   reversal), which is the form some references print. The complement
//!   of [`RESIDUE`] is 0x2144DF1C, the value `zlib.crc32` returns on a
//!   frame with its FCS appended.
//!
//! [`FrameCrc`] consumes a frame byte stream (T0.1 contract, FCS not
//! stripped) and sends one [`Crc`] per frame when it sees the byte with
//! `last`: the complemented CRC of all bytes of the frame, and whether the
//! last four bytes were a correct FCS.
use crate::stream::FrameByte;
use txhdl::comp::{mux, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{check, cover, lower, Trace, Transaction as TransactionDerive, Value as ValueDerive};

/// Reflected polynomial of IEEE 802.3 CRC-32.
pub const POLY: u32 = 0xEDB8_8320;
/// Register value at the start of a frame, and the final complement.
pub const INIT: u32 = 0xFFFF_FFFF;
/// Register value after a frame and its FCS, before the final complement.
pub const RESIDUE: u32 = 0xDEBB_20E3;

/// One bit step of the reflected CRC: shift right, and fold the polynomial
/// in when the bit leaving the bottom is one.
#[lower]
fn crc_bit(c: U<32>) -> U<32> {
    mux(c.bit(0), (c >> 1u32) ^ U::<32>::from(POLY), c >> 1u32)
}

/// One byte of the reflected CRC-32: the register exclusive-or the byte,
/// then eight bit steps. `crc32_step(INIT, b)` is the CRC of the one byte
/// `b` before the complement, and the frame's CRC is the complement of the
/// register after its last byte.
#[lower]
pub fn crc32_step(crc: U<32>, byte: U<8>) -> U<32> {
    let x0 = crc ^ byte.zext::<32>();
    let x1 = crc_bit(x0);
    let x2 = crc_bit(x1);
    let x3 = crc_bit(x2);
    let x4 = crc_bit(x3);
    let x5 = crc_bit(x4);
    let x6 = crc_bit(x5);
    let x7 = crc_bit(x6);
    crc_bit(x7)
}

/// The CRC of one frame, sent when its last byte is seen.
#[derive(TransactionDerive, ValueDerive, Clone, Copy, Default, Debug)]
pub struct Crc {
    /// Complemented CRC of every byte the frame carried, FCS included when
    /// the stream carries it (then it is the residue complement 0x2144DF1C).
    pub value: U<32>,
    /// 1 when the last four bytes of the frame were a correct FCS.
    pub residue_ok: Bit,
}

/// Running CRC over a frame byte stream, one byte per cycle.
#[derive(Trace)]
pub struct FrameCrc {
    /// The register: reflected, before the final complement.
    pub crc: Reg<U<32>>,
    /// Set by the byte that ended the previous frame.
    pub after_last: Reg<Bit>,
}

impl Default for FrameCrc {
    fn default() -> Self {
        FrameCrc {
            crc: Reg::new(INIT),
            after_last: Reg::new(Bit::Zero),
        }
    }
}

#[lower]
impl Unit<Rx<FrameByte>, Tx<Crc>> for FrameCrc {
    async fn run(&mut self, bytes: Rx<FrameByte>, out: Tx<Crc>) {
        loop {
            // Right after a frame's last byte the register is back at INIT.
            // Stated before the wait, so it holds in every cycle and not only
            // in those where a byte is taken: the induction needs that.
            let seen = self.after_last.get();
            let at_init = Bit::from(self.crc.get() == U::<32>::from(INIT));
            check!(!seen | at_init, "the CRC register starts a frame at INIT");
            let b = bytes.wait().await;
            let crc = self.crc.get();
            let next = crc32_step(crc, b.data);
            let good = Bit::from(next == U::<32>::from(RESIDUE));
            cover!(b.last & good, "a frame ends with a good FCS");
            cover!(b.last & !good, "a frame ends with a bad FCS");
            if b.last.to_bool() {
                let value = next ^ U::<32>::from(INIT);
                out.send(Crc {
                    value,
                    residue_ok: good,
                });
                self.crc.set(INIT);
            } else {
                self.crc.set(next);
            }
            self.after_last.set(b.last);
        }
    }
}
