// SPDX-License-Identifier: Apache-2.0
//! Interface contract between every module of the NTP server (task T0.1).
//!
//! Modules talk only through the types below, on TxHDL channels
//! (`Tx<T>`/`Rx<T>`: `valid`/`ready` handshake, registered). No other
//! ad hoc port crosses a module boundary.
//!
//! # Frame stream: `Tx<FrameByte>` / `Rx<FrameByte>`
//!
//! One transaction is one byte of an Ethernet frame, destination MAC first.
//! Contract (the formal monitor of task F0.5 states these as `check!`):
//!
//! 1. A frame is a run of transactions ending with exactly one with
//!    `last = 1`. Bytes of the next frame follow only after it.
//! 2. `err` is meaningful only together with `last`. `last & err` means
//!    the frame is bad (RX: `rx_er`, bad FCS, overrun; TX: aborted) and the
//!    consumer must discard it. `err` is 0 on every other byte.
//! 3. A frame has at most [`MAX_FRAME`] bytes (1518 on the wire with FCS), at least 1.
//! 4. Which bytes a frame carries depends on the point of the pipeline:
//!    * `mii_rx` output (domain `ClkRx`): everything after the SFD up to the
//!      end of `rx_dv`, **FCS included**; `err` on `last` if `rx_er` was seen
//!      or a nibble was left over. The FCS is checked and stripped by
//!      `fcs_check` (T1.12): its output is destination MAC through last
//!      payload byte, `err` set on `last` when the FCS is bad.
//!    * Ethernet builder output (T1.6): destination MAC through last payload
//!      byte. `eth_tx` (T1.6) pads to 60 bytes and appends the FCS, so the
//!      stream entering `mii_tx` (domain `ClkTx`) is the **complete frame:
//!      padded, FCS included**. `mii_tx` adds only preamble, SFD and the
//!      inter-frame gap.
//!    * between layers above Ethernet (IP, UDP, ...) the stream carries the
//!      payload of that layer only, with the layer's header already parsed
//!      into [`Meta`] (see below).
//! 5. The consumer may stall (`ready = 0`) at any time **except** where the
//!    producer is a wire (MII RX): the receiver side of the MAC must be able
//!    to absorb a whole frame (FIFO) or flag it with `err`.
//!
//! # Netlist note (TxHDL)
//!
//! In the generated Verilog a sender's `valid` waits on its `ready` and a
//! receiver's `ready` waits on its `valid`, so two modules must never be
//! wired port to port: join them through a registered channel
//! (`<top>_txhdl_chan` or a FIFO whose `ready` is a register). In the Rust
//! simulation `chan()` already is one. `FrameByte` is packed as
//! `{data[7:0], last, err}` (`err` is bit 0, `last` bit 1).
//!
//! # Timestamps
//!
//! [`Ts`] is NTP time, 32.32 fixed point seconds since 1900 (RFC 5905 §6),
//! i.e. `acc >> 32` of [`crate::clock::NtpClock`]. The MAC captures one
//! timestamp per frame at the SFD, on a channel `Tx<Ts>` separate from the
//! byte stream: exactly one `Ts` per frame, delivered no later than the
//! frame's first byte reaches the filter (RX), or after the frame's last
//! byte has been accepted by the MAC transmitter (TX).
//!
//! # Packet metadata
//!
//! A layer that has parsed headers presents the payload on a
//! `Tx<FrameByte>` and the associated [`Meta`] on a parallel `Tx<Meta>`,
//! one `Meta` per packet, delivered before the payload's first byte.
use txhdl::comp::{Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{lower, Trace};
use txhdl::{Transaction as TransactionDerive, Value as ValueDerive};

/// Largest frame without FCS: 14-byte header + 1500 payload.
pub const MAX_FRAME: usize = 1514;

/// NTP time: 32.32 fixed point seconds since 1900-01-01.
pub type Ts = U<64>;

/// One byte of a frame; see the module documentation for the contract.
#[derive(TransactionDerive, ValueDerive, Clone, Copy, Default, Debug)]
pub struct FrameByte {
    pub data: U<8>,
    /// Last byte of the frame.
    pub last: Bit,
    /// With `last`: the frame is bad and must be discarded.
    pub err: Bit,
}

impl FrameByte {
    pub fn new(data: u8, last: bool, err: bool) -> Self {
        FrameByte {
            data: U::<8>::from(data),
            last: Bit::from_bool(last),
            err: Bit::from_bool(err),
        }
    }
}

/// IP address family of a packet.
pub const FAMILY_V4: bool = false;
pub const FAMILY_V6: bool = true;

/// Parsed headers of one packet, presented once per packet.
#[derive(TransactionDerive, ValueDerive, Clone, Copy, Default, Debug)]
pub struct Meta {
    /// 0 = IPv4, 1 = IPv6.
    pub family: Bit,
    /// Source address; IPv4 in the low 32 bits, upper bits zero.
    pub src: U<128>,
    /// Destination address, same layout.
    pub dst: U<128>,
    pub sport: U<16>,
    pub dport: U<16>,
    /// Length of the payload that follows, in bytes.
    pub len: U<16>,
    /// Source MAC of the frame that carried the packet: where a reply goes
    /// (the server keeps no ARP/NDP table to answer a request).
    pub eth_src: U<48>,
    /// Hardware receive timestamp of the frame that carried the packet.
    pub rx_ts: U<64>,
}

/// Example consumer: counts the bytes of each frame and reports the length
/// of each on a second channel when `last` is seen. Used by tests as the
/// reference producer/consumer pair of the contract.
#[derive(Trace, Default)]
pub struct FrameCounter {
    pub count: Reg<U<16>>,
}

#[lower]
impl Unit<Rx<FrameByte>, Tx<U<16>>> for FrameCounter {
    async fn run(&mut self, bytes: Rx<FrameByte>, lens: Tx<U<16>>) {
        loop {
            let b = bytes.wait().await;
            let n = self.count + 1;
            if b.last.to_bool() {
                lens.send(n);
                self.count.set(0);
            } else {
                self.count.set(n);
            }
        }
    }
}
