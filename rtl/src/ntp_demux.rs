// SPDX-License-Identifier: Apache-2.0
//! NTP mode demultiplexer (task T4.2a), domain `DefaultClock` (`clk_sys`,
//! 100 MHz).
//!
//! Our NTP client uses UDP source port 123, so the replies to its requests
//! arrive on port 123 together with the requests that clients send to our
//! server. Both reach one channel pair; [`NtpDemux`] splits them by NTP mode
//! (RFC 5905 section 7.3, low 3 bits of byte 0).
//!
//! Ports (flat): inputs `meta` (`Rx<Meta>`) and `payload` (`Rx<FrameByte>`)
//! are the `ntp_meta` / `ntp` outputs of `udp4_rx` or `udp6_rx` (the UDP
//! payload of a datagram to port 123); outputs `srv_meta` (`Tx<Meta>`) and
//! `srv` (`Tx<FrameByte>`) go to `ntp_server`, `cli_meta` and `cli` to the
//! NTP client (replies to our requests).
//!
//! # Behaviour
//!
//! * One [`Meta`] is taken per datagram, and the datagram is decided on its
//!   **first payload byte** (not taken yet): mode = `byte0 & 7`. Mode 4
//!   (server reply) selects the `cli` pair; every other mode, 3 (client
//!   request) included, selects the `srv` pair. The server ignores what it
//!   does not serve, so modes 0..2, 5..7 are sent to it and it drops them.
//! * The `Meta` is forwarded on the selected meta channel, unchanged, and
//!   then ALL payload bytes are forwarded on the selected byte channel,
//!   unchanged (`data`, `last`, `err`, including a late `err` on the last
//!   byte). Cut-through: nothing is buffered. The selected pair is the only
//!   one written during a datagram; the next datagram is decided only after
//!   the `last` of this one has been forwarded.
//! * A datagram's `Meta` is always sent before its first payload byte. A
//!   payload of one byte is a valid datagram.
//! * The decision waits for room on the selected meta channel, and each
//!   byte waits for room on the selected byte channel. Nothing else is
//!   checked: the contract of `stream.rs` is assumed on the input.
//!
//! # Timing
//!
//! Cycle `c` is the cycle the decision is taken: the input `meta` and the
//! selected output `*_meta` are both handled in `c` (the `Meta` is sent in
//! `c`, so the consumer sees it at `c + 1` on a registered channel). The
//! first payload byte is taken from `payload` and sent on the selected byte
//! channel in `c + 1` at the earliest, so the consumer sees it at `c + 2`.
//! Afterwards one byte per cycle at best, in the cycle its input byte is
//! taken, as long as the output has room and the input has a byte. The
//! Verilog does not add any latency: the unit has no pipeline register of
//! its own.
//!
//! # Head-of-line blocking
//!
//! Datagrams leave in arrival order. A datagram is decided only when it is
//! at the head, so a stalled consumer blocks everything behind it, on both
//! pairs. In particular, if `cli` stalls on a reply, a `srv` datagram that
//! arrives after that reply waits until `cli` drains; a `srv` datagram that
//! arrived before it is already delivered and is not affected. Nothing is
//! lost or reordered in either case. The alternative (a buffer per pair)
//! would cost one datagram of RAM per pair and is not needed at NTP rates.
//!
//! # Integration
//!
//! Inputs and outputs must be registered channels or FIFOs (never port to
//! port, see the netlist note of `stream.rs`).
use crate::stream::{FrameByte, Meta};
use txhdl::comp::{mux, Clock, DefaultClock, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{lower, with, Trace};

/// NTP mode of a server reply (RFC 5905 section 7.3).
pub const MODE_SERVER: u8 = 4;

#[derive(Trace, Default)]
pub struct NtpDemux {
    /// A datagram is being forwarded (its `Meta` already sent).
    pub fwd: Reg<Bit>,
    /// Destination of the datagram in progress: 1 = `cli`, 0 = `srv`.
    pub to_cli: Reg<Bit>,
}

#[lower]
impl
    Unit<
        (Rx<Meta>, Rx<FrameByte>),
        (Tx<Meta>, Tx<FrameByte>, Tx<Meta>, Tx<FrameByte>),
    > for NtpDemux
{
    async fn run(
        &mut self,
        (meta, payload): (Rx<Meta>, Rx<FrameByte>),
        (srv_meta, srv, cli_meta, cli): (Tx<Meta>, Tx<FrameByte>, Tx<Meta>, Tx<FrameByte>),
    ) {
        loop {
            DefaultClock::rising().await;
            let fwd = self.fwd.get();
            let sel = self.to_cli.get();
            let have = Bit::from(payload.peek().is_some());
            let b = payload.head();
            let mv = Bit::from(meta.peek().is_some());
            let m = meta.head();

            // Decision on the first byte: mode 4 goes to the client pair.
            let is4 = Bit::from((b.data & U::<8>::from(7u8)) == U::<8>::from(MODE_SERVER));

            // Idle: take the meta and decide on byte 0 (not taken yet).
            let room_m = mux(is4, cli_meta.ready(), srv_meta.ready());
            let go = !fwd & have & mv & room_m;
            let _ = meta.recv_if(go);
            if (go & is4).to_bool() {
                cli_meta.send(m);
            }
            if (go & !is4).to_bool() {
                srv_meta.send(m);
            }

            // Forwarding: one payload byte per cycle to the selected pair.
            let room_b = mux(sel, cli.ready(), srv.ready());
            let take = fwd & have & room_b;
            let _ = payload.recv_if(take);
            let out = FrameByte { data: b.data, last: b.last, err: b.err };
            if (take & sel).to_bool() {
                cli.send(out);
            }
            if (take & !sel).to_bool() {
                srv.send(out);
            }

            with!(self <= {
                go ? { fwd: Bit::One, to_cli: is4 },
                take & b.last ? fwd: Bit::Zero,
            });
        }
    }
}
