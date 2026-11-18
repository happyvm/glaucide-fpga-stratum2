// SPDX-License-Identifier: Apache-2.0
//! Ethernet frame builder (task T1.6): (header, payload) pairs to complete
//! frames, in the `DefaultClock` domain (the CDC to `ClkTx` is T1.4's job).
//!
//! Ports: `hdr` ([`EthHdr`]: destination MAC and EtherType), `payload`
//! (`FrameByte`, `last` ends it, `err` on `last` = upstream aborted),
//! `local_mac` (config, 48 bits) in; `frames` (`Tx<FrameByte>`) out.
//!
//! # Wire byte order
//!
//! MAC addresses are 48-bit values whose **most significant byte is byte 0
//! on the wire** (`0x0123_4567_89ab` goes out as 01 23 45 67 89 ab). The
//! EtherType is sent big endian (`0x0800` as 08 00).
//!
//! # Behaviour
//!
//! One frame per (`hdr`, `payload`) pair. A `hdr` is taken while idle (the
//! source MAC `local_mac` is latched at the same time), then `frames`
//! carries: destination MAC (6), source MAC (6), EtherType (2), the payload
//! bytes, zero padding until the frame before the FCS has 60 bytes, and the
//! FCS (4 bytes: complement of the CRC-32 of everything before it, least
//! significant byte first, as the wire wants; `crc32_step` from `crc32.rs`,
//! computed inline). `last` is on the final FCS byte. The stream is the
//! complete frame as `mii_tx` expects, FCS included (stream.rs item 4).
//!
//! The payload must have at least one byte per `hdr` (stream contract: a
//! frame has at least one byte).
//!
//! # Abort
//!
//! * `payload` ends with `last & err`: the unit sends one more byte (zero)
//!   with `last & err` and drops the frame; the consumer discards it. No
//!   FCS is sent.
//! * the payload would make the frame longer than [`MAX_FRAME`] (1514)
//!   bytes before the FCS: same, at the first payload byte that does not
//!   fit; the rest of that payload is swallowed (up to its `last`) so the
//!   next frame starts clean.
//!
//! # Stalls (integration contract, T1.11)
//!
//! This unit **stalls mid-frame**: it waits for `payload` bytes (which may
//! have gaps) and for `frames.ready()`, so its output stream has gaps
//! between the bytes of a frame. It never emits `last` early or out of
//! order. `mii_tx` needs a gap-free input, so T1.11 must put a FIFO that
//! buffers the **whole frame** between `frames` and `mii_tx` (and release
//! a frame to `mii_tx` only once its `last` is in, or drop it if it ends
//! with `err`). Two modules are never wired port to port in the netlist:
//! `hdr` and `payload` are fed through registered channels, and so is
//! `frames`.
//!
//! One byte goes out per cycle at best; a frame costs one idle cycle for
//! taking `hdr`.
//!
//! # Formal
//!
//! `check!`/`cover!` in `run` state the Ethernet row of `docs/formal.md`
//! (frame at least 64 bytes with FCS unless aborted, header byte order,
//! padding before the FCS, FCS = complemented CRC register, frame-stream
//! contract) and are proved with `tools/formal/prove.sh eth_tx prove`.
//! `assume!` is used only for the payload stream contract (`err` only with
//! `last`).
use crate::crc32::{crc32_step, INIT};
use crate::stream::{FrameByte, MAX_FRAME};
use txhdl::comp::{mux, Clock, DefaultClock, In, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{assume, check, cover, lower, with, Trace, Transaction as TransactionDerive, Value as ValueDerive};

/// Minimum frame length before the FCS (64 - 4).
pub const MIN_BODY: usize = 60;

/// Ethernet header fields the layer above supplies, one per frame.
#[derive(TransactionDerive, ValueDerive, Clone, Copy, Default, Debug)]
pub struct EthHdr {
    /// Destination MAC; byte 0 on the wire is the most significant byte.
    pub dst: U<48>,
    pub ethertype: U<16>,
}

// States.
const IDLE: u8 = 0;
const HDR: u8 = 1;
const PAY: u8 = 2;
const PAD: u8 = 3;
const FCS: u8 = 4;
const DRAIN: u8 = 5;

#[derive(Trace)]
pub struct EthTx {
    pub st: Reg<U<3>>,
    /// Bytes of the frame before the FCS sent so far.
    pub cnt: Reg<U<11>>,
    /// FCS bytes sent so far.
    pub k: Reg<U<2>>,
    /// CRC register (reflected, before the complement).
    pub crc: Reg<U<32>>,
    pub dst: Reg<U<48>>,
    pub src: Reg<U<48>>,
    pub et: Reg<U<16>>,
    // Monitors (only read by check!/cover!; pruned in synthesis).
    /// Bytes of the current frame sent so far, FCS included.
    pub m_n: Reg<U<11>>,
    /// The CRC register must hold `m_exp` (set when a byte went in).
    pub m_chk: Reg<Bit>,
    pub m_exp: Reg<U<32>>,
    /// The final CRC, captured when the FCS starts.
    pub m_cf: Reg<U<32>>,
}

impl Default for EthTx {
    fn default() -> Self {
        EthTx {
            st: Reg::default(),
            cnt: Reg::default(),
            k: Reg::default(),
            crc: Reg::new(INIT),
            dst: Reg::default(),
            src: Reg::default(),
            et: Reg::default(),
            m_n: Reg::default(),
            m_chk: Reg::default(),
            m_exp: Reg::default(),
            m_cf: Reg::default(),
        }
    }
}

#[lower]
impl Unit<(Rx<EthHdr>, Rx<FrameByte>, In<U<48>>), Tx<FrameByte>> for EthTx {
    async fn run(
        &mut self,
        (hdr, payload, local_mac): (Rx<EthHdr>, Rx<FrameByte>, In<U<48>>),
        frames: Tx<FrameByte>,
    ) {
        loop {
            DefaultClock::rising().await;
            let st = self.st.get();
            let cnt = self.cnt.get();
            let crc = self.crc.get();
            let room = frames.ready();
            let is_idle = Bit::from(st == IDLE);
            let is_hdr = Bit::from(st == HDR);
            let is_pay = Bit::from(st == PAY);
            let is_pad = Bit::from(st == PAD);
            let is_fcs = Bit::from(st == FCS);
            let is_drain = Bit::from(st == DRAIN);

            let h = hdr.head();
            let take_h = is_idle & Bit::from(hdr.peek().is_some());
            let _ = hdr.recv_if(take_h);
            let pb = payload.head();
            let pv = Bit::from(payload.peek().is_some());
            assume!(!pb.err | pb.last, "payload: err only together with last (stream contract 2)");

            // What happens this cycle.
            let over = Bit::from(cnt == MAX_FRAME);
            let send_hdr = is_hdr & room;
            let pay_in = is_pay & pv & room;
            let abort = pay_in & (over | (pb.last & pb.err));
            let send_pay = pay_in & !abort;
            let send_pad = is_pad & room;
            let send_fcs = is_fcs & room;
            let pre = send_hdr | send_pay | send_pad;
            let go = pre | send_fcs | abort;
            let _ = payload.recv_if(pay_in | (is_drain & pv));

            // The byte.
            let hb = mux(cnt == 0, self.dst.get().slice::<40,8>(), mux(cnt == 1, self.dst.get().slice::<32,8>(), mux(cnt == 2, self.dst.get().slice::<24,8>(), mux(cnt == 3, self.dst.get().slice::<16,8>(), mux(cnt == 4, self.dst.get().slice::<8,8>(), mux(cnt == 5, self.dst.get().slice::<0,8>(), mux(cnt == 6, self.src.get().slice::<40,8>(), mux(cnt == 7, self.src.get().slice::<32,8>(), mux(cnt == 8, self.src.get().slice::<24,8>(), mux(cnt == 9, self.src.get().slice::<16,8>(), mux(cnt == 10, self.src.get().slice::<8,8>(), mux(cnt == 11, self.src.get().slice::<0,8>(), mux(cnt == 12, self.et.get().slice::<8,8>(), mux(cnt == 13, self.et.get().slice::<0,8>(), U::<8>::from(0u8)))))))))))))));
            let fb = (crc ^ U::<32>::from(INIT)).slice::<0, 8>();
            let data = mux(send_hdr, hb, mux(send_pay, pb.data, mux(send_fcs, fb, U::<8>::from(0u8))));
            let last = abort | (send_fcs & Bit::from(self.k.get() == 3));
            let errb = abort;
            let next = crc32_step(crc, data);
            let m_n = self.m_n.get();

            // Invariants (stated before any wait/branch so they hold every cycle).
            check!(!is_idle | Bit::from(m_n == 0), "idle: nothing of a frame sent");
            check!(!is_drain | Bit::from(m_n == 0), "draining: the aborted frame is closed");
            check!(!(is_hdr | is_pay | is_pad) | Bit::from(m_n == cnt), "bytes sent = bytes counted before the FCS");
            check!(!is_fcs | Bit::from(m_n == cnt + self.k.get().zext::<11>()), "in the FCS: sent = counted + FCS bytes");
            check!(cnt <= MAX_FRAME, "a frame has at most 1514 bytes before the FCS");
            check!(!is_hdr | Bit::from(cnt < 14), "header state ends at byte 14");
            check!(!is_pad | (Bit::from(cnt >= 14) & Bit::from(cnt < MIN_BODY)), "padding only below 60 bytes");
            check!(!is_pay | Bit::from(cnt >= 14), "payload follows the header");
            check!(!is_fcs | Bit::from(cnt >= MIN_BODY), "the FCS comes only after padding to 60 bytes");
            check!(!(is_hdr & Bit::from(cnt == 0)) | Bit::from(crc == INIT), "the CRC register starts a frame at INIT");
            check!(!self.m_chk.get() | Bit::from(crc == self.m_exp.get()), "the CRC register is the step of the byte sent");
            check!(
                !(is_fcs & Bit::from(self.k.get() == 0)) | Bit::from(crc == (self.m_cf.get() >> 0u32)),
                "the CRC register shifts out byte 0 of the final CRC"
            );
            check!(
                !(send_fcs & Bit::from(self.k.get() == 0)) | Bit::from(data == !(self.m_cf.get() >> 0u32).slice::<0, 8>()),
                "FCS byte 0 is the complemented CRC byte 0 (little endian on the wire)"
            );
            check!(
                !(is_fcs & Bit::from(self.k.get() == 1)) | Bit::from(crc == (self.m_cf.get() >> 8u32)),
                "the CRC register shifts out byte 1 of the final CRC"
            );
            check!(
                !(send_fcs & Bit::from(self.k.get() == 1)) | Bit::from(data == !(self.m_cf.get() >> 8u32).slice::<0, 8>()),
                "FCS byte 1 is the complemented CRC byte 1 (little endian on the wire)"
            );
            check!(
                !(is_fcs & Bit::from(self.k.get() == 2)) | Bit::from(crc == (self.m_cf.get() >> 16u32)),
                "the CRC register shifts out byte 2 of the final CRC"
            );
            check!(
                !(send_fcs & Bit::from(self.k.get() == 2)) | Bit::from(data == !(self.m_cf.get() >> 16u32).slice::<0, 8>()),
                "FCS byte 2 is the complemented CRC byte 2 (little endian on the wire)"
            );
            check!(
                !(is_fcs & Bit::from(self.k.get() == 3)) | Bit::from(crc == (self.m_cf.get() >> 24u32)),
                "the CRC register shifts out byte 3 of the final CRC"
            );
            check!(
                !(send_fcs & Bit::from(self.k.get() == 3)) | Bit::from(data == !(self.m_cf.get() >> 24u32).slice::<0, 8>()),
                "FCS byte 3 is the complemented CRC byte 3 (little endian on the wire)"
            );
            check!(
                !(send_hdr & Bit::from(cnt == 0)) | Bit::from(data == (self.dst.get() >> 40u32).slice::<0, 8>()),
                "header byte 0 is destination MAC byte, most significant first"
            );
            check!(
                !(send_hdr & Bit::from(cnt == 1)) | Bit::from(data == (self.dst.get() >> 32u32).slice::<0, 8>()),
                "header byte 1 is destination MAC byte, most significant first"
            );
            check!(
                !(send_hdr & Bit::from(cnt == 2)) | Bit::from(data == (self.dst.get() >> 24u32).slice::<0, 8>()),
                "header byte 2 is destination MAC byte, most significant first"
            );
            check!(
                !(send_hdr & Bit::from(cnt == 3)) | Bit::from(data == (self.dst.get() >> 16u32).slice::<0, 8>()),
                "header byte 3 is destination MAC byte, most significant first"
            );
            check!(
                !(send_hdr & Bit::from(cnt == 4)) | Bit::from(data == (self.dst.get() >> 8u32).slice::<0, 8>()),
                "header byte 4 is destination MAC byte, most significant first"
            );
            check!(
                !(send_hdr & Bit::from(cnt == 5)) | Bit::from(data == (self.dst.get() >> 0u32).slice::<0, 8>()),
                "header byte 5 is destination MAC byte, most significant first"
            );
            check!(
                !(send_hdr & Bit::from(cnt == 6)) | Bit::from(data == (self.src.get() >> 40u32).slice::<0, 8>()),
                "header byte 6 is source MAC byte, most significant first"
            );
            check!(
                !(send_hdr & Bit::from(cnt == 7)) | Bit::from(data == (self.src.get() >> 32u32).slice::<0, 8>()),
                "header byte 7 is source MAC byte, most significant first"
            );
            check!(
                !(send_hdr & Bit::from(cnt == 8)) | Bit::from(data == (self.src.get() >> 24u32).slice::<0, 8>()),
                "header byte 8 is source MAC byte, most significant first"
            );
            check!(
                !(send_hdr & Bit::from(cnt == 9)) | Bit::from(data == (self.src.get() >> 16u32).slice::<0, 8>()),
                "header byte 9 is source MAC byte, most significant first"
            );
            check!(
                !(send_hdr & Bit::from(cnt == 10)) | Bit::from(data == (self.src.get() >> 8u32).slice::<0, 8>()),
                "header byte 10 is source MAC byte, most significant first"
            );
            check!(
                !(send_hdr & Bit::from(cnt == 11)) | Bit::from(data == (self.src.get() >> 0u32).slice::<0, 8>()),
                "header byte 11 is source MAC byte, most significant first"
            );
            check!(
                !(send_hdr & Bit::from(cnt == 12)) | Bit::from(data == (self.et.get() >> 8u32).slice::<0, 8>()),
                "header byte 12 is EtherType byte, most significant first"
            );
            check!(
                !(send_hdr & Bit::from(cnt == 13)) | Bit::from(data == (self.et.get() >> 0u32).slice::<0, 8>()),
                "header byte 13 is EtherType byte, most significant first"
            );
            if go.to_bool() {
                check!(!errb | last, "err only together with last (stream contract 2)");
                check!(!last | go, "last is a byte of the stream");
                check!(Bit::from(m_n + 1 <= 1518), "a frame has at most 1518 bytes");
                check!(!(last & !errb) | Bit::from(m_n + 1 >= 64), "a good frame has at least 64 bytes with FCS");
                check!(!(last & !errb) | send_fcs, "a good frame ends with its FCS");
                check!(!send_fcs | Bit::from(m_n >= MIN_BODY), "the FCS follows 60 padded bytes");
                check!(!pre | !last, "no last before the FCS");
                check!(!(last & !errb) | Bit::from(self.k.get() == 3), "last on the fourth FCS byte");
            }
            cover!(send_fcs & last & Bit::from(m_n >= 63) & Bit::from(cnt >= 64), "a long frame is sent complete");
            cover!(send_pad & Bit::from(cnt == 20), "a short frame is padded");
            cover!(send_fcs & last & Bit::from(cnt == MIN_BODY), "a frame ends with its FCS at the minimum size");
            cover!(abort & !over, "a frame is aborted by the payload");
            cover!(abort & over, "a frame is aborted for length");

            if go.to_bool() {
                frames.send(FrameByte { data, last, err: errb });
            }

            let end_pay = send_pay & pb.last;
            let enter_fcs = end_pay & Bit::from(cnt >= 59) | send_pad & Bit::from(cnt == 59);
            with!(self <= {
                take_h ? {
                    st: U::<3>::from(HDR),
                    cnt: U::<11>::from(0u16),
                    crc: INIT,
                    dst: h.dst,
                    src: local_mac.get(),
                    et: h.ethertype,
                },
                pre ? { cnt: cnt + 1, crc: next },
                send_hdr & Bit::from(cnt == 13) ? st: U::<3>::from(PAY),
                end_pay ? st: U::<3>::from(PAD),
                enter_fcs ? { st: U::<3>::from(FCS), k: U::<2>::from(0u8), m_cf: next },
                send_fcs ? { k: self.k.get() + 1, crc: crc >> 8u32 },
                send_fcs & last ? st: U::<3>::from(IDLE),
                abort ? st: U::<3>::from(DRAIN),
                (abort & pb.last) | (is_drain & pv & pb.last) ? st: U::<3>::from(IDLE),
                Bit::One ? m_chk: pre,
                Bit::One ? m_exp: next,
                go ? m_n: m_n + 1,
                go & last ? m_n: U::<11>::from(0u16),
            });
        }
    }
}
