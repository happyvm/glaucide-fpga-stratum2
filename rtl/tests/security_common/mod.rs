// SPDX-License-Identifier: Apache-2.0
//! Shared bench of the security tests (task T5.5): the whole server top
//! (`ArtyTop<Test, S, phy_init::Sim>`, real `mii_rx`/`mii_tx`) driven on
//! its MII pins, as `rtl/tests/top.rs` does (`PinBench`, copied here: test
//! files are separate crates), plus frame builders with correct checksums
//! and a parser of the transmitted frames.
//!
//! Addresses are those of `config::Test` (the golden model): our MAC
//! 02:00:00:00:00:01, 192.0.2.1, fe80::ff:fe00:1, 2001:db8::1.
#![allow(dead_code)]

use ntp_fpga::clocks::{ClkRx, ClkTx};
use ntp_fpga::config::Test;
use ntp_fpga::phy_init;
use ntp_fpga::top::{ArtyTop, SyncSource};
use txhdl::comp::{now, signal, In, Out, Reg, Running, Unit};
use txhdl::types::{Bit, U};

pub const OUR_MAC: u64 = 0x0200_0000_0001;
pub const OUR_IP4: u32 = 0xC000_0201;
pub const OUR_GUA: u128 = 0x2001_0DB8_0000_0000_0000_0000_0000_0001;
pub const OUR_LL: u128 = 0xFE80_0000_0000_0000_0000_00FF_FE00_0001;

// ------------------------------------------------------------ vectors

pub fn from_hex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

/// (is_rx, frame) lines of a golden vector file.
pub fn vector(name: &str) -> Vec<(bool, Vec<u8>)> {
    let p = format!(
        "{}/../tools/golden/vectors/{}",
        env!("CARGO_MANIFEST_DIR"),
        name
    );
    let text = std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{p}: {e}"));
    let mut v = vec![];
    for l in text.lines() {
        let l = l.trim();
        if l.is_empty() || l.starts_with('#') {
            continue;
        }
        let (d, h) = l.split_once(' ').unwrap();
        v.push((d == "rx", from_hex(h)));
    }
    v
}

/// Every `rx` frame of every golden vector (valid and negative ones).
pub fn all_rx_frames() -> Vec<Vec<u8>> {
    let dir = format!("{}/../tools/golden/vectors", env!("CARGO_MANIFEST_DIR"));
    let mut names: Vec<String> = std::fs::read_dir(&dir)
        .unwrap()
        .filter_map(|e| e.ok())
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .filter(|n| n.ends_with(".hex"))
        .collect();
    names.sort();
    names
        .iter()
        .flat_map(|n| vector(n))
        .filter(|(rx, _)| *rx)
        .map(|(_, f)| f)
        .collect()
}

// ------------------------------------------------------------ checksums

pub fn crc32(bytes: &[u8]) -> u32 {
    let mut c = 0xffff_ffffu32;
    for &b in bytes {
        for i in 0..8 {
            let bit = ((c ^ (b as u32 >> i)) & 1) != 0;
            c >>= 1;
            if bit {
                c ^= 0xedb8_8320;
            }
        }
    }
    !c
}

/// RFC 1071 sum, not complemented, not folded.
pub fn sum16(acc: u32, b: &[u8]) -> u32 {
    let mut a = acc;
    for c in b.chunks(2) {
        a += ((c[0] as u32) << 8) | *c.get(1).unwrap_or(&0) as u32;
    }
    a
}

pub fn fold(mut a: u32) -> u16 {
    while a > 0xffff {
        a = (a & 0xffff) + (a >> 16);
    }
    a as u16
}

// ------------------------------------------------------------ builders

pub fn mac_bytes(m: u64) -> [u8; 6] {
    let b = m.to_be_bytes();
    [b[2], b[3], b[4], b[5], b[6], b[7]]
}

/// Ethernet frame (no FCS) padded with zeros to 60 bytes.
pub fn eth(dst: u64, src: u64, et: u16, payload: &[u8]) -> Vec<u8> {
    let mut v = mac_bytes(dst).to_vec();
    v.extend_from_slice(&mac_bytes(src));
    v.extend_from_slice(&et.to_be_bytes());
    v.extend_from_slice(payload);
    while v.len() < 60 {
        v.push(0);
    }
    v
}

/// IPv4 header (IHL 5, DF, the given TTL and protocol) with its checksum.
pub fn ipv4_hdr(src: u32, dst: u32, proto: u8, ttl: u8, data_len: usize) -> Vec<u8> {
    let tl = 20 + data_len;
    let mut h = vec![0x45, 0, (tl >> 8) as u8, tl as u8, 0, 0, 0x40, 0, ttl, proto, 0, 0];
    h.extend_from_slice(&src.to_be_bytes());
    h.extend_from_slice(&dst.to_be_bytes());
    let c = !fold(sum16(0, &h));
    h[10] = (c >> 8) as u8;
    h[11] = c as u8;
    h
}

/// UDP datagram over IPv4 with a correct checksum.
pub fn udp4(src: u32, dst: u32, sport: u16, dport: u16, pay: &[u8]) -> Vec<u8> {
    let len = 8 + pay.len();
    let mut u = sport.to_be_bytes().to_vec();
    u.extend_from_slice(&dport.to_be_bytes());
    u.extend_from_slice(&(len as u16).to_be_bytes());
    u.extend_from_slice(&[0, 0]);
    u.extend_from_slice(pay);
    let mut a = sum16(0, &src.to_be_bytes());
    a = sum16(a, &dst.to_be_bytes());
    a += 17 + len as u32;
    a = sum16(a, &u);
    let mut c = !fold(a);
    if c == 0 {
        c = 0xffff;
    }
    u[6] = (c >> 8) as u8;
    u[7] = c as u8;
    u
}

/// UDP datagram over IPv6 with a correct checksum.
pub fn udp6(src: u128, dst: u128, sport: u16, dport: u16, pay: &[u8]) -> Vec<u8> {
    let len = 8 + pay.len();
    let mut u = sport.to_be_bytes().to_vec();
    u.extend_from_slice(&dport.to_be_bytes());
    u.extend_from_slice(&(len as u16).to_be_bytes());
    u.extend_from_slice(&[0, 0]);
    u.extend_from_slice(pay);
    let mut a = sum16(0, &src.to_be_bytes());
    a = sum16(a, &dst.to_be_bytes());
    a = sum16(a, &(len as u32).to_be_bytes());
    a += 17;
    a = sum16(a, &u);
    let mut c = !fold(a);
    if c == 0 {
        c = 0xffff;
    }
    u[6] = (c >> 8) as u8;
    u[7] = c as u8;
    u
}

/// IPv6 header.
pub fn ipv6_hdr(src: u128, dst: u128, next: u8, hop: u8, len: usize) -> Vec<u8> {
    let mut h = vec![0x60, 0, 0, 0, (len >> 8) as u8, len as u8, next, hop];
    h.extend_from_slice(&src.to_be_bytes());
    h.extend_from_slice(&dst.to_be_bytes());
    h
}

/// An NTP packet of `len` bytes (>= 1): byte 0 `b0`, poll 6, Transmit
/// timestamp `xmit` at bytes 40..47 when they exist, the rest zero.
pub fn ntp_payload(b0: u8, len: usize, xmit: u64) -> Vec<u8> {
    let mut p = vec![0u8; len.max(1)];
    p[0] = b0;
    if len > 2 {
        p[2] = 6;
    }
    let x = xmit.to_be_bytes();
    for i in 0..8 {
        if 40 + i < len {
            p[40 + i] = x[i];
        }
    }
    p
}

/// A complete NTP request frame over IPv4 to us (to port 123).
pub fn ntp4_frame(src_mac: u64, src: u32, sport: u16, pay: &[u8]) -> Vec<u8> {
    let u = udp4(src, OUR_IP4, sport, 123, pay);
    let mut ip = ipv4_hdr(src, OUR_IP4, 17, 64, u.len());
    ip.extend_from_slice(&u);
    eth(OUR_MAC, src_mac, 0x0800, &ip)
}

/// A complete NTP request frame over IPv6 to our global address.
pub fn ntp6_frame(src_mac: u64, src: u128, sport: u16, pay: &[u8]) -> Vec<u8> {
    let u = udp6(src, OUR_GUA, sport, 123, pay);
    let mut ip = ipv6_hdr(src, OUR_GUA, 17, 64, u.len());
    ip.extend_from_slice(&u);
    eth(OUR_MAC, src_mac, 0x86dd, &ip)
}

// ------------------------------------------------------------ parsed reply

/// What a transmitted frame is (FCS removed and checked).
#[derive(Debug, Clone)]
pub struct Sent {
    pub bytes: Vec<u8>,
    pub dst_mac: u64,
    pub ethertype: u16,
    /// For a UDP frame: (family 6?, src ip, dst ip, sport, dport, payload).
    pub udp: Option<(bool, u128, u128, u16, u16, Vec<u8>)>,
    pub sfd_tick: u64,
}

fn be(b: &[u8]) -> u128 {
    b.iter().fold(0u128, |a, &x| (a << 8) | x as u128)
}

/// Parses a transmitted frame (FCS included), checking the FCS and, for
/// IPv4/IPv6 UDP, the IP header checksum and the UDP checksum.
pub fn parse_sent(raw: &[u8], sfd_tick: u64) -> Sent {
    let n = raw.len();
    assert!(n >= 64, "transmitted frame of {n} bytes (< 64 with FCS)");
    let (body, fcs) = raw.split_at(n - 4);
    assert_eq!(
        u32::from_le_bytes([fcs[0], fcs[1], fcs[2], fcs[3]]),
        crc32(body),
        "FCS of a transmitted frame"
    );
    let dst_mac = be(&body[0..6]) as u64;
    let ethertype = u16::from_be_bytes([body[12], body[13]]);
    let mut udp = None;
    if ethertype == 0x0800 && body.len() >= 42 {
        let ip = &body[14..];
        assert_eq!(fold(sum16(0, &ip[..20])), 0xffff, "IPv4 header checksum");
        if ip[9] == 17 {
            let tl = u16::from_be_bytes([ip[2], ip[3]]) as usize;
            let u = &ip[20..tl];
            let len = u16::from_be_bytes([u[4], u[5]]) as usize;
            assert_eq!(len, u.len(), "UDP length");
            let mut a = sum16(0, &ip[12..20]);
            a += 17 + len as u32;
            a = sum16(a, u);
            assert_eq!(fold(a), 0xffff, "UDP checksum over IPv4");
            udp = Some((
                false,
                be(&ip[12..16]),
                be(&ip[16..20]),
                u16::from_be_bytes([u[0], u[1]]),
                u16::from_be_bytes([u[2], u[3]]),
                u[8..].to_vec(),
            ));
        }
    } else if ethertype == 0x86dd && body.len() >= 62 {
        let ip = &body[14..];
        let pl = u16::from_be_bytes([ip[4], ip[5]]) as usize;
        if ip[6] == 17 {
            let u = &ip[40..40 + pl];
            let len = u16::from_be_bytes([u[4], u[5]]) as usize;
            assert_eq!(len, u.len(), "UDP length");
            let mut a = sum16(0, &ip[8..40]);
            a = sum16(a, &(len as u32).to_be_bytes());
            a += 17;
            a = sum16(a, u);
            assert_eq!(fold(a), 0xffff, "UDP checksum over IPv6");
            udp = Some((
                true,
                be(&ip[8..24]),
                be(&ip[24..40]),
                u16::from_be_bytes([u[0], u[1]]),
                u16::from_be_bytes([u[2], u[3]]),
                u[8..].to_vec(),
            ));
        }
    }
    Sent {
        bytes: body.to_vec(),
        dst_mac,
        ethertype,
        udp,
        sfd_tick,
    }
}

// ------------------------------------------------------------ pins bench

const CLOCK_START: u128 = 0xECC5_8CC0_3FFF_0000u128 << 32;
const INCR: u128 = ntp_fpga::clock::INCR;

fn start_clock(c: &mut ntp_fpga::clock::DisciplinedClock) {
    c.acc = Reg::new(U::<96>::from(CLOCK_START));
    c.g_valid = Reg::new(Bit::One);
    c.g_prev = Reg::new(U::<96>::from(CLOCK_START - INCR));
    c.g_inc = Reg::new(U::<96>::from(INCR));
    c.g_any = Reg::new(Bit::One);
}

/// `ArtyTop<Test, S, phy_init::Sim>` driven on its MII pins, the MII at the
/// `clk_sys` rate (one nibble per cycle), as `rtl/tests/top.rs`.
/// The simulated clock starts 0.75 s before an NTP second boundary, so a
/// run shorter than 75 M cycles stays inside one rate-limiter window.
pub struct PinBench {
    /// Per cycle: (rxd, rx_dv, rx_er).
    pub wire: Vec<(u8, bool, bool)>,
    /// Cycle index of each frame's SFD nibble.
    pub rx_sfd: Vec<usize>,
    /// Bytes on the wire (frame + FCS) of each frame.
    pub rx_len: Vec<usize>,
}

impl Default for PinBench {
    fn default() -> Self {
        Self::new()
    }
}

impl PinBench {
    pub fn new() -> Self {
        PinBench {
            wire: vec![(0, false, false); 40],
            rx_sfd: vec![],
            rx_len: vec![],
        }
    }

    /// Appends `gap` idle cycles, then a frame with preamble, SFD and FCS
    /// (`fcs_flip` corrupts one FCS bit).
    pub fn frame(&mut self, f: &[u8], gap: usize, fcs_flip: bool) {
        let mut w = f.to_vec();
        w.extend_from_slice(&crc32(f).to_le_bytes());
        if fcs_flip {
            let n = w.len();
            w[n - 1] ^= 0x10;
        }
        self.raw(&w, gap);
    }

    /// Appends `gap` idle cycles, then preamble, SFD and exactly `w` (no FCS
    /// added: `w` carries its own, good or not).
    pub fn raw(&mut self, w: &[u8], gap: usize) {
        self.wire.extend(std::iter::repeat_n((0, false, false), gap));
        self.wire.extend(std::iter::repeat_n((5, true, false), 15));
        self.rx_sfd.push(self.wire.len());
        self.rx_len.push(w.len());
        self.wire.push((0xd, true, false));
        for &b in w {
            self.wire.push((b & 15, true, false));
            self.wire.push((b >> 4, true, false));
        }
    }

    /// Appends `n` idle cycles.
    pub fn idle(&mut self, n: usize) {
        self.wire.extend(std::iter::repeat_n((0, false, false), n));
    }

    /// Current length of the wire script (cycles).
    pub fn len(&self) -> usize {
        self.wire.len()
    }

    /// Runs the wire then `tail` more cycles; returns the frames on `txd`
    /// (FCS included) with the cycle of their SFD.
    pub fn run<S: SyncSource>(&self, tail: usize) -> Vec<(Vec<u8>, u64)> {
        let (rxd_o, rxd) = signal::<U<4>, ClkRx>();
        let (dv_o, dv) = signal::<Bit, ClkRx>();
        let (er_o, er) = signal::<Bit, ClkRx>();
        let (mdio_o_, mdio_i) = signal::<Bit, txhdl::comp::DefaultClock>();
        let (txd_o, txd) = signal::<U<4>, ClkTx>();
        let (en_o, en) = signal::<Bit, ClkTx>();
        let outs: Vec<(Out<Bit>, In<Bit>)> = (0..7)
            .map(|_| signal::<Bit, txhdl::comp::DefaultClock>())
            .collect();
        let mut outs = outs.into_iter();
        let mut o = || outs.next().unwrap().0;
        let mut top = ArtyTop::<Test, S, phy_init::Sim>::default();
        start_clock(&mut top.core.clock);
        mdio_o_.set(Bit::One);
        let t0 = now();
        let mut sim = Running::new(top.run(
            (rxd, dv, er, mdio_i),
            (txd_o, en_o, o(), o(), o(), o(), o(), o(), o()),
        ));
        let mut frames = vec![];
        let mut nib: Vec<u8> = vec![];
        let mut sfd_at = 0u64;
        for t in 0..self.wire.len() + tail {
            let (d, v, e) = self.wire.get(t).copied().unwrap_or((0, false, false));
            rxd_o.set(U::<4>::from(d));
            dv_o.set(Bit::from_bool(v));
            er_o.set(Bit::from_bool(e));
            sim.cycle();
            if en.get().to_bool() {
                let n = txd.get().raw() as u8;
                if nib.len() == 15 {
                    sfd_at = (now() - t0) / 2;
                }
                nib.push(n);
            } else if !nib.is_empty() {
                frames.push(decode(&nib, sfd_at));
                nib.clear();
            }
        }
        assert!(nib.is_empty(), "a frame still on the wire at the end of the run");
        frames
    }
}

fn decode(nib: &[u8], sfd: u64) -> (Vec<u8>, u64) {
    assert!(
        nib.len() >= 16 && nib.len().is_multiple_of(2),
        "odd or short frame on txd: {} nibbles",
        nib.len()
    );
    assert!(nib[..15].iter().all(|&n| n == 5), "preamble");
    assert_eq!(nib[15], 0xd, "SFD");
    (nib[16..].chunks(2).map(|c| c[0] | (c[1] << 4)).collect(), sfd)
}

// ------------------------------------------------------------ random

/// xorshift64*: deterministic, seeded.
pub struct Rng(pub u64);

impl Rng {
    pub fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    /// Uniform in `0..n` (n > 0).
    pub fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
    pub fn byte(&mut self) -> u8 {
        self.next() as u8
    }
    pub fn bytes(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| self.byte()).collect()
    }
}
