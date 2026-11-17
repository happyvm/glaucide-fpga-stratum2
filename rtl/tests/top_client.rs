// SPDX-License-Identifier: Apache-2.0
//! System tests of the stratum-2 top (task T1.14): the NTP client wired into
//! the top (`ArtyCoreClient`), cycle accurate.
//!
//! Two benches, as in `rtl/tests/top.rs`:
//!
//! * **Pins** ([`Pins`]): `ArtyTopOf<Sim, ArtyCoreClient<..>>` (real
//!   `mii_rx`, `mii_tx`, `phy_init`) driven on its MII pins at the `clk_sys`
//!   rate (one nibble per cycle; functionally the design, timestamps only
//!   plausible). The test plays the upstream servers: it answers the ARP
//!   requests, the neighbor solicitations and the NTP requests it sees on
//!   `txd` by frames on `rxd`.
//! * **Clocks** ([`Clocks`]): `ArtyCoreClient` with models of `mii_rx` and
//!   `mii_tx` at 25 MHz (4 `clk_sys` periods, several phases): the transmit
//!   timestamps (client T1, server T3) against the SFD instants.
//!
//! # Time acceleration
//!
//! The client's timers are in seconds (lookup retries 15.6..31.3 ms, iburst
//! every 2 s, reply deadline 1..2 s): hundreds of millions of cycles at the
//! real rate. The benches that need them use `WarpClock<A>` (simulation-only
//! clock of `top_client.rs`): one NTP second lasts `1e8 / A` cycles. It does
//! not change the cycle behaviour of the design (only the values of `time`),
//! so a transmit timestamp is checked by finding the cycle whose warped time
//! it was computed from (probe on the clock's registers) and converting that
//! cycle back to real time: exactly the timestamp the unwarped design would
//! have sent. The absolute values of the timestamps on the wire are
//! meaningless otherwise (the fixed biases in ns are not scaled).
//!
//! # Probes
//!
//! The client's effect inside the design is read on the registers of the
//! real units (zero cost, nothing added to the netlist): the per-slot flags
//! and T1 of `ntp_client` (`m_fl`, `m_t1`), the per-peer state stored by the
//! selection (`rid0`, `str0`, `off0`, `dly0`, `t40`, the fields of the last
//! `PeerStat` of peer 0, i.e. of the last `Sample` through the clock
//! filter), the servo's step register and the clock's accumulator.
use ntp_fpga::clock::{DisciplinedClock, INCR};
use ntp_fpga::clocks::{ClkRx, ClkTx};
use ntp_fpga::config::{eui64_link_local, ConfigSet, Test};
use ntp_fpga::phy_init;
use ntp_fpga::refid::refid_ipv4;
use ntp_fpga::stream::FrameByte;
use ntp_fpga::top::ArtyTopOf;
use ntp_fpga::top_client::{
    ArtyCoreClient, ClockSource, WarpClock, CLIENT_TX_LATENCY_V4, CLIENT_TX_LATENCY_V6,
};
use std::cell::RefCell;
use std::collections::VecDeque;
use std::future::Future;
use std::rc::Rc;
use txhdl::comp::{chan, join2, now, signal, Clock, In, Out, Reg, Running, Rx, Tx, Unit};
use txhdl::types::{Bit, U};

// ------------------------------------------------------------ configuration

/// The golden `Test` addresses with the upstream slots of this test: slot 0
/// IPv4 on link (192.0.2.99), slot 1 IPv6 link-local (fe80::99), 2 and 3
/// off.
#[derive(Default)]
struct CliCfg;
impl ConfigSet for CliCfg {
    const LOCAL_MAC: u64 = Test::LOCAL_MAC;
    const LOCAL_IP4: u32 = Test::LOCAL_IP4;
    const GATEWAY_IP4: u32 = Test::GATEWAY_IP4;
    const NETMASK4: u32 = Test::NETMASK4;
    const GATEWAY_IP6: u128 = Test::GATEWAY_IP6;
    const LOCAL_GUA: u128 = Test::LOCAL_GUA;
    const GUA_ENABLED: bool = Test::GUA_ENABLED;
    const UPSTREAM: [(bool, u128, bool); 4] = [
        (false, UP4 as u128, true),
        (true, UP6, true),
        (false, 0, false),
        (false, 0, false),
    ];
    const POLL_LOG2: u8 = Test::POLL_LOG2;
    const PRECISION_LOG2: i8 = Test::PRECISION_LOG2;
    const STRATUM_OFFSET: u8 = Test::STRATUM_OFFSET;
    const RATE_LIMIT_PER_S: u16 = Test::RATE_LIMIT_PER_S;
}

/// [`CliCfg`] with the IPv4 upstream only (the synchronisation test: one
/// peer, no request queued behind another).
#[derive(Default)]
struct CliCfg4;
impl ConfigSet for CliCfg4 {
    const LOCAL_MAC: u64 = CliCfg::LOCAL_MAC;
    const LOCAL_IP4: u32 = CliCfg::LOCAL_IP4;
    const GATEWAY_IP4: u32 = CliCfg::GATEWAY_IP4;
    const NETMASK4: u32 = CliCfg::NETMASK4;
    const GATEWAY_IP6: u128 = CliCfg::GATEWAY_IP6;
    const LOCAL_GUA: u128 = CliCfg::LOCAL_GUA;
    const GUA_ENABLED: bool = CliCfg::GUA_ENABLED;
    const UPSTREAM: [(bool, u128, bool); 4] = [
        (false, UP4 as u128, true),
        (false, 0, false),
        (false, 0, false),
        (false, 0, false),
    ];
    const POLL_LOG2: u8 = CliCfg::POLL_LOG2;
    const PRECISION_LOG2: i8 = CliCfg::PRECISION_LOG2;
    const STRATUM_OFFSET: u8 = CliCfg::STRATUM_OFFSET;
    const RATE_LIMIT_PER_S: u16 = CliCfg::RATE_LIMIT_PER_S;
}

/// [`CliCfg`] with the IPv6 upstream only (slot 1).
#[derive(Default)]
struct CliCfg6;
impl ConfigSet for CliCfg6 {
    const LOCAL_MAC: u64 = CliCfg::LOCAL_MAC;
    const LOCAL_IP4: u32 = CliCfg::LOCAL_IP4;
    const GATEWAY_IP4: u32 = CliCfg::GATEWAY_IP4;
    const NETMASK4: u32 = CliCfg::NETMASK4;
    const GATEWAY_IP6: u128 = CliCfg::GATEWAY_IP6;
    const LOCAL_GUA: u128 = CliCfg::LOCAL_GUA;
    const GUA_ENABLED: bool = CliCfg::GUA_ENABLED;
    const UPSTREAM: [(bool, u128, bool); 4] = [
        (false, 0, false),
        (true, UP6, true),
        (false, 0, false),
        (false, 0, false),
    ];
    const POLL_LOG2: u8 = CliCfg::POLL_LOG2;
    const PRECISION_LOG2: i8 = CliCfg::PRECISION_LOG2;
    const STRATUM_OFFSET: u8 = CliCfg::STRATUM_OFFSET;
    const RATE_LIMIT_PER_S: u16 = CliCfg::RATE_LIMIT_PER_S;
}

const MAC: u64 = Test::LOCAL_MAC;
const IP4: u32 = Test::LOCAL_IP4;
const LL: u128 = eui64_link_local(Test::LOCAL_MAC);
/// Upstream 0: 192.0.2.99 and its MAC.
const UP4: u32 = 0xC000_0263;
const UP4_MAC: u64 = 0x0200_0000_0063;
/// Upstream 1: fe80::99 and its MAC.
const UP6: u128 = 0xFE80_0000_0000_0000_0000_0000_0000_0099;
const UP6_MAC: u64 = 0x0200_0000_0699;
/// Our poll and precision fields (configuration).
const POLL: u8 = Test::POLL_LOG2;
const PREC: u8 = Test::PRECISION_LOG2 as u8;

/// Acceleration of the pins benches: one NTP second = 10 000 cycles.
const A_PINS: u64 = 10_000;
/// Acceleration of the clocks bench (slower MII: more room for the lookup
/// retries, 15.6 ms = 15 625 cycles).
const A_CLK: u64 = 1_000;

type CoreW<R, T, C, K> = ArtyCoreClient<R, T, C, K>;
type TopW<C, K> = ArtyTopOf<phy_init::Sim, CoreW<ClkRx, ClkTx, C, K>>;

// ------------------------------------------------------------ bytes

fn from_hex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

/// (rx frame, tx frames) of a golden vector file.
fn exchanges(name: &str) -> Vec<(Vec<u8>, Vec<Vec<u8>>)> {
    let p = format!(
        "{}/../tools/golden/vectors/{}",
        env!("CARGO_MANIFEST_DIR"),
        name
    );
    let text = std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{p}: {e}"));
    let mut out: Vec<(Vec<u8>, Vec<Vec<u8>>)> = vec![];
    for l in text.lines() {
        let l = l.trim();
        if l.is_empty() || l.starts_with('#') {
            continue;
        }
        let (d, h) = l.split_once(' ').unwrap();
        if d == "rx" {
            out.push((from_hex(h), vec![]));
        } else {
            out.last_mut().unwrap().1.push(from_hex(h));
        }
    }
    out
}

fn crc32(bytes: &[u8]) -> u32 {
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

fn with_fcs(f: &[u8]) -> Vec<u8> {
    let mut v = f.to_vec();
    v.extend_from_slice(&crc32(f).to_le_bytes());
    v
}

fn be64(b: &[u8]) -> u64 {
    b.iter().fold(0u64, |a, &x| (a << 8) | x as u64)
}

fn be32(b: &[u8]) -> u32 {
    b.iter().fold(0u32, |a, &x| (a << 8) | x as u32)
}

fn sum16(acc: u32, b: &[u8]) -> u32 {
    let mut a = acc;
    for c in b.chunks(2) {
        a += ((c[0] as u32) << 8) | *c.get(1).unwrap_or(&0) as u32;
    }
    a
}

fn fold(mut a: u32) -> u16 {
    while a > 0xffff {
        a = (a & 0xffff) + (a >> 16);
    }
    a as u16
}

fn mac6(m: u64) -> [u8; 6] {
    m.to_be_bytes()[2..].try_into().unwrap()
}

fn eth(dst: u64, src: u64, et: u16, body: &[u8]) -> Vec<u8> {
    let mut f = mac6(dst).to_vec();
    f.extend_from_slice(&mac6(src));
    f.extend_from_slice(&et.to_be_bytes());
    f.extend_from_slice(body);
    while f.len() < 60 {
        f.push(0);
    }
    f
}

/// IPv4 header (ipv4_tx's: id 0, DF, TTL 64) with its checksum.
fn ip4_hdr(src: u32, dst: u32, proto: u8, len: usize) -> Vec<u8> {
    let tot = (20 + len) as u16;
    let mut h = vec![0x45, 0];
    h.extend_from_slice(&tot.to_be_bytes());
    h.extend_from_slice(&[0, 0, 0x40, 0, 64, proto, 0, 0]);
    h.extend_from_slice(&src.to_be_bytes());
    h.extend_from_slice(&dst.to_be_bytes());
    let c = !fold(sum16(0, &h));
    h[10..12].copy_from_slice(&c.to_be_bytes());
    h
}

fn udp_csum(pseudo: u32, u: &mut [u8]) {
    u[6] = 0;
    u[7] = 0;
    let mut c = !fold(sum16(pseudo, u));
    if c == 0 {
        c = 0xffff;
    }
    u[6..8].copy_from_slice(&c.to_be_bytes());
}

fn udp(sport: u16, dport: u16, payload: &[u8]) -> Vec<u8> {
    let mut u = sport.to_be_bytes().to_vec();
    u.extend_from_slice(&dport.to_be_bytes());
    u.extend_from_slice(&((8 + payload.len()) as u16).to_be_bytes());
    u.extend_from_slice(&[0, 0]);
    u.extend_from_slice(payload);
    u
}

/// An UDP/IPv4 frame, checksums computed.
fn udp4_frame(
    dmac: u64,
    smac: u64,
    src: u32,
    dst: u32,
    sport: u16,
    dport: u16,
    p: &[u8],
) -> Vec<u8> {
    let mut u = udp(sport, dport, p);
    let ps = sum16(0, &src.to_be_bytes()) + sum16(0, &dst.to_be_bytes()) + 17 + u.len() as u32;
    udp_csum(ps, &mut u);
    let mut body = ip4_hdr(src, dst, 17, u.len());
    body.extend_from_slice(&u);
    eth(dmac, smac, 0x0800, &body)
}

fn ip6_hdr(src: u128, dst: u128, nh: u8, hop: u8, len: usize) -> Vec<u8> {
    let mut h = vec![0x60, 0, 0, 0];
    h.extend_from_slice(&(len as u16).to_be_bytes());
    h.extend_from_slice(&[nh, hop]);
    h.extend_from_slice(&src.to_be_bytes());
    h.extend_from_slice(&dst.to_be_bytes());
    h
}

fn pseudo6(src: u128, dst: u128, nh: u8, len: usize) -> u32 {
    sum16(0, &src.to_be_bytes()) + sum16(0, &dst.to_be_bytes()) + nh as u32 + len as u32
}

/// An UDP/IPv6 frame (hop limit 64), checksum computed.
fn udp6_frame(
    dmac: u64,
    smac: u64,
    src: u128,
    dst: u128,
    sport: u16,
    dport: u16,
    p: &[u8],
) -> Vec<u8> {
    let mut u = udp(sport, dport, p);
    udp_csum(pseudo6(src, dst, 17, u.len()), &mut u);
    let mut body = ip6_hdr(src, dst, 17, 64, u.len());
    body.extend_from_slice(&u);
    eth(dmac, smac, 0x86dd, &body)
}

fn icmp6_frame(dmac: u64, smac: u64, src: u128, dst: u128, mut m: Vec<u8>) -> Vec<u8> {
    m[2] = 0;
    m[3] = 0;
    let c = !fold(sum16(pseudo6(src, dst, 58, m.len()), &m));
    m[2..4].copy_from_slice(&c.to_be_bytes());
    let mut body = ip6_hdr(src, dst, 58, 255, m.len());
    body.extend_from_slice(&m);
    eth(dmac, smac, 0x86dd, &body)
}

/// ARP packet (Ethernet/IPv4).
fn arp(op: u16, sha: u64, spa: u32, tha: u64, tpa: u32) -> Vec<u8> {
    let mut p = vec![0, 1, 8, 0, 6, 4];
    p.extend_from_slice(&op.to_be_bytes());
    p.extend_from_slice(&mac6(sha));
    p.extend_from_slice(&spa.to_be_bytes());
    p.extend_from_slice(&mac6(tha));
    p.extend_from_slice(&tpa.to_be_bytes());
    p
}

/// The ARP request the DUT must send for `ip`.
fn want_arp_request(ip: u32) -> Vec<u8> {
    eth(0xffff_ffff_ffff, MAC, 0x0806, &arp(1, MAC, IP4, 0, ip))
}

/// The upstream's ARP reply to it.
fn arp_reply(ip: u32, mac: u64) -> Vec<u8> {
    eth(MAC, mac, 0x0806, &arp(2, mac, ip, MAC, IP4))
}

fn sn_addr(a: u128) -> u128 {
    0xff02_0000_0000_0000_0000_0001_ff00_0000 | (a & 0xFF_FFFF)
}

/// The neighbor solicitation the DUT must send for `target` (link-local:
/// source = our link-local address, to the solicited-node group).
fn want_ns(target: u128) -> Vec<u8> {
    let mut m = vec![135, 0, 0, 0, 0, 0, 0, 0];
    m.extend_from_slice(&target.to_be_bytes());
    m.extend_from_slice(&[1, 1]);
    m.extend_from_slice(&mac6(MAC));
    icmp6_frame(
        0x3333_ff00_0000 | (target & 0xFF_FFFF) as u64,
        MAC,
        LL,
        sn_addr(target),
        m,
    )
}

/// The upstream's solicited neighbor advertisement (S, O, target LLA).
fn na(target: u128, mac: u64) -> Vec<u8> {
    let mut m = vec![136, 0, 0, 0, 0x60, 0, 0, 0];
    m.extend_from_slice(&target.to_be_bytes());
    m.extend_from_slice(&[2, 1]);
    m.extend_from_slice(&mac6(mac));
    icmp6_frame(MAC, mac, target, LL, m)
}

/// `tools/golden/ntpclient/client.py::build_request`.
fn build_request(poll: u8, precision: u8, t1: u64) -> Vec<u8> {
    let mut p = vec![0x23, 0, poll, precision];
    p.extend_from_slice(&[0; 36]);
    p.extend_from_slice(&t1.to_be_bytes());
    p
}

/// A server reply (`client.py::build_reply` with LI 0, VN 4, mode 4, poll 6,
/// precision -20).
fn build_reply(stratum: u8, ref_id: u32, org: u64, t2: u64, t3: u64) -> Vec<u8> {
    let mut p = vec![0x24, stratum, 6, 0xEC];
    p.extend_from_slice(&[0; 8]); // root delay, root dispersion
    p.extend_from_slice(&ref_id.to_be_bytes());
    p.extend_from_slice(&(t2 & !0xFFFF_FFFF).to_be_bytes()); // reference timestamp
    p.extend_from_slice(&org.to_be_bytes());
    p.extend_from_slice(&t2.to_be_bytes());
    p.extend_from_slice(&t3.to_be_bytes());
    p
}

/// The client's IPv4 request frame for transmit timestamp `t1`.
fn want_req4(t1: u64) -> Vec<u8> {
    udp4_frame(
        UP4_MAC,
        MAC,
        IP4,
        UP4,
        123,
        123,
        &build_request(POLL, PREC, t1),
    )
}

/// The client's IPv6 request frame (link-local upstream: source = our LL).
fn want_req6(t1: u64) -> Vec<u8> {
    udp6_frame(
        UP6_MAC,
        MAC,
        LL,
        UP6,
        123,
        123,
        &build_request(POLL, PREC, t1),
    )
}

/// Checks the FCS and the zero padding of a transmitted frame; returns the
/// frame without FCS and padding (length `want_len`).
fn body(got: &[u8], want_len: usize, what: &str) -> Vec<u8> {
    let n = got.len();
    assert!(n >= 64, "{what}: {n} bytes");
    let (b, fcs) = got.split_at(n - 4);
    assert_eq!(
        u32::from_le_bytes(fcs.try_into().unwrap()),
        crc32(b),
        "{what}: FCS"
    );
    assert_eq!(b.len(), want_len.max(60), "{what}: length");
    assert!(b[want_len..].iter().all(|x| *x == 0), "{what}: padding");
    b[..want_len].to_vec()
}

fn same(got: &[u8], want: &[u8], what: &str) {
    let b = body(got, want.len(), what);
    if b != want {
        let d = (0..want.len()).find(|&i| b[i] != want[i]).unwrap();
        panic!(
            "{what}: byte {d} differs\n got  {}\n want {}",
            hex(&b),
            hex(want)
        );
    }
}

// ------------------------------------------------------------ frame kinds

#[derive(Debug, PartialEq)]
enum Kind {
    ArpReq(u32),
    Ns(u128),
    /// Client request (family 6 = true, T1).
    Req(bool, u64),
    /// Server reply (family, stratum, LI, ref id).
    Reply(bool, u8, u8, u32),
    Other,
}

fn kind(f: &[u8]) -> Kind {
    let et = u16::from_be_bytes([f[12], f[13]]);
    if et == 0x0806 && f[20..22] == [0, 1] {
        return Kind::ArpReq(be32(&f[38..42]));
    }
    if et == 0x0800 && f[23] == 17 {
        let p = &f[42..];
        let (sp, dp) = (
            u16::from_be_bytes([f[34], f[35]]),
            u16::from_be_bytes([f[36], f[37]]),
        );
        if p[0] & 7 == 3 && sp == 123 && dp == 123 {
            return Kind::Req(false, be64(&p[40..48]));
        }
        if p[0] & 7 == 4 {
            return Kind::Reply(false, p[1], p[0] >> 6, be32(&p[12..16]));
        }
    }
    if et == 0x86dd && f[20] == 58 && f[54] == 135 {
        return Kind::Ns(u128::from_be_bytes(f[62..78].try_into().unwrap()));
    }
    if et == 0x86dd && f[20] == 17 {
        let p = &f[62..];
        let (sp, dp) = (
            u16::from_be_bytes([f[54], f[55]]),
            u16::from_be_bytes([f[56], f[57]]),
        );
        if p[0] & 7 == 3 && sp == 123 && dp == 123 {
            return Kind::Req(true, be64(&p[40..48]));
        }
        if p[0] & 7 == 4 {
            return Kind::Reply(true, p[1], p[0] >> 6, be32(&p[12..16]));
        }
    }
    Kind::Other
}

// ------------------------------------------------------------ time

/// Start of the simulated clock (32.64), as in `top.rs`.
const CLOCK_START: u128 = 0xECC5_8CC0_3FFF_0000u128 << 32;

fn start_clock(c: &mut DisciplinedClock) {
    c.acc = Reg::new(U::<96>::from(CLOCK_START));
    c.g_valid = Reg::new(Bit::One);
    c.g_prev = Reg::new(U::<96>::from(CLOCK_START - INCR));
    c.g_inc = Reg::new(U::<96>::from(INCR));
    c.g_any = Reg::new(Bit::One);
}

/// Real (unwarped) 32.32 time at tick `t` of a run (`clk_sys` = 2 ticks).
fn ntp_at(t: u64) -> u64 {
    ((CLOCK_START + t as u128 * INCR / 2) >> 32) as u64
}

/// Warped 32.32 time at tick `t` with acceleration `a` (no step).
fn wt(a: u64, t: u64) -> u64 {
    ((CLOCK_START + t as u128 * a as u128 * INCR / 2) >> 32) as u64
}

fn diff_ns(a: u64, b: u64) -> f64 {
    (a.wrapping_sub(b) as i64) as f64 * 1e9 / 4_294_967_296.0
}

/// Per `clk_sys` cycle of a run: (tick, real accumulator, warp) as the
/// design's `time` consumers saw them (probe on the clock's registers).
#[derive(Default)]
struct TimeLog(Vec<(u64, u128, u128)>);

impl TimeLog {
    /// The real-time equivalent of a transmit timestamp `ts = (time >> 32) +
    /// k` computed in some cycle: the cycle is found by its warped time, the
    /// timestamp recomputed from its real accumulator.
    fn unwarp(&self, ts: u64, k: u64) -> u64 {
        let v = ts.wrapping_sub(k);
        let hits: Vec<_> = self
            .0
            .iter()
            .filter(|(_, acc, w)| (((acc + w) % (1u128 << 96)) >> 32) as u64 == v)
            .collect();
        assert_eq!(hits.len(), 1, "one cycle has the warped time {v:#x}");
        ((hits[0].1 >> 32) as u64).wrapping_add(k)
    }
}

// ------------------------------------------------------------ pins bench

/// A frame seen on `txd`: bytes after the SFD (FCS included), tick of its
/// SFD nibble.
#[derive(Clone)]
struct TxFrame {
    bytes: Vec<u8>,
    sfd_tick: u64,
}

/// Registers of the design read by the tests.
struct Probes {
    acc: Reg<U<96>>,
    warp: Option<Reg<U<96>>>,
    m_fl: txhdl::comp::Mem<U<10>, 4>,
    m_t1: txhdl::comp::Mem<U<64>, 4>,
    rid0: Reg<U<32>>,
    str0: Reg<U<8>>,
    off0: Reg<U<64>>,
    dly0: Reg<U<64>>,
    t40: Reg<U<64>>,
    servo_stp: Reg<U<96>>,
    servo_st: Reg<U<3>>,
    ua_st: Reg<U<1>>,
    ua_gr: Reg<Bit>,
}

/// A clock source the tests can start and probe.
trait TestClock: ClockSource {
    const A: u64;
    fn start(&mut self);
    fn regs(&self) -> (Reg<U<96>>, Option<Reg<U<96>>>);
}
impl TestClock for DisciplinedClock {
    const A: u64 = 1;
    fn start(&mut self) {
        start_clock(self)
    }
    fn regs(&self) -> (Reg<U<96>>, Option<Reg<U<96>>>) {
        (self.acc.clone(), None)
    }
}
impl<const A: u64> TestClock for WarpClock<A> {
    const A: u64 = A;
    fn start(&mut self) {
        start_clock(&mut self.clock)
    }
    fn regs(&self) -> (Reg<U<96>>, Option<Reg<U<96>>>) {
        (self.clock.acc.clone(), Some(self.add.warp.clone()))
    }
}

fn probes<R: Clock, T: Clock, C: ConfigSet, K: TestClock>(c: &CoreW<R, T, C, K>) -> Probes {
    let (acc, warp) = c.clock.regs();
    Probes {
        acc,
        warp,
        m_fl: c.client.m_fl.clone(),
        m_t1: c.client.m_t1.clone(),
        rid0: c.sync.selection.rid0.clone(),
        str0: c.sync.selection.str0.clone(),
        off0: c.sync.selection.off0.clone(),
        dly0: c.sync.selection.dly0.clone(),
        t40: c.sync.selection.t40.clone(),
        servo_stp: c.sync.servo.stp.clone(),
        servo_st: c.sync.servo.st.clone(),
        ua_st: c.uarb4.st.clone(),
        ua_gr: c.uarb4.gr.clone(),
    }
}

impl Probes {
    fn time(&self) -> (u128, u128) {
        let a = self.acc.get().raw();
        let w = self.warp.as_ref().map(|w| w.get().raw()).unwrap_or(0);
        (a, w)
    }
    /// `out` flag of client slot `s`.
    fn out(&self, s: usize) -> bool {
        self.m_fl.read(s).raw() & 1 == 1
    }
}

/// The whole top on its MII pins, MII at the `clk_sys` rate, with a queue of
/// nibbles for `rxd`.
struct Pins<F: Future<Output = ()>> {
    sim: Running<F>,
    rxd_o: Out<U<4>, ClkRx>,
    dv_o: Out<Bit, ClkRx>,
    txd: In<U<4>, ClkTx>,
    en: In<Bit, ClkTx>,
    queue: VecDeque<(u8, bool)>,
    nib: Vec<u8>,
    sfd_at: u64,
    t0: u64,
    n: u64,
    p: Probes,
    a: u64,
    times: TimeLog,
    /// Frames seen on `txd`, in order.
    log: Vec<TxFrame>,
    /// `uarb4` (IDLE/PAY, granted source) at each change: (cycle, state).
    ua: Vec<(u64, (u128, bool))>,
}

fn pins<C: ConfigSet + 'static, K: TestClock + 'static>() -> Pins<impl Future<Output = ()>> {
    let (rxd_o, rxd) = signal::<U<4>, ClkRx>();
    let (dv_o, dv) = signal::<Bit, ClkRx>();
    let (er_o, er) = signal::<Bit, ClkRx>();
    let (mdio_o_, mdio_i) = signal::<Bit, txhdl::comp::DefaultClock>();
    let (txd_o, txd) = signal::<U<4>, ClkTx>();
    let (en_o, en) = signal::<Bit, ClkTx>();
    let mut outs = (0..7).map(|_| signal::<Bit, txhdl::comp::DefaultClock>().0);
    let mut o = || outs.next().unwrap();
    let top: &'static mut TopW<C, K> = Box::leak(Box::default());
    top.core.clock.start();
    let p = probes(&top.core);
    mdio_o_.set(Bit::One);
    er_o.set(Bit::Zero);
    let t0 = now();
    let sim = Running::new(top.run(
        (rxd, dv, er, mdio_i),
        (txd_o, en_o, o(), o(), o(), o(), o(), o(), o()),
    ));
    Pins {
        sim,
        rxd_o,
        dv_o,
        txd,
        en,
        queue: VecDeque::new(),
        nib: vec![],
        sfd_at: 0,
        t0,
        n: 0,
        p,
        a: K::A,
        times: TimeLog::default(),
        log: vec![],
        ua: vec![],
    }
}

impl<F: Future<Output = ()>> Pins<F> {
    /// Queues `gap` idle cycles then `f` (preamble, SFD, FCS added); returns
    /// the tick of its SFD nibble.
    fn send(&mut self, f: &[u8], gap: usize) -> u64 {
        self.queue.extend(std::iter::repeat_n((0, false), gap));
        self.queue.extend(std::iter::repeat_n((5, true), 15));
        let sfd = 2 * (self.n + self.queue.len() as u64);
        self.queue.push_back((0xd, true));
        for b in with_fcs(f) {
            self.queue.push_back((b & 15, true));
            self.queue.push_back((b >> 4, true));
        }
        sfd
    }

    /// One `clk_sys` cycle; returns a frame completed on `txd`.
    fn cycle(&mut self) -> Option<TxFrame> {
        let (d, v) = self.queue.pop_front().unwrap_or((0, false));
        self.rxd_o.set(U::<4>::from(d));
        self.dv_o.set(Bit::from_bool(v));
        let (acc, w) = self.p.time();
        self.times.0.push((2 * self.n, acc, w));
        self.sim.cycle();
        self.n += 1;
        let ua = (self.p.ua_st.get().raw(), self.p.ua_gr.get().to_bool());
        if self.ua.last().map(|x| x.1) != Some(ua) {
            self.ua.push((self.n, ua));
        }
        if self.en.get().to_bool() {
            if self.nib.len() == 15 {
                self.sfd_at = now() - self.t0 - 2;
            }
            self.nib.push(self.txd.get().raw() as u8);
            None
        } else if !self.nib.is_empty() {
            let nib = std::mem::take(&mut self.nib);
            assert!(
                nib[..15].iter().all(|&n| n == 5) && nib[15] == 0xd,
                "preamble/SFD"
            );
            assert!(nib.len().is_multiple_of(2), "odd nibble count");
            let bytes = nib[16..].chunks(2).map(|c| c[0] | (c[1] << 4)).collect();
            let f = TxFrame {
                bytes,
                sfd_tick: self.sfd_at,
            };
            self.log.push(f.clone());
            Some(f)
        } else {
            None
        }
    }

    /// Runs until a frame completes on `txd`, at most `limit` cycles.
    fn next(&mut self, limit: u64) -> Option<TxFrame> {
        for _ in 0..limit {
            if let Some(f) = self.cycle() {
                return Some(f);
            }
        }
        None
    }

    /// Runs `k` cycles; returns the frames completed meanwhile.
    fn run(&mut self, k: u64) -> Vec<TxFrame> {
        (0..k).filter_map(|_| self.cycle()).collect()
    }

    /// Warped time of the upstream at tick `t`, plus `delta` (2^-32 s).
    fn up_time(&self, t: u64, delta: u64) -> u64 {
        wt(self.a, t).wrapping_add(delta)
    }
}

/// The upstream: answers an ARP request for UP4 or an NS for UP6 seen on
/// the wire; returns whether `f` was one.
fn answer_resolution<F: Future<Output = ()>>(b: &mut Pins<F>, f: &TxFrame) -> bool {
    match kind(&f.bytes) {
        Kind::ArpReq(ip) if ip == UP4 => {
            b.send(&arp_reply(UP4, UP4_MAC), 12);
            true
        }
        Kind::Ns(t) if t == UP6 => {
            b.send(&na(UP6, UP6_MAC), 12);
            true
        }
        _ => false,
    }
}

/// The upstream's reply to a client request seen on the wire (`origin`
/// replaced by `org` when given), T2/T3 = its clock (warped time + `delta`)
/// at the request's SFD and at the reply's SFD, moved apart by `proc`
/// (2^-32 s, half each way: a longer processing time in the server, which
/// shortens the measured delay and leaves the offset unchanged). Returns
/// (T2, T3).
#[allow(clippy::too_many_arguments)]
fn answer_request<F: Future<Output = ()>>(
    b: &mut Pins<F>,
    f: &TxFrame,
    stratum: u8,
    ref_id: u32,
    org: Option<u64>,
    delta: u64,
    proc: u64,
    gap: usize,
) -> (u64, u64) {
    let Kind::Req(v6, t1) = kind(&f.bytes) else {
        panic!("not a request")
    };
    let t2 = b.up_time(f.sfd_tick, delta).wrapping_sub(proc / 2);
    // The reply's SFD comes after the queue, the gap and the preamble.
    let sfd = 2 * (b.n + b.queue.len() as u64 + gap as u64 + 15);
    let t3 = b.up_time(sfd, delta).wrapping_add(proc / 2);
    let p = build_reply(stratum, ref_id, org.unwrap_or(t1), t2, t3);
    let fr = if v6 {
        udp6_frame(MAC, UP6_MAC, UP6, LL, 123, 123, &p)
    } else {
        udp4_frame(MAC, UP4_MAC, UP4, IP4, 123, 123, &p)
    };
    let s = b.send(&fr, gap);
    assert_eq!(s, sfd);
    (t2, t3)
}

/// RFC 5905 on-wire arithmetic of the client (`client.py::arith`):
/// (offset, delay).
fn arith(t1: u64, t2: u64, t3: u64, t4: u64) -> (u64, u64) {
    let a = t2.wrapping_sub(t1) as i64 as i128;
    let b = t3.wrapping_sub(t4) as i64 as i128;
    let off = ((a + b) >> 1) as u64;
    let d1 = t4.wrapping_sub(t1) as i64 as i128;
    let d2 = t3.wrapping_sub(t2) as i64 as i128;
    let d = (d1 - d2).max(0) as u64;
    (off, d)
}

/// Start-up of a pins bench: plays the upstreams' resolution until both
/// client requests are out. Returns them (IPv4, IPv6) and the resolution
/// frames seen.
///
/// The NS is answered only once the IPv4 request is out, so that the two
/// requests do not leave back to back (the second one would wait for the
/// first on the wire and carry that wait in its T1).
fn startup<F: Future<Output = ()>>(b: &mut Pins<F>) -> (TxFrame, TxFrame, Vec<TxFrame>) {
    let (mut r4, mut r6, mut res) = (None::<TxFrame>, None, vec![]);
    let mut ns_seen = false;
    while r4.is_none() || r6.is_none() {
        let f = b
            .next(20_000)
            .expect("the client's frames within 20 000 cycles");
        match kind(&f.bytes) {
            Kind::ArpReq(_) => {
                assert!(answer_resolution(b, &f));
                res.push(f);
            }
            Kind::Ns(_) => {
                if r4.is_some() {
                    assert!(answer_resolution(b, &f));
                }
                ns_seen = true;
                res.push(f);
            }
            Kind::Req(false, _) => {
                if ns_seen {
                    b.send(&na(UP6, UP6_MAC), 12);
                }
                r4 = Some(f)
            }
            Kind::Req(true, _) => r6 = Some(f),
            k => panic!("unexpected frame {k:?}: {}", hex(&f.bytes)),
        }
    }
    (r4.unwrap(), r6.unwrap(), res)
}

/// (a): after reset the client resolves its two upstreams (ARP request,
/// NS to the solicited-node group, byte-exact), and once answered sends one
/// request to each: byte-exact against `build_request` (48 bytes, ports
/// 123 -> 123, valid IPv4 header and UDP checksums, destination MAC learnt
/// from the ARP reply / NA, source = our link-local address for the
/// link-local upstream), T1 = the timestamp the client stored, consistent
/// with the SFD on `txd`.
#[test]
fn startup_resolves_and_sends_requests() {
    // The Python model's own bytes for one T1 (client.py::build_request).
    assert_eq!(
        hex(&build_request(6, 0xE8, 0xECC5_8CC0_4000_1234)),
        "230006e8000000000000000000000000000000000000000000000000000000000000000000000000ecc58cc040001234"
    );
    let mut b = pins::<CliCfg, WarpClock<A_PINS>>();
    let (r4, r6, res) = startup(&mut b);
    // The first resolution frames: one ARP request, one NS, byte exact (the
    // client may repeat a resolution while the answer is in flight).
    let arps: Vec<_> = res
        .iter()
        .filter(|f| kind(&f.bytes) == Kind::ArpReq(UP4))
        .collect();
    let nss: Vec<_> = res
        .iter()
        .filter(|f| kind(&f.bytes) == Kind::Ns(UP6))
        .collect();
    assert!(
        !arps.is_empty() && !nss.is_empty(),
        "ARP request and NS seen"
    );
    for f in &arps {
        same(&f.bytes, &want_arp_request(UP4), "ARP request");
    }
    for f in &nss {
        same(&f.bytes, &want_ns(UP6), "neighbor solicitation");
    }
    assert!(
        r4.sfd_tick > arps[0].sfd_tick && r6.sfd_tick > nss[0].sfd_tick,
        "requests after their resolution"
    );
    println!(
        "ARP requests {}, NS {}; IPv4 request at cycle {}, IPv6 at {}",
        arps.len(),
        nss.len(),
        r4.sfd_tick / 2,
        r6.sfd_tick / 2
    );
    for (f, v6, slot) in [(&r4, false, 0), (&r6, true, 1)] {
        let Kind::Req(_, t1) = kind(&f.bytes) else {
            unreachable!()
        };
        let want = if v6 { want_req6(t1) } else { want_req4(t1) };
        same(
            &f.bytes,
            &want,
            if v6 { "IPv6 request" } else { "IPv4 request" },
        );
        assert_eq!(b.p.m_t1.read(slot).raw() as u64, t1, "stored T1 = sent T1");
        assert!(b.p.out(slot), "request outstanding");
        let k = if v6 {
            CLIENT_TX_LATENCY_V6
        } else {
            CLIENT_TX_LATENCY_V4
        };
        let e = diff_ns(b.times.unwarp(t1, k), ntp_at(f.sfd_tick));
        println!("slot {slot}: T1 - tx SFD = {e:+.1} ns (real time, MII at the clk_sys rate)");
        assert!(e.abs() < 2000.0, "T1 implausible ({e} ns)");
    }
}

/// (b): a spoofed reply (Origin != T1) does not reach the clock filter and
/// leaves the request outstanding; the valid reply closes it and its sample
/// (offset, delay bit-exact with `client.py::arith` from the T1 sent, the
/// chosen T2/T3 and the receive stamp T4) is the state of peer 0 in the
/// selection, with the reply's stratum and reference id.
#[test]
fn reply_reaches_the_filter_and_spoof_does_not() {
    let mut b = pins::<CliCfg, WarpClock<A_PINS>>();
    let (r4, _r6, _) = startup(&mut b);
    let Kind::Req(_, t1) = kind(&r4.bytes) else {
        unreachable!()
    };
    let delta = 5u64 << 32; // the upstream is 5 s ahead
    answer_request(&mut b, &r4, 3, 0xBAD0_BAD0, Some(t1 ^ 0xFFFF), delta, 0, 12);
    b.run(3000);
    assert!(b.p.out(0), "spoofed reply: request still outstanding");
    assert_eq!(b.p.rid0.get().raw(), 0, "spoofed reply: no sample");
    let (t2, t3) = answer_request(&mut b, &r4, 2, 0x0A0B_0C0D, None, delta, 0, 12);
    b.run(3000);
    assert!(!b.p.out(0), "valid reply: request closed");
    assert_eq!(
        b.p.rid0.get().raw(),
        0x0A0B_0C0D,
        "reference id of the reply"
    );
    assert_eq!(b.p.str0.get().raw(), 2, "stratum of the reply");
    let t4 = b.p.t40.get().raw() as u64;
    let (off, dly) = arith(t1, t2, t3, t4);
    assert_eq!(b.p.off0.get().raw() as u64, off, "offset");
    assert_eq!(b.p.dly0.get().raw() as u64, dly, "delay");
    let off_s = off as i64 as f64 / 4_294_967_296.0;
    println!("sample: offset {off_s:.6} s (upstream 5 s ahead, warped time), T4 {t4:#x}");
    assert!((off_s - 5.0).abs() < 0.05, "offset near the upstream's 5 s");
    // Nothing else came out (no reply to a mode-4 packet, no extra request).
    for f in b.run(100) {
        panic!("unexpected frame {:?}", kind(&f.bytes));
    }
}

/// The golden server reply to `ntp4_basic.hex`'s request as an
/// unsynchronised `ClientSync` sends it (LI 3, stratum 16, reference
/// fields 0 after reset), T2/T3 copied from `got`.
fn want_unsync_reply(got: &TxFrame) -> Vec<u8> {
    let (_, w4) = exchanges("ntp4_basic.hex").remove(0);
    let mut w = w4[0].clone();
    let b = body(&got.bytes, w.len(), "server reply");
    w[42] = 0xE4;
    w[43] = 16;
    w[46..66].fill(0);
    w[74..90].copy_from_slice(&b[74..90]);
    let mut u = w[34..].to_vec();
    let ps = sum16(0, &w[26..34]) + 17 + u.len() as u32;
    udp_csum(ps, &mut u);
    w[34..].copy_from_slice(&u);
    w
}

/// Cycles at which `uarb4` granted a packet of source `src` (0 server,
/// 1 client), and the cycles at which it was idle again.
fn grants(ua: &[(u64, (u128, bool))], src: bool) -> Vec<(u64, u64)> {
    let mut v = vec![];
    for (i, &(n, (st, gr))) in ua.iter().enumerate() {
        if st == 1 && gr == src {
            let end = ua[i + 1..]
                .iter()
                .find(|x| x.1 .0 == 0)
                .map(|x| x.0)
                .unwrap_or(u64::MAX);
            v.push((n, end));
        }
    }
    v
}

/// (c): a server request is placed so that its reply reaches `uarb4` while
/// the arbiter forwards the client's IPv4 request (aimed from a first,
/// identical run): the arbiter grants the server's reply the cycle after the
/// client's packet (whole packets, no interleaving), and both frames come
/// out byte-exact.
#[test]
fn server_answers_while_client_polls() {
    let (q4, _) = exchanges("ntp4_basic.hex").remove(0);
    // First run (deterministic): when uarb4 grants the client's request,
    // then how long after its SFD a lone server request reaches uarb4.
    let mut b = pins::<CliCfg, WarpClock<A_PINS>>();
    let (r4, _, _) = startup(&mut b);
    let (cg, cend) = grants(&b.ua, true)[0];
    b.run(500);
    let s = b.send(&q4, 0) / 2;
    b.run(1500);
    let (sg, _) = *grants(&b.ua, false)
        .last()
        .expect("server reply through uarb4");
    let ls = sg - s;
    println!("client granted at cycle {cg} (until {cend}); a server request reaches uarb4 {ls} cycles after its SFD");
    drop(b);
    // Second run: the server request's SFD at cg + 10 - ls.
    let mut b = pins::<CliCfg, WarpClock<A_PINS>>();
    let target = cg + 10 - ls;
    let mut ns_seen = false;
    let (mut got_req, mut got_rep, mut sent_srv) = (None, None, false);
    while got_req.is_none() || got_rep.is_none() {
        let f = b.next(20_000).expect("frames within 20 000 cycles");
        match kind(&f.bytes) {
            Kind::ArpReq(_) => {
                answer_resolution(&mut b, &f);
                if !sent_srv {
                    let gap = target - (b.n + b.queue.len() as u64 + 15);
                    assert_eq!(b.send(&q4, gap as usize) / 2, target);
                    sent_srv = true;
                }
            }
            Kind::Ns(_) => ns_seen = true,
            Kind::Req(false, t1) => {
                same(&f.bytes, &want_req4(t1), "client request");
                assert_eq!(f.sfd_tick, r4.sfd_tick, "the client did not move");
                if ns_seen {
                    b.send(&na(UP6, UP6_MAC), 12);
                }
                got_req = Some(f);
            }
            Kind::Reply(false, ..) => {
                let w = want_unsync_reply(&f);
                same(&f.bytes, &w, "server reply");
                got_rep = Some(f);
            }
            k => panic!("unexpected {k:?}"),
        }
    }
    let c = grants(&b.ua, true);
    let sv = grants(&b.ua, false);
    println!("uarb4 grants: client {c:?}, server {sv:?}");
    assert_eq!(c[0], (cg, cend), "the client's packet as in the first run");
    assert_eq!(sv.len(), 1);
    assert!(
        sv[0].0 == cend || sv[0].0 == cend + 1,
        "the server's reply waited for the client's packet and was granted right after it"
    );
    let (q, r) = (got_req.unwrap(), got_rep.unwrap());
    println!(
        "client request at cycle {}, server reply at cycle {}",
        q.sfd_tick / 2,
        r.sfd_tick / 2
    );
    assert!(
        r.sfd_tick > q.sfd_tick,
        "the reply leaves after the request"
    );
}

/// (d): the IPv4 upstream (the only one, [`CliCfg4`]) answers the four
/// iburst polls 100 s ahead, each reply with a slightly longer processing
/// time (T3 - T2 + 1 ms per reply: each new sample has the smallest delay so
/// far, so the clock filter takes it, RFC 5905 §10). The fourth sample makes
/// the peer eligible (filter dispersion < 1 s), the selection picks it, the
/// servo (NSET) steps the real clock by the selected offset: the accumulator
/// moves by INCR + the commanded step in one cycle. The server then answers
/// IPv4 and IPv6 requests synchronised: LI 0, stratum 3 (upstream 2 + 1), the
/// reference id of 192.0.2.99 in both families, T3 on the upstream's time.
#[test]
fn iburst_steps_the_clock_and_server_becomes_stratum_3() {
    let delta = 100u64 << 32;
    let ms = (1u64 << 32) / 1000;
    let mut b = pins::<CliCfg4, WarpClock<A_PINS>>();
    let mut answered = 0u64;
    let mut stepped = None;
    let mut prev = b.p.time().0;
    // Four iburst polls, 2 s = 20 000 cycles apart, then the step.
    for _ in 0..80_000u64 {
        if let Some(f) = b.cycle() {
            if !answer_resolution(&mut b, &f) {
                match kind(&f.bytes) {
                    Kind::Req(false, _) => {
                        if stepped.is_none() {
                            answer_request(
                                &mut b,
                                &f,
                                2,
                                0x4750_5300,
                                None,
                                delta,
                                answered * ms,
                                12,
                            );
                            answered += 1;
                        }
                    }
                    k => panic!("unexpected {k:?}"),
                }
            }
        }
        let acc = b.p.time().0;
        let d = acc.wrapping_sub(prev) % (1u128 << 96);
        if d != INCR && stepped.is_none() {
            stepped = Some((b.n, d));
            println!("step at cycle {} after {answered} answered polls", b.n);
        }
        prev = acc;
        if stepped.is_some_and(|(n, _)| b.n > n + 200) {
            break;
        }
    }
    let (_, d) = stepped.expect("the servo stepped the clock");
    assert_eq!(
        answered, 4,
        "the peer is eligible after the four iburst samples"
    );
    assert_eq!(b.p.servo_st.get().raw(), 3, "servo in FREQ after the step");
    assert_eq!(
        d - INCR,
        b.p.servo_stp.get().raw(),
        "the clock moved by exactly the commanded step"
    );
    let step_s = (d - INCR) as f64 / 2f64.powi(64);
    println!("step {step_s:.6} s (upstream 100 s ahead)");
    assert!((step_s - 100.0).abs() < 0.05, "step near 100 s");
    // The server now answers synchronised, in both families.
    let (q4, _) = exchanges("ntp4_basic.hex").remove(0);
    let (q6, _) = exchanges("ntp6_basic.hex").remove(0);
    b.send(&q4, 50);
    b.send(&q6, 200);
    let mut replies = vec![];
    for f in b.run(3000) {
        match kind(&f.bytes) {
            Kind::Reply(v6, st, li, rid) => replies.push((v6, st, li, rid, f)),
            // The client's next poll (due at once after the step).
            Kind::Req(false, _) => {}
            k => panic!("unexpected {k:?}"),
        }
    }
    assert_eq!(replies.len(), 2, "two server replies");
    for (v6, st, li, rid, f) in &replies {
        println!("server reply (IPv6 {v6}): LI {li}, stratum {st}, reference id {rid:#010x}");
        assert_eq!((*st, *li), (3, 0), "stratum 3, LI 0");
        assert_eq!(*rid, refid_ipv4(UP4), "reference id of the system peer");
        let o = if *v6 { 62 } else { 42 };
        let t3 = be64(&f.bytes[o + 40..o + 48]);
        let e = diff_ns(t3, b.up_time(f.sfd_tick, delta)) / 1e9;
        println!("  T3 - upstream time = {e:+.6} s (warped)");
        assert!(e.abs() < 0.05, "T3 on the upstream's time");
    }
}

// ------------------------------------------------------------ clocks bench

#[derive(Clone, Copy, Default)]
struct Act {
    sfd: bool,
    byte: Option<(u8, bool)>,
}

/// `mii_rx` at clock `R` (port timing of `mii_rx.rs`, see `top.rs`), fed
/// from a queue the test extends while the simulation runs.
struct MiiRxModel {
    acts: Rc<RefCell<VecDeque<Act>>>,
    sfd_ticks: Rc<RefCell<Vec<u64>>>,
}

impl<R: Clock> Unit<(), (Tx<FrameByte, R>, Out<Bit, R>)> for MiiRxModel {
    async fn run(&mut self, _i: (), (raw, sfd): (Tx<FrameByte, R>, Out<Bit, R>)) {
        let mut sfd_r = false;
        loop {
            R::rising().await;
            let a = self.acts.borrow_mut().pop_front().unwrap_or_default();
            sfd.set(Bit::from_bool(sfd_r));
            sfd_r = a.sfd;
            if a.sfd {
                self.sfd_ticks.borrow_mut().push(now());
            }
            if let Some((d, last)) = a.byte {
                assert!(raw.ready().to_bool(), "the receive path stalled the wire");
                raw.send(FrameByte::new(d, last, false));
            }
        }
    }
}

/// The acts of one frame (`top.rs::rx_script`): SFD after `gap` + 15.
fn frame_acts(f: &[u8], gap: usize) -> Vec<Act> {
    let k = gap + 15;
    let n = f.len();
    let end = k + 2 * n + 1;
    let mut acts = vec![Act::default(); end + 2];
    acts[k].sfd = true;
    for (i, &d) in f.iter().enumerate() {
        let last = i + 1 == n;
        acts[if last { end } else { k + 4 + 2 * i }].byte = Some((d, last));
    }
    acts
}

type TxLog = (Vec<u8>, u64);

/// `mii_tx` at clock `T` (`top.rs`).
struct MiiTxModel {
    log: Rc<RefCell<Vec<TxLog>>>,
}

impl<T: Clock> Unit<Rx<FrameByte, T>, (Out<Bit, T>, Out<Bit, T>)> for MiiTxModel {
    async fn run(&mut self, frames: Rx<FrameByte, T>, (sfd, underrun): (Out<Bit, T>, Out<Bit, T>)) {
        let (mut st, mut cnt, mut o_sfd) = (0u8, 23u32, false);
        let mut last = false;
        let mut cur: Vec<u8> = vec![];
        let mut sfd_tick = 0u64;
        loop {
            T::rising().await;
            sfd.set(Bit::from_bool(o_sfd));
            underrun.set(Bit::Zero);
            o_sfd = false;
            match st {
                0 => {
                    if cnt > 0 {
                        cnt -= 1;
                    } else if let Some(b) = frames.recv() {
                        cur = vec![b.data.raw() as u8];
                        last = b.last.to_bool();
                        st = 1;
                        cnt = 1;
                    }
                }
                1 => {
                    if cnt == 15 {
                        o_sfd = true;
                        sfd_tick = now();
                    }
                    if cnt == 16 {
                        st = 2;
                    } else {
                        cnt += 1;
                    }
                }
                2 => st = 3,
                _ => {
                    if last {
                        self.log
                            .borrow_mut()
                            .push((std::mem::take(&mut cur), sfd_tick));
                        st = 0;
                        cnt = 23;
                    } else {
                        let b = frames.recv().expect("mii_tx underrun");
                        cur.push(b.data.raw() as u8);
                        last = b.last.to_bool();
                        st = 2;
                    }
                }
            }
        }
    }
}

/// Runs `ArtyCoreClient<R, T, CliCfg, K>` with the MII models. Every
/// transmitted frame is handed to `react` with the tick of its SFD edge
/// (relative to the run's start) and the time log; `react` returns frames
/// to receive (with their idle gap in `clk_rx` cycles) and whether to stop.
/// Returns the ticks of the received SFD edges.
fn clocks<R: Clock, T: Clock, C: ConfigSet, K: TestClock>(
    pre: &[(Vec<u8>, usize)],
    max_ticks: u64,
    mut react: impl FnMut(&[u8], u64, &TimeLog) -> (Vec<(Vec<u8>, usize)>, bool),
) -> Vec<u64> {
    let acts = Rc::new(RefCell::new(VecDeque::new()));
    for (f, g) in pre {
        acts.borrow_mut().extend(frame_acts(&with_fcs(f), *g));
    }
    let log = Rc::new(RefCell::new(vec![]));
    let sfd_ticks = Rc::new(RefCell::new(vec![]));
    let (raw_tx, raw_rx) = chan::<FrameByte, R>();
    let (rsfd_o, rsfd) = signal::<Bit, R>();
    let (txf_tx, txf_rx) = chan::<FrameByte, T>();
    let (tsfd_o, tsfd) = signal::<Bit, T>();
    let (und_o, und) = signal::<Bit, T>();
    let mut rxm = MiiRxModel {
        acts: acts.clone(),
        sfd_ticks: sfd_ticks.clone(),
    };
    let mut txm = MiiTxModel { log: log.clone() };
    let mut core = CoreW::<R, T, C, K>::default();
    core.clock.start();
    let p = probes(&core);
    let mut times = TimeLog::default();
    let t0 = now();
    assert_eq!(t0 % 2, 0);
    {
        let mut sim = Running::new(join2(
            join2(
                <MiiRxModel as Unit<(), (Tx<FrameByte, R>, Out<Bit, R>)>>::run(
                    &mut rxm,
                    (),
                    (raw_tx, rsfd_o),
                ),
                <MiiTxModel as Unit<Rx<FrameByte, T>, (Out<Bit, T>, Out<Bit, T>)>>::run(
                    &mut txm,
                    txf_rx,
                    (tsfd_o, und_o),
                ),
            ),
            core.run((raw_rx, rsfd, tsfd, und), txf_tx),
        ));
        let mut seen = 0;
        while now() - t0 < max_ticks {
            if (now() - t0) % 2 == 0 {
                let (acc, w) = p.time();
                times.0.push((now() - t0, acc, w));
            }
            sim.step();
            let n = log.borrow().len();
            if n > seen {
                let (f, t) = log.borrow()[seen].clone();
                seen = n;
                let (more, stop) = react(&f, t - t0, &times);
                if stop {
                    // End on a clk_sys edge: the next run starts there.
                    while (now() - t0) % 2 != 0 {
                        sim.step();
                    }
                    break;
                }
                let mut q = acts.borrow_mut();
                for (f, g) in more {
                    q.extend(frame_acts(&with_fcs(&f), g));
                }
            }
        }
    }
    let rx = sfd_ticks.borrow().iter().map(|t| t - t0).collect();
    rx
}

macro_rules! clock {
    ($name:ident, $period:expr, $phase:expr) => {
        struct $name;
        impl Clock for $name {
            const NAME: &'static str = stringify!($name);
            const PERIOD: u64 = $period;
            const PHASE: u64 = $phase;
        }
    };
}
clock!(Rx40a, 8, 1);
clock!(Tx40a, 8, 3);
clock!(Rx40b, 8, 7);
clock!(Tx40b, 8, 5);
clock!(Rx40c, 8, 2);
clock!(Tx40c, 8, 6);
clock!(Rx40d, 8, 5);
clock!(Tx40d, 8, 0);
clock!(Rx40e, 8, 3);
clock!(Tx40e, 8, 7);
clock!(Tx40f, 8, 1);
clock!(Tx40g, 8, 2);
clock!(Tx40h, 8, 4);

/// Client T1 against the SFD instant at 25 MHz MII (warped clock, T1
/// converted back to real time, see the module header), with one upstream
/// only ([`CliCfg4`] or [`CliCfg6`]): its request finds the transmit path
/// idle (checked). Error in ns.
fn t1_through_core<C: ConfigSet, R: Clock, T: Clock>() -> f64 {
    let mut e = None;
    let mut prev_end = 0u64;
    clocks::<R, T, C, WarpClock<A_CLK>>(&[], 400_000, |f, sfd, times| {
        let f = &f[..f.len() - 4];
        let mut more = vec![];
        let idle = sfd > prev_end + (24 + 16 + 8) * T::PERIOD;
        prev_end = sfd + 2 * (f.len() as u64 + 4) * T::PERIOD;
        match kind(f) {
            Kind::ArpReq(ip) if ip == UP4 => more.push((arp_reply(UP4, UP4_MAC), 12)),
            Kind::Ns(t) if t == UP6 => more.push((na(UP6, UP6_MAC), 12)),
            Kind::Req(v6, t1) => {
                let (k, want) = if v6 {
                    (CLIENT_TX_LATENCY_V6, want_req6(t1))
                } else {
                    (CLIENT_TX_LATENCY_V4, want_req4(t1))
                };
                assert_eq!(f, &want[..], "request bytes");
                assert!(idle, "the request found the transmit path idle");
                let x = diff_ns(times.unwarp(t1, k), ntp_at(sfd));
                let fam = if v6 { "IPv6" } else { "IPv4" };
                println!("{}/{} {fam}: T1 - tx SFD = {x:+.1} ns", R::NAME, T::NAME);
                e = Some(x);
            }
            k => panic!("unexpected {k:?}"),
        }
        (more, e.is_some())
    });
    e.expect("a request")
}

/// The server's T3 against the SFD instant, by phase of the transmit MII
/// clock (all eight 5 ns phases of a 40 ns `clk_tx`): over a full period the
/// error spans -25..+10 ns for the server-only top too (`rtl/tests/top.rs`
/// samples five of these phases, -10..+10 ns).
const T3_WINDOW: (f64, f64) = (-25.5, 10.5);

/// The client's T1 at 25 MHz MII, eight transmit phases, both families: in
/// the server's T3 window, so the route's K constants apply unchanged to the
/// client's path (same number of channel stages).
#[test]
fn client_t1_error_at_mii_granularity() {
    let mut all = vec![];
    macro_rules! both {
        ($r:ty, $t:ty) => {
            all.push(t1_through_core::<CliCfg4, $r, $t>());
            all.push(t1_through_core::<CliCfg6, $r, $t>());
        };
    }
    both!(Rx40a, Tx40a);
    both!(Rx40b, Tx40b);
    both!(Rx40c, Tx40c);
    both!(Rx40d, Tx40d);
    both!(Rx40e, Tx40e);
    both!(Rx40a, Tx40f);
    both!(Rx40b, Tx40g);
    both!(Rx40c, Tx40h);
    let lo = all.iter().cloned().fold(f64::MAX, f64::min);
    let hi = all.iter().cloned().fold(f64::MIN, f64::max);
    println!("25 MHz MII: client T1 error {lo:+.1}..{hi:+.1} ns");
    for e in all {
        assert!(e >= T3_WINDOW.0 && e <= T3_WINDOW.1, "T1 error {e} ns");
    }
}

/// Server replies (IPv4, IPv6) through the stratum-2 core with the real
/// clock at clocks `R`, `T`: (T2 error, T3 error) per reply, ns.
fn server_through_core<R: Clock, T: Clock>() -> Vec<(f64, f64)> {
    let v4 = exchanges("ntp4_basic.hex").remove(0);
    let v6 = exchanges("ntp6_basic.hex").remove(0);
    let mut got = vec![];
    let ticks = (8 + 2 * (1500 + 15 + 240)) * R::PERIOD + 6000;
    let pre = [(v4.0.clone(), 1500), (v6.0.clone(), 1500)];
    let rx = clocks::<R, T, CliCfg, DisciplinedClock>(&pre, ticks, |f, sfd, _| {
        let f = &f[..f.len() - 4];
        match kind(f) {
            Kind::Reply(v6, ..) => {
                let o = if v6 { 62 } else { 42 };
                got.push((be64(&f[o + 32..o + 40]), be64(&f[o + 40..o + 48]), sfd));
            }
            // The client's own resolution at start-up (not answered).
            Kind::ArpReq(_) | Kind::Ns(_) => {}
            k => panic!("unexpected {k:?}"),
        }
        (vec![], false)
    });
    assert_eq!(got.len(), 2, "two replies");
    got.iter()
        .enumerate()
        .map(|(i, (t2, t3, s))| {
            let e = (diff_ns(*t2, ntp_at(rx[i])), diff_ns(*t3, ntp_at(*s)));
            println!(
                "{}/{} reply {i}: T2 - rx SFD = {:+.1} ns, T3 - tx SFD = {:+.1} ns",
                R::NAME,
                T::NAME,
                e.0,
                e.1
            );
            e
        })
        .collect()
}

/// The server's T2/T3 in the stratum-2 core (real clock, eight transmit
/// phases): the extra channel stage of the UDP arbiter is compensated
/// (`SERVER_ARB_ADJ`), the T3 window is the server-only top's.
#[test]
fn server_t3_error_with_client_wired() {
    let mut e = vec![];
    e.extend(server_through_core::<Rx40a, Tx40a>());
    e.extend(server_through_core::<Rx40b, Tx40b>());
    e.extend(server_through_core::<Rx40c, Tx40c>());
    e.extend(server_through_core::<Rx40d, Tx40d>());
    e.extend(server_through_core::<Rx40e, Tx40e>());
    e.extend(server_through_core::<Rx40a, Tx40f>());
    e.extend(server_through_core::<Rx40b, Tx40g>());
    e.extend(server_through_core::<Rx40c, Tx40h>());
    let lo = e.iter().map(|x| x.1).fold(f64::MAX, f64::min);
    let hi = e.iter().map(|x| x.1).fold(f64::MIN, f64::max);
    println!("25 MHz MII, client wired: server T3 error {lo:+.1}..{hi:+.1} ns");
    for (t2, t3) in e {
        assert!(t2.abs() <= 50.0, "T2 error {t2} ns");
        assert!(t3 >= T3_WINDOW.0 && t3 <= T3_WINDOW.1, "T3 error {t3} ns");
    }
}

// ------------------------------------------------------------ netlist

/// The stratum-2 board lowers to one module tree with the client side, and
/// without the simulation clock.
#[test]
fn with_client_netlist_has_the_client_side() {
    use ntp_fpga::top::{fix_rst_clash, ArtyBoardWithClient};
    let v = fix_rst_clash(&ArtyBoardWithClient::verilog("arty_top"));
    for m in [
        "core_client",
        "core_demux4",
        "core_demux6",
        "core_cmerge",
        "core_nbr",
        "core_uarb4",
        "core_uarb6",
        "core_lat",
        "core_clock",
        "core_sync_servo",
    ] {
        assert!(v.contains(&format!("module arty_top_{m}(")), "module {m}");
    }
    assert!(
        !v.contains("warp"),
        "no simulation clock in the board build"
    );
    assert!(
        !v.contains("idle_samples") && !v.contains("drain_lookup") && !v.contains("idle_nd"),
        "client hooks connected"
    );
}
