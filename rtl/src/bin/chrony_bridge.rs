// SPDX-License-Identifier: Apache-2.0
//! `chrony_bridge`: co-simulation of the server-only design with a real NTP
//! client (chronyd), task CHRONY. Sim-only tool, not part of the design.
//!
//! The whole server top (`ArtyTopClk<ClkRx25, ClkTx25, ..>`, real `mii_rx`
//! and `mii_tx` at the real 25 MHz MII clocks, one simulation tick = 5 ns)
//! is driven on its MII pins from a UDP socket on 127.0.0.1:`--port`:
//!
//! * every datagram received from chronyd becomes an Ethernet/IPv4/UDP
//!   frame (correct IP and UDP checksums, FCS) from a fake client on the
//!   board's subnet (02:00:00:00:00:02, 192.0.2.2, UDP source port = the
//!   datagram's real source port) to the board (`config::Test` addresses:
//!   02:00:00:00:00:01, 192.0.2.1, port 123), clocked into `rxd`/`rx_dv`
//!   one nibble per `clk_rx` edge after a 7-byte preamble and the SFD;
//! * every frame on `txd`/`tx_en` is collected, its FCS and IP/UDP
//!   checksums checked, and a UDP datagram from port 123 to the fake client
//!   is sent back to chronyd from the bridge's own socket (so its source
//!   address and port are those chronyd polls). An ARP request for the fake
//!   client (the server-only top never sends one: it answers the request's
//!   source MAC) would be answered; any other frame is logged.
//!
//! # Time coupling (lockstep virtual time)
//!
//! chronyd runs under `tools/chrony/vtime_shim.so` (LD_PRELOAD): every clock
//! it reads is the simulation's TRUE time, published by this bridge in a
//! shared page (`--shm`, layout in the shim). True time is
//! `T0 + 5 ns x ticks + skipped time`, T0 being the host's wall clock when
//! the bridge starts; the design's `DisciplinedClock` is loaded with the
//! NTP time of T0 at the first simulated edge, so the offset between the
//! design and chronyd starts at zero (up to the 2^-64 s rounding of the
//! accumulator) and the design then runs free (`trim` = 0).
//! The bridge advances the simulation only while chronyd is blocked in a
//! wait (select), up to the virtual deadline of that wait or up to the
//! delivery of a reply, whichever is first; while chronyd computes, virtual
//! time stands still. So chronyd sees an infinitely fast host and a wire
//! with the real MII timing, whatever the speed of the simulator.
//!
//! # Fast-forward (`--no-ff` disables it)
//!
//! Between polls the design is idle. When nothing has moved on the MII pins
//! for [`IDLE_CYCLES`] cycles and chronyd waits for a deadline more than
//! [`FF_MIN_CYCLES`] away, the bridge skips the idle cycles: it asks the
//! sync source below for a clock step of exactly `J x INCR` (J cycles) and
//! adds `J x 10 ns` to the true time. The design's clock thus reads what it
//! would have read after J idle cycles; what is NOT simulated during a skip:
//! the free-running state that only matters when traffic flows (the
//! rate-limiter leak generator, the PHY management polling). The
//! design-minus-true clock difference (`clk_err` in the status line) checks
//! the bookkeeping: it must stay constant.
//!
//! # Synchronisation source
//!
//! [`BridgeSync`] stands for `top::FixedSync` (`--sync fixed`: synchronised,
//! stratum 2, golden reference fields) or `top::Standalone` (`--sync
//! standalone`: stratum 16, LI 3, zero reference fields), with the same
//! output values, plus the sim-only step input of the fast-forward. It is a
//! simulation-only unit (no netlist).
//!
//! # Rate limit
//!
//! `--rate-limit N` (0, 1, 2, 4 or 8, the configured `RATE_LIMIT_PER_S`)
//! selects a configuration set equal to `config::Test` but for that
//! constant. `kod_enable` is tied to 0 in `ArtyCore`: limited requests get
//! silence (with the 1/16 leak), never a KoD; the bridge cannot change that.
//!
//! # Output
//!
//! A status line every `--status` wall seconds (stderr); with `--log FILE`
//! one CSV line per exchange: the instants chronyd sees (T1 = virtual time
//! of the request's pickup, T4 = virtual time of the reply's delivery), the
//! stamps of the reply (T2, T3), the SFD instants on the MII and the stamp
//! errors against them, and the offset/delay chronyd should compute from
//! these four stamps; `--hexdump` prints every payload.
use ntp_fpga::clock::INCR;
use ntp_fpga::clocks::{ClkRx25, ClkTx25};
use ntp_fpga::config::ConfigSet;
use ntp_fpga::ntp_types::Sample;
use ntp_fpga::phy_init;
use ntp_fpga::top::{
    ArtyTopClk, SyncSource, GOLDEN_REF_ID, GOLDEN_REF_ID6, GOLDEN_REF_TS, GOLDEN_ROOT_DELAY,
    GOLDEN_ROOT_DISP,
};
use std::cell::Cell;
use std::collections::{HashMap, VecDeque};
use std::io::Write;
use std::net::{SocketAddr, UdpSocket};
use std::os::fd::AsRawFd;
use std::rc::Rc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use txhdl::comp::trace::{Scope, Traceable};
use txhdl::comp::{now, settle, signal, Clock, DefaultClock, In, Out, Reg, Rx, Running, Unit};
use txhdl::netlist::{Lower, Lowered, Port};
use txhdl::types::{Bit, U};

// ------------------------------------------------------------ addresses

const OUR_MAC: u64 = 0x0200_0000_0001;
const OUR_IP4: u32 = 0xC000_0201;
/// The fake client standing for chronyd on the board's subnet.
const CLI_MAC: u64 = 0x0200_0000_0002;
const CLI_IP4: u32 = 0xC000_0202;

/// Seconds between the NTP era 0 origin (1900) and the Unix epoch.
const NTP_UNIX: u128 = 2_208_988_800;
const NS: u128 = 1_000_000_000;

/// Cycles without activity on the pins before the design counts as idle.
const IDLE_CYCLES: u64 = 3000;
/// Smallest skip worth a step (cycles).
const FF_MIN_CYCLES: u64 = 2000;
/// Left before the deadline after a skip (cycles), simulated normally.
const FF_MARGIN_CYCLES: u64 = 100;

// ------------------------------------------------------------ configuration

/// `config::Test` with `RATE_LIMIT_PER_S` = `RL`.
struct ChronyCfg<const RL: u16>;
impl<const RL: u16> ConfigSet for ChronyCfg<RL> {
    const LOCAL_MAC: u64 = <ntp_fpga::config::Test as ConfigSet>::LOCAL_MAC;
    const LOCAL_IP4: u32 = <ntp_fpga::config::Test as ConfigSet>::LOCAL_IP4;
    const GATEWAY_IP4: u32 = <ntp_fpga::config::Test as ConfigSet>::GATEWAY_IP4;
    const NETMASK4: u32 = <ntp_fpga::config::Test as ConfigSet>::NETMASK4;
    const GATEWAY_IP6: u128 = <ntp_fpga::config::Test as ConfigSet>::GATEWAY_IP6;
    const LOCAL_GUA: u128 = <ntp_fpga::config::Test as ConfigSet>::LOCAL_GUA;
    const GUA_ENABLED: bool = <ntp_fpga::config::Test as ConfigSet>::GUA_ENABLED;
    const UPSTREAM: [(bool, u128, bool); 4] = <ntp_fpga::config::Test as ConfigSet>::UPSTREAM;
    const POLL_LOG2: u8 = <ntp_fpga::config::Test as ConfigSet>::POLL_LOG2;
    const PRECISION_LOG2: i8 = <ntp_fpga::config::Test as ConfigSet>::PRECISION_LOG2;
    const RATE_LIMIT_PER_S: u16 = RL;
    const RATE_SECRET: u64 = <ntp_fpga::config::Test as ConfigSet>::RATE_SECRET;
    const ENTROPY_SEED: u64 = 0x0123_4567_89AB_CDEF;
}

// ------------------------------------------------------------ sync source

/// Shared between [`BridgeSync`] and the bench.
#[derive(Default)]
struct SyncCtl {
    /// Synchronised stratum 2 (golden fields) or stratum 16 (zero fields).
    fixed: Cell<bool>,
    /// The design's `time` (32.64) seen at the last `clk_sys` edge.
    time: Cell<u128>,
    /// Pending clock step (fast-forward), applied once.
    step: Cell<u128>,
}

/// `FixedSync` / `Standalone` outputs plus a sim-only clock step.
#[derive(Default)]
struct BridgeSync {
    ctl: Rc<SyncCtl>,
}

impl Traceable for BridgeSync {
    fn trace(&self, _scope: &Scope) {}
}
impl Port for BridgeSync {}
impl Lower for BridgeSync {
    fn lowered_as(_name: &str) -> Lowered {
        panic!("BridgeSync is a simulation-only unit: it has no netlist")
    }
}

type Outs = (
    Out<Bit>,
    Out<U<8>>,
    Out<U<32>>,
    Out<U<32>>,
    Out<U<64>>,
    Out<U<32>>,
    Out<U<32>>,
    Out<U<32>>,
    Out<U<96>>,
    Out<Bit>,
);

impl Unit<(In<U<96>>, In<U<8>>, In<U<8>>, Rx<Sample>, Rx<U<2>>), Outs> for BridgeSync {
    async fn run(
        &mut self,
        (time, _precision, _poll, samples, miss): (In<U<96>>, In<U<8>>, In<U<8>>, Rx<Sample>, Rx<U<2>>),
        (synced, stratum, ref_id, ref_id6, ref_ts, root_delay, root_disp, trim, step, step_en): Outs,
    ) {
        loop {
            DefaultClock::rising().await;
            let _ = samples.recv();
            let _ = miss.recv();
            self.ctl.time.set(time.get().raw());
            let f = self.ctl.fixed.get();
            // Same values as top::Fixed<true> / top::Standalone.
            synced.set(Bit::from_bool(f));
            stratum.set(U::<8>::from(if f { 2u8 } else { 16u8 }));
            ref_id.set(U::<32>::from(if f { GOLDEN_REF_ID } else { 0 }));
            ref_id6.set(U::<32>::from(if f { GOLDEN_REF_ID6 } else { 0 }));
            ref_ts.set(U::<64>::from(if f { GOLDEN_REF_TS } else { 0 }));
            root_delay.set(U::<32>::from(if f { GOLDEN_ROOT_DELAY } else { 0 }));
            root_disp.set(U::<32>::from(if f { GOLDEN_ROOT_DISP } else { 0 }));
            trim.set(U::<32>::from(0u32));
            let s = self.ctl.step.replace(0);
            step.set(U::<96>::from(s));
            step_en.set(Bit::from_bool(s != 0));
        }
    }
}
impl SyncSource for BridgeSync {}

// ------------------------------------------------------------ frames

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

/// RFC 1071 sum, not complemented, not folded.
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

fn mac_bytes(m: u64) -> [u8; 6] {
    let b = m.to_be_bytes();
    [b[2], b[3], b[4], b[5], b[6], b[7]]
}

/// Ethernet frame (no FCS) padded with zeros to 60 bytes.
fn eth(dst: u64, src: u64, et: u16, payload: &[u8]) -> Vec<u8> {
    let mut v = mac_bytes(dst).to_vec();
    v.extend_from_slice(&mac_bytes(src));
    v.extend_from_slice(&et.to_be_bytes());
    v.extend_from_slice(payload);
    while v.len() < 60 {
        v.push(0);
    }
    v
}

/// IPv4 header (IHL 5, DF, TTL 64) with its checksum; `id` in bytes 4-5.
fn ipv4_hdr(src: u32, dst: u32, proto: u8, id: u16, data_len: usize) -> Vec<u8> {
    let tl = 20 + data_len;
    let i = id.to_be_bytes();
    let mut h = vec![0x45, 0, (tl >> 8) as u8, tl as u8, i[0], i[1], 0x40, 0, 64, proto, 0, 0];
    h.extend_from_slice(&src.to_be_bytes());
    h.extend_from_slice(&dst.to_be_bytes());
    let c = !fold(sum16(0, &h));
    h[10] = (c >> 8) as u8;
    h[11] = c as u8;
    h
}

/// UDP datagram over IPv4 with a correct checksum.
fn udp4(src: u32, dst: u32, sport: u16, dport: u16, pay: &[u8]) -> Vec<u8> {
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

/// A frame received on `txd`, decoded.
enum Sent {
    /// IPv4 UDP: (src ip, dst ip, sport, dport, payload, dst mac).
    Udp4(u32, u32, u16, u16, Vec<u8>, u64),
    /// ARP request: (sender ip, target ip).
    ArpReq(u32, u32),
    Other(String),
}

fn be(b: &[u8]) -> u128 {
    b.iter().fold(0u128, |a, &x| (a << 8) | x as u128)
}

/// Decodes a transmitted frame (bytes after the SFD, FCS included),
/// checking the FCS and, for IPv4 UDP, both checksums.
fn parse_sent(raw: &[u8]) -> Sent {
    let n = raw.len();
    if n < 64 {
        return Sent::Other(format!("runt frame of {n} bytes"));
    }
    let (body, fcs) = raw.split_at(n - 4);
    if u32::from_le_bytes([fcs[0], fcs[1], fcs[2], fcs[3]]) != crc32(body) {
        return Sent::Other("bad FCS".into());
    }
    let dst_mac = be(&body[0..6]) as u64;
    let et = u16::from_be_bytes([body[12], body[13]]);
    if et == 0x0806 && body.len() >= 42 {
        let a = &body[14..];
        if a[6..8] == [0, 1] {
            return Sent::ArpReq(be(&a[14..18]) as u32, be(&a[24..28]) as u32);
        }
        return Sent::Other("ARP reply".into());
    }
    if et != 0x0800 {
        return Sent::Other(format!("ethertype {et:04x}"));
    }
    let ip = &body[14..];
    if fold(sum16(0, &ip[..20])) != 0xffff {
        return Sent::Other("bad IPv4 header checksum".into());
    }
    if ip[9] != 17 {
        return Sent::Other(format!("IPv4 protocol {}", ip[9]));
    }
    let tl = u16::from_be_bytes([ip[2], ip[3]]) as usize;
    let u = &ip[20..tl];
    let len = u16::from_be_bytes([u[4], u[5]]) as usize;
    if len != u.len() {
        return Sent::Other("bad UDP length".into());
    }
    let mut a = sum16(0, &ip[12..20]);
    a += 17 + len as u32;
    a = sum16(a, u);
    if fold(a) != 0xffff {
        return Sent::Other("bad UDP checksum".into());
    }
    Sent::Udp4(
        be(&ip[12..16]) as u32,
        be(&ip[16..20]) as u32,
        u16::from_be_bytes([u[0], u[1]]),
        u16::from_be_bytes([u[2], u[3]]),
        u[8..].to_vec(),
        dst_mac,
    )
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

fn be64(b: &[u8]) -> u64 {
    u64::from_be_bytes(b[..8].try_into().unwrap())
}

// ------------------------------------------------------------ time

/// NTP 32.32 of a Unix time in ns.
fn ntp_of_ns(ns: u128) -> u64 {
    ((((ns + NTP_UNIX * NS) << 32) + NS / 2) / NS) as u64
}

/// Signed difference of two 32.32 times, in ns.
fn diff_ns(a: u64, b: u64) -> f64 {
    (a.wrapping_sub(b) as i64) as f64 * 1e9 / 4_294_967_296.0
}

// ------------------------------------------------------------ shared page

const VT_MAGIC: u64 = 0x5654_4d43_4852_4e59;
const W_MAGIC: usize = 0;
const W_VT_NS: usize = 1;
const W_WAIT_SEQ: usize = 2;
const W_WAITING: usize = 3;
const W_DEADLINE: usize = 4;
const W_T0_NS: usize = 5;
const W_STOP: usize = 6;
const NONE_DEADLINE: u64 = !0;

extern "C" {
    fn mmap(addr: *mut u8, len: usize, prot: i32, flags: i32, fd: i32, off: i64) -> *mut u8;
}

/// The page shared with `vtime_shim.so` (a 4096-byte file, mmap'ed).
struct Shm {
    p: *const AtomicU64,
}

impl Shm {
    fn create(path: &str, t0: u64) -> Shm {
        let f = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(true)
            .open(path)
            .unwrap_or_else(|e| panic!("{path}: {e}"));
        f.set_len(4096).unwrap();
        // SAFETY: a fresh 4096-byte shared mapping of a file we own; it is
        // never unmapped (lives until exit), accessed only through atomics.
        let p = unsafe { mmap(std::ptr::null_mut(), 4096, 3, 1, f.as_raw_fd(), 0) };
        assert!(!p.is_null() && p as isize != -1, "mmap of {path} failed");
        let s = Shm { p: p as *const AtomicU64 };
        s.st(W_VT_NS, t0);
        s.st(W_T0_NS, t0);
        s.st(W_DEADLINE, NONE_DEADLINE);
        s.st(W_MAGIC, VT_MAGIC);
        s
    }
    fn w(&self, i: usize) -> &AtomicU64 {
        // SAFETY: i < 512 words of the live 4096-byte mapping.
        unsafe { &*self.p.add(i) }
    }
    fn ld(&self, i: usize) -> u64 {
        self.w(i).load(Ordering::SeqCst)
    }
    fn st(&self, i: usize, v: u64) {
        self.w(i).store(v, Ordering::SeqCst)
    }
}

// ------------------------------------------------------------ options

struct Opts {
    port: u16,
    fixed: bool,
    rate_limit: u16,
    duration_s: f64,
    shm: String,
    log: Option<String>,
    hexdump: bool,
    ff: bool,
    status_s: f64,
    max_wall_s: f64,
}

fn usage() -> ! {
    eprintln!(
        "usage: chrony_bridge --shm FILE [--port P (12300)] [--sync fixed|standalone] \
         [--rate-limit 0|1|2|4|8 (8)] [--duration VIRTUAL_S (60)] [--log CSV] [--hexdump] \
         [--no-ff] [--status WALL_S (5)] [--max-wall WALL_S (3600)]"
    );
    std::process::exit(2)
}

fn opts() -> Opts {
    let mut o = Opts {
        port: 12300,
        fixed: true,
        rate_limit: 8,
        duration_s: 60.0,
        shm: String::new(),
        log: None,
        hexdump: false,
        ff: true,
        status_s: 5.0,
        max_wall_s: 3600.0,
    };
    let a: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    let val = |i: usize| a.get(i + 1).cloned().unwrap_or_else(|| usage());
    while i < a.len() {
        match a[i].as_str() {
            "--port" => o.port = val(i).parse().unwrap_or_else(|_| usage()),
            "--sync" => {
                o.fixed = match val(i).as_str() {
                    "fixed" => true,
                    "standalone" => false,
                    _ => usage(),
                }
            }
            "--rate-limit" => o.rate_limit = val(i).parse().unwrap_or_else(|_| usage()),
            "--duration" => o.duration_s = val(i).parse().unwrap_or_else(|_| usage()),
            "--shm" => o.shm = val(i),
            "--log" => o.log = Some(val(i)),
            "--status" => o.status_s = val(i).parse().unwrap_or_else(|_| usage()),
            "--max-wall" => o.max_wall_s = val(i).parse().unwrap_or_else(|_| usage()),
            "--hexdump" => {
                o.hexdump = true;
                i += 1;
                continue;
            }
            "--no-ff" => {
                o.ff = false;
                i += 1;
                continue;
            }
            _ => usage(),
        }
        i += 2;
    }
    if o.shm.is_empty() {
        usage();
    }
    o
}

fn main() {
    let o = opts();
    match o.rate_limit {
        0 => run::<ChronyCfg<0>>(&o),
        1 => run::<ChronyCfg<1>>(&o),
        2 => run::<ChronyCfg<2>>(&o),
        4 => run::<ChronyCfg<4>>(&o),
        8 => run::<ChronyCfg<8>>(&o),
        _ => {
            eprintln!("--rate-limit: one of 0, 1, 2, 4, 8");
            std::process::exit(2)
        }
    }
}

// ------------------------------------------------------------ the bench

/// A request on its way (pickup until its reply, if any).
struct Pending {
    /// Virtual time (ns) of the pickup: chronyd's T1.
    t1_ns: u128,
    /// Virtual time of the SFD nibble on `rxd` (the instant T2 estimates).
    rx_sfd_ns: Option<u128>,
}

/// One nibble on the receive pins; `Some(id)` on a frame's SFD nibble.
type Nib = (u8, bool, Option<u64>);

fn run<C: ConfigSet + 'static>(o: &Opts) {
    let sock = UdpSocket::bind(("127.0.0.1", o.port)).unwrap_or_else(|e| panic!("bind: {e}"));
    sock.set_nonblocking(true).unwrap();
    let wall0 = Instant::now();
    let t0_ns = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_nanos();
    let shm = Shm::create(&o.shm, t0_ns as u64);
    let mut log = o
        .log
        .as_ref()
        .map(|p| std::fs::File::create(p).unwrap_or_else(|e| panic!("{p}: {e}")));
    if let Some(f) = log.as_mut() {
        writeln!(
            f,
            "idx,sport,len,t1_ns,t4_ns,t2,t3,rx_sfd_ns,tx_sfd_ns,e2_ns,e3_ns,offset_ns,delay_ns,\
             req_hex,reply_hex"
        )
        .unwrap();
    }

    // Pins.
    let (rxd_o, rxd) = signal::<U<4>, ClkRx25>();
    let (dv_o, dv) = signal::<Bit, ClkRx25>();
    let (er_o, er) = signal::<Bit, ClkRx25>();
    let (mdio_o_, mdio_i) = signal::<Bit, DefaultClock>();
    let (txd_o, txd) = signal::<U<4>, ClkTx25>();
    let (en_o, en) = signal::<Bit, ClkTx25>();
    let outs: Vec<(Out<Bit>, In<Bit>)> = (0..7).map(|_| signal::<Bit, DefaultClock>()).collect();
    let mut outs = outs.into_iter();
    let mut ob = || outs.next().unwrap().0;

    let mut top = ArtyTopClk::<ClkRx25, ClkTx25, C, BridgeSync, phy_init::Sim>::default();
    let ctl = top.core.sync.ctl.clone();
    ctl.fixed.set(o.fixed);
    // The design's clock starts at the NTP time of T0 (32.64), with its
    // ghost registers in the state of a running clock (as the system tests'
    // `start_clock`).
    let start = ((t0_ns + NTP_UNIX * NS) << 64) / NS;
    {
        let c = &mut top.core.clock;
        c.acc = Reg::new(U::<96>::from(start));
        c.g_valid = Reg::new(Bit::One);
        c.g_prev = Reg::new(U::<96>::from(start - INCR));
        c.g_inc = Reg::new(U::<96>::from(INCR));
        c.g_any = Reg::new(Bit::One);
    }
    mdio_o_.set(Bit::One); // no PHY: the pulled-up line
    while now() % 2 != 0 {
        settle();
    }
    let tick0 = now();
    let mut sim = Running::new(top.run(
        (rxd, dv, er, mdio_i),
        (txd_o, en_o, ob(), ob(), ob(), ob(), ob(), ob(), ob()),
    ));

    // Virtual true time (ns) at tick t: T0 + 5 ns per tick + skipped time.
    let mut ff_ns: u128 = 0;
    let vt_at = |t: u64, ff: u128| t0_ns + 5 * (t - tick0) as u128 + ff;
    let end_ns = t0_ns + (o.duration_s * 1e9) as u128;

    let mut wire: VecDeque<Nib> = VecDeque::new();
    let mut nib: Vec<u8> = vec![];
    let mut tx_sfd_ns = 0u128;
    let mut pending: HashMap<(u16, u64), (u64, Pending, Vec<u8>)> = HashMap::new();
    let mut by_id: HashMap<u64, (u16, u64)> = HashMap::new();
    let mut next_id = 0u64;
    let mut ip_id = 0u16;
    let mut last_activity = 0u64; // cycle
    let mut cycles = 0u64; // simulated clk_sys cycles
    let mut skipped = 0u64; // skipped cycles
    let (mut n_req, mut n_rep, mut n_other, mut n_unmatched, mut n_stuck) = (0u64, 0u64, 0u64, 0u64, 0u64);
    let mut blocked_seq: Option<(u64, Instant)> = None;
    let mut chrony_addr: HashMap<u16, SocketAddr> = HashMap::new();
    let mut last_status = Instant::now();
    let mut buf = [0u8; 2048];
    let mut sum_off = 0f64;
    // Wall time spent stepping the simulation (speed measurement).
    let mut sim_wall = Duration::ZERO;
    let (mut clk_err_min, mut clk_err_max) = (f64::MAX, f64::MIN);

    eprintln!(
        "chrony_bridge: 127.0.0.1:{} -> board 192.0.2.1:123, sync {}, rate limit {}/s, \
         duration {} virtual s, fast-forward {}, T0 {} ns",
        o.port,
        if o.fixed { "fixed (stratum 2)" } else { "standalone (stratum 16)" },
        C::RATE_LIMIT_PER_S,
        o.duration_s,
        o.ff,
        t0_ns
    );

    'outer: loop {
        let vt = vt_at(now(), ff_ns);
        shm.st(W_VT_NS, vt as u64);
        if vt >= end_ns {
            break;
        }
        if wall0.elapsed().as_secs_f64() > o.max_wall_s {
            eprintln!("chrony_bridge: --max-wall reached");
            break;
        }
        if last_status.elapsed().as_secs_f64() >= o.status_s {
            last_status = Instant::now();
            let w = wall0.elapsed().as_secs_f64();
            eprintln!(
                "status: virt {:.3} s, wall {:.1} s, simulated {} cycles ({:.0} cycles/s), \
                 skipped {} cycles, requests {}, replies {}, clk_err {:+.1} ns, \
                 chronyd waiting {} (wait {}, deadline in {:.6} s)",
                (vt - t0_ns) as f64 / 1e9,
                w,
                cycles,
                cycles as f64 / w,
                skipped,
                n_req,
                n_rep,
                diff_ns((ctl.time.get() >> 32) as u64, ntp_of_ns(vt)),
                shm.ld(W_WAITING),
                shm.ld(W_WAIT_SEQ),
                match shm.ld(W_DEADLINE) {
                    NONE_DEADLINE => f64::INFINITY,
                    d => (d as f64 - vt as f64) / 1e9,
                }
            );
        }
        // May the simulation advance? Only while chronyd waits, and not
        // again in the same wait after a delivery or its deadline.
        let waiting = shm.ld(W_WAITING) == 1;
        let seq = shm.ld(W_WAIT_SEQ);
        let deadline = shm.ld(W_DEADLINE);
        if !waiting || shm.ld(W_WAIT_SEQ) != seq {
            std::thread::sleep(Duration::from_micros(20));
            continue;
        }
        if let Some((s, since)) = blocked_seq {
            if s == seq {
                if since.elapsed() < Duration::from_millis(500) {
                    std::thread::sleep(Duration::from_micros(20));
                    continue;
                }
                // chronyd did not wake up for what we delivered (a socket it
                // had already closed): resume.
                n_stuck += 1;
                eprintln!("chrony_bridge: chronyd did not take a delivery; resuming");
            }
            blocked_seq = None;
        }
        // Requests chronyd sent before it blocked.
        loop {
            match sock.recv_from(&mut buf) {
                Ok((n, src)) => {
                    let SocketAddr::V4(a) = src else { continue };
                    if !a.ip().is_loopback() {
                        continue;
                    }
                    let pay = &buf[..n];
                    n_req += 1;
                    chrony_addr.insert(a.port(), src);
                    let xmit = if n >= 48 { be64(&pay[40..48]) } else { 0 };
                    if o.hexdump {
                        eprintln!(
                            "req  #{} at {:.9} from :{} ({} bytes): {}",
                            n_req,
                            (vt - t0_ns) as f64 / 1e9,
                            a.port(),
                            n,
                            hex(pay)
                        );
                    }
                    ip_id = ip_id.wrapping_add(1);
                    let u = udp4(CLI_IP4, OUR_IP4, a.port(), 123, pay);
                    let mut ip = ipv4_hdr(CLI_IP4, OUR_IP4, 17, ip_id, u.len());
                    ip.extend_from_slice(&u);
                    let f = eth(OUR_MAC, CLI_MAC, 0x0800, &ip);
                    let id = next_id;
                    next_id += 1;
                    // Inter-frame gap (12 bytes) before a frame queued
                    // behind another, preamble, SFD, frame, FCS.
                    if !wire.is_empty() {
                        wire.extend(std::iter::repeat_n((0, false, None), 24));
                    }
                    wire.extend(std::iter::repeat_n((5, true, None), 15));
                    wire.push_back((0xd, true, Some(id)));
                    let mut w = f.clone();
                    w.extend_from_slice(&crc32(&f).to_le_bytes());
                    for b in w {
                        wire.push_back((b & 15, true, None));
                        wire.push_back((b >> 4, true, None));
                    }
                    let key = (a.port(), xmit);
                    by_id.insert(id, key);
                    pending.insert(
                        key,
                        (n_req, Pending { t1_ns: vt, rx_sfd_ns: None }, pay.to_vec()),
                    );
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(e) => panic!("recv: {e}"),
            }
        }
        if deadline != NONE_DEADLINE && vt >= deadline as u128 {
            blocked_seq = Some((seq, Instant::now()));
            continue;
        }
        // Fast-forward over idle time.
        if o.ff
            && deadline != NONE_DEADLINE
            && wire.is_empty()
            && nib.is_empty()
            && cycles - last_activity >= IDLE_CYCLES
            && ctl.step.get() == 0
        {
            let left = (deadline as u128 - vt) / 10;
            if left > (FF_MIN_CYCLES + FF_MARGIN_CYCLES) as u128 {
                // A multiple of 4 cycles keeps the MII clock phases.
                let j = ((left as u64 - FF_MARGIN_CYCLES) / 4) * 4;
                ctl.step.set(j as u128 * INCR);
                ff_ns += j as u128 * 10;
                skipped += j;
                continue;
            }
        }
        // Simulate up to 256 cycles (512 ticks), the deadline or a delivery.
        let ts = Instant::now();
        for _ in 0..512 {
            let t = now();
            if ClkRx25::rising_at(t) {
                let (d, v, mark) = wire.pop_front().unwrap_or((0, false, None));
                rxd_o.set(U::<4>::from(d));
                dv_o.set(Bit::from_bool(v));
                er_o.set(Bit::Zero);
                if v {
                    last_activity = cycles;
                }
                if let Some(id) = mark {
                    if let Some(k) = by_id.remove(&id) {
                        if let Some(p) = pending.get_mut(&k) {
                            p.1.rx_sfd_ns = Some(vt_at(t, ff_ns));
                        }
                    }
                }
            }
            sim.step();
            if t % 2 == 0 {
                cycles += 1;
            }
            if ClkTx25::rising_at(t) {
                if en.get().to_bool() {
                    last_activity = cycles;
                    if nib.len() == 15 {
                        // As RealBench: the pin shows the register one
                        // clk_tx period after the edge that put the SFD
                        // on the wire.
                        tx_sfd_ns = vt_at(t - ClkTx25::PERIOD, ff_ns);
                    }
                    nib.push(txd.get().raw() as u8);
                } else if !nib.is_empty() {
                    let ok = nib.len() >= 16
                        && nib.len() % 2 == 0
                        && nib[..15].iter().all(|&n| n == 5)
                        && nib[15] == 0xd;
                    let bytes: Vec<u8> = if ok {
                        nib[16..].chunks(2).map(|c| c[0] | (c[1] << 4)).collect()
                    } else {
                        vec![]
                    };
                    nib.clear();
                    // T4: the instant chronyd will read once it wakes up,
                    // i.e. the time published from now on (after this tick).
                    let t4 = vt_at(now(), ff_ns);
                    shm.st(W_VT_NS, t4 as u64);
                    match parse_sent(&bytes) {
                        Sent::Udp4(src, dst, 123, dport, pay, mac)
                            if src == OUR_IP4 && dst == CLI_IP4 && mac == CLI_MAC =>
                        {
                            n_rep += 1;
                            let origin = if pay.len() >= 48 { be64(&pay[24..32]) } else { 0 };
                            if o.hexdump {
                                eprintln!(
                                    "rep  #{} at {:.9} to :{} ({} bytes): {}",
                                    n_rep,
                                    (t4 - t0_ns) as f64 / 1e9,
                                    dport,
                                    pay.len(),
                                    hex(&pay)
                                );
                            }
                            if let Some((idx, p, req)) = pending.remove(&(dport, origin)) {
                                let t2 = be64(&pay[32..40]);
                                let t3 = be64(&pay[40..48]);
                                let rx_sfd = p.rx_sfd_ns.unwrap_or(0);
                                let e2 = diff_ns(t2, ntp_of_ns(rx_sfd));
                                let e3 = diff_ns(t3, ntp_of_ns(tx_sfd_ns));
                                let t1n = ntp_of_ns(p.t1_ns);
                                let t4n = ntp_of_ns(t4);
                                let off = (diff_ns(t2, t1n) + diff_ns(t3, t4n)) / 2.0;
                                let del = diff_ns(t4n, t1n) - diff_ns(t3, t2);
                                sum_off += off;
                                if let Some(f) = log.as_mut() {
                                    writeln!(
                                        f,
                                        "{idx},{dport},{},{},{t4},{t2:016x},{t3:016x},{rx_sfd},{tx_sfd_ns},\
                                         {e2:.2},{e3:.2},{off:.2},{del:.2},{},{}",
                                        req.len(),
                                        p.t1_ns,
                                        hex(&req),
                                        hex(&pay)
                                    )
                                    .unwrap();
                                    f.flush().unwrap();
                                }
                            } else {
                                n_unmatched += 1;
                            }
                            if let Some(a) = chrony_addr.get(&dport) {
                                let _ = sock.send_to(&pay, a);
                            }
                            // Stop here: chronyd must see the reply at T4.
                            blocked_seq = Some((seq, Instant::now()));
                            let e = diff_ns((ctl.time.get() >> 32) as u64, ntp_of_ns(t4));
                            clk_err_min = clk_err_min.min(e);
                            clk_err_max = clk_err_max.max(e);
                            sim_wall += ts.elapsed();
                            continue 'outer;
                        }
                        Sent::ArpReq(spa, tpa) if tpa == CLI_IP4 => {
                            // Answer for the fake client.
                            let mut a = vec![0, 1, 8, 0, 6, 4, 0, 2];
                            a.extend_from_slice(&mac_bytes(CLI_MAC));
                            a.extend_from_slice(&CLI_IP4.to_be_bytes());
                            a.extend_from_slice(&mac_bytes(OUR_MAC));
                            a.extend_from_slice(&spa.to_be_bytes());
                            let f = eth(OUR_MAC, CLI_MAC, 0x0806, &a);
                            if !wire.is_empty() {
                                wire.extend(std::iter::repeat_n((0, false, None), 24));
                            }
                            wire.extend(std::iter::repeat_n((5, true, None), 15));
                            wire.push_back((0xd, true, None));
                            let mut w = f.clone();
                            w.extend_from_slice(&crc32(&f).to_le_bytes());
                            for b in w {
                                wire.push_back((b & 15, true, None));
                                wire.push_back((b >> 4, true, None));
                            }
                            eprintln!("chrony_bridge: answered an ARP request of the design");
                        }
                        Sent::Udp4(..) => {
                            n_other += 1;
                            eprintln!("chrony_bridge: unexpected UDP frame: {}", hex(&bytes));
                        }
                        Sent::ArpReq(..) => {
                            n_other += 1;
                            eprintln!("chrony_bridge: ARP request not for the client: {}", hex(&bytes));
                        }
                        Sent::Other(why) => {
                            n_other += 1;
                            eprintln!("chrony_bridge: transmitted frame ignored ({why}): {}", hex(&bytes));
                        }
                    }
                }
            }
            let v = vt_at(now(), ff_ns);
            shm.st(W_VT_NS, v as u64);
            if deadline != NONE_DEADLINE && v >= deadline as u128 {
                break;
            }
        }
        sim_wall += ts.elapsed();
    }
    shm.st(W_STOP, 1);
    let w = wall0.elapsed().as_secs_f64();
    let vt = vt_at(now(), ff_ns);
    eprintln!(
        "chrony_bridge: done: virt {:.3} s, wall {:.1} s, simulated {} cycles ({:.0} cycles/s, \
         {:.0} cycles per second of simulation, {:.0}x slower than the hardware), skipped {} \
         cycles, requests {}, replies {}, unanswered requests {}, unmatched replies {}, other frames {}, deliveries not \
         taken {}, mean expected offset {:+.1} ns, clk_err {:+.1}..{:+.1} ns at deliveries",
        (vt - t0_ns) as f64 / 1e9,
        w,
        cycles,
        cycles as f64 / w,
        cycles as f64 / sim_wall.as_secs_f64().max(1e-9),
        1e8 * sim_wall.as_secs_f64() / cycles.max(1) as f64,
        skipped,
        n_req,
        n_rep,
        pending.len(),
        n_unmatched,
        n_other,
        n_stuck,
        if n_rep > 0 { sum_off / n_rep as f64 } else { 0.0 },
        if n_rep > 0 { clk_err_min } else { f64::NAN },
        if n_rep > 0 { clk_err_max } else { f64::NAN }
    );
    drop(sim);
}
