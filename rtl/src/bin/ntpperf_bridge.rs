// SPDX-License-Identifier: Apache-2.0
//! `ntpperf_bridge`: load test of the server-only design with ntpperf
//! (Miroslav Lichvar's NTP server benchmark), task NTPPERF. Sim-only tool,
//! not part of the design.
//!
//! ```text
//! ntpperf_bridge [OPTIONS] -- NTPPERF ARGS...
//! ```
//!
//! The bridge runs the whole server top (`ArtyTopClk<ClkRx25, ClkTx25, ..>`,
//! the core of `ArtyBoard`: real `mii_rx`/`mii_tx` at the real 25 MHz MII
//! clocks, one simulation tick = 5 ns) and starts ntpperf as its child, with
//! the replacement libpcap of `tools/ntpperf/pcap_shim/` (built in) and the
//! time shim `ntpperf_vtime.so` (LD_PRELOAD). The contract between both
//! sides is `tools/ntpperf/pcap_shim/simlink.h`.
//!
//! # Ethernet level
//!
//! ntpperf writes raw Ethernet frames (requests from many simulated clients
//! of its `-s` network) on its "interface", an AF_UNIX SOCK_SEQPACKET
//! socketpair; each frame gets its FCS here and goes into a model of the
//! sender's NIC / switch egress queue: a FIFO of at most `--queue-bytes`
//! frame bytes (FCS included) waiting for the wire, tail drop, served at the
//! 100 Mb/s line rate (preamble + SFD, frame, 12-byte inter-frame gap) on
//! `rxd`/`rx_dv`, one nibble per `clk_rx` edge. Every frame on `txd`/`tx_en`
//! is collected, its FCS checked and sent back to ntpperf (without FCS) with
//! a virtual timestamp: with `-H` (time stamp type ADAPTER*) the instant of
//! its SFD on the TX pins (as `RealBench` of `rtl/tests/top.rs`), otherwise
//! the instant its last nibble left the pins (a host software stamp).
//!
//! # Time coupling
//!
//! The virtual time is the simulation's true time, `T0 + 5 ns x ticks +
//! skipped time`, published in a shared page (`simlink.h`). The design's
//! `DisciplinedClock` is loaded with the NTP time of T0 (fixed: `--t0`, for
//! reproducible runs) and runs free. Both ntpperf processes (the measuring
//! parent and the forked sender) read only this time (shim). The bridge
//! advances time only when every process is "ready" (waiting for a virtual
//! deadline, or blocked in I/O that needs time to advance), and then up to
//! the EARLIEST deadline: the sender's deadline is the send time of its next
//! request (exact), the parent's a quantum (1 ms) after its last useful
//! action. So ntpperf behaves as on an infinitely fast host: each request
//! leaves exactly at its scheduled virtual instant.
//!
//! # Fast-forward (`--no-ff` disables it)
//!
//! As in `chrony_bridge`: when nothing moved on the MII pins for
//! `--idle-cycles` cycles and no frame is due, the bridge skips the idle
//! cycles up to the next frame or deadline with an exact clock step of the
//! design (`J x INCR`, J a multiple of 4 cycles to keep the MII phases).
//! Not simulated during a skip: the free-running state that only matters
//! with traffic (the leak generator of the rate limiter, the MDIO polling).
//!
//! # Configuration
//!
//! `PerfCfg<RL, KOD>` = `config::Test` but for the IPv4 addresses
//! (192.168.123.1/24: ntpperf refuses non-private networks, 192.0.2.1 of
//! `Test` is one), `RATE_LIMIT_PER_S` = `--rate-limit` (0 unlimited, 1, 2,
//! 4, 8, 16) and `KOD_ENABLE` = `--kod`. MAC 02:00:00:00:00:01, IPv6
//! addresses and `RATE_SECRET` are `Test`'s. `--sync fixed` = `FixedSync`
//! (stratum 2, golden reference fields), `standalone` = stratum 16.
//!
//! # Output
//!
//! ntpperf's table on stdout (inherited), status lines and a summary on
//! stderr, and with `--log FILE` one CSV line per request (send instant,
//! queue drop, wire start, RX SFD, source, port, rate-limiter bucket, reply
//! stamps and their errors against the SFD instants).
use ntp_fpga::clock::INCR;
use ntp_fpga::clocks::{ClkRx25, ClkTx25};
use ntp_fpga::config::ConfigSet;
use ntp_fpga::ntp_server::rl_index;
use ntp_fpga::ntp_types::Sample;
use ntp_fpga::phy_init;
use ntp_fpga::top::{
    ArtyTopClk, SyncSource, GOLDEN_REF_ID, GOLDEN_REF_ID6, GOLDEN_REF_TS, GOLDEN_ROOT_DELAY,
    GOLDEN_ROOT_DISP,
};
use std::cell::Cell;
use std::collections::{HashMap, VecDeque};
use std::io::Write;
use std::os::fd::AsRawFd;
use std::process::{Child, Command};
use std::rc::Rc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};
use txhdl::comp::trace::{Scope, Traceable};
use txhdl::comp::{now, settle, signal, Clock, DefaultClock, In, Out, Reg, Rx, Running, Unit};
use txhdl::netlist::{Lower, Lowered, Port};
use txhdl::types::{Bit, U};

// ------------------------------------------------------------ constants

/// Seconds between the NTP era 0 origin (1900) and the Unix epoch.
const NTP_UNIX: u128 = 2_208_988_800;
const NS: u128 = 1_000_000_000;
/// Default virtual start: 2026-05-28T13:20:00Z (a whole second).
const DEFAULT_T0_S: u64 = 1_780_000_000;
/// Wire bytes added to a frame (with FCS): preamble + SFD.
const PREAMBLE: usize = 8;
/// Inter-frame gap, in nibble times (12 bytes).
const IFG_NIBBLES: u32 = 24;
/// Smallest skip worth a clock step (cycles).
const FF_MIN_CYCLES: u64 = 16;

// ------------------------------------------------------------ configuration

/// `config::Test` with ntpperf-compatible IPv4 addresses, `RL` and `KOD`.
struct PerfCfg<const RL: u16, const KOD: bool>;
impl<const RL: u16, const KOD: bool> ConfigSet for PerfCfg<RL, KOD> {
    const LOCAL_MAC: u64 = <ntp_fpga::config::Test as ConfigSet>::LOCAL_MAC;
    const LOCAL_IP4: u32 = 0xC0A8_7B01; // 192.168.123.1
    const GATEWAY_IP4: u32 = 0xC0A8_7BFE; // 192.168.123.254
    const NETMASK4: u32 = 0xFFFF_FF00; // /24
    const GATEWAY_IP6: u128 = <ntp_fpga::config::Test as ConfigSet>::GATEWAY_IP6;
    const LOCAL_GUA: u128 = <ntp_fpga::config::Test as ConfigSet>::LOCAL_GUA;
    const GUA_ENABLED: bool = <ntp_fpga::config::Test as ConfigSet>::GUA_ENABLED;
    const UPSTREAM: [(bool, u128, bool); 4] = <ntp_fpga::config::Test as ConfigSet>::UPSTREAM;
    const POLL_LOG2: u8 = <ntp_fpga::config::Test as ConfigSet>::POLL_LOG2;
    const PRECISION_LOG2: i8 = <ntp_fpga::config::Test as ConfigSet>::PRECISION_LOG2;
    const RATE_LIMIT_PER_S: u16 = RL;
    const KOD_ENABLE: bool = KOD;
    const RATE_SECRET: u64 = <ntp_fpga::config::Test as ConfigSet>::RATE_SECRET;
    const ENTROPY_SEED: u64 = <ntp_fpga::config::Test as ConfigSet>::ENTROPY_SEED;
}

// ------------------------------------------------------------ sync source

/// Shared between [`BridgeSync`] and the bench.
#[derive(Default)]
struct SyncCtl {
    fixed: Cell<bool>,
    /// The design's `time` (32.64) seen at the last `clk_sys` edge.
    time: Cell<u128>,
    /// Pending clock step (fast-forward), applied once.
    step: Cell<u128>,
}

/// `FixedSync` / `Standalone` outputs plus a sim-only clock step (as in
/// `chrony_bridge`).
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

fn be(b: &[u8]) -> u64 {
    b.iter().fold(0u64, |a, &x| (a << 8) | x as u64)
}

/// An NTP reply decoded from a transmitted frame.
struct NtpOut {
    dst_ip: u32,
    dport: u16,
    stratum: u8,
    origin: u64,
    t2: u64,
    t3: u64,
}

/// Decodes a transmitted frame (bytes after the SFD, FCS included): `Err`
/// for a bad frame, `Ok(None)` for a good frame that is not an IPv4 UDP
/// reply from port 123.
fn parse_tx(raw: &[u8]) -> Result<Option<NtpOut>, String> {
    let n = raw.len();
    if n < 64 {
        return Err(format!("runt frame of {n} bytes"));
    }
    let (body, fcs) = raw.split_at(n - 4);
    if u32::from_le_bytes([fcs[0], fcs[1], fcs[2], fcs[3]]) != crc32(body) {
        return Err("bad FCS".into());
    }
    if u16::from_be_bytes([body[12], body[13]]) != 0x0800 {
        return Ok(None);
    }
    let ip = &body[14..];
    if fold(sum16(0, &ip[..20])) != 0xffff {
        return Err("bad IPv4 header checksum".into());
    }
    if ip[9] != 17 {
        return Ok(None);
    }
    let tl = u16::from_be_bytes([ip[2], ip[3]]) as usize;
    if tl > ip.len() || tl < 28 {
        return Err("bad IPv4 length".into());
    }
    let u = &ip[20..tl];
    let len = u16::from_be_bytes([u[4], u[5]]) as usize;
    if len != u.len() {
        return Err("bad UDP length".into());
    }
    let mut a = sum16(0, &ip[12..20]);
    a += 17 + len as u32;
    a = sum16(a, u);
    if fold(a) != 0xffff {
        return Err("bad UDP checksum".into());
    }
    if u16::from_be_bytes([u[0], u[1]]) != 123 || u.len() < 8 + 48 {
        return Ok(None);
    }
    let p = &u[8..];
    Ok(Some(NtpOut {
        dst_ip: be(&ip[16..20]) as u32,
        dport: u16::from_be_bytes([u[2], u[3]]),
        stratum: p[1],
        origin: be(&p[24..32]),
        t2: be(&p[32..40]),
        t3: be(&p[40..48]),
    }))
}

// ------------------------------------------------------------ time

/// NTP 32.32 of a Unix time in ns (rounded).
fn ntp_of_ns(ns: u128) -> u64 {
    ((((ns + NTP_UNIX * NS) << 32) + NS / 2) / NS) as u64
}

/// Signed difference of two 32.32 times, in ns.
fn diff_ns(a: u64, b: u64) -> f64 {
    (a.wrapping_sub(b) as i64) as f64 * 1e9 / 4_294_967_296.0
}

// ------------------------------------------------------------ libc

extern "C" {
    fn mmap(addr: *mut u8, len: usize, prot: i32, flags: i32, fd: i32, off: i64) -> *mut u8;
    fn socketpair(domain: i32, ty: i32, proto: i32, sv: *mut i32) -> i32;
    fn send(fd: i32, buf: *const u8, len: usize, flags: i32) -> isize;
    fn recv(fd: i32, buf: *mut u8, len: usize, flags: i32) -> isize;
    fn setsockopt(fd: i32, level: i32, name: i32, val: *const u8, len: u32) -> i32;
    fn fcntl(fd: i32, cmd: i32, ...) -> i32;
    fn syscall(n: i64, ...) -> i64;
    fn getpid() -> i32;
    fn close(fd: i32) -> i32;
}
const AF_UNIX: i32 = 1;
const SOCK_SEQPACKET: i32 = 5;
const SOL_SOCKET: i32 = 1;
const SO_SNDBUF: i32 = 7;
const MSG_DONTWAIT: i32 = 0x40;
const F_SETFD: i32 = 2;
const FD_CLOEXEC: i32 = 1;
const SYS_FUTEX: i64 = 202;
const FUTEX_WAIT: i64 = 0;
const FUTEX_WAKE: i64 = 1;

// ------------------------------------------------------------ shared page

// Words, as in tools/ntpperf/pcap_shim/simlink.h.
const SIM_MAGIC: u64 = 0x314d_5456_5050_544e;
const SW_MAGIC: usize = 0;
const SW_VT_NS: usize = 1;
const SW_T0_NS: usize = 2;
const SW_STOP: usize = 3;
const SW_INFLIGHT: usize = 4;
const SW_NSLOTS: usize = 5;
const SW_EPOCH: usize = 6;
const SW_SEQ: usize = 7;
const SW_BRIDGE_PID: usize = 8;
const SW_QUANTUM_NS: usize = 9;
const SW_SLOTS: usize = 16;
const SW_SLOT_WORDS: usize = 8;
const SW_MAX_SLOTS: usize = 32;
const SL_STATE: usize = 0;
const SL_PID: usize = 1;
const SL_DEADLINE: usize = 2;
const SL_WAITS: usize = 3;
const SL_IOWAITS: usize = 4;
const ST_TIME_WAIT: u64 = 2;
const ST_BLOCKED_READ: u64 = 3;
const ST_BLOCKED_WRITE: u64 = 4;
const ST_EXITED: u64 = 5;

struct Shm {
    p: *const AtomicU64,
}

impl Shm {
    fn create(path: &str, t0: u64, quantum: u64) -> Shm {
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
        s.st(SW_VT_NS, t0);
        s.st(SW_T0_NS, t0);
        s.st(SW_QUANTUM_NS, quantum);
        // SAFETY: getpid has no preconditions.
        s.st(SW_BRIDGE_PID, unsafe { getpid() } as u64);
        s.st(SW_MAGIC, SIM_MAGIC);
        s
    }
    fn w(&self, i: usize) -> &AtomicU64 {
        assert!(i < 512);
        // SAFETY: i < 512 words of the live 4096-byte mapping.
        unsafe { &*self.p.add(i) }
    }
    fn ld(&self, i: usize) -> u64 {
        self.w(i).load(Ordering::SeqCst)
    }
    fn st(&self, i: usize, v: u64) {
        self.w(i).store(v, Ordering::SeqCst)
    }
    fn slot(&self, i: usize, k: usize) -> usize {
        SW_SLOTS + i * SW_SLOT_WORDS + k
    }
    fn futex_wait(&self, i: usize, val: u64, timeout: Duration) {
        let ts: [i64; 2] = [timeout.as_secs() as i64, timeout.subsec_nanos() as i64];
        // SAFETY: FUTEX_WAIT on the low 32 bits of a live word of the
        // mapping (little endian), with a valid timespec.
        unsafe {
            syscall(SYS_FUTEX, self.w(i).as_ptr() as *mut u32, FUTEX_WAIT, val as u32, ts.as_ptr(), 0usize, 0i64);
        }
    }
    fn futex_wake(&self, i: usize) {
        // SAFETY: FUTEX_WAKE on a live word of the mapping.
        unsafe {
            syscall(SYS_FUTEX, self.w(i).as_ptr() as *mut u32, FUTEX_WAKE, i32::MAX as i64, 0usize, 0usize, 0i64);
        }
    }
}

/// Result of a readiness check of ntpperf's processes.
enum Ready {
    /// Every process waits: the earliest deadline, if any.
    Yes(Option<u64>),
    No,
}

/// Checks that every registered process is ready (see `vtime.c`), on a
/// snapshot validated by the state-change counter.
fn readiness(shm: &Shm, vt: u64) -> Ready {
    let s1 = shm.ld(SW_SEQ);
    // The in-flight count first, then the states (a reader leaves
    // BLOCKED_READ before it lowers the count).
    let inflight = shm.ld(SW_INFLIGHT) as i64;
    let n = (shm.ld(SW_NSLOTS) as usize).min(SW_MAX_SLOTS);
    if n == 0 {
        return Ready::No;
    }
    let mut dl: Option<u64> = None;
    for i in 0..n {
        match shm.ld(shm.slot(i, SL_STATE)) {
            ST_TIME_WAIT => {
                let d = shm.ld(shm.slot(i, SL_DEADLINE));
                if d <= vt {
                    return Ready::No; // woken, not running yet
                }
                dl = Some(dl.map_or(d, |x| x.min(d)));
            }
            ST_BLOCKED_WRITE | ST_EXITED => {}
            ST_BLOCKED_READ => {
                if inflight != 0 {
                    return Ready::No;
                }
            }
            _ => return Ready::No,
        }
    }
    if shm.ld(SW_SEQ) != s1 {
        return Ready::No;
    }
    Ready::Yes(dl)
}

/// Marks as exited the slots of processes that died without saying so.
fn reap_dead_slots(shm: &Shm) {
    let n = (shm.ld(SW_NSLOTS) as usize).min(SW_MAX_SLOTS);
    for i in 0..n {
        let st = shm.ld(shm.slot(i, SL_STATE));
        let pid = shm.ld(shm.slot(i, SL_PID)) as i32;
        if st == ST_EXITED || pid <= 0 {
            continue;
        }
        let dead = match std::fs::read_to_string(format!("/proc/{pid}/stat")) {
            Err(_) => true,
            Ok(s) => s.rsplit(')').next().map(|r| r.trim_start().starts_with('Z')).unwrap_or(false),
        };
        if dead {
            eprintln!("ntpperf_bridge: process {pid} (slot {i}) died without notice");
            shm.st(shm.slot(i, SL_STATE), ST_EXITED);
            shm.w(SW_SEQ).fetch_add(1, Ordering::SeqCst);
        }
    }
}

// ------------------------------------------------------------ options

struct Opts {
    fixed: bool,
    rate_limit: u16,
    kod: bool,
    queue_bytes: usize,
    shm: String,
    shim: String,
    log: Option<String>,
    ff: bool,
    idle_cycles: u64,
    status_s: f64,
    max_wall_s: f64,
    max_virtual_s: f64,
    t0_ns: u128,
    quantum_ns: u64,
    iface: String,
    cmd: Vec<String>,
}

fn usage() -> ! {
    eprintln!(
        "usage: ntpperf_bridge --shim ntpperf_vtime.so --shm FILE [--sync fixed|standalone] \
         [--rate-limit 0|1|2|4|8|16 (0)] [--kod] [--queue-bytes N (65536)] [--log CSV] [--no-ff] \
         [--idle-cycles N (400)] [--status WALL_S (10)] [--max-wall WALL_S (3600)] \
         [--max-virtual S (100)] [--t0 UNIX_NS] [--quantum-ns N (1000000)] [--iface NAME (sim0)] \
         -- NTPPERF ARGS..."
    );
    std::process::exit(2)
}

fn opts() -> Opts {
    let mut o = Opts {
        fixed: true,
        rate_limit: 0,
        kod: false,
        queue_bytes: 65536,
        shm: String::new(),
        shim: String::new(),
        log: None,
        ff: true,
        idle_cycles: 400,
        status_s: 10.0,
        max_wall_s: 3600.0,
        max_virtual_s: 100.0,
        t0_ns: DEFAULT_T0_S as u128 * NS,
        quantum_ns: 1_000_000,
        iface: "sim0".into(),
        cmd: vec![],
    };
    let a: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    let val = |i: usize| a.get(i + 1).cloned().unwrap_or_else(|| usage());
    fn num<T: std::str::FromStr>(s: String) -> T {
        s.parse().unwrap_or_else(|_| usage())
    }
    while i < a.len() {
        match a[i].as_str() {
            "--" => {
                o.cmd = a[i + 1..].to_vec();
                break;
            }
            "--sync" => {
                o.fixed = match val(i).as_str() {
                    "fixed" => true,
                    "standalone" => false,
                    _ => usage(),
                }
            }
            "--rate-limit" => o.rate_limit = num(val(i)),
            "--queue-bytes" => o.queue_bytes = num(val(i)),
            "--shm" => o.shm = val(i),
            "--shim" => o.shim = val(i),
            "--log" => o.log = Some(val(i)),
            "--idle-cycles" => o.idle_cycles = num(val(i)),
            "--status" => o.status_s = num(val(i)),
            "--max-wall" => o.max_wall_s = num(val(i)),
            "--max-virtual" => o.max_virtual_s = num(val(i)),
            "--t0" => o.t0_ns = num(val(i)),
            "--quantum-ns" => o.quantum_ns = num(val(i)),
            "--iface" => o.iface = val(i),
            "--kod" => {
                o.kod = true;
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
    if o.shm.is_empty() || o.shim.is_empty() || o.cmd.is_empty() {
        usage();
    }
    o
}

fn main() {
    let o = opts();
    let code = match (o.rate_limit, o.kod) {
        (0, false) => run::<PerfCfg<0, false>>(&o),
        (1, false) => run::<PerfCfg<1, false>>(&o),
        (2, false) => run::<PerfCfg<2, false>>(&o),
        (4, false) => run::<PerfCfg<4, false>>(&o),
        (8, false) => run::<PerfCfg<8, false>>(&o),
        (16, false) => run::<PerfCfg<16, false>>(&o),
        (1, true) => run::<PerfCfg<1, true>>(&o),
        (8, true) => run::<PerfCfg<8, true>>(&o),
        _ => {
            eprintln!("--rate-limit: one of 0, 1, 2, 4, 8, 16 (with --kod: 1 or 8)");
            2
        }
    };
    std::process::exit(code)
}

// ------------------------------------------------------------ the bench

/// One request of ntpperf, from its send to its reply (if any).
#[derive(Default, Clone)]
struct Req {
    send_ns: u128,
    src_ip: u32,
    sport: u16,
    len: usize,
    bucket: usize,
    qdrop: bool,
    wire_ns: Option<u128>,
    rx_sfd_ns: Option<u128>,
    reply: Option<Reply>,
}

#[derive(Clone)]
struct Reply {
    tx_sfd_ns: u128,
    end_ns: u128,
    stratum: u8,
    t2: u64,
    t3: u64,
}

/// A frame waiting in the sender's queue model.
struct Frame {
    id: Option<usize>,
    ready_ns: u128,
    nibbles: Vec<u8>,
}

/// Running statistics (Welford).
#[derive(Default)]
struct Stat {
    n: u64,
    mean: f64,
    m2: f64,
    min: f64,
    max: f64,
}
impl Stat {
    fn add(&mut self, x: f64) {
        if self.n == 0 {
            self.min = x;
            self.max = x;
        }
        self.n += 1;
        let d = x - self.mean;
        self.mean += d / self.n as f64;
        self.m2 += d * (x - self.mean);
        self.min = self.min.min(x);
        self.max = self.max.max(x);
    }
    fn show(&self) -> String {
        if self.n == 0 {
            return "n=0".into();
        }
        let sd = if self.n > 1 { (self.m2 / (self.n - 1) as f64).sqrt() } else { 0.0 };
        format!(
            "n={} min {:+.1} mean {:+.2} max {:+.1} sd {:.2}",
            self.n, self.min, self.mean, self.max, sd
        )
    }
}

fn spawn(o: &Opts, client_fd: i32) -> Child {
    let mut c = Command::new(&o.cmd[0]);
    c.args(&o.cmd[1..])
        .env("NTPPERF_SIM_FD", client_fd.to_string())
        .env("NTPPERF_VTIME_SHM", &o.shm)
        .env("NTPPERF_IFACE", &o.iface)
        .env("LD_PRELOAD", &o.shim);
    c.spawn().unwrap_or_else(|e| panic!("{}: {e}", o.cmd[0]))
}

fn run<C: ConfigSet + 'static>(o: &Opts) -> i32 {
    let wall0 = Instant::now();
    let t0_ns = o.t0_ns;
    let shm = Shm::create(&o.shm, t0_ns as u64, o.quantum_ns);

    // The "interface": a SOCK_SEQPACKET pair; [0] is ours, [1] ntpperf's.
    let mut sv = [0i32; 2];
    // SAFETY: plain socketpair(2) into a 2-element array.
    let r = unsafe { socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sv.as_mut_ptr()) };
    assert_eq!(r, 0, "socketpair: {}", std::io::Error::last_os_error());
    let sock = sv[0];
    let big: i32 = 4 << 20;
    for fd in sv {
        // SAFETY: setsockopt with a valid int option value.
        unsafe { setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &big as *const i32 as *const u8, 4) };
    }
    // SAFETY: fcntl(F_SETFD) on our own descriptor: not inherited by ntpperf.
    unsafe { fcntl(sock, F_SETFD, FD_CLOEXEC) };

    let mut log = o
        .log
        .as_ref()
        .map(|p| std::io::BufWriter::new(std::fs::File::create(p).unwrap_or_else(|e| panic!("{p}: {e}"))));

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
    // The design's clock starts at the NTP time of T0 (32.64), ghost
    // registers of a running clock (as `chrony_bridge`).
    let start = ((t0_ns + NTP_UNIX * NS) << 64) / NS;
    {
        let c = &mut top.core.clock;
        c.acc = Reg::new(U::<96>::from(start));
        c.g_valid = Reg::new(Bit::One);
        c.g_prev = Reg::new(U::<96>::from(start - INCR));
        c.g_inc = Reg::new(U::<96>::from(INCR));
        c.g_any = Reg::new(Bit::One);
    }
    mdio_o_.set(Bit::One);
    while now() % 2 != 0 {
        settle();
    }
    let tick0 = now();
    let mut sim = Running::new(top.run(
        (rxd, dv, er, mdio_i),
        (txd_o, en_o, ob(), ob(), ob(), ob(), ob(), ob(), ob()),
    ));

    let mut ff_ns: u128 = 0;
    let vt_at = |t: u64, ff: u128| t0_ns + 5 * (t - tick0) as u128 + ff;
    let max_vt = t0_ns + (o.max_virtual_s * 1e9) as u128;

    eprintln!(
        "ntpperf_bridge: board 02:00:00:00:00:01 192.168.123.1, sync {}, rate limit {}/s, KoD {}, \
         queue {} bytes, fast-forward {} (idle {} cycles), T0 {} ns, quantum {} ns; running {:?}",
        if o.fixed { "fixed (stratum 2)" } else { "standalone (stratum 16)" },
        C::RATE_LIMIT_PER_S,
        C::KOD_ENABLE,
        o.queue_bytes,
        o.ff,
        o.idle_cycles,
        t0_ns,
        o.quantum_ns,
        o.cmd
    );
    let mut child = spawn(o, sv[1]);
    // SAFETY: ntpperf has its copy; ours is not used.
    unsafe { close(sv[1]) };

    // Bench state.
    let mut reqs: Vec<Req> = vec![];
    let mut by_key: HashMap<(u32, u16, u64), usize> = HashMap::new();
    let mut queue: VecDeque<Frame> = VecDeque::new();
    let mut queued_bytes = 0usize;
    let (mut max_queued, mut n_qdrop, mut n_ctrl) = (0usize, 0u64, 0u64);
    let mut cur: Option<(Option<usize>, Vec<u8>, usize)> = None;
    let mut gap = 0u32;
    let mut nib: Vec<u8> = vec![];
    let mut tx_sfd_ns = 0u128;
    let mut adapter_ts = false;
    let mut out_q: VecDeque<Vec<u8>> = VecDeque::new();
    let (mut n_rep, mut n_kod, mut n_unmatched, mut n_other, mut n_bad) = (0u64, 0u64, 0u64, 0u64, 0u64);
    let (mut st_resp, mut st_e2, mut st_e3, mut st_off) = (Stat::default(), Stat::default(), Stat::default(), Stat::default());
    let mut last_activity = 0u64;
    let (mut cycles, mut skipped, mut advances) = (0u64, 0u64, 0u64);
    let mut sim_wall = Duration::ZERO;
    let mut last_status = Instant::now();
    let mut buf = vec![0u8; 4096];
    let mut clk_err = Stat::default();
    let mut exit_code = 0;
    let mut not_ready_since: Option<Instant> = None;
    // Longest silence between the end of a received frame and the next
    // transmitted SFD while a reply was due (the idle threshold must exceed it).
    let mut rx_end_cycle: Option<u64> = None;
    let mut max_rx_to_tx = 0u64;

    let mut vt: u128 = vt_at(now(), ff_ns);
    let child_status;
    let debug = std::env::var_os("NTPPERF_BRIDGE_DEBUG").is_some();
    'main: loop {
        // ---- 1. wait until every ntpperf process is ready
        let target = loop {
            // Frames sent so far (stamped with the current virtual time).
            loop {
                // SAFETY: recv into our buffer.
                let r = unsafe { recv(sock, buf.as_mut_ptr(), buf.len(), MSG_DONTWAIT) };
                if r < 0 {
                    break;
                }
                let r = r as usize;
                if r == 0 {
                    break;
                }
                if r < 14 {
                    if r == 12 && &buf[..4] == b"HELO" {
                        let tt = u16::from_ne_bytes(buf[4..6].try_into().unwrap());
                        adapter_ts = tt == 3 || tt == 4;
                        eprintln!(
                            "ntpperf_bridge: pcap activated, time stamps {}",
                            if adapter_ts { "adapter (TX SFD instant)" } else { "host (end of frame)" }
                        );
                    }
                    n_ctrl += 1;
                    continue;
                }
                let f = &buf[..r];
                let mut w = f.to_vec();
                while w.len() < 60 {
                    w.push(0);
                }
                w.extend_from_slice(&crc32(&w).to_le_bytes());
                let wl = w.len();
                // A request? (IPv4 UDP to 123 with 48 bytes of NTP)
                let id = if r >= 90 && f[12..14] == [8, 0] && f[23] == 17 && f[36..38] == [0, 123] {
                    let src = be(&f[26..30]) as u32;
                    let sport = u16::from_be_bytes([f[34], f[35]]);
                    let xmt = be(&f[82..90]);
                    let i = reqs.len();
                    reqs.push(Req {
                        send_ns: vt,
                        src_ip: src,
                        sport,
                        len: r,
                        bucket: rl_index(src as u64, C::RATE_SECRET),
                        ..Default::default()
                    });
                    by_key.insert((src, sport, xmt), i);
                    Some(i)
                } else {
                    None
                };
                if queued_bytes + wl > o.queue_bytes {
                    n_qdrop += 1;
                    if let Some(i) = id {
                        reqs[i].qdrop = true;
                    }
                    continue;
                }
                let mut nibbles = Vec::with_capacity(2 * (wl + PREAMBLE));
                nibbles.extend(std::iter::repeat_n(5u8, 15));
                nibbles.push(0xd);
                for b in w {
                    nibbles.push(b & 15);
                    nibbles.push(b >> 4);
                }
                queued_bytes += wl;
                max_queued = max_queued.max(queued_bytes);
                queue.push_back(Frame { id, ready_ns: vt, nibbles });
            }
            // Replies not delivered yet.
            while let Some(m) = out_q.front() {
                // SAFETY: send of our buffer.
                let r = unsafe { send(sock, m.as_ptr(), m.len(), MSG_DONTWAIT) };
                if r < 0 {
                    break;
                }
                out_q.pop_front();
            }
            if let Ok(Some(s)) = child.try_wait() {
                child_status = s;
                break 'main;
            }
            if wall0.elapsed().as_secs_f64() > o.max_wall_s {
                eprintln!("ntpperf_bridge: --max-wall reached, stopping ntpperf");
                shm.st(SW_STOP, 1);
                shm.futex_wake(SW_EPOCH);
                let _ = child.kill();
                child_status = child.wait().unwrap();
                exit_code = 3;
                break 'main;
            }
            let seq = shm.ld(SW_SEQ);
            match readiness(&shm, vt as u64) {
                Ready::Yes(d) => {
                    not_ready_since = None;
                    break d;
                }
                _ => {}
            }
            let since = *not_ready_since.get_or_insert_with(Instant::now);
            if since.elapsed() > Duration::from_secs(2) {
                reap_dead_slots(&shm);
                not_ready_since = Some(Instant::now());
            }
            shm.futex_wait(SW_SEQ, seq, Duration::from_micros(500));
        };
        // Frames sent just before the last process became ready.
        loop {
            // SAFETY: recv into our buffer (peek for presence only).
            let r = unsafe { recv(sock, buf.as_mut_ptr(), 1, MSG_DONTWAIT | 0x2) };
            if r <= 0 {
                break;
            }
            // Something arrived after the readiness check: go round again
            // (the processes are still waiting, nothing can change).
            continue 'main;
        }
        let target = match target {
            Some(d) => d as u128,
            None => vt + 1000,
        };
        if target > max_vt {
            eprintln!("ntpperf_bridge: --max-virtual reached, stopping ntpperf");
            shm.st(SW_STOP, 1);
            shm.futex_wake(SW_EPOCH);
            let _ = child.kill();
            child_status = child.wait().unwrap();
            exit_code = 3;
            break 'main;
        }

        // ---- 2. simulate up to the target
        advances += 1;
        if debug && advances <= 12 {
            eprintln!(
                "debug: advance {advances}: vt {} -> {}, queue {} frames, slots {:?}",
                vt - t0_ns,
                target - t0_ns,
                queue.len(),
                (0..(shm.ld(SW_NSLOTS) as usize).min(SW_MAX_SLOTS))
                    .map(|i| (shm.ld(shm.slot(i, SL_STATE)), shm.ld(shm.slot(i, SL_DEADLINE)).wrapping_sub(t0_ns as u64)))
                    .collect::<Vec<_>>()
            );
        }
        let ts = Instant::now();
        while vt < target {
            let t = now();
            if ClkRx25::rising_at(t) {
                // Fast-forward over idle time.
                if o.ff
                    && cur.is_none()
                    && gap == 0
                    && nib.is_empty()
                    && cycles - last_activity >= o.idle_cycles
                    && ctl.step.get() == 0
                {
                    let next = queue.front().map_or(target, |f| f.ready_ns.min(target));
                    if next > vt {
                        // 40 ns = 4 cycles = one MII period: keeps the phases.
                        let j = ((next - vt) / 40) as u64 * 4;
                        if j >= FF_MIN_CYCLES {
                            ctl.step.set(j as u128 * INCR);
                            ff_ns += j as u128 * 10;
                            skipped += j;
                            vt = vt_at(t, ff_ns);
                            continue;
                        }
                    }
                }
                // Receive pins.
                let mut out = (0u8, false);
                if cur.is_none() && gap == 0 {
                    if let Some(f) = queue.front() {
                        if f.ready_ns <= vt {
                            let f = queue.pop_front().unwrap();
                            queued_bytes -= f.nibbles.len() / 2 - PREAMBLE;
                            if let Some(i) = f.id {
                                reqs[i].wire_ns = Some(vt);
                            }
                            cur = Some((f.id, f.nibbles, 0));
                        }
                    }
                }
                if let Some((id, nibs, pos)) = cur.as_mut() {
                    let n = nibs[*pos];
                    out = (n, true);
                    // The SFD is the 16th nibble.
                    if *pos == 15 {
                        if let Some(i) = *id {
                            reqs[i].rx_sfd_ns = Some(vt);
                        }
                    }
                    *pos += 1;
                    if *pos == nibs.len() {
                        cur = None;
                        gap = IFG_NIBBLES;
                        rx_end_cycle = Some(cycles);
                    }
                } else if gap > 0 {
                    gap -= 1;
                }
                rxd_o.set(U::<4>::from(out.0));
                dv_o.set(Bit::from_bool(out.1));
                er_o.set(Bit::Zero);
                if out.1 {
                    last_activity = cycles;
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
                        tx_sfd_ns = vt_at(t - ClkTx25::PERIOD, ff_ns);
                        if let Some(c) = rx_end_cycle.take() {
                            max_rx_to_tx = max_rx_to_tx.max(cycles - c);
                        }
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
                    let end_ns = vt_at(now(), ff_ns);
                    match parse_tx(&bytes) {
                        Err(why) => {
                            n_bad += 1;
                            eprintln!("ntpperf_bridge: bad transmitted frame ({why})");
                        }
                        Ok(p) => {
                            if let Some(p) = p {
                                match by_key.remove(&(p.dst_ip, p.dport, p.origin)) {
                                    Some(i) => {
                                        n_rep += 1;
                                        if p.stratum == 0 {
                                            n_kod += 1;
                                        }
                                        let rq = &mut reqs[i];
                                        rq.reply = Some(Reply {
                                            tx_sfd_ns,
                                            end_ns,
                                            stratum: p.stratum,
                                            t2: p.t2,
                                            t3: p.t3,
                                        });
                                        if p.stratum != 0 {
                                            st_resp.add(diff_ns(p.t3, p.t2));
                                            st_e3.add(diff_ns(p.t3, ntp_of_ns(tx_sfd_ns)));
                                            st_off.add(-diff_ns(p.t3, ntp_of_ns(tx_sfd_ns)));
                                            if let Some(s) = rq.rx_sfd_ns {
                                                st_e2.add(diff_ns(p.t2, ntp_of_ns(s)));
                                            }
                                        }
                                    }
                                    None => n_unmatched += 1,
                                }
                            } else {
                                n_other += 1;
                            }
                            let body = &bytes[..bytes.len() - 4];
                            let ts = if adapter_ts { tx_sfd_ns } else { end_ns };
                            let mut m = Vec::with_capacity(16 + body.len());
                            m.extend_from_slice(&(ts as u64).to_ne_bytes());
                            m.extend_from_slice(&(body.len() as u32).to_ne_bytes());
                            m.extend_from_slice(&0u32.to_ne_bytes());
                            m.extend_from_slice(body);
                            out_q.push_back(m);
                            clk_err.add(diff_ns((ctl.time.get() >> 32) as u64, ntp_of_ns(end_ns)));
                        }
                    }
                }
            }
            vt = vt_at(now(), ff_ns);
        }
        sim_wall += ts.elapsed();
        // Deliver what the design sent, then publish the time.
        while let Some(m) = out_q.front() {
            // SAFETY: send of our buffer.
            let r = unsafe { send(sock, m.as_ptr(), m.len(), MSG_DONTWAIT) };
            if r < 0 {
                break;
            }
            out_q.pop_front();
        }
        shm.st(SW_VT_NS, vt as u64);
        shm.w(SW_EPOCH).fetch_add(1, Ordering::SeqCst);
        shm.futex_wake(SW_EPOCH);

        if last_status.elapsed().as_secs_f64() >= o.status_s {
            last_status = Instant::now();
            let w = wall0.elapsed().as_secs_f64();
            eprintln!(
                "status: virt {:.4} s, wall {:.0} s, simulated {} cycles ({:.0}/s of simulation), \
                 skipped {}, advances {}, requests {}, queue drops {}, queue {} B, replies {}",
                (vt - t0_ns) as f64 / 1e9,
                w,
                cycles,
                cycles as f64 / sim_wall.as_secs_f64().max(1e-9),
                skipped,
                advances,
                reqs.len(),
                n_qdrop,
                queued_bytes,
                n_rep
            );
        }
    }
    shm.st(SW_STOP, 1);
    let w = wall0.elapsed().as_secs_f64();
    let n = reqs.len();
    let answered = reqs.iter().filter(|r| r.reply.is_some()).count();
    let n_qd = reqs.iter().filter(|r| r.qdrop).count();
    let slots = (shm.ld(SW_NSLOTS) as usize).min(SW_MAX_SLOTS);
    let waits: Vec<String> = (0..slots)
        .map(|i| format!("{}/{}", shm.ld(shm.slot(i, SL_WAITS)), shm.ld(shm.slot(i, SL_IOWAITS))))
        .collect();
    eprintln!(
        "ntpperf_bridge: done ({child_status}): virt {:.4} s, wall {:.1} s, simulated {} cycles \
         ({:.0} cycles per second of simulation), skipped {} cycles, {} time advances, \
         time/io waits per process {:?}",
        (vt - t0_ns) as f64 / 1e9,
        w,
        cycles,
        cycles as f64 / sim_wall.as_secs_f64().max(1e-9),
        skipped,
        advances,
        waits
    );
    eprintln!(
        "ntpperf_bridge: requests {n}, queue drops {n_qd} (max queue {max_queued} B), \
         answered {answered} ({} KoD), not answered {}, unmatched replies {n_unmatched}, \
         other frames {n_other}, bad frames {n_bad}, control messages {n_ctrl}, \
         max silence rx end -> tx SFD {max_rx_to_tx} cycles",
        n_kod,
        n - answered - n_qd
    );
    eprintln!("ntpperf_bridge: response time T3-T2 (ns): {}", st_resp.show());
    eprintln!("ntpperf_bridge: T2 - RX SFD (ns): {}", st_e2.show());
    eprintln!("ntpperf_bridge: T3 - TX SFD (ns): {}", st_e3.show());
    eprintln!("ntpperf_bridge: TX SFD - T3 (ntpperf -o -H offset, ns): {}", st_off.show());
    eprintln!("ntpperf_bridge: design clock - true time at frame ends (ns): {}", clk_err.show());
    if let Some(f) = log.as_mut() {
        writeln!(
            f,
            "idx,send_ns,qdrop,wire_ns,rx_sfd_ns,src_ip,sport,len,bucket,answered,stratum,t2,t3,\
             tx_sfd_ns,end_ns,resp_ns,e2_ns,e3_ns"
        )
        .unwrap();
        for (i, r) in reqs.iter().enumerate() {
            let rel = |x: u128| (x - t0_ns).to_string();
            let (ans, strat, t2, t3, tsfd, end, resp, e2, e3) = match &r.reply {
                Some(p) => (
                    1,
                    p.stratum.to_string(),
                    format!("{:016x}", p.t2),
                    format!("{:016x}", p.t3),
                    rel(p.tx_sfd_ns),
                    rel(p.end_ns),
                    format!("{:.2}", diff_ns(p.t3, p.t2)),
                    r.rx_sfd_ns.map_or(String::new(), |s| format!("{:.2}", diff_ns(p.t2, ntp_of_ns(s)))),
                    format!("{:.2}", diff_ns(p.t3, ntp_of_ns(p.tx_sfd_ns))),
                ),
                None => (0, String::new(), String::new(), String::new(), String::new(), String::new(), String::new(), String::new(), String::new()),
            };
            writeln!(
                f,
                "{i},{},{},{},{},{}.{}.{}.{},{},{},{},{ans},{strat},{t2},{t3},{tsfd},{end},{resp},{e2},{e3}",
                rel(r.send_ns),
                r.qdrop as u8,
                r.wire_ns.map_or(String::new(), rel),
                r.rx_sfd_ns.map_or(String::new(), rel),
                r.src_ip >> 24,
                (r.src_ip >> 16) & 255,
                (r.src_ip >> 8) & 255,
                r.src_ip & 255,
                r.sport,
                r.len,
                r.bucket
            )
            .unwrap();
        }
        f.flush().unwrap();
    }
    drop(sim);
    if exit_code == 0 && !child_status.success() {
        exit_code = 1;
    }
    exit_code
}
