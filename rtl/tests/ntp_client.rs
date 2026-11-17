// SPDX-License-Identifier: Apache-2.0
//! Tests of task T4.2, `ntp_client`: the Python vectors
//! (`tools/golden/ntpclient/vectors/client.txt`: every Sample field, every
//! reject, the request bytes), the T1 sampling cycle, the iburst and regular
//! schedule, next-hop and source selection (IPv4 and IPv6), lookup miss /
//! resolve / retry cap, deadline misses, spoofed / duplicate / `err` replies,
//! two slots in flight, a disabled slot, era wrap, and the Verilog lowering.
use ntp_fpga::arp::ArpEntry;
use ntp_fpga::ntp_client::{NtpClient, DEADLINE_S, RES_TRIES};
use ntp_fpga::ntp_types::{Sample, PHI_Q32};
use ntp_fpga::stream::{FrameByte, Meta};
use ntp_fpga::udp4_tx::UdpTxReq;
use ntp_fpga::udp6_tx::Udp6TxReq;
use std::collections::{HashMap, VecDeque};
use std::future::Future;
use txhdl::comp::{chan, signal, DefaultClock, Out, Running, Rx, Tx, Unit};
use txhdl::types::{Bit, U};

const SEC: u64 = 1 << 32;
const LOCAL4: u32 = 0xC000_0201; // 192.0.2.1
const MASK4: u32 = 0xFFFF_FF00;
const GW4: u32 = 0xC000_02FE;
const LL: u128 = 0xFE80_0000_0000_0000_0000_00FF_FE00_0001;
const GUA: u128 = 0x2001_0DB8_0000_0000_0000_0000_0000_0001;
const GW6: u128 = 0xFE80_0000_0000_0000_0000_0000_0000_0001;
const PEER4: u32 = 0xC000_0263; // 192.0.2.99, on link
const PEER4_OFF: u32 = 0xC633_6401; // 198.51.100.1, via the gateway
const PEER6: u128 = 0x2001_0DB8_0000_0000_0000_0000_0000_0099; // same /64
const PEER6_LL: u128 = 0xFE80_0000_0000_0000_0000_0000_0000_0063;
const PEER6_OFF: u128 = 0x2001_0DB8_0001_0000_0000_0000_0000_0001;
const T0: u64 = 0xEC00_0000_0000_0000; // 2025 or so
const PREC: u8 = 0xE8;

fn mac_of(ip: u128) -> u64 {
    0x0200_0000_0000 | (ip as u64 & 0xFFFF_FFFF)
}

// ------------------------------------------------------------ bench

#[derive(Clone, Debug)]
struct ReqOut {
    v6: bool,
    dst_mac: u64,
    dst_ip: u128,
    src_ip: u128,
    sport: u16,
    dport: u16,
    len: u16,
    bytes: Vec<u8>,
    /// Bench cycle at which the request (`req4`/`req6`) and byte 40 were read.
    t_req: usize,
    t40: usize,
    /// `time` (96-bit) as driven in every cycle up to t40, for T1 checks.
    sec: u32,
}

impl ReqOut {
    fn t1(&self) -> u64 {
        u64::from_be_bytes(self.bytes[40..48].try_into().unwrap())
    }
}

#[derive(Clone, Debug)]
struct Reply {
    v6: bool,
    src: u128,
    sport: u16,
    rx_ts: u64,
    pay: Vec<u8>,
    err: bool,
}

struct Bench<F: Future<Output = ()>> {
    sim: Running<F>,
    up: Vec<(Out<U<128>>, Out<Bit>, Out<Bit>)>,
    local_ip4: Out<U<32>>,
    netmask: Out<U<32>>,
    gateway: Out<U<32>>,
    gateway6: Out<U<128>>,
    local_ll: Out<U<128>>,
    local_gua: Out<U<128>>,
    gua_valid: Out<Bit>,
    poll: Out<U<8>>,
    prec: Out<U<8>>,
    lat4: Out<U<64>>,
    lat6: Out<U<64>>,
    time: Out<U<96>>,
    meta_t: Tx<Meta>,
    rep_t: Tx<FrameByte>,
    arp_rsp_t: Tx<ArpEntry>,
    nd_rsp_t: Tx<ArpEntry>,
    arp_lookup: Rx<U<32>>,
    arp_resolve: Rx<U<32>>,
    nd_lookup: Rx<U<128>>,
    nd_resolve: Rx<U<128>>,
    req4: Rx<UdpTxReq>,
    pay4: Rx<FrameByte>,
    req6: Rx<Udp6TxReq>,
    pay6: Rx<FrameByte>,
    samples: Rx<Sample>,
    miss: Rx<U<2>>,
    // ---- test side state
    t: usize,
    /// Driven `time` (96 bits); `tick` is added after every cycle.
    now: u128,
    tick: u128,
    /// Neighbor tables (IPv4 keys zero extended).
    tab4: HashMap<u128, u64>,
    tab6: HashMap<u128, u64>,
    /// Every lookup is answered `found` (the table is ignored).
    all_found: bool,
    /// A resolve teaches its address at once.
    learn: bool,
    rsp4: VecDeque<ArpEntry>,
    rsp6: VecDeque<ArpEntry>,
    lookups: Vec<(bool, u128, usize)>,
    resolves: Vec<(bool, u128, usize)>,
    open: Option<ReqOut>,
    reqs: Vec<ReqOut>,
    samples_got: Vec<(Sample, usize)>,
    misses: Vec<(u8, usize)>,
    /// Replies to feed: (meta sent?, reply, next byte).
    inq: VecDeque<(bool, Reply, usize)>,
    /// `pay4`/`pay6` are not read while set.
    hold_pay: bool,
}

fn bench() -> Bench<impl Future<Output = ()>> {
    let mut up = vec![];
    let mut up_in = vec![];
    for _ in 0..4 {
        let (a, ai) = signal::<U<128>, DefaultClock>();
        let (v, vi) = signal::<Bit, DefaultClock>();
        let (e, ei) = signal::<Bit, DefaultClock>();
        up.push((a, v, e));
        up_in.push((ai, vi, ei));
    }
    let (local_ip4, lip_i) = signal::<U<32>, DefaultClock>();
    let (netmask, nm_i) = signal::<U<32>, DefaultClock>();
    let (gateway, gw_i) = signal::<U<32>, DefaultClock>();
    let (gateway6, gw6_i) = signal::<U<128>, DefaultClock>();
    let (local_ll, ll_i) = signal::<U<128>, DefaultClock>();
    let (local_gua, gua_i) = signal::<U<128>, DefaultClock>();
    let (gua_valid, gv_i) = signal::<Bit, DefaultClock>();
    let (local_mac, mac_i) = signal::<U<48>, DefaultClock>();
    let (poll, poll_i) = signal::<U<8>, DefaultClock>();
    let (prec, prec_i) = signal::<U<8>, DefaultClock>();
    let (lat4, lat4_i) = signal::<U<64>, DefaultClock>();
    let (lat6, lat6_i) = signal::<U<64>, DefaultClock>();
    let (time, time_i) = signal::<U<96>, DefaultClock>();
    let (meta_t, meta_r) = chan::<Meta, DefaultClock>();
    let (rep_t, rep_r) = chan::<FrameByte, DefaultClock>();
    let (arp_rsp_t, arp_rsp_r) = chan::<ArpEntry, DefaultClock>();
    let (nd_rsp_t, nd_rsp_r) = chan::<ArpEntry, DefaultClock>();
    let (al_t, arp_lookup) = chan::<U<32>, DefaultClock>();
    let (ar_t, arp_resolve) = chan::<U<32>, DefaultClock>();
    let (nl_t, nd_lookup) = chan::<U<128>, DefaultClock>();
    let (nr_t, nd_resolve) = chan::<U<128>, DefaultClock>();
    let (q4_t, req4) = chan::<UdpTxReq, DefaultClock>();
    let (p4_t, pay4) = chan::<FrameByte, DefaultClock>();
    let (q6_t, req6) = chan::<Udp6TxReq, DefaultClock>();
    let (p6_t, pay6) = chan::<FrameByte, DefaultClock>();
    let (s_t, samples) = chan::<Sample, DefaultClock>();
    let (m_t, miss) = chan::<U<2>, DefaultClock>();
    let c: &'static mut NtpClient = Box::leak(Box::default());
    let mut it = up_in.into_iter();
    let (a0, v0, e0) = it.next().unwrap();
    let (a1, v1, e1) = it.next().unwrap();
    let (a2, v2, e2) = it.next().unwrap();
    let (a3, v3, e3) = it.next().unwrap();
    let sim = Running::new(c.run(
        (
            a0, v0, e0, a1, v1, e1, a2, v2, e2, a3, v3, e3, lip_i, nm_i, gw_i, gw6_i, ll_i, gua_i,
            gv_i, mac_i, poll_i, prec_i, lat4_i, lat6_i, time_i, meta_r, rep_r, arp_rsp_r,
            nd_rsp_r,
        ),
        (al_t, ar_t, nl_t, nr_t, q4_t, p4_t, q6_t, p6_t, s_t, m_t),
    ));
    local_mac.set(U::<48>::from(0x0200_0000_0001u64));
    let b = Bench {
        sim,
        up,
        local_ip4,
        netmask,
        gateway,
        gateway6,
        local_ll,
        local_gua,
        gua_valid,
        poll,
        prec,
        lat4,
        lat6,
        time,
        meta_t,
        rep_t,
        arp_rsp_t,
        nd_rsp_t,
        arp_lookup,
        arp_resolve,
        nd_lookup,
        nd_resolve,
        req4,
        pay4,
        req6,
        pay6,
        samples,
        miss,
        t: 0,
        now: (T0 as u128) << 32,
        tick: 0,
        tab4: HashMap::new(),
        tab6: HashMap::new(),
        all_found: true,
        learn: false,
        rsp4: VecDeque::new(),
        rsp6: VecDeque::new(),
        lookups: vec![],
        resolves: vec![],
        open: None,
        reqs: vec![],
        samples_got: vec![],
        misses: vec![],
        inq: VecDeque::new(),
        hold_pay: false,
    };
    b.local_ip4.set(U::<32>::from(LOCAL4));
    b.netmask.set(U::<32>::from(MASK4));
    b.gateway.set(U::<32>::from(GW4));
    b.gateway6.set(U::<128>::new(GW6));
    b.local_ll.set(U::<128>::new(LL));
    b.local_gua.set(U::<128>::new(GUA));
    b.gua_valid.set(Bit::One);
    b.poll.set(U::<8>::from(6u8));
    b.prec.set(U::<8>::from(PREC));
    b.lat4.set(U::<64>::from(0u64));
    b.lat6.set(U::<64>::from(0u64));
    for i in 0..4 {
        b.slot(i, 0, false, false);
    }
    b
}

impl<F: Future<Output = ()>> Bench<F> {
    fn slot(&self, i: usize, addr: u128, v6: bool, en: bool) {
        self.up[i].0.set(U::<128>::new(addr));
        self.up[i].1.set(Bit::from_bool(v6));
        self.up[i].2.set(Bit::from_bool(en));
    }

    /// NTP time (32.32) driven now.
    fn ntp(&self) -> u64 {
        (self.now >> 32) as u64
    }

    fn set_ntp(&mut self, ts: u64) {
        self.now = (ts as u128) << 32;
    }

    fn found(&self, v6: bool, ip: u128) -> ArpEntry {
        let hit = if self.all_found {
            Some(mac_of(ip))
        } else if v6 {
            self.tab6.get(&ip).copied()
        } else {
            self.tab4.get(&ip).copied()
        };
        ArpEntry {
            found: Bit::from_bool(hit.is_some()),
            mac: U::<48>::from(hit.unwrap_or(0)),
        }
    }

    fn cycle(&mut self) {
        self.time.set(U::<96>::new(self.now & ((1u128 << 96) - 1)));
        let t = self.t;
        // ---- outputs (always ready, except the payload when held)
        if let Some(ip) = self.arp_lookup.recv() {
            let ip = ip.raw();
            self.lookups.push((false, ip, t));
            let r = self.found(false, ip);
            self.rsp4.push_back(r);
        }
        if let Some(ip) = self.nd_lookup.recv() {
            let ip = ip.raw();
            self.lookups.push((true, ip, t));
            let r = self.found(true, ip);
            self.rsp6.push_back(r);
        }
        if let Some(ip) = self.arp_resolve.recv() {
            self.resolves.push((false, ip.raw(), t));
            if self.learn {
                self.tab4.insert(ip.raw(), mac_of(ip.raw()));
            }
        }
        if let Some(ip) = self.nd_resolve.recv() {
            self.resolves.push((true, ip.raw(), t));
            if self.learn {
                self.tab6.insert(ip.raw(), mac_of(ip.raw()));
            }
        }
        if let Some(q) = self.req4.recv() {
            assert!(self.open.is_none(), "request while a payload is open");
            self.open = Some(ReqOut {
                v6: false,
                dst_mac: q.dst_mac.raw() as u64,
                dst_ip: q.dst_ip.raw(),
                src_ip: 0,
                sport: q.sport.raw() as u16,
                dport: q.dport.raw() as u16,
                len: q.len.raw() as u16,
                bytes: vec![],
                t_req: t,
                t40: 0,
                sec: 0,
            });
        }
        if let Some(q) = self.req6.recv() {
            assert!(self.open.is_none(), "request while a payload is open");
            self.open = Some(ReqOut {
                v6: true,
                dst_mac: q.dst_mac.raw() as u64,
                dst_ip: q.dst_ip.raw(),
                src_ip: q.src_ip.raw(),
                sport: q.sport.raw() as u16,
                dport: q.dport.raw() as u16,
                len: q.len.raw() as u16,
                bytes: vec![],
                t_req: t,
                t40: 0,
                sec: 0,
            });
        }
        if !self.hold_pay {
            for v6 in [false, true] {
                let ch = if v6 { &self.pay6 } else { &self.pay4 };
                if let Some(b) = ch.recv() {
                    let o = self.open.as_mut().expect("payload byte before its request");
                    assert_eq!(o.v6, v6, "payload on the other family");
                    assert!(!b.err.to_bool(), "err in a request");
                    if o.bytes.len() == 40 {
                        o.t40 = t;
                        o.sec = (self.now >> 64) as u32;
                    }
                    o.bytes.push(b.data.raw() as u8);
                    let last = b.last.to_bool();
                    assert_eq!(last, o.bytes.len() == 48, "last on byte 47 only");
                    if last {
                        self.reqs.push(self.open.take().unwrap());
                    }
                }
            }
        }
        if let Some(s) = self.samples.recv() {
            self.samples_got.push((s, t));
        }
        if let Some(m) = self.miss.recv() {
            self.misses.push((m.raw() as u8, t));
        }
        // ---- inputs
        if let Some(r) = self.rsp4.front() {
            if self.arp_rsp_t.ready().to_bool() {
                self.arp_rsp_t.send(*r);
                self.rsp4.pop_front();
            }
        }
        if let Some(r) = self.rsp6.front() {
            if self.nd_rsp_t.ready().to_bool() {
                self.nd_rsp_t.send(*r);
                self.rsp6.pop_front();
            }
        }
        if let Some((sent, r, k)) = self.inq.front_mut() {
            if !*sent {
                if self.meta_t.ready().to_bool() {
                    self.meta_t.send(Meta {
                        family: Bit::from_bool(r.v6),
                        src: U::<128>::new(r.src),
                        dst: U::<128>::new(if r.v6 { GUA } else { LOCAL4 as u128 }),
                        sport: U::<16>::from(r.sport),
                        dport: U::<16>::from(123u16),
                        len: U::<16>::from(r.pay.len() as u16),
                        eth_src: U::<48>::from(0x0200_0000_0099u64),
                        rx_ts: U::<64>::from(r.rx_ts),
                    });
                    *sent = true;
                }
            } else if self.rep_t.ready().to_bool() {
                let last = *k + 1 == r.pay.len();
                self.rep_t
                    .send(FrameByte::new(r.pay[*k], last, last && r.err));
                *k += 1;
                if last {
                    self.inq.pop_front();
                }
            }
        }
        self.sim.cycle();
        self.t += 1;
        self.now = (self.now + self.tick) & ((1u128 << 96) - 1);
    }

    fn run(&mut self, n: usize) {
        for _ in 0..n {
            self.cycle();
        }
    }

    /// Runs until `n` requests in total were seen (panics after `max` cycles).
    fn until_reqs(&mut self, n: usize, max: usize) {
        for _ in 0..max {
            if self.reqs.len() >= n {
                return;
            }
            self.cycle();
        }
        panic!("{} requests expected, {} seen", n, self.reqs.len());
    }

    fn reply(&mut self, r: Reply) {
        self.inq.push_back((false, r, 0));
    }

    /// Feeds `r` and runs until it is consumed, plus 40 cycles.
    fn feed(&mut self, r: Reply) {
        self.reply(r);
        for _ in 0..2000 {
            if self.inq.is_empty() {
                break;
            }
            self.cycle();
        }
        assert!(self.inq.is_empty(), "reply not consumed");
        self.run(40);
    }

    /// Advances the driven time by `dt` (32.32) in `steps` steps of `cyc` cycles.
    fn advance(&mut self, dt: u64, steps: u64, cyc: usize) {
        for _ in 0..steps {
            self.now = (self.now + (((dt / steps) as u128) << 32)) & ((1u128 << 96) - 1);
            self.run(cyc);
        }
    }
}

// ------------------------------------------------------------ reply helpers

#[allow(clippy::too_many_arguments)]
fn reply_bytes(
    li: u8,
    vn: u8,
    mode: u8,
    stratum: u8,
    prec: u8,
    org: u64,
    t2: u64,
    t3: u64,
) -> Vec<u8> {
    let mut v = vec![(li << 6) | (vn << 3) | mode, stratum, 6, prec];
    v.extend_from_slice(&0x0000_0010u32.to_be_bytes());
    v.extend_from_slice(&0x0000_0020u32.to_be_bytes());
    v.extend_from_slice(&0x4750_5300u32.to_be_bytes());
    v.extend_from_slice(&(T0 - 5 * SEC).to_be_bytes());
    v.extend_from_slice(&org.to_be_bytes());
    v.extend_from_slice(&t2.to_be_bytes());
    v.extend_from_slice(&t3.to_be_bytes());
    v
}

/// A good reply to `q` (server `off` ahead, 1 ms each way and of processing).
fn good_reply(q: &ReqOut, src: u128, off: u64) -> Reply {
    let t1 = q.t1();
    let ms = SEC / 1000;
    let t2 = t1.wrapping_add(ms).wrapping_add(off);
    let t3 = t2.wrapping_add(ms);
    Reply {
        v6: q.v6,
        src,
        sport: 123,
        rx_ts: t1.wrapping_add(3 * ms),
        pay: reply_bytes(0, 4, 4, 1, 0xE9, t1, t2, t3),
        err: false,
    }
}

/// Python `arith` (offset, delay, disp) for the checks of the local tests.
fn arith(t1: u64, t2: u64, t3: u64, t4: u64, pp: u8, po: u8) -> (u64, u64, u64) {
    let w = |x: u64| x as i64 as i128;
    let a = w(t2.wrapping_sub(t1));
    let b = w(t3.wrapping_sub(t4));
    let off = ((a + b) >> 1) as u64;
    let d1 = w(t4.wrapping_sub(t1));
    let d2 = w(t3.wrapping_sub(t2));
    let dly = (d1 - d2).max(0) as u64;
    let iv = d1.max(0) as u128;
    let phi = ((PHI_Q32 as u128 * iv + 0xFFFF_FFFF) >> 32) as u64;
    let pf = |p: u8| -> u64 {
        let p = p as i8 as i32;
        if p >= 0 {
            1 << 32
        } else if p < -32 {
            1
        } else {
            1 << (32 + p)
        }
    };
    (off, dly, pf(pp) + pf(po) + phi)
}

fn check_good(s: &Sample, q: &ReqOut, r: &Reply, peer: u8) {
    let p = &r.pay;
    let rd = |o: usize| u64::from_be_bytes(p[o..o + 8].try_into().unwrap());
    let (off, dly, disp) = arith(q.t1(), rd(32), rd(40), r.rx_ts, p[3], PREC);
    assert_eq!(s.peer.raw() as u8, peer);
    assert_eq!(s.offset.raw() as u64, off, "offset");
    assert_eq!(s.delay.raw() as u64, dly, "delay");
    assert_eq!(s.disp.raw() as u64, disp, "disp");
    assert_eq!(s.t4.raw() as u64, r.rx_ts);
    assert_eq!(s.stratum.raw(), p[1] as u128);
    assert_eq!(s.leap.raw(), (p[0] >> 6) as u128);
    assert_eq!(s.ref_id.raw(), 0x4750_5300);
    assert_eq!(s.root_delay.raw(), 0x10);
    assert_eq!(s.root_disp.raw(), 0x20);
}

// ------------------------------------------------------------ vectors

fn hex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

fn num(s: &str) -> u128 {
    match s.strip_prefix("0x") {
        Some(h) => u128::from_str_radix(h, 16).unwrap(),
        None => s.parse().unwrap(),
    }
}

fn vector_lines() -> Vec<String> {
    let p = format!(
        "{}/../tools/golden/ntpclient/vectors/client.txt",
        env!("CARGO_MANIFEST_DIR")
    );
    let text = std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{p}: {e}"));
    text.lines()
        .map(|l| l.trim().to_string())
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .collect()
}

/// Replays every exchange of client.txt through the RTL, one fresh unit per
/// line: the slot's request is sent with T1 = the vector's t1 (latency 0,
/// `time` = t1 frozen), then the vector's reply; the outputs must be exactly
/// the expected Sample, the expected miss, or nothing.
#[test]
fn vectors_replay() {
    let mut n_s = 0;
    let mut n_r = 0;
    for line in vector_lines() {
        if line.starts_with("request") {
            continue;
        }
        let (lhs, rhs) = line.split_once(" -> ").unwrap();
        let f: Vec<&str> = lhs.split(' ').collect();
        let slot = num(f[0]) as usize;
        let v6 = num(f[1]) == 1;
        let addr = num(f[2]);
        let t1 = num(f[3]) as u64;
        let our = num(f[4]) as u8;
        let src = num(f[5]);
        let fam = num(f[6]) == 1;
        let sport = num(f[7]) as u16;
        let err = num(f[8]) == 1;
        let t4 = num(f[9]) as u64;
        let pay = hex(f[10]);
        let mut b = bench();
        b.slot(slot, addr, v6, true);
        b.prec.set(U::<8>::from(our));
        b.set_ntp(t1);
        b.until_reqs(1, 300);
        assert_eq!(b.reqs[0].t1(), t1, "{line}");
        b.feed(Reply {
            v6: fam,
            src,
            sport,
            rx_ts: t4,
            pay,
            err,
        });
        let e: Vec<&str> = rhs.split(' ').collect();
        if e[0] == "S" {
            n_s += 1;
            assert_eq!(b.samples_got.len(), 1, "{line}");
            assert!(b.misses.is_empty(), "{line}");
            let s = b.samples_got[0].0;
            let got = [
                s.peer.raw(),
                s.offset.raw(),
                s.delay.raw(),
                s.disp.raw(),
                s.t4.raw(),
                s.stratum.raw(),
                s.leap.raw(),
                s.ref_id.raw(),
                s.root_delay.raw(),
                s.root_disp.raw(),
            ];
            let want: Vec<u128> = e[1..].iter().map(|x| num(x)).collect();
            assert_eq!(got.to_vec(), want, "{line}");
        } else {
            n_r += 1;
            assert!(b.samples_got.is_empty(), "{line}");
            let want_miss = e[2] == "1";
            assert_eq!(
                b.misses.iter().map(|m| m.0).collect::<Vec<_>>(),
                if want_miss { vec![slot as u8] } else { vec![] },
                "{line}"
            );
        }
    }
    assert!(n_s > 150 && n_r > 100, "{n_s} samples, {n_r} rejects");
}

/// The request bytes against `build_request` (the `request` lines), and the
/// request's addressing for both families.
#[test]
fn request_bytes() {
    let mut n = 0;
    for line in vector_lines() {
        let Some(rest) = line.strip_prefix("request ") else {
            continue;
        };
        let (lhs, rhs) = rest.split_once(" -> ").unwrap();
        let f: Vec<&str> = lhs.split(' ').collect();
        let (poll, prec, t1) = (num(f[0]) as u8, num(f[1]) as u8, num(f[2]) as u64);
        for v6 in [false, true] {
            let mut b = bench();
            if v6 {
                b.slot(2, PEER6, true, true);
            } else {
                b.slot(2, PEER4 as u128, false, true);
            }
            b.poll.set(U::<8>::from(poll));
            b.prec.set(U::<8>::from(prec));
            b.set_ntp(t1);
            b.until_reqs(1, 300);
            let q = &b.reqs[0];
            assert_eq!(q.bytes, hex(rhs), "{line}");
            assert_eq!((q.v6, q.sport, q.dport, q.len), (v6, 123, 123, 48));
            if v6 {
                assert_eq!((q.dst_ip, q.src_ip, q.dst_mac), (PEER6, GUA, mac_of(PEER6)));
            } else {
                assert_eq!((q.dst_ip, q.dst_mac), (PEER4 as u128, mac_of(PEER4 as u128)));
            }
            assert!(q.t_req < q.t40);
            n += 1;
        }
    }
    assert!(n >= 40);
}

/// T1 = (time >> 32) + tx_latency of the cycle before byte 40 is sent:
/// with `time` advancing by one 2^-32 s per cycle, T1 - latency names the
/// cycle; it moves with a stall of the payload channel before byte 40.
#[test]
fn t1_sampling_cycle() {
    let mut deltas = vec![];
    for (v6, hold) in [(false, 0usize), (true, 0), (false, 7), (true, 13)] {
        let mut b = bench();
        b.slot(0, if v6 { PEER6 } else { PEER4 as u128 }, v6, true);
        b.lat4.set(U::<64>::from(8032u64));
        b.lat6.set(U::<64>::from(8891u64));
        b.tick = 1 << 32;
        let base = b.ntp();
        // hold the payload channel for `hold` cycles once the request is out
        for _ in 0..300 {
            if b.open.is_some() {
                break;
            }
            b.cycle();
        }
        assert!(b.open.is_some());
        b.hold_pay = true;
        b.run(hold);
        b.hold_pay = false;
        b.until_reqs(1, 300);
        let q = &b.reqs[0];
        let lat = if v6 { 8891 } else { 8032 };
        let cyc = q.t1() - lat - base;
        // T1's cycle relative to the cycle the bench read byte 40
        deltas.push(q.t40 as i64 - cyc as i64);
    }
    assert!(deltas.iter().all(|&d| d == deltas[0]), "{deltas:?}");
    // byte 40 is written into the channel in cycle S and read by the bench
    // in S + 1; t1p was loaded at the end of S - 1 from the time of S - 1.
    assert_eq!(deltas[0], 2, "{deltas:?}");
}

/// Era wrap of T1 itself: (time >> 32) + latency crosses 2^64.
#[test]
fn t1_era_wrap() {
    let mut b = bench();
    b.slot(0, PEER4 as u128, false, true);
    b.lat4.set(U::<64>::from(100u64));
    b.set_ntp(u64::MAX - 9);
    b.until_reqs(1, 300);
    assert_eq!(b.reqs[0].t1(), 90);
    let q = b.reqs[0].clone();
    b.feed(good_reply(&q, PEER4 as u128, 0));
    assert_eq!(b.samples_got.len(), 1);
    check_good(&b.samples_got[0].0, &q, &good_reply(&q, PEER4 as u128, 0), 0);
}

// ------------------------------------------------------------ schedule

/// Requests answered at once: 4 requests 2 s apart, then every 2^poll s
/// (poll 6: 64 s), measured in NTP seconds.
#[test]
fn iburst_then_regular() {
    let mut b = bench();
    b.slot(1, PEER4 as u128, false, true);
    let start = (b.ntp() >> 32) as u32;
    let mut answered = 0;
    // 0.25 s steps over 140 s
    for _ in 0..(140 * 4) {
        b.now += ((SEC / 4) as u128) << 32;
        b.run(130);
        while answered < b.reqs.len() {
            let q = b.reqs[answered].clone();
            b.feed(good_reply(&q, PEER4 as u128, 0));
            answered += 1;
        }
    }
    let secs: Vec<u32> = b.reqs.iter().map(|q| q.sec - start).collect();
    assert_eq!(secs, vec![0, 2, 4, 6, 70, 134], "{secs:?}");
    assert_eq!(b.samples_got.len(), 6);
    assert!(b.samples_got.iter().all(|s| s.0.peer.raw() == 1));
    assert!(b.misses.is_empty());
}

/// The regular interval follows `poll_log2` (clamped to 4..=17).
#[test]
fn poll_interval_clamped() {
    for (poll, gap) in [(2u8, 16u32), (4, 16), (5, 32)] {
        let mut b = bench();
        b.poll.set(U::<8>::from(poll));
        b.slot(0, PEER4 as u128, false, true);
        let start = (b.ntp() >> 32) as u32;
        let mut answered = 0;
        for _ in 0..(6 + 2 * gap + 2) * 2 {
            b.now += ((SEC / 2) as u128) << 32;
            b.run(130);
            while answered < b.reqs.len() {
                let q = b.reqs[answered].clone();
                b.feed(good_reply(&q, PEER4 as u128, 0));
                answered += 1;
            }
        }
        let secs: Vec<u32> = b.reqs.iter().map(|q| q.sec - start).collect();
        assert_eq!(secs, vec![0, 2, 4, 6, 6 + gap, 6 + 2 * gap], "poll {poll}");
    }
}

// ------------------------------------------------------------ addressing

fn first_request(slot_addr: u128, v6: bool, gua_valid: bool) -> (Vec<(bool, u128, usize)>, ReqOut) {
    let mut b = bench();
    b.gua_valid.set(Bit::from_bool(gua_valid));
    b.slot(3, slot_addr, v6, true);
    b.until_reqs(1, 300);
    (b.lookups.clone(), b.reqs[0].clone())
}

#[test]
fn next_hop_ipv4() {
    let (l, q) = first_request(PEER4 as u128, false, true);
    assert_eq!(l.iter().map(|x| (x.0, x.1)).collect::<Vec<_>>(), vec![(false, PEER4 as u128)]);
    assert_eq!((q.dst_ip, q.dst_mac), (PEER4 as u128, mac_of(PEER4 as u128)));
    let (l, q) = first_request(PEER4_OFF as u128, false, true);
    assert_eq!(l.iter().map(|x| (x.0, x.1)).collect::<Vec<_>>(), vec![(false, GW4 as u128)]);
    assert_eq!((q.dst_ip, q.dst_mac), (PEER4_OFF as u128, mac_of(GW4 as u128)));
}

#[test]
fn next_hop_ipv6() {
    // (peer, gua_valid, expected next hop, expected source)
    let cases = [
        (PEER6_LL, true, PEER6_LL, LL),
        (PEER6_LL, false, PEER6_LL, LL),
        (PEER6, true, PEER6, GUA),
        (PEER6, false, GW6, LL),
        (PEER6_OFF, true, GW6, GUA),
    ];
    for (peer, gv, nh, src) in cases {
        let (l, q) = first_request(peer, true, gv);
        assert_eq!(l.iter().map(|x| (x.0, x.1)).collect::<Vec<_>>(), vec![(true, nh)]);
        assert_eq!((q.v6, q.dst_ip, q.src_ip, q.dst_mac), (true, peer, src, mac_of(nh)));
    }
}

// ------------------------------------------------------------ resolution

/// Lookup miss, resolve, retry after the retry delay, hit, request; no miss.
#[test]
fn lookup_miss_resolve_hit() {
    for v6 in [false, true] {
        let peer = if v6 { PEER6 } else { PEER4 as u128 };
        let mut b = bench();
        b.all_found = false;
        b.learn = true;
        b.slot(0, peer, v6, true);
        b.run(200);
        assert_eq!(b.lookups.len(), 1);
        assert_eq!(b.resolves.iter().map(|r| (r.0, r.1)).collect::<Vec<_>>(), vec![(v6, peer)]);
        assert!(b.reqs.is_empty());
        // one tick (2^-6 s) is not enough, two are
        b.advance(SEC / 64, 1, 200);
        assert_eq!(b.lookups.len(), 1, "retry before RETRY_TICKS");
        b.advance(SEC / 64, 1, 200);
        assert_eq!(b.lookups.len(), 2);
        assert_eq!(b.reqs.len(), 1);
        assert_eq!(b.reqs[0].dst_mac, mac_of(peer));
        assert!(b.misses.is_empty());
    }
}

/// A next hop that never resolves: RES_TRIES lookups, then the poll is
/// abandoned without a miss (no request was ever sent); once a request has
/// been sent, an abandoned poll is one miss.
#[test]
fn resolution_cap() {
    let mut b = bench();
    b.all_found = false;
    b.slot(0, PEER4 as u128, false, true);
    for _ in 0..40 {
        b.advance(SEC / 64, 1, 100);
    }
    assert_eq!(b.lookups.len(), RES_TRIES as usize);
    assert_eq!(b.resolves.len(), RES_TRIES as usize);
    assert!(b.reqs.is_empty() && b.misses.is_empty());
    // the next iburst poll (2 s after the first) retries; now it resolves
    b.tab4.insert(PEER4 as u128, mac_of(PEER4 as u128));
    b.advance(2 * SEC, 8, 100);
    assert_eq!(b.reqs.len(), 1);
    let q = b.reqs[0].clone();
    b.feed(good_reply(&q, PEER4 as u128, 0));
    assert_eq!(b.samples_got.len(), 1);
    // the neighbor disappears: the next poll is abandoned and counted
    b.tab4.clear();
    let l0 = b.lookups.len();
    b.advance(3 * SEC, 192, 30);
    assert_eq!(b.lookups.len() - l0, RES_TRIES as usize);
    assert_eq!(b.misses.iter().map(|m| m.0).collect::<Vec<_>>(), vec![0]);
    assert_eq!(b.reqs.len(), 1);
}

// ------------------------------------------------------------ deadline

#[test]
fn deadline_miss_once() {
    let mut b = bench();
    b.set_ntp(T0 + SEC / 2);
    b.slot(2, PEER6, true, true);
    b.until_reqs(1, 300);
    // 1.25 s later (second s + 1.75): still waiting
    b.advance(SEC + SEC / 4, 5, 50);
    assert!(b.misses.is_empty());
    assert_eq!(b.reqs.len(), 1);
    // DEADLINE_S whole seconds after T1's second: one miss, then the next
    // iburst request (due 2 s after the first) goes out
    b.advance(SEC / 4, 1, 300);
    assert_eq!(DEADLINE_S, 2);
    assert_eq!(b.misses.iter().map(|m| m.0).collect::<Vec<_>>(), vec![2]);
    assert_eq!(b.reqs.len(), 2);
    // a late reply to the first request is dropped
    let q0 = b.reqs[0].clone();
    b.feed(good_reply(&q0, PEER6, 0));
    assert!(b.samples_got.is_empty());
    // no more misses for the first one
    b.run(500);
    assert_eq!(b.misses.len(), 1);
    // the second one expires too: exactly one more
    b.advance(2 * SEC, 4, 100);
    assert_eq!(b.misses.len(), 2);
}

// ------------------------------------------------------------ bad replies

fn one_request(v6: bool) -> (Bench<impl Future<Output = ()>>, ReqOut, u128) {
    let peer = if v6 { PEER6 } else { PEER4 as u128 };
    let mut b = bench();
    b.slot(0, peer, v6, true);
    b.until_reqs(1, 300);
    let q = b.reqs[0].clone();
    (b, q, peer)
}

#[test]
fn spoofed_replies_dropped() {
    for v6 in [false, true] {
        let (mut b, q, peer) = one_request(v6);
        let t1 = q.t1();
        let g = good_reply(&q, peer, 0);
        let mut bad = vec![];
        // wrong origin
        let mut r = g.clone();
        r.pay[31] ^= 1;
        bad.push(r);
        // wrong source address, wrong family, wrong port
        let mut r = g.clone();
        r.src ^= 1;
        bad.push(r);
        let mut r = g.clone();
        r.v6 = !v6;
        bad.push(r);
        let mut r = g.clone();
        r.sport = 1123;
        bad.push(r);
        // mode 3, VN 2, VN 5
        for (vn, mode) in [(4, 3), (2, 4), (5, 4)] {
            let mut r = g.clone();
            r.pay[0] = (vn << 3) | mode;
            bad.push(r);
        }
        // KoD with a wrong origin
        let mut r = g.clone();
        r.pay[1] = 0;
        r.pay[24..32].copy_from_slice(&(t1 ^ 0x100).to_be_bytes());
        bad.push(r);
        // short (47 bytes)
        let mut r = g.clone();
        r.pay.truncate(47);
        bad.push(r);
        for r in bad {
            b.feed(r);
        }
        assert!(b.samples_got.is_empty() && b.misses.is_empty());
        // the request is still outstanding: the genuine reply is accepted
        b.feed(g.clone());
        assert_eq!(b.samples_got.len(), 1);
        check_good(&b.samples_got[0].0, &q, &g, 0);
    }
}

/// Authentic but unusable replies (KoD, stratum 16, leap 3): no sample, one
/// miss at once, the request is closed.
#[test]
fn unusable_replies_are_misses() {
    for (li, st) in [(0u8, 0u8), (3, 16), (3, 1), (0, 16)] {
        let (mut b, q, peer) = one_request(false);
        let mut r = good_reply(&q, peer, 0);
        r.pay[0] = (li << 6) | (4 << 3) | 4;
        r.pay[1] = st;
        b.feed(r);
        assert!(b.samples_got.is_empty());
        assert_eq!(b.misses.iter().map(|m| m.0).collect::<Vec<_>>(), vec![0]);
        b.feed(good_reply(&q, peer, 0));
        assert!(b.samples_got.is_empty(), "request closed");
        b.advance(SEC + SEC / 2, 3, 50);
        assert_eq!(b.misses.len(), 1, "no deadline miss after it");
    }
}

#[test]
fn duplicate_reply_ignored() {
    let (mut b, q, peer) = one_request(true);
    let g = good_reply(&q, peer, 3 * SEC);
    b.feed(g.clone());
    b.feed(g.clone());
    assert_eq!(b.samples_got.len(), 1);
    check_good(&b.samples_got[0].0, &q, &g, 0);
    b.advance(3 * SEC / 2, 3, 50);
    assert!(b.misses.is_empty());
}

#[test]
fn err_reply_ignored() {
    let (mut b, q, peer) = one_request(false);
    let mut r = good_reply(&q, peer, 0);
    r.err = true;
    b.feed(r);
    assert!(b.samples_got.is_empty() && b.misses.is_empty());
    // with extension bytes after the 48: accepted
    let mut g = good_reply(&q, peer, 0);
    g.pay.extend_from_slice(&[0xAB; 20]);
    b.feed(g.clone());
    assert_eq!(b.samples_got.len(), 1);
    check_good(&b.samples_got[0].0, &q, &g, 0);
}

/// Replies flooding the input do not starve the scheduler.
#[test]
fn reply_flood_does_not_starve() {
    let mut b = bench();
    b.slot(0, PEER4 as u128, false, true);
    let junk = Reply {
        v6: false,
        src: 0xC000_0277,
        sport: 123,
        rx_ts: 0,
        pay: vec![0x24; 48],
        err: false,
    };
    for _ in 0..30 {
        b.reply(junk.clone());
    }
    b.until_reqs(1, 3000);
    assert!(!b.inq.is_empty(), "the request went out during the flood");
}

// ------------------------------------------------------------ several slots

#[test]
fn two_slots_in_flight() {
    let mut b = bench();
    b.slot(0, PEER4 as u128, false, true);
    b.slot(1, PEER6_LL, true, true);
    b.slot(3, PEER4_OFF as u128, false, true);
    b.until_reqs(3, 1000);
    let by_ip = |ip: u128| b.reqs.iter().find(|q| q.dst_ip == ip).unwrap().clone();
    let q0 = by_ip(PEER4 as u128);
    let q1 = by_ip(PEER6_LL);
    let q3 = by_ip(PEER4_OFF as u128);
    // answers in another order, each to its own request; slot 3 answered
    // with slot 0's origin is dropped
    let mut cross = good_reply(&q3, PEER4_OFF as u128, 0);
    cross.pay[24..32].copy_from_slice(&q0.t1().to_be_bytes());
    if q0.t1() != q3.t1() {
        b.feed(cross);
        assert!(b.samples_got.is_empty());
    }
    let r1 = good_reply(&q1, PEER6_LL, SEC);
    let r3 = good_reply(&q3, PEER4_OFF as u128, 2 * SEC);
    let r0 = good_reply(&q0, PEER4 as u128, 7);
    b.feed(r1.clone());
    b.feed(r3.clone());
    b.feed(r0.clone());
    assert_eq!(b.samples_got.len(), 3);
    check_good(&b.samples_got[0].0, &q1, &r1, 1);
    check_good(&b.samples_got[1].0, &q3, &r3, 3);
    check_good(&b.samples_got[2].0, &q0, &r0, 0);
    assert!(b.misses.is_empty());
}

/// Same address in two slots: the lowest index gets the replies.
#[test]
fn slot_disabled() {
    let mut b = bench();
    b.slot(0, PEER4 as u128, false, true);
    b.slot(2, PEER6, true, false);
    b.until_reqs(1, 300);
    b.run(500);
    assert_eq!(b.reqs.len(), 1, "no request for a disabled slot");
    let q = b.reqs[0].clone();
    // disabled with a request outstanding: no miss, the reply is dropped
    b.slot(0, PEER4 as u128, false, false);
    b.run(10);
    b.feed(good_reply(&q, PEER4 as u128, 0));
    b.advance(5 * SEC, 10, 50);
    assert!(b.samples_got.is_empty() && b.misses.is_empty());
    assert_eq!(b.reqs.len(), 1);
    // re-enabled: the iburst restarts at once
    b.slot(0, PEER4 as u128, false, true);
    b.until_reqs(2, 300);
    let q2 = b.reqs[1].clone();
    b.feed(good_reply(&q2, PEER4 as u128, 0));
    assert_eq!(b.samples_got.len(), 1);
}

// ------------------------------------------------------------ extremes

/// Offsets near ±2^31 s and an era change between T1 and T4.
#[test]
fn extreme_timestamps() {
    let half = (1u64 << 63) - 2 * SEC;
    for off in [half, half.wrapping_neg(), 0u64.wrapping_sub(SEC / 3)] {
        for start in [T0, u64::MAX - SEC / 1000] {
            let mut b = bench();
            b.set_ntp(start);
            b.slot(0, PEER6, true, true);
            b.until_reqs(1, 300);
            let q = b.reqs[0].clone();
            let r = good_reply(&q, PEER6, off);
            b.feed(r.clone());
            assert_eq!(b.samples_got.len(), 1);
            check_good(&b.samples_got[0].0, &q, &r, 0);
            let o = b.samples_got[0].0.offset.raw() as u64 as i64;
            let want = off.wrapping_add(SEC / 2000 * 0) as i64;
            assert!((o - want).unsigned_abs() < SEC, "offset {o:x} {want:x}");
        }
    }
}

// ------------------------------------------------------------ Verilog

#[test]
fn lowers_to_verilog() {
    let v = NtpClient::verilog("ntp_client");
    assert!(v.contains("module ntp_client"));
}
