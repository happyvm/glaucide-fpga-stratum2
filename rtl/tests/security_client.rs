// SPDX-License-Identifier: Apache-2.0
//! Security review (task T5.5), the client side: what a forged upstream
//! reply can do to `ntp_client`, how predictable our Origin (T1) is, and
//! what ARP and NDP spoofing do to the neighbor caches the client uses.
//! Findings and context: `docs/security.md`.
//!
//! Benches (copied, reduced, from `rtl/tests/ntp_client.rs`, `arp.rs` and
//! `nd_client.rs`: test files are separate crates):
//!
//! * `NtpClient` alone, with every channel driven by the test (lookups
//!   always answered `found`, outputs always ready);
//! * `Arp` alone, frames as `eth_rx` delivers them;
//! * `eth_rx -> ipv6_rx -> icmpv6 -> nbr_cache` (the learning path of the
//!   IPv6 neighbor cache), frames as `fcs_check` delivers them.
use ntp_fpga::arp::{Arp, ArpEntry};
use ntp_fpga::arp::{ArpTiming, Board, Sim};
use ntp_fpga::eth_rx::{EthMeta, EthRx};
use ntp_fpga::eth_tx::EthHdr;
use ntp_fpga::icmpv6::{Icmpv6With, NdLearn};
use ntp_fpga::ipv6_rx::{Ip6Meta, Ipv6Rx};
use ntp_fpga::ipv6_tx::Ip6TxReq;
use ntp_fpga::nbr_cache::NbrCache;
use ntp_fpga::entropy::Entropy;
use ntp_fpga::ntp_client::NtpClient;
use ntp_fpga::top_client::ENTROPY_SEED;
use ntp_fpga::ntp_types::Sample;
use ntp_fpga::stream::{FrameByte, Meta, Ts};
use ntp_fpga::udp4_tx::UdpTxReq;
use ntp_fpga::udp6_tx::Udp6TxReq;
use std::collections::VecDeque;
use std::future::Future;
use txhdl::comp::{chan, join2, signal, tie, DefaultClock, Mem, Out, Running, Rx, Tx, Unit};
use txhdl::types::{Bit, U};

const SEC: u64 = 1 << 32;
const LOCAL4: u32 = 0xC000_0201; // 192.0.2.1
const GW4: u32 = 0xC000_02FE; // 192.0.2.254
const LL: u128 = 0xFE80_0000_0000_0000_0000_00FF_FE00_0001;
const GUA: u128 = 0x2001_0DB8_0000_0000_0000_0000_0000_0001;
const GW6: u128 = 0xFE80_0000_0000_0000_0000_0000_0000_0001;
const PEER4: u32 = 0xC000_0263; // 192.0.2.99
const PEER6: u128 = 0x2001_0DB8_0000_0000_0000_0000_0000_0099;
const T0: u64 = 0xEC00_0000_0000_0000;
const OUR_MAC: u64 = 0x0200_0000_0001;
const GW_MAC: u64 = 0x0200_0000_00fe;
const ATT_MAC: u64 = 0x0200_0000_0bad;
const INCR: u128 = ntp_fpga::clock::INCR;

// ------------------------------------------------------------ client bench

#[derive(Clone, Debug)]
struct ReqOut {
    v6: bool,
    bytes: Vec<u8>,
    /// Slot 0's clock reading T1 (`m_t1[0]`) once the request is complete:
    /// the T1 of the request when it is slot 0's (every test here uses
    /// slot 0).
    clock: u64,
}

impl ReqOut {
    /// The Transmit Timestamp sent: the Origin nonce (F3).
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
    lat4: Out<U<64>>,
    lat6: Out<U<64>>,
    time: Out<U<96>>,
    /// The client's per-slot clock reading T1 (`m_t1`), never sent since F3.
    m_t1: Mem<U<64>, 4>,
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
    /// Driven `time` (96 bits); `tick` is added after every cycle.
    now: u128,
    tick: u128,
    rsp4: VecDeque<ArpEntry>,
    rsp6: VecDeque<ArpEntry>,
    open: Option<ReqOut>,
    reqs: Vec<ReqOut>,
    samples_got: Vec<Sample>,
    misses: Vec<u8>,
    inq: VecDeque<(bool, Reply, usize)>,
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
    let (rnd_o, rnd_i) = signal::<U<64>, DefaultClock>();
    let (rxst_o, rx_stamp) = signal::<U<64>, DefaultClock>();
    let time_e = time_i.clone();
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
    // The client with the entropy unit of the board build (F3): the seed of
    // `top_client.rs`, `time`, the client's `rx_stamp`.
    let c: &'static mut NtpClient = Box::leak(Box::default());
    let ent: &'static mut Entropy = Box::leak(Box::default());
    let m_t1 = c.m_t1.clone();
    let mut it = up_in.into_iter();
    let (a0, v0, e0) = it.next().unwrap();
    let (a1, v1, e1) = it.next().unwrap();
    let (a2, v2, e2) = it.next().unwrap();
    let (a3, v3, e3) = it.next().unwrap();
    let sim = Running::new(join2(c.run(
        (
            a0, v0, e0, a1, v1, e1, a2, v2, e2, a3, v3, e3, lip_i, nm_i, gw_i, gw6_i, ll_i, gua_i,
            gv_i, mac_i, poll_i, prec_i, lat4_i, lat6_i, time_i, rnd_i, meta_r, rep_r, arp_rsp_r,
            nd_rsp_r,
        ),
        (al_t, ar_t, nl_t, nr_t, q4_t, p4_t, q6_t, p6_t, s_t, m_t, rxst_o),
    ), ent.run((tie(U::<64>::from(ENTROPY_SEED)), time_e, rx_stamp), rnd_o)));
    local_mac.set(U::<48>::from(OUR_MAC));
    local_ip4.set(U::<32>::from(LOCAL4));
    netmask.set(U::<32>::from(0xFFFF_FF00u32));
    gateway.set(U::<32>::from(GW4));
    gateway6.set(U::<128>::new(GW6));
    local_ll.set(U::<128>::new(LL));
    local_gua.set(U::<128>::new(GUA));
    gua_valid.set(Bit::One);
    poll.set(U::<8>::from(6u8));
    prec.set(U::<8>::from(0xE8u8));
    // Keep the configuration wires alive for the whole run.
    std::mem::forget((local_ip4, netmask, gateway, gateway6, local_ll, local_gua, gua_valid, poll, prec, local_mac));
    let b = Bench {
        sim,
        up,
        lat4,
        lat6,
        time,
        m_t1,
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
        now: (T0 as u128) << 32,
        tick: 0,
        rsp4: VecDeque::new(),
        rsp6: VecDeque::new(),
        open: None,
        reqs: vec![],
        samples_got: vec![],
        misses: vec![],
        inq: VecDeque::new(),
    };
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

    fn cycle(&mut self) {
        self.time.set(U::<96>::new(self.now & ((1u128 << 96) - 1)));
        // Lookups always answered `found`, the MAC of the address.
        if let Some(ip) = self.arp_lookup.recv() {
            self.rsp4.push_back(ArpEntry { found: Bit::One, mac: U::<48>::from(ip.raw() as u64 & 0xFFFF) });
        }
        if let Some(ip) = self.nd_lookup.recv() {
            self.rsp6.push_back(ArpEntry { found: Bit::One, mac: U::<48>::from(ip.raw() as u64 & 0xFFFF) });
        }
        let _ = (self.arp_resolve.recv(), self.nd_resolve.recv());
        if self.req4.recv().is_some() {
            self.open = Some(ReqOut { v6: false, bytes: vec![], clock: 0 });
        }
        if self.req6.recv().is_some() {
            self.open = Some(ReqOut { v6: true, bytes: vec![], clock: 0 });
        }
        for v6 in [false, true] {
            let ch = if v6 { &self.pay6 } else { &self.pay4 };
            if let Some(b) = ch.recv() {
                let o = self.open.as_mut().expect("payload byte before its request");
                o.bytes.push(b.data.raw() as u8);
                if b.last.to_bool() {
                    let mut q = self.open.take().unwrap();
                    q.clock = self.m_t1.read(0usize).raw() as u64;
                    self.reqs.push(q);
                }
            }
        }
        if let Some(s) = self.samples.recv() {
            self.samples_got.push(s);
        }
        if let Some(m) = self.miss.recv() {
            self.misses.push(m.raw() as u8);
        }
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
                        eth_src: U::<48>::from(ATT_MAC),
                        rx_ts: U::<64>::from(r.rx_ts),
                    });
                    *sent = true;
                }
            } else if self.rep_t.ready().to_bool() {
                let last = *k + 1 == r.pay.len();
                self.rep_t.send(FrameByte::new(r.pay[*k], last, last && r.err));
                *k += 1;
                if last {
                    self.inq.pop_front();
                }
            }
        }
        self.sim.cycle();
        self.now = (self.now + self.tick) & ((1u128 << 96) - 1);
    }

    fn run(&mut self, n: usize) {
        for _ in 0..n {
            self.cycle();
        }
    }

    fn until_reqs(&mut self, n: usize, max: usize) {
        for _ in 0..max {
            if self.reqs.len() >= n {
                return;
            }
            self.cycle();
        }
        panic!("{} requests expected, {} seen", n, self.reqs.len());
    }

    /// Feeds `r` and runs until it is consumed, plus 40 cycles.
    fn feed(&mut self, r: Reply) {
        self.inq.push_back((false, r, 0));
        for _ in 0..3000 {
            if self.inq.is_empty() {
                break;
            }
            self.cycle();
        }
        assert!(self.inq.is_empty(), "reply not consumed");
        self.run(40);
    }
}

fn reply_bytes(li: u8, vn: u8, mode: u8, stratum: u8, org: u64, t2: u64, t3: u64) -> Vec<u8> {
    let mut v = vec![(li << 6) | (vn << 3) | mode, stratum, 6, 0xE9];
    v.extend_from_slice(&0x10u32.to_be_bytes());
    v.extend_from_slice(&0x20u32.to_be_bytes());
    v.extend_from_slice(&0x4750_5300u32.to_be_bytes());
    v.extend_from_slice(&(T0 - 5 * SEC).to_be_bytes());
    v.extend_from_slice(&org.to_be_bytes());
    v.extend_from_slice(&t2.to_be_bytes());
    v.extend_from_slice(&t3.to_be_bytes());
    v
}

/// A reply to `q` from `src` claiming the server is `off` ahead: Origin =
/// the nonce sent, T2 / T3 / T4 from the real T1.
fn reply(q: &ReqOut, src: u128, off: u64) -> Reply {
    let t1 = q.clock;
    let ms = SEC / 1000;
    let t2 = t1.wrapping_add(ms).wrapping_add(off);
    Reply {
        v6: q.v6,
        src,
        sport: 123,
        rx_ts: t1.wrapping_add(3 * ms),
        pay: reply_bytes(0, 4, 4, 1, q.t1(), t2, t2.wrapping_add(ms)),
        err: false,
    }
}

fn one_request(v6: bool) -> (Bench<impl Future<Output = ()>>, ReqOut, u128) {
    let peer = if v6 { PEER6 } else { PEER4 as u128 };
    let mut b = bench();
    b.slot(0, peer, v6, true);
    b.until_reqs(1, 300);
    let q = b.reqs[0].clone();
    (b, q, peer)
}

// ------------------------------------------------------------ forged replies

/// Every off-path forgery that does not carry the exact 64-bit Origin, the
/// upstream's address and port 123, mode 4, VN 3/4, 48 bytes and a good
/// checksum produces neither a `Sample` nor a `miss`, and leaves the
/// request open: the genuine reply is still accepted afterwards. The
/// forged replies claim a 1000 s offset (what an attacker would want).
#[test]
fn forged_replies_never_make_a_sample() {
    for v6 in [false, true] {
        let (mut b, q, peer) = one_request(v6);
        let t1 = q.t1();
        let g = reply(&q, peer, 1000 * SEC);
        let mut bad: Vec<(String, Reply)> = vec![];
        // Origin wrong by any single bit, all zero, all one.
        for bit in 0..64 {
            let mut r = g.clone();
            r.pay[24..32].copy_from_slice(&(t1 ^ (1u64 << bit)).to_be_bytes());
            bad.push((format!("origin bit {bit}"), r));
        }
        for org in [0u64, u64::MAX] {
            let mut r = g.clone();
            r.pay[24..32].copy_from_slice(&org.to_be_bytes());
            bad.push((format!("origin {org:#x}"), r));
        }
        // Source address wrong in any byte, other family, other ports.
        for byte in 0..(if v6 { 16 } else { 4 }) {
            let mut r = g.clone();
            r.src ^= 1u128 << (8 * byte);
            bad.push((format!("source byte {byte}"), r));
        }
        let mut r = g.clone();
        r.v6 = !v6;
        bad.push(("other family".into(), r));
        for port in [0u16, 122, 124, 1123, 40000, 65535] {
            let mut r = g.clone();
            r.sport = port;
            bad.push((format!("source port {port}"), r));
        }
        // Modes other than 4, versions other than 3 and 4.
        for mode in [0u8, 1, 2, 3, 5, 6, 7] {
            let mut r = g.clone();
            r.pay[0] = (4 << 3) | mode;
            bad.push((format!("mode {mode}"), r));
        }
        for vn in [0u8, 1, 2, 5, 6, 7] {
            let mut r = g.clone();
            r.pay[0] = (vn << 3) | 4;
            bad.push((format!("version {vn}"), r));
        }
        // Truncated, and a bad UDP checksum / FCS (err on last).
        for n in [1usize, 8, 32, 40, 47] {
            let mut r = g.clone();
            r.pay.truncate(n);
            bad.push((format!("{n} bytes"), r));
        }
        let mut r = g.clone();
        r.err = true;
        bad.push(("err".into(), r));
        // KoD and unsynchronised replies with a wrong origin.
        for (li, st) in [(3u8, 0u8), (0, 0), (3, 16)] {
            let mut r = g.clone();
            r.pay[0] = (li << 6) | (4 << 3) | 4;
            r.pay[1] = st;
            r.pay[24..32].copy_from_slice(&(t1 ^ 0x8000_0000).to_be_bytes());
            bad.push((format!("LI {li} stratum {st}, wrong origin"), r));
        }
        let n = bad.len();
        for (what, r) in bad {
            b.feed(r);
            assert!(b.samples_got.is_empty(), "v6={v6}: {what} made a sample");
            assert!(b.misses.is_empty(), "v6={v6}: {what} made a miss");
        }
        // Still outstanding: the genuine reply is accepted (offset 1000 s).
        b.feed(g.clone());
        assert_eq!(b.samples_got.len(), 1, "v6={v6}: genuine reply after {n} forgeries");
        let off = b.samples_got[0].offset.raw() as u64;
        assert!(off.abs_diff(1000 * SEC) < SEC / 100, "offset as claimed");
    }
}

/// A reply replayed from an earlier poll (a capture of a genuine reply)
/// is rejected: its Origin is the previous T1. Only the reply to the
/// current request counts, and only once.
#[test]
fn replayed_reply_is_rejected() {
    let (mut b, q1, peer) = one_request(false);
    let g1 = reply(&q1, peer, 0);
    b.feed(g1.clone());
    assert_eq!(b.samples_got.len(), 1);
    b.feed(g1.clone());
    assert_eq!(b.samples_got.len(), 1, "duplicate of the answered request");
    // The next iburst poll (2 s later).
    b.now += ((2 * SEC + SEC / 8) as u128) << 32;
    b.until_reqs(2, 600);
    let q2 = b.reqs[1].clone();
    assert_ne!(q2.t1(), q1.t1());
    b.feed(g1.clone());
    assert_eq!(b.samples_got.len(), 1, "replay of poll 1 during poll 2");
    assert!(b.misses.is_empty());
    b.feed(reply(&q2, peer, 0));
    assert_eq!(b.samples_got.len(), 2);
}

/// What a KoD can do once its Origin is right (on-path, or a guess of the
/// 32-bit nonce, F3): one miss for that poll, no sample. A kiss code other
/// than RATE / DENY / RSTR changes nothing else (the iburst continues).
/// Since F9 a genuine RATE / DENY is honoured (see
/// `kod_rate_and_deny_are_honoured`), so a forged one can slow down or
/// silence one upstream slot: the price is the same Origin guess as for a
/// forged reply, and with three or more upstreams the selection keeps a
/// majority.
#[test]
fn kod_with_right_origin_costs_one_poll() {
    let (mut b, q1, peer) = one_request(false);
    let mut kod = reply(&q1, peer, 0);
    kod.pay[0] = (3 << 6) | (4 << 3) | 4;
    kod.pay[1] = 0;
    kod.pay[12..16].copy_from_slice(b"INIT");
    b.feed(kod);
    assert!(b.samples_got.is_empty());
    assert_eq!(b.misses, vec![0], "a KoD is one miss");
    // The schedule continues: the next iburst request goes out 2 s later.
    b.now += ((2 * SEC + SEC / 8) as u128) << 32;
    b.until_reqs(2, 600);
    b.feed(reply(&b.reqs[1].clone(), peer, 0));
    assert_eq!(b.samples_got.len(), 1, "polling resumed normally");
}

/// F9 fixed: an authentic RATE ends the iburst and doubles the slot's
/// interval (poll 6: the next request 2^7 s after the previous poll start
/// instead of 2 s); an authentic DENY (or RSTR) stops the slot's requests
/// for good (until reset), each later poll counting as a miss. A KoD with
/// a wrong Origin does neither.
#[test]
fn kod_rate_and_deny_are_honoured() {
    let kod = |q: &ReqOut, peer: u128, code: &[u8; 4], org: u64| {
        let mut r = reply(q, peer, 0);
        r.pay[0] = (3 << 6) | (4 << 3) | 4;
        r.pay[1] = 0;
        r.pay[12..16].copy_from_slice(code);
        r.pay[24..32].copy_from_slice(&org.to_be_bytes());
        r
    };
    // RATE
    let (mut b, q1, peer) = one_request(false);
    b.feed(kod(&q1, peer, b"RATE", q1.t1() ^ (1 << 7)));
    assert!(b.misses.is_empty(), "wrong Origin: nothing");
    b.feed(kod(&q1, peer, b"RATE", q1.t1()));
    assert_eq!(b.misses, vec![0]);
    let t0 = (b.now >> 64) as u64;
    for _ in 0..(130 * 4) {
        b.now += ((SEC / 4) as u128) << 32;
        b.run(40);
        if b.reqs.len() > 1 {
            break;
        }
    }
    assert_eq!(b.reqs.len(), 2);
    let dt = ((b.now >> 64) as u64).wrapping_sub(t0);
    assert!((127..=129).contains(&dt), "next request {dt} s later");
    // DENY
    let (mut b, q1, peer) = one_request(true);
    b.feed(kod(&q1, peer, b"DENY", q1.t1() ^ (1 << 40)));
    assert!(b.misses.is_empty(), "wrong Origin: nothing");
    b.feed(kod(&q1, peer, b"DENY", q1.t1()));
    assert_eq!(b.misses, vec![0]);
    for _ in 0..(200 * 4) {
        b.now += ((SEC / 4) as u128) << 32;
        b.run(40);
    }
    assert_eq!(b.reqs.len(), 1, "no request after DENY");
    assert!(b.misses.len() >= 3, "later polls are misses: {:?}", b.misses);
}

// ------------------------------------------------------------ T1 entropy

/// One poll of `t1_offsets`: the Transmit Timestamp sent and the clock
/// reading T1, both minus the latency and the poll tick boundary.
struct PollT1 {
    sent: u64,
    clock: u64,
}

/// Runs one IPv4 slot for 41 polls (iburst then regular, poll 6) with the
/// clock at its real rate and the board's entropy unit; returns, for polls
/// 1..=40 (the first poll is not tick aligned: it starts when the slot is
/// armed), the Transmit Timestamp sent and the clock reading T1, both
/// minus the poll tick boundary and the latency.
fn t1_offsets() -> Vec<PollT1> {
    let mut b = bench();
    b.slot(0, PEER4 as u128, false, true);
    b.lat4.set(U::<64>::from(8032u64));
    b.tick = INCR;
    let mut rng = 0x1234_5678_9abc_def1u64;
    let mut next = || {
        rng ^= rng << 13;
        rng ^= rng >> 7;
        rng ^= rng << 17;
        rng
    };
    let mut offs = vec![];
    b.until_reqs(1, 2000);
    b.feed(reply(&b.reqs[0].clone(), PEER4 as u128, 0));
    for k in 1..=40usize {
        // The poll tick: `last` was the 2^-6 s tick of the previous poll start
        // (its T1 minus the latency, a few cycles after the tick).
        let prev = b.reqs[k - 1].clock - 8032;
        let last_tick = prev >> 26;
        let period_ticks = if k < 4 { 2 * 64 } else { 64 * 64 };
        let boundary = (last_tick + period_ticks) << 26; // 32.32
        // Jump to 50..400 cycles before the boundary, random sub-cycle phase
        // (the time in between is idle for the client).
        let before = 50 + (next() % 350) as u128;
        let phase = (next() as u128) % INCR;
        b.now = ((boundary as u128) << 32) - before * INCR - phase;
        b.until_reqs(k + 1, 2000);
        let q = b.reqs[k].clone();
        offs.push(PollT1 {
            sent: q.t1().wrapping_sub(8032).wrapping_sub(boundary),
            clock: q.clock.wrapping_sub(8032).wrapping_sub(boundary),
        });
        b.feed(reply(&q, PEER4 as u128, 0));
    }
    assert_eq!(b.samples_got.len(), 41);
    offs
}

/// Why F3 needed a nonce: our clock reading T1 is predictable. The client
/// starts a poll at the first scheduler examination after the poll tick
/// (2^-6 s grid) is crossed, and reads the clock a fixed number of cycles
/// later. With the clock running at its real rate, sub-cycle phase chosen
/// at random, over 40 polls of one slot, T1 - (poll tick boundary) -
/// latency takes values in a window of a few cycles: about 7 to 8 bits of
/// uncertainty for someone who knows our clock (our own server tells it to
/// anyone) and the poll tick (our server's Reference Timestamp follows each
/// update; the poll period is fixed). Before F3 this T1 was the Origin; it
/// is now only kept for the arithmetic (`m_t1`) and never sent: the
/// Transmit Timestamp sent shares only its seconds.
#[test]
fn t1_is_predictable_from_the_poll_tick() {
    let offs = t1_offsets();
    let c: Vec<u64> = offs.iter().map(|o| o.clock).collect();
    let (lo, hi) = (*c.iter().min().unwrap(), *c.iter().max().unwrap());
    let ns = |x: u64| x as f64 * 1e9 / SEC as f64;
    let mut distinct = c.clone();
    distinct.sort();
    distinct.dedup();
    println!(
        "clock T1 - tick - latency over 40 polls: {:.1} .. {:.1} ns ({} 2^-32 s units wide, {} distinct values, {:.1} bits)",
        ns(lo),
        ns(hi),
        hi - lo + 1,
        distinct.len(),
        ((hi - lo + 1) as f64).log2()
    );
    // The whole spread is a few clock cycles: less than 2^9 units of
    // 2^-32 s (120 ns), i.e. < 9 bits of uncertainty given the tick.
    assert!(hi - lo < 512, "spread {} units", hi - lo);
    assert!(ns(hi) < 1000.0, "a fixed number of cycles after the tick");
    // It is not what goes on the wire (only its seconds are).
    for o in &offs {
        assert_ne!(o.sent, o.clock);
        assert_eq!(
            (o.sent.wrapping_add(8032)) >> 32,
            (o.clock.wrapping_add(8032)) >> 32
        );
    }
}

/// F3 fixed: the Transmit Timestamp of a request (our Origin) is
/// unpredictable from our clock and schedule: its 32 fraction bits come
/// from the entropy unit, so its offset to the poll tick is spread over the
/// whole second (at least 2^20 values here, no repeat, every fraction bit
/// both 0 and 1 over the 40 polls); its seconds are T1's.
#[test]
fn t1_is_unpredictable() {
    let offs = t1_offsets();
    let sent: Vec<u64> = offs.iter().map(|o| o.sent).collect();
    let mut distinct = sent.clone();
    distinct.sort();
    distinct.dedup();
    assert_eq!(distinct.len(), sent.len(), "no repeated value");
    let fr: Vec<u32> = sent.iter().map(|&x| x.wrapping_add(8032) as u32).collect();
    let (lo, hi) = (fr.iter().min().unwrap(), fr.iter().max().unwrap());
    assert!(hi - lo >= 1 << 20, "spread {} units only", hi - lo);
    let or = fr.iter().fold(0u32, |a, &x| a | x);
    let and = fr.iter().fold(u32::MAX, |a, &x| a & x);
    assert_eq!((or, and), (u32::MAX, 0), "a fraction bit is stuck");
}

// ------------------------------------------------------------ ARP cache

fn mac_bytes(m: u64) -> [u8; 6] {
    let b = m.to_be_bytes();
    [b[2], b[3], b[4], b[5], b[6], b[7]]
}

fn arp_pkt(op: u16, sha: u64, spa: u32, tha: u64, tpa: u32) -> Vec<u8> {
    let mut v = vec![0, 1, 8, 0, 6, 4];
    v.extend_from_slice(&op.to_be_bytes());
    v.extend_from_slice(&mac_bytes(sha));
    v.extend_from_slice(&spa.to_be_bytes());
    v.extend_from_slice(&mac_bytes(tha));
    v.extend_from_slice(&tpa.to_be_bytes());
    v.resize(46, 0);
    v
}

/// (eth dst, eth src, ARP payload) frames into `Arp`, then lookups of
/// `look`, one after the other; returns (found, mac) per lookup. `resolve`
/// is an address the unit starts resolving before the first frame.
fn arp_run(resolve: Option<u32>, frames: &[(u64, u64, Vec<u8>)], look: &[u32]) -> Vec<(bool, u64)> {
    let (mac_o, mac) = signal::<U<48>, DefaultClock>();
    let (ip_o, ip) = signal::<U<32>, DefaultClock>();
    let (fl_o, fl) = signal::<Bit, DefaultClock>();
    let (mt, mr) = chan::<EthMeta, DefaultClock>();
    let (pt, pr) = chan::<FrameByte, DefaultClock>();
    let (rt, rr) = chan::<U<32>, DefaultClock>();
    let (lt, lr) = chan::<U<32>, DefaultClock>();
    let (ht, hr) = chan::<EthHdr, DefaultClock>();
    let (ot, or) = chan::<FrameByte, DefaultClock>();
    let (st, sr) = chan::<ArpEntry, DefaultClock>();
    mac_o.set(U::<48>::from(OUR_MAC));
    ip_o.set(U::<32>::from(LOCAL4));
    fl_o.set(Bit::Zero);
    let mut dut = Arp::default();
    let mut sim = Running::new(dut.run((mr, pr, mac, ip, rr, lr, fl), (ht, ot, st)));
    let (mut fi, mut bi) = (0usize, 0usize);
    let mut li = 0;
    let mut out = vec![];
    let mut resolve = resolve;
    for _ in 0..(frames.len() * 120 + look.len() * 10 + 200) {
        if let Some(r) = resolve {
            if rt.ready().to_bool() {
                rt.send(U::<32>::from(r));
                resolve = None;
            }
        }
        if fi < frames.len() {
            let (d, s, p) = &frames[fi];
            if bi == 0 {
                if mt.ready().to_bool() {
                    mt.send(EthMeta {
                        dst: U::<48>::from(*d),
                        src: U::<48>::from(*s),
                        ethertype: U::<16>::from(0x0806u16),
                        multicast: Bit::from_bool((d >> 40) & 1 == 1),
                        ts: U::<64>::from(0u64),
                    });
                    bi = 1;
                }
            } else if pt.ready().to_bool() {
                let last = bi == p.len();
                pt.send(FrameByte::new(p[bi - 1], last, false));
                bi += 1;
                if last {
                    fi += 1;
                    bi = 0;
                }
            }
        } else if li < look.len() && lt.ready().to_bool() {
            lt.send(U::<32>::from(look[li]));
            li += 1;
        }
        if let Some(r) = sr.recv() {
            out.push((r.found.to_bool(), r.mac.raw() as u64));
        }
        let _ = (hr.recv(), or.recv());
        sim.cycle();
    }
    assert_eq!(out.len(), look.len());
    out
}

const BCAST: u64 = 0xFFFF_FFFF_FFFF;

/// ARP cache poisoning (finding F5, fixed): the cache learns only the reply
/// that answers our own pending resolution (op 2, sender = the address being
/// resolved, addressed to our IP and MAC). The gateway is being resolved and
/// answers first; whatever an on-link attacker sends afterwards (unsolicited
/// reply, request carrying the gateway as sender, gratuitous ARP, wrong
/// target...) never rebinds the entry.
#[test]
fn arp_poisoning_of_the_gateway() {
    let legit = (OUR_MAC, GW_MAC, arp_pkt(2, GW_MAC, GW4, OUR_MAC, LOCAL4));
    let cases: Vec<(&str, (u64, u64, Vec<u8>))> = vec![
        ("unsolicited reply to us", (OUR_MAC, ATT_MAC, arp_pkt(2, ATT_MAC, GW4, OUR_MAC, LOCAL4))),
        ("request for us, broadcast", (BCAST, ATT_MAC, arp_pkt(1, ATT_MAC, GW4, 0, LOCAL4))),
        ("reply, eth src differs from sha", (OUR_MAC, GW_MAC, arp_pkt(2, ATT_MAC, GW4, OUR_MAC, LOCAL4))),
        ("gratuitous request (tpa = spa)", (BCAST, ATT_MAC, arp_pkt(1, ATT_MAC, GW4, 0, GW4))),
        ("gratuitous reply (tpa = spa)", (BCAST, ATT_MAC, arp_pkt(2, ATT_MAC, GW4, BCAST, GW4))),
        ("request for another host", (BCAST, ATT_MAC, arp_pkt(1, ATT_MAC, GW4, 0, 0xC000_0277))),
    ];
    for (what, f) in cases {
        let r = arp_run(Some(GW4), &[legit.clone(), f], &[GW4]);
        assert_eq!(r, vec![(true, GW_MAC)], "{what}");
    }
}

/// Attacks that reach us while the gateway is being resolved but are not a
/// valid reply to our request: they are ignored and the genuine reply that
/// follows is learned. Without any resolution nothing is ever learned.
#[test]
fn arp_invalid_replies_while_resolving_and_without_resolution() {
    let legit = (OUR_MAC, GW_MAC, arp_pkt(2, GW_MAC, GW4, OUR_MAC, LOCAL4));
    let bad = vec![
        (BCAST, ATT_MAC, arp_pkt(1, ATT_MAC, GW4, 0, LOCAL4)),
        (OUR_MAC, ATT_MAC, arp_pkt(2, ATT_MAC, GW4, OUR_MAC, LOCAL4 + 1)),
        (OUR_MAC, ATT_MAC, arp_pkt(2, ATT_MAC, GW4, ATT_MAC, LOCAL4)),
        (OUR_MAC, ATT_MAC, arp_pkt(2, ATT_MAC, GW4 + 1, OUR_MAC, LOCAL4)),
        (BCAST, ATT_MAC, arp_pkt(2, ATT_MAC, GW4, BCAST, GW4)),
    ];
    for f in &bad {
        assert_eq!(arp_run(Some(GW4), &[f.clone(), legit.clone()], &[GW4, GW4 + 1]), vec![(true, GW_MAC), (false, 0)]);
        // No resolution: neither the attack nor even the genuine reply is learned.
        assert_eq!(arp_run(None, &[f.clone(), legit.clone()], &[GW4]), vec![(false, 0)]);
    }
}

/// Residual risk, stated (docs/security.md F5): an attacker that answers our
/// own request BEFORE the gateway wins the race (ARP without DAI/SEND); the
/// genuine reply that follows is then unsolicited and ignored.
#[test]
fn arp_residual_race_attacker_answers_first() {
    let att = (OUR_MAC, ATT_MAC, arp_pkt(2, ATT_MAC, GW4, OUR_MAC, LOCAL4));
    let legit = (OUR_MAC, GW_MAC, arp_pkt(2, GW_MAC, GW4, OUR_MAC, LOCAL4));
    assert_eq!(arp_run(Some(GW4), &[att, legit], &[GW4]), vec![(true, ATT_MAC)]);
}

/// The desired property behind F5, formerly ignored: the gateway entry is
/// not rebound by unsolicited packets.
#[test]
fn arp_gateway_entry_is_not_rebound_by_unsolicited_packets() {
    let legit = (OUR_MAC, GW_MAC, arp_pkt(2, GW_MAC, GW4, OUR_MAC, LOCAL4));
    for f in [
        (OUR_MAC, ATT_MAC, arp_pkt(2, ATT_MAC, GW4, OUR_MAC, LOCAL4)),
        (BCAST, ATT_MAC, arp_pkt(1, ATT_MAC, GW4, 0, LOCAL4)),
    ] {
        assert_eq!(arp_run(Some(GW4), &[legit.clone(), f], &[GW4]), vec![(true, GW_MAC)]);
    }
}

/// Cache churn (finding F6, fixed by F5): any number of requests for us
/// (and unsolicited replies) from other hosts create no entry, so they cannot
/// evict the gateway.
#[test]
fn arp_cache_eviction_by_ordinary_requests() {
    let mut frames = vec![(OUR_MAC, GW_MAC, arp_pkt(2, GW_MAC, GW4, OUR_MAC, LOCAL4))];
    for k in 0..6u64 {
        let m = 0x0200_0000_1000 + k;
        if k % 2 == 0 {
            frames.push((BCAST, m, arp_pkt(1, m, 0xC000_0210 + k as u32, 0, LOCAL4)));
        } else {
            frames.push((OUR_MAC, m, arp_pkt(2, m, 0xC000_0210 + k as u32, OUR_MAC, LOCAL4)));
        }
    }
    let r = arp_run(Some(GW4), &frames, &[GW4, 0xC000_0210]);
    assert_eq!(r, vec![(true, GW_MAC), (false, 0)], "the gateway stays, the clients are not learned");
}

// ------------------------------------------------------------ IPv6 neighbor cache

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

/// An ICMPv6 message (type, code, body after the checksum) in a full
/// Ethernet frame, checksum computed.
fn icmp6_frame(dst_mac: u64, src_mac: u64, src: u128, dst: u128, hop: u8, ty: u8, body: &[u8]) -> Vec<u8> {
    let mut m = vec![ty, 0, 0, 0];
    m.extend_from_slice(body);
    let mut a = sum16(0, &src.to_be_bytes());
    a = sum16(a, &dst.to_be_bytes());
    a = sum16(a, &(m.len() as u32).to_be_bytes());
    a += 58;
    a = sum16(a, &m);
    let c = !fold(a);
    m[2] = (c >> 8) as u8;
    m[3] = c as u8;
    let mut f = mac_bytes(dst_mac).to_vec();
    f.extend_from_slice(&mac_bytes(src_mac));
    f.extend_from_slice(&[0x86, 0xdd, 0x60, 0, 0, 0]);
    f.extend_from_slice(&(m.len() as u16).to_be_bytes());
    f.extend_from_slice(&[58, hop]);
    f.extend_from_slice(&src.to_be_bytes());
    f.extend_from_slice(&dst.to_be_bytes());
    f.extend_from_slice(&m);
    f
}

/// NA body: flags (R S O in the top bits), target, optional TLLA.
fn na(flags: u8, target: u128, tlla: Option<u64>) -> Vec<u8> {
    let mut v = vec![flags, 0, 0, 0];
    v.extend_from_slice(&target.to_be_bytes());
    if let Some(m) = tlla {
        v.extend_from_slice(&[2, 1]);
        v.extend_from_slice(&mac_bytes(m));
    }
    v
}

/// NS body: target, optional SLLA.
fn ns(target: u128, slla: Option<u64>) -> Vec<u8> {
    let mut v = vec![0, 0, 0, 0];
    v.extend_from_slice(&target.to_be_bytes());
    if let Some(m) = slla {
        v.extend_from_slice(&[1, 1]);
        v.extend_from_slice(&mac_bytes(m));
    }
    v
}

/// Frames through eth_rx -> ipv6_rx -> icmpv6 -> nbr_cache, then lookups;
/// returns (found, mac) per lookup. `resolve` is the address the unit
/// solicits before the first frame (None: no solicitation). An empty frame
/// stands for an idle gap of 400 cycles.
fn nd_run(resolve: Option<u128>, frames: &[Vec<u8>], look: &[u128]) -> Vec<(bool, u64)> {
    nd_run_t::<Board>(resolve, frames, look)
}

fn nd_run_t<T: ArpTiming>(resolve: Option<u128>, frames: &[Vec<u8>], look: &[u128]) -> Vec<(bool, u64)> {
    let (mac_o, mac1) = signal::<U<48>, DefaultClock>();
    let mac2 = mac1.clone();
    let (sn_o, sn1) = signal::<U<24>, DefaultClock>();
    let sn2 = sn1.clone();
    let (ll_o, ll1) = signal::<U<128>, DefaultClock>();
    let ll2 = ll1.clone();
    let (gua_o, gua1) = signal::<U<128>, DefaultClock>();
    let gua2 = gua1.clone();
    let (gv_o, gv1) = signal::<Bit, DefaultClock>();
    let gv2 = gv1.clone();
    let (fl_o, fl) = signal::<Bit, DefaultClock>();
    mac_o.set(U::<48>::from(OUR_MAC));
    sn_o.set(U::<24>::from(1u32));
    ll_o.set(U::<128>::from(LL));
    gua_o.set(U::<128>::from(GUA));
    gv_o.set(Bit::One);
    fl_o.set(Bit::Zero);
    let (in_t, in_r) = chan::<FrameByte, DefaultClock>();
    let (am_t, am) = chan::<EthMeta, DefaultClock>();
    let (ap_t, ap) = chan::<FrameByte, DefaultClock>();
    let (v4m_t, v4m) = chan::<EthMeta, DefaultClock>();
    let (v4p_t, v4p) = chan::<FrameByte, DefaultClock>();
    let (v6m_t, v6m) = chan::<EthMeta, DefaultClock>();
    let (v6p_t, v6p) = chan::<FrameByte, DefaultClock>();
    let (im_t, im) = chan::<Ip6Meta, DefaultClock>();
    let (i_t, i) = chan::<FrameByte, DefaultClock>();
    let (um_t, um) = chan::<Ip6Meta, DefaultClock>();
    let (u_t, u) = chan::<FrameByte, DefaultClock>();
    let (rq_t, rq) = chan::<Ip6TxReq, DefaultClock>();
    let (op_t, op) = chan::<FrameByte, DefaultClock>();
    let (st_t, st_r) = chan::<Ts, DefaultClock>();
    let (rs_t, rs_r) = chan::<U<128>, DefaultClock>();
    let (ln_t, ln_r) = chan::<NdLearn, DefaultClock>();
    let (lk_t, lk_r) = chan::<U<128>, DefaultClock>();
    let (rsp_t, rsp_r) = chan::<ArpEntry, DefaultClock>();
    let (mut d_eth, mut d_rx, mut d_icmp, mut d_cache) =
        (EthRx::default(), Ipv6Rx::default(), Icmpv6With::<T>::default(), NbrCache::default());
    let mut sim = Running::new(join2(
        join2(
            d_eth.run((in_r, mac1, sn1, st_r), (am_t, ap_t, v4m_t, v4p_t, v6m_t, v6p_t)),
            d_rx.run((v6m, v6p, ll1, gua1, gv1, sn2), (im_t, i_t, um_t, u_t)),
        ),
        join2(
            d_icmp.run((im, i, ll2, gua2, gv2, mac2, rs_r), (rq_t, op_t, ln_t)),
            d_cache.run((ln_r, lk_r, fl), (rsp_t,)),
        ),
    ));
    let (mut fi, mut bi, mut li, mut stamps) = (0usize, 0usize, 0usize, 0u64);
    let mut out = vec![];
    // Lookups start 150 cycles after the last frame byte (the learn has
    // reached the cache by then).
    let mut quiet = 0usize;
    let mut resolve = resolve;
    let mut gap = 0usize;
    for _ in 0..(frames.len() * 200 + look.len() * 10 + 300 + 400 * frames.iter().filter(|f| f.is_empty()).count()) {
        if let Some(r) = resolve {
            if rs_t.ready().to_bool() {
                rs_t.send(U::<128>::from(r));
                resolve = None;
            }
        }
        if stamps < frames.len() as u64 && st_t.ready().to_bool() {
            stamps += 1;
            st_t.send(Ts::from(stamps));
        }
        if fi < frames.len() && frames[fi].is_empty() {
            gap += 1;
            if gap == 400 {
                gap = 0;
                fi += 1;
            }
        } else if fi < frames.len() {
            if in_t.ready().to_bool() {
                let f = &frames[fi];
                let last = bi + 1 == f.len();
                in_t.send(FrameByte::new(f[bi], last, false));
                bi += 1;
                if last {
                    fi += 1;
                    bi = 0;
                }
            }
        } else if quiet < 150 {
            quiet += 1;
        } else if li < look.len() && lk_t.ready().to_bool() {
            lk_t.send(U::<128>::from(look[li]));
            li += 1;
        }
        let _ = (am.recv(), ap.recv(), v4m.recv(), v4p.recv(), um.recv(), u.recv(), rq.recv(), op.recv());
        if let Some(r) = rsp_r.recv() {
            out.push((r.found.to_bool(), r.mac.raw() as u64));
        }
        sim.cycle();
    }
    assert_eq!(out.len(), look.len());
    out
}

/// NDP spoofing (finding F5, IPv6 side, fixed): an NA is learned only when
/// it answers our outstanding solicitation (target = the address being
/// resolved, S = 1, source = target, hop limit 255); an NS never teaches.
/// The gateway fe80::1 is solicited and answers; whatever the attacker sends
/// afterwards, or before the genuine NA, never wins.
#[test]
fn nd_spoofing_of_the_gateway() {
    let legit = icmp6_frame(OUR_MAC, GW_MAC, GW6, LL, 255, 136, &na(0x60, GW6, Some(GW_MAC)));
    let all_nodes = 0xff02_0000_0000_0000_0000_0000_0000_0001u128;
    let other = 0xfe80_0000_0000_0000_0000_0000_0000_0bad_u128;
    let cases: Vec<(&str, Vec<u8>)> = vec![
        ("unsolicited NA, S=0 O=1", icmp6_frame(OUR_MAC, ATT_MAC, GW6, LL, 255, 136, &na(0x20, GW6, Some(ATT_MAC)))),
        ("NA with S=0 O=0", icmp6_frame(OUR_MAC, ATT_MAC, GW6, LL, 255, 136, &na(0x00, GW6, Some(ATT_MAC)))),
        ("NA without TLLA, S=0", icmp6_frame(OUR_MAC, ATT_MAC, GW6, LL, 255, 136, &na(0x20, GW6, None))),
        ("NA S=1 from another source address", icmp6_frame(OUR_MAC, ATT_MAC, other, LL, 255, 136, &na(0x60, GW6, Some(ATT_MAC)))),
        ("NA S=1 for another target", icmp6_frame(OUR_MAC, ATT_MAC, other, LL, 255, 136, &na(0x60, other, Some(ATT_MAC)))),
        ("NS for us with SLLA, source = gateway", icmp6_frame(OUR_MAC, ATT_MAC, GW6, LL, 255, 135, &ns(LL, Some(ATT_MAC)))),
        ("NA S=1 hop limit 64 (off link)", icmp6_frame(OUR_MAC, ATT_MAC, GW6, LL, 64, 136, &na(0x60, GW6, Some(ATT_MAC)))),
        ("NA to all-nodes, S=0", icmp6_frame(0x3333_0000_0001, ATT_MAC, GW6, all_nodes, 255, 136, &na(0x20, GW6, Some(ATT_MAC)))),
    ];
    for (what, f) in cases {
        let r = nd_run(Some(GW6), &[legit.clone(), f.clone()], &[GW6]);
        assert_eq!(r, vec![(true, GW_MAC)], "{what}: after the genuine NA");
        let r = nd_run(Some(GW6), &[f.clone(), legit.clone()], &[GW6]);
        assert_eq!(r, vec![(true, GW_MAC)], "{what}: while resolving");
        let r = nd_run(None, &[f, legit.clone()], &[GW6]);
        assert_eq!(r, vec![(false, 0)], "{what}: no solicitation, nothing is learned");
    }
}

/// Residual risk (docs/security.md F5): an attacker's valid-looking solicited
/// NA that arrives before the genuine one wins the race (no SEND).
#[test]
fn nd_residual_race_attacker_answers_first() {
    let att = icmp6_frame(OUR_MAC, ATT_MAC, GW6, LL, 255, 136, &na(0x60, GW6, Some(ATT_MAC)));
    let legit = icmp6_frame(OUR_MAC, GW_MAC, GW6, LL, 255, 136, &na(0x60, GW6, Some(GW_MAC)));
    assert_eq!(nd_run(Some(GW6), &[att, legit], &[GW6]), vec![(true, ATT_MAC)]);
}

/// Neighbor cache churn (finding F6, fixed by F5): unsolicited NAs for
/// unrelated targets, even with S = 1, no longer evict the gateway.
#[test]
fn nd_cache_eviction_by_unsolicited_na() {
    let mut frames = vec![icmp6_frame(OUR_MAC, GW_MAC, GW6, LL, 255, 136, &na(0x60, GW6, Some(GW_MAC)))];
    for k in 0..6u128 {
        let t = 0x2001_0db8_0000_0000_0000_0000_0000_1000 + k;
        frames.push(icmp6_frame(OUR_MAC, ATT_MAC, t, GUA, 255, 136, &na(0x60, t, Some(ATT_MAC))));
    }
    let r = nd_run(Some(GW6), &frames, &[GW6]);
    assert_eq!(r, vec![(true, GW_MAC)], "the gateway stays");
}

/// An NA that arrives after the pending-solicitation timeout is ignored
/// (`Sim` timeout: 300 cycles); on time it is learned.
#[test]
fn nd_na_after_the_timeout_is_not_learned() {
    let legit = icmp6_frame(OUR_MAC, GW_MAC, GW6, LL, 255, 136, &na(0x60, GW6, Some(GW_MAC)));
    let r = nd_run_t::<Sim>(Some(GW6), &[vec![], legit.clone()], &[GW6]);
    assert_eq!(r, vec![(false, 0)], "late NA");
    let r = nd_run_t::<Sim>(Some(GW6), &[legit.clone()], &[GW6]);
    assert_eq!(r, vec![(true, GW_MAC)], "on-time NA");
    // A second genuine NA after the entry was learned is ignored (one learn per solicitation).
    let again = icmp6_frame(OUR_MAC, ATT_MAC, GW6, LL, 255, 136, &na(0x60, GW6, Some(ATT_MAC)));
    let r = nd_run_t::<Sim>(Some(GW6), &[legit, again], &[GW6]);
    assert_eq!(r, vec![(true, GW_MAC)], "a second solicited-looking NA");
}
