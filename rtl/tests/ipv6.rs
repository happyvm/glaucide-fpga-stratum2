// SPDX-License-Identifier: Apache-2.0
//! Tests of `ipv6_tx` (header bytes, length abort, vectors of the Python
//! model, tx_arb source 3) and `ipv6_rx` (rules G1-G8, truncation, err,
//! chain from `eth_rx`) for task T3.1, plus the Verilog lowering.
use ntp_fpga::eth_rx::{EthMeta, EthRx};
use ntp_fpga::eth_tx::EthHdr;
use ntp_fpga::ipv6_rx::{Ip6Meta, Ipv6Rx};
use ntp_fpga::ipv6_tx::{Ip6TxReq, Ipv6Tx};
use ntp_fpga::stream::FrameByte;
use ntp_fpga::tx_arb::TxArb;
use txhdl::comp::{chan, join2, signal, DefaultClock, Running, Unit};
use txhdl::types::{Bit, U};

const MAC: u64 = 0x0200_0000_0001;
const PEER_MAC: u64 = 0x0200_0000_0002;
const LL: u128 = 0xfe80_0000_0000_0000_0000_00ff_fe00_0001; // fe80::ff:fe00:1
const GUA: u128 = 0x2001_0db8_0000_0000_0000_0000_0000_0001; // 2001:db8::1
const PEER: u128 = 0x2001_0db8_0000_0000_0000_0000_0000_0002;
const PEER_LL: u128 = 0xfe80_0000_0000_0000_0000_00ff_fe00_0002;
const SN_LOW: u32 = 1;
const ALL_NODES: u128 = 0xff02_0000_0000_0000_0000_0000_0000_0001;

fn sn_of(low: u32) -> u128 {
    0xff02_0000_0000_0000_0000_0001_ff00_0000u128 | (low & 0xFF_FFFF) as u128
}
fn sn_mac(low: u32) -> u64 {
    0x3333_ff00_0000u64 | (low & 0xFF_FFFF) as u64
}

// ------------------------------------------------------------ ipv6_tx

struct TxFrame {
    dst_mac: u64,
    ethertype: u16,
    bytes: Vec<FrameByte>,
}

impl TxFrame {
    fn data(&self) -> Vec<u8> {
        self.bytes.iter().map(|b| b.data.raw() as u8).collect()
    }
    fn err(&self) -> bool {
        self.bytes.last().unwrap().err.to_bool()
    }
}

/// One request and the payload the test offers for it.
#[derive(Clone)]
struct Job {
    dst_mac: u64,
    dst_ip: u128,
    src_ip: u128,
    next: u8,
    hop: u8,
    len: u16,
    payload: Vec<u8>,
    err: bool,
}

fn run_tx(jobs: &[Job], gap: usize, take: impl Fn(usize) -> bool) -> Vec<TxFrame> {
    let (rq_t, rq) = chan::<Ip6TxReq, DefaultClock>();
    let (pl_t, pl) = chan::<FrameByte, DefaultClock>();
    let (h_t, h) = chan::<EthHdr, DefaultClock>();
    let (o_t, o) = chan::<FrameByte, DefaultClock>();
    let mut dut = Ipv6Tx::default();
    let mut sim = Running::new(dut.run((rq, pl), (h_t, o_t)));
    let mut next_req = 0;
    let mut pf = 0;
    let mut pi = 0;
    let total: usize = jobs.iter().map(|j| j.payload.len() + 50).sum();
    let mut frames: Vec<TxFrame> = vec![];
    let mut open: Vec<FrameByte> = vec![];
    let mut hdrs: Vec<(u64, u16)> = vec![];
    for t in 0..(total * (gap + 2) * 2 + 400) {
        if next_req < jobs.len() && rq_t.ready().to_bool() {
            let j = &jobs[next_req];
            rq_t.send(Ip6TxReq {
                dst_mac: U::<48>::from(j.dst_mac),
                dst_ip: U::<128>::from(j.dst_ip),
                src_ip: U::<128>::from(j.src_ip),
                next: U::<8>::from(j.next),
                len: U::<16>::from(j.len),
                hop: U::<8>::from(j.hop),
            });
            next_req += 1;
        }
        if pf < jobs.len() && (t % (gap + 1)) == 0 && pl_t.ready().to_bool() {
            let j = &jobs[pf];
            let last = pi + 1 == j.payload.len();
            pl_t.send(FrameByte::new(j.payload[pi], last, last && j.err));
            if last {
                pf += 1;
                pi = 0;
            } else {
                pi += 1;
            }
        }
        if take(t) {
            if let Some(x) = h.recv() {
                hdrs.push((x.dst.raw() as u64, x.ethertype.raw() as u16));
            }
            if let Some(b) = o.recv() {
                assert!(
                    !hdrs.is_empty() && hdrs.len() > frames.len(),
                    "payload byte before its hdr"
                );
                assert!(!b.err.to_bool() || b.last.to_bool(), "err without last");
                open.push(b);
                if b.last.to_bool() {
                    let (d, e) = hdrs[frames.len()];
                    frames.push(TxFrame {
                        dst_mac: d,
                        ethertype: e,
                        bytes: std::mem::take(&mut open),
                    });
                }
            }
        }
        sim.cycle();
    }
    assert!(open.is_empty(), "frame left open");
    assert_eq!(hdrs.len(), frames.len(), "a hdr without a complete frame");
    frames
}

fn job(dst_ip: u128, src_ip: u128, next: u8, hop: u8, n: usize) -> Job {
    Job {
        dst_mac: PEER_MAC,
        dst_ip,
        src_ip,
        next,
        hop,
        len: n as u16,
        payload: (0..n).map(|i| (i * 7 + 3) as u8).collect(),
        err: false,
    }
}

/// The 40-byte header ipv6_tx must send, built independently.
fn ref_header(dst: u128, src: u128, next: u8, hop: u8, len: u16) -> Vec<u8> {
    let mut h = vec![0x60, 0, 0, 0, (len >> 8) as u8, len as u8, next, hop];
    h.extend_from_slice(&src.to_be_bytes());
    h.extend_from_slice(&dst.to_be_bytes());
    h
}

fn from_hex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

/// (is_rx, frame) lines of a vector file.
fn vector(name: &str) -> Vec<(bool, Vec<u8>)> {
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
        v.push((
            match d {
                "rx" => true,
                "tx" => false,
                _ => panic!("bad line {l}"),
            },
            from_hex(h),
        ));
    }
    v
}

fn u128_at(b: &[u8], i: usize) -> u128 {
    u128::from_be_bytes(b[i..i + 16].try_into().unwrap())
}

fn mac_of(f: &[u8]) -> u64 {
    u64::from_be_bytes([0, 0, f[0], f[1], f[2], f[3], f[4], f[5]])
}

/// Jobs rebuilt from the IPv6 `tx` frames of the vectors.
fn vector_jobs(names: &[&str]) -> Vec<(Job, Vec<u8>)> {
    let mut v = vec![];
    for n in names {
        for (rx, f) in vector(n) {
            if rx || f[12] != 0x86 || f[13] != 0xDD {
                continue;
            }
            let plen = f.len() - 54;
            let j = Job {
                dst_mac: mac_of(&f),
                src_ip: u128_at(&f, 22),
                dst_ip: u128_at(&f, 38),
                next: f[20],
                hop: f[21],
                len: plen as u16,
                payload: f[54..].to_vec(),
                err: false,
            };
            v.push((j, f[14..].to_vec()));
        }
    }
    v
}

#[test]
fn tx_header_vectors_from_python_model() {
    let jobs = vector_jobs(&["ping6.hex", "ntp6_basic.hex", "ndp_ns_na.hex"]);
    assert!(jobs.len() >= 4, "vectors not found");
    for (gap, tk) in [(0usize, false), (2, true)] {
        let js: Vec<Job> = jobs.iter().map(|(j, _)| j.clone()).collect();
        let frames = run_tx(&js, gap, |t| !tk || (t % 3 != 1));
        assert_eq!(frames.len(), js.len());
        for ((j, exp), f) in jobs.iter().zip(&frames) {
            assert_eq!(f.ethertype, 0x86DD);
            assert_eq!(f.dst_mac, j.dst_mac);
            assert!(!f.err());
            let d = f.data();
            assert_eq!(&d[..40], &exp[..40], "40-byte header");
            assert_eq!(&d[..], &exp[..], "whole packet");
            assert_eq!(&d[..40], &ref_header(j.dst_ip, j.src_ip, j.next, j.hop, j.len)[..]);
        }
    }
}

#[test]
fn tx_headers_many_combinations() {
    let addrs = [
        0u128,
        1,
        GUA,
        PEER_LL,
        u128::MAX,
        ALL_NODES,
        0x0123_4567_89ab_cdef_0011_2233_4455_6677,
        0x8000_0000_0000_0000_0000_0000_0000_0001,
    ];
    let nexts = [0u8, 6, 17, 58, 255];
    let hops = [0u8, 1, 64, 255];
    let lens = [1usize, 2, 8, 41, 255, 256, 1000, 1460];
    let mut jobs = vec![];
    let mut x = 0x1234_5678u32;
    for &d in &addrs {
        for &s in &addrs {
            x = x.wrapping_mul(1664525).wrapping_add(1013904223);
            jobs.push(job(
                d,
                s,
                nexts[(x >> 8) as usize % 5],
                hops[(x >> 12) as usize % 4],
                lens[(x >> 16) as usize % lens.len()],
            ));
        }
    }
    for &n in &nexts {
        for &hp in &hops {
            jobs.push(job(GUA, LL, n, hp, 5));
        }
    }
    for i in 0..40 {
        x = x.wrapping_mul(1664525).wrapping_add(1013904223);
        let d = ((x as u128) << 96) | ((x as u128).wrapping_mul(0x9E37_79B9) << 32) | i;
        jobs.push(job(d, !d, (x >> 24) as u8, (x >> 8) as u8, 1 + (i as usize * 37) % 1460));
    }
    let frames = run_tx(&jobs, 0, |_| true);
    assert_eq!(frames.len(), jobs.len());
    for (j, f) in jobs.iter().zip(&frames) {
        let d = f.data();
        assert_eq!(d.len(), 40 + j.len as usize);
        assert_eq!(&d[..40], &ref_header(j.dst_ip, j.src_ip, j.next, j.hop, j.len)[..]);
        assert_eq!(&d[40..], &j.payload[..]);
        assert!(!f.err());
        assert_eq!(f.dst_mac, j.dst_mac);
        assert_eq!(f.ethertype, 0x86DD);
    }
}

#[test]
fn tx_backpressure_and_gaps() {
    let jobs: Vec<Job> = (1..6)
        .map(|i| job(PEER + i, LL, 58, 255 - i as u8, 10 * i as usize))
        .collect();
    let frames = run_tx(&jobs, 3, |t| t % 5 != 0 && t % 7 != 1);
    assert_eq!(frames.len(), jobs.len());
    for (j, f) in jobs.iter().zip(&frames) {
        let mut exp = ref_header(j.dst_ip, j.src_ip, j.next, j.hop, j.len);
        exp.extend_from_slice(&j.payload);
        assert_eq!(f.data(), exp);
        assert!(!f.err());
    }
}

#[test]
fn tx_length_mismatch_and_upstream_abort() {
    let mut short = job(PEER, GUA, 58, 64, 10);
    short.len = 20;
    let mut long = job(PEER, GUA, 58, 64, 30);
    long.len = 12;
    let mut ab = job(PEER, GUA, 17, 64, 10);
    ab.err = true;
    let mut zero = job(PEER, GUA, 17, 64, 3);
    zero.len = 0;
    let mut big = job(PEER, GUA, 17, 64, 5);
    big.len = 1461;
    let mut max = job(PEER, GUA, 17, 64, 1460);
    max.len = 1460;
    let mut one_short = job(PEER, GUA, 17, 64, 1);
    one_short.len = 1; // exactly right, one byte
    let good = job(PEER, GUA, 17, 64, 16);
    let jobs = vec![
        short,
        good.clone(),
        long,
        good.clone(),
        ab,
        good.clone(),
        zero,
        good.clone(),
        big,
        good.clone(),
        max,
        one_short,
        good.clone(),
    ];
    for gap in [0usize, 2] {
        let frames = run_tx(&jobs, gap, |_| true);
        assert_eq!(frames.len(), jobs.len());
        assert!(frames[0].err());
        assert_eq!(frames[0].bytes.len(), 50);
        assert!(frames[2].err());
        assert_eq!(frames[2].bytes.len(), 52);
        assert!(frames[4].err());
        assert_eq!(frames[4].bytes.len(), 50);
        assert!(frames[6].err());
        assert_eq!(frames[6].bytes.len(), 40);
        assert!(frames[8].err());
        assert_eq!(frames[8].bytes.len(), 40);
        assert!(!frames[10].err());
        assert_eq!(frames[10].bytes.len(), 1500);
        assert!(!frames[11].err());
        assert_eq!(frames[11].bytes.len(), 41);
        for k in [1usize, 3, 5, 7, 9, 12] {
            let mut exp = ref_header(PEER, GUA, 17, 64, 16);
            exp.extend_from_slice(&good.payload);
            assert_eq!(frames[k].data(), exp, "frame {k}");
            assert!(!frames[k].err());
        }
        // The header of an aborted packet is still the right one.
        assert_eq!(&frames[6].data()[..], &ref_header(PEER, GUA, 17, 64, 0)[..]);
        for f in &frames {
            for b in &f.bytes[..f.bytes.len() - 1] {
                assert!(!b.err.to_bool() && !b.last.to_bool());
            }
        }
    }
}

// ------------------------------------------------------------ ipv6_rx

/// A frame offered to ipv6_rx: the Ethernet meta and the payload of the frame.
#[derive(Clone)]
struct RxFr {
    dst_mac: u64,
    src_mac: u64,
    bytes: Vec<u8>,
    err: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct M {
    src: u128,
    dst: u128,
    next: u8,
    len: u16,
    hop: u8,
    eth_src: u64,
    multicast: bool,
}

#[derive(Default, Debug)]
struct RxOut {
    /// Channel 0 = icmp, 1 = udp: metas and the byte streams.
    meta: [Vec<M>; 2],
    bytes: [Vec<FrameByte>; 2],
}

fn grab(m: Ip6Meta) -> M {
    M {
        src: m.src.raw(),
        dst: m.dst.raw(),
        next: m.next.raw() as u8,
        len: m.len.raw() as u16,
        hop: m.hop.raw() as u8,
        eth_src: m.eth_src.raw() as u64,
        multicast: m.multicast.to_bool(),
    }
}

fn run_rx(
    frames: &[RxFr],
    gap: usize,
    gua_valid: bool,
    take: impl Fn(usize, usize) -> bool,
) -> RxOut {
    let (ll_o, ll) = signal::<U<128>, DefaultClock>();
    let (gua_o, gua) = signal::<U<128>, DefaultClock>();
    let (gv_o, gv) = signal::<Bit, DefaultClock>();
    let (sn_o, sn) = signal::<U<24>, DefaultClock>();
    ll_o.set(U::<128>::from(LL));
    gua_o.set(U::<128>::from(GUA));
    gv_o.set(Bit::from_bool(gua_valid));
    sn_o.set(U::<24>::from(SN_LOW));
    let (m_t, m) = chan::<EthMeta, DefaultClock>();
    let (p_t, p) = chan::<FrameByte, DefaultClock>();
    let (im_t, im) = chan::<Ip6Meta, DefaultClock>();
    let (i_t, i) = chan::<FrameByte, DefaultClock>();
    let (um_t, um) = chan::<Ip6Meta, DefaultClock>();
    let (u_t, u) = chan::<FrameByte, DefaultClock>();
    let mut dut = Ipv6Rx::default();
    let mut sim = Running::new(dut.run((m, p, ll, gua, gv, sn), (im_t, i_t, um_t, u_t)));
    let mut out = RxOut::default();
    let (mut fi, mut bi) = (0usize, 0usize);
    let mut meta_sent = false;
    let total: usize = frames.iter().map(|f| f.bytes.len() + gap + 4).sum();
    for t in 0..(total * (gap + 2) * 2 + 400) {
        if fi < frames.len() {
            let f = &frames[fi];
            // eth_rx sends the meta strictly before the first payload byte.
            if !meta_sent {
                if m_t.ready().to_bool() {
                    m_t.send(EthMeta {
                        dst: U::<48>::from(f.dst_mac),
                        src: U::<48>::from(f.src_mac),
                        ethertype: U::<16>::from(0x86DDu16),
                        multicast: Bit::from_bool(f.dst_mac & 0x0100_0000_0000 != 0),
                    });
                    meta_sent = true;
                }
            } else if t % (gap + 1) == 0 && p_t.ready().to_bool() {
                let last = bi + 1 == f.bytes.len();
                p_t.send(FrameByte::new(f.bytes[bi], last, last && f.err));
                if last {
                    fi += 1;
                    bi = 0;
                    meta_sent = false;
                } else {
                    bi += 1;
                }
            }
        }
        for (c, (mm, pp)) in [(&im, &i), (&um, &u)].into_iter().enumerate() {
            if take(t, c) {
                if let Some(x) = mm.recv() {
                    out.meta[c].push(grab(x));
                }
                if let Some(b) = pp.recv() {
                    out.bytes[c].push(b);
                }
            }
        }
        sim.cycle();
    }
    out
}

#[derive(Clone)]
struct Pk {
    ver: u8,
    next: u8,
    hop: u8,
    src: u128,
    dst: u128,
    /// None: the correct payload length.
    plen: Option<u16>,
    data: Vec<u8>,
    dst_mac: u64,
    src_mac: u64,
}

impl Pk {
    fn new(next: u8, n: usize) -> Pk {
        Pk {
            ver: 6,
            next,
            hop: 64,
            src: PEER,
            dst: LL,
            plen: None,
            data: (0..n).map(|i| (i * 5 + 1) as u8).collect(),
            dst_mac: MAC,
            src_mac: PEER_MAC,
        }
    }
    fn to(mut self, dst: u128, mac: u64) -> Pk {
        self.dst = dst;
        self.dst_mac = mac;
        self
    }
    fn build(&self) -> Vec<u8> {
        let plen = self.plen.unwrap_or(self.data.len() as u16);
        // Non-zero traffic class and flow label: they must be ignored.
        let first = ((self.ver as u32) << 28) | 0x0ab1_2345;
        let mut h = first.to_be_bytes().to_vec();
        h.extend_from_slice(&[(plen >> 8) as u8, plen as u8, self.next, self.hop]);
        h.extend_from_slice(&self.src.to_be_bytes());
        h.extend_from_slice(&self.dst.to_be_bytes());
        h.extend_from_slice(&self.data);
        h
    }
    /// Ethernet payload: padded to 46 bytes like a minimum frame.
    fn frame(&self) -> RxFr {
        let mut b = self.build();
        while b.len() < 46 {
            b.push(0);
        }
        RxFr {
            dst_mac: self.dst_mac,
            src_mac: self.src_mac,
            bytes: b,
            err: false,
        }
    }
}

fn clean(o: &RxOut, c: usize) -> Vec<Vec<u8>> {
    let mut v = vec![];
    let mut cur = vec![];
    for b in &o.bytes[c] {
        assert!(!b.err.to_bool() || b.last.to_bool());
        cur.push(b.data.raw() as u8);
        if b.last.to_bool() {
            assert!(!b.err.to_bool(), "unexpected err");
            v.push(std::mem::take(&mut cur));
        }
    }
    assert!(cur.is_empty());
    v
}

/// Independent statement of rules G1-G8 (no use of the RTL's `sn_low`: the
/// multicast MAC is derived from the destination as the model does).
/// Returns (channel, payload) or None. Packets whose payload length exceeds
/// the frame are the caller's business.
fn ref_rx(f: &RxFr, gua_valid: bool) -> Option<(usize, Vec<u8>)> {
    let b = &f.bytes;
    if f.src_mac & 0x0100_0000_0000 != 0 || b.len() < 40 {
        return None;
    }
    if b[0] >> 4 != 6 {
        return None;
    }
    let plen = ((b[4] as usize) << 8) | b[5] as usize;
    if 40 + plen > b.len() {
        return None;
    }
    let next = b[6];
    if next != 17 && next != 58 {
        return None;
    }
    if b[7] == 0 || b[8] == 0xFF {
        return None;
    }
    let dst = u128_at(b, 24);
    let ours = dst == LL || (gua_valid && dst == GUA);
    let is_sn = dst == sn_of(SN_LOW);
    let dmac_ours_uni = f.dst_mac == MAC;
    let ok_dst = if ours {
        dmac_ours_uni
    } else if is_sn {
        let d = dst.to_be_bytes();
        f.dst_mac == (0x3333u64 << 32 | u32::from_be_bytes([d[12], d[13], d[14], d[15]]) as u64)
    } else {
        false
    };
    if !ok_dst {
        return None;
    }
    if next == 17 && !ours {
        return None;
    }
    if plen == 0 {
        return None;
    }
    Some((if next == 58 { 0 } else { 1 }, b[40..40 + plen].to_vec()))
}

#[test]
fn rx_table() {
    // (name, packet, expected channel, gua_valid)
    let mut cases: Vec<(&str, Pk, Option<usize>, bool)> = vec![];
    let icmp = |n| Pk::new(58, n);
    let udp = |n| Pk::new(17, n);
    // Destination classes.
    cases.push(("icmp to ll", icmp(12), Some(0), true));
    cases.push(("udp to ll", udp(30), Some(1), true));
    cases.push(("icmp to gua", icmp(12).to(GUA, MAC), Some(0), true));
    cases.push(("udp to gua", udp(30).to(GUA, MAC), Some(1), true));
    cases.push(("icmp to gua, gua_valid 0", icmp(12).to(GUA, MAC), None, false));
    cases.push(("udp to gua, gua_valid 0", udp(12).to(GUA, MAC), None, false));
    cases.push(("icmp to ll, gua_valid 0", icmp(12), Some(0), false));
    cases.push(("icmp to sn", icmp(12).to(sn_of(SN_LOW), sn_mac(SN_LOW)), Some(0), true));
    cases.push(("icmp to sn, gua_valid 0", icmp(12).to(sn_of(SN_LOW), sn_mac(SN_LOW)), Some(0), false));
    cases.push(("udp to sn (G8)", udp(12).to(sn_of(SN_LOW), sn_mac(SN_LOW)), None, true));
    cases.push(("icmp to sn on unicast mac (G6)", icmp(12).to(sn_of(SN_LOW), MAC), None, true));
    cases.push(("icmp to sn, wrong mac", icmp(12).to(sn_of(SN_LOW), sn_mac(2)), None, true));
    cases.push(("icmp to sn, all-nodes mac", icmp(12).to(sn_of(SN_LOW), 0x3333_0000_0001), None, true));
    cases.push(("icmp to sn of other low", icmp(12).to(sn_of(2), sn_mac(2)), None, true));
    cases.push(("icmp to sn of other low, our mac", icmp(12).to(sn_of(2), sn_mac(SN_LOW)), None, true));
    cases.push(("icmp to sn, other prefix", icmp(12).to(sn_of(SN_LOW) ^ (1u128 << 112), sn_mac(SN_LOW)), None, true));
    cases.push(("icmp to ff02::1", icmp(12).to(ALL_NODES, 0x3333_0000_0001), None, true));
    cases.push(("udp to ff02::1", udp(12).to(ALL_NODES, 0x3333_0000_0001), None, true));
    cases.push(("icmp to ff02::2", icmp(12).to(ALL_NODES + 1, 0x3333_0000_0002), None, true));
    cases.push(("icmp to ff02::1 on our mac", icmp(12).to(ALL_NODES, MAC), None, true));
    cases.push(("icmp to someone else", icmp(12).to(PEER_LL, MAC), None, true));
    cases.push(("icmp to peer gua", icmp(12).to(PEER, MAC), None, true));
    cases.push(("icmp to ::1", icmp(12).to(1, MAC), None, true));
    cases.push(("icmp to ::", icmp(12).to(0, MAC), None, true));
    cases.push(("icmp to ll, multicast mac", icmp(12).to(LL, 0x3333_0000_0001), None, true));
    cases.push(("icmp to ll, broadcast mac", icmp(12).to(LL, 0xFFFF_FFFF_FFFF), None, true));
    cases.push(("icmp to ll off by one bit", icmp(12).to(LL ^ 1, MAC), None, true));
    cases.push(("icmp to ll off by top bit", icmp(12).to(LL ^ (1 << 127), MAC), None, true));
    // Sources.
    for (n, s, exp) in [
        ("src ff02::1", ALL_NODES, None),
        ("src ff00::", 0xff00u128 << 112, None),
        ("src ffff..", u128::MAX, None),
        ("src fe80::2", PEER_LL, Some(0)),
        ("src ::", 0u128, Some(0)),
        ("src ::1", 1, Some(0)),
        ("src fd..", 0xfd00u128 << 112, Some(0)),
        ("src just below multicast", (0xfeffu128 << 112) | 5, Some(0)),
    ] {
        let mut p = icmp(12);
        p.src = s;
        cases.push((n, p, exp, true));
    }
    let mut p = icmp(24).to(sn_of(SN_LOW), sn_mac(SN_LOW));
    p.src = 0;
    cases.push(("DAD: src :: to sn", p, Some(0), true));
    let mut p = udp(24);
    p.src = 0;
    cases.push(("udp from :: to ll (passes this layer)", p, Some(1), true));
    // Hop limit.
    for (h, exp) in [(0u8, None), (1, Some(0)), (2, Some(0)), (64, Some(0)), (254, Some(0)), (255, Some(0))] {
        let mut p = icmp(12);
        p.hop = h;
        cases.push(("hop limit", p, exp, true));
    }
    // Version.
    for v in [0u8, 1, 4, 5, 7, 15] {
        let mut p = icmp(12);
        p.ver = v;
        cases.push(("version != 6", p, None, true));
    }
    // Next header.
    for n in [0u8, 1, 6, 43, 44, 50, 51, 59, 60, 135, 255] {
        cases.push(("next header dropped", Pk::new(n, 12), None, true));
    }
    // Payload length against the frame.
    cases.push(("padding stripped", icmp(4), Some(0), true));
    cases.push(("one byte payload", udp(1), Some(1), true));
    let mut p = icmp(8);
    p.plen = Some(0);
    cases.push(("plen 0", p, None, true));
    let mut p = udp(8);
    p.plen = Some(0);
    cases.push(("plen 0 udp", p, None, true));
    let mut p = icmp(8);
    p.plen = Some(3);
    cases.push(("plen shorter than data", p, Some(0), true));
    cases.push(("full size 1440", udp(1440), Some(1), true));
    cases.push(("full size 1440 icmp", icmp(1440), Some(0), true));
    cases.push(("icmp 46 (no padding)", icmp(6), Some(0), true));
    cases.push(("icmp 47", icmp(7), Some(0), true));
    // Ethernet source with the group bit.
    let mut p = icmp(12);
    p.src_mac = 0x0300_0000_0002;
    cases.push(("group-addressed eth src", p, None, true));

    for gap in [0usize, 2] {
        for (name, p, exp, gv) in &cases {
            // Each case alone, preceded and followed by a good packet so that
            // state leaking between frames would show.
            let g = Pk::new(17, 9);
            let f = p.frame();
            let frames = vec![g.frame(), f.clone(), g.frame()];
            let o = run_rx(&frames, gap, *gv, |_, _| true);
            let mut want = [vec![], vec![g.data.clone(), g.data.clone()]];
            let mut metas = [0usize, 2];
            if let Some(c) = exp {
                let plen = p.plen.unwrap_or(p.data.len() as u16) as usize;
                let d = p.data[..plen].to_vec();
                if *c == 0 {
                    want[0].push(d);
                    metas = [1, 2];
                } else {
                    want[1].insert(1, d);
                    metas = [0, 3];
                }
            }
            assert_eq!(clean(&o, 0), want[0], "{name} (gap {gap}): icmp");
            assert_eq!(clean(&o, 1), want[1], "{name} (gap {gap}): udp");
            assert_eq!(o.meta[0].len(), metas[0], "{name}: icmp metas");
            assert_eq!(o.meta[1].len(), metas[1], "{name}: udp metas");
            // The reference agrees with the table.
            assert_eq!(ref_rx(&f, *gv).map(|r| r.0), *exp, "{name}: ref_rx");
            if let Some(c) = exp {
                let m = o.meta[*c][if *c == 0 { 0 } else { 1 }];
                let plen = p.plen.unwrap_or(p.data.len() as u16);
                assert_eq!(m.next, p.next, "{name}: next");
                assert_eq!(m.len, plen, "{name}: meta len");
                assert_eq!(m.src, p.src, "{name}: src");
                assert_eq!(m.dst, p.dst, "{name}: dst");
                assert_eq!(m.hop, p.hop, "{name}: hop");
                assert_eq!(m.eth_src, p.src_mac, "{name}: eth_src");
                assert_eq!(m.multicast, p.dst == sn_of(SN_LOW), "{name}: multicast");
            }
        }
    }
}

#[test]
fn rx_truncated_and_err() {
    // Payload length longer than the frame: forwarded then aborted with err.
    let mut p = Pk::new(58, 4);
    p.plen = Some(60);
    let o = run_rx(&[p.frame()], 0, true, |_, _| true);
    // frame = 46 bytes: 6 payload bytes arrive (4 data + 2 padding), last with err.
    assert_eq!(o.meta[0].len(), 1);
    assert_eq!(o.meta[0][0].len, 60);
    assert_eq!(o.bytes[0].len(), 6);
    let l = o.bytes[0].last().unwrap();
    assert!(l.last.to_bool() && l.err.to_bool());
    assert!(o.bytes[0][..5]
        .iter()
        .all(|b| !b.last.to_bool() && !b.err.to_bool()));
    // Longer by one byte only.
    let mut p = Pk::new(17, 20);
    p.plen = Some(21);
    let o = run_rx(&[p.frame()], 0, true, |_, _| true);
    assert_eq!(o.bytes[1].len(), 20);
    assert!(o.bytes[1][19].last.to_bool() && o.bytes[1][19].err.to_bool());
    // Frame ending inside or right after the header: nothing.
    let b = Pk::new(58, 8).build();
    for n in [1usize, 5, 7, 8, 23, 24, 39, 40] {
        let f = RxFr {
            dst_mac: MAC,
            src_mac: PEER_MAC,
            bytes: b[..n].to_vec(),
            err: false,
        };
        let o = run_rx(&[f], 0, true, |_, _| true);
        assert!(o.meta[0].is_empty() && o.bytes[0].is_empty(), "len {n}");
        // And the unit recovers.
        let g = Pk::new(58, 5).frame();
        let f2 = RxFr {
            dst_mac: MAC,
            src_mac: PEER_MAC,
            bytes: b[..n].to_vec(),
            err: false,
        };
        let o = run_rx(&[f2, g], 0, true, |_, _| true);
        assert_eq!(clean(&o, 0), vec![Pk::new(58, 5).data], "recover after {n}");
    }
    // err on the input last: copied to the output last, with and without
    // padding after the packet.
    let mut f = Pk::new(17, 8).frame();
    f.err = true;
    let o = run_rx(&[f], 0, true, |_, _| true);
    assert_eq!(o.bytes[1].len(), 8);
    assert!(o.bytes[1][7].last.to_bool() && o.bytes[1][7].err.to_bool());
    assert!(o.bytes[1][..7]
        .iter()
        .all(|b| !b.last.to_bool() && !b.err.to_bool()));
    let mut f = Pk::new(17, 26).frame(); // no padding
    f.err = true;
    let o = run_rx(&[f], 0, true, |_, _| true);
    assert_eq!(o.bytes[1].len(), 26);
    assert!(o.bytes[1][25].last.to_bool() && o.bytes[1][25].err.to_bool());
    // err on a frame with err cut inside the packet (plen too long) stays one err.
    let mut p = Pk::new(17, 20);
    p.plen = Some(40);
    let mut f = p.frame();
    f.err = true;
    let o = run_rx(&[f], 0, true, |_, _| true);
    assert_eq!(o.bytes[1].len(), 26);
    assert!(o.bytes[1][25].last.to_bool() && o.bytes[1][25].err.to_bool());
    // A good frame after an err one is clean.
    let mut f = Pk::new(17, 8).frame();
    f.err = true;
    let g = Pk::new(17, 9);
    let o = run_rx(&[f, g.frame()], 0, true, |_, _| true);
    assert_eq!(o.bytes[1].len(), 8 + 9);
    assert!(!o.bytes[1][16].err.to_bool());
    // err on a dropped frame: nothing.
    let mut p = Pk::new(58, 8);
    p.hop = 0;
    let mut f = p.frame();
    f.err = true;
    let o = run_rx(&[f], 0, true, |_, _| true);
    assert!(o.bytes[0].is_empty() && o.meta[0].is_empty());
}

#[test]
fn rx_backpressure_and_meta_before_payload() {
    let frames: Vec<RxFr> = (0..6)
        .map(|i| Pk::new(if i % 2 == 0 { 58 } else { 17 }, 5 + 7 * i).frame())
        .collect();
    let o = run_rx(&frames, 0, true, |t, c| t % 3 == c || t % 11 == 0);
    assert_eq!(clean(&o, 0).len(), 3);
    assert_eq!(clean(&o, 1).len(), 3);
    for (k, d) in clean(&o, 0).iter().enumerate() {
        assert_eq!(d, &Pk::new(58, 5 + 14 * k).data);
    }
    assert_eq!(o.meta[0].len(), 3);
    assert_eq!(o.meta[1].len(), 3);
    // Same with gaps on the input and a very slow sink.
    let o = run_rx(&frames, 3, true, |t, _| t % 7 == 0);
    assert_eq!(clean(&o, 0).len(), 3);
    assert_eq!(clean(&o, 1).len(), 3);
}

#[test]
fn rx_back_to_back_differential() {
    // Pseudo-random packets against the reference, in one long run.
    let macs = [MAC, sn_mac(1), sn_mac(2), 0x3333_0000_0001, 0x0200_0000_0003, 0xFFFF_FFFF_FFFF];
    let dsts = [LL, GUA, sn_of(1), sn_of(2), ALL_NODES, ALL_NODES + 1, PEER, PEER_LL, 0];
    let srcs = [PEER, 0, ALL_NODES, PEER_LL, u128::MAX];
    let nexts = [58u8, 58, 17, 17, 6, 0, 43, 44, 60];
    let hops = [0u8, 1, 64, 255];
    let mut x = 0xCAFE_F00Du32;
    let mut rnd = move || {
        x = x.wrapping_mul(1664525).wrapping_add(1013904223);
        (x >> 8) as usize
    };
    let mut frames = vec![];
    let mut want: Vec<Vec<(usize, Vec<u8>)>> = vec![vec![], vec![]];
    for gv in [true, false] {
        frames.clear();
        let mut exp = vec![];
        for _ in 0..400 {
            let mut p = Pk::new(nexts[rnd() % 9], rnd() % 90);
            p.dst = dsts[rnd() % 9];
            p.dst_mac = macs[rnd() % 6];
            // Make the "good" combinations frequent.
            if rnd() % 3 == 0 {
                p.dst = LL;
                p.dst_mac = MAC;
            } else if rnd() % 5 == 0 {
                p.dst = sn_of(1);
                p.dst_mac = sn_mac(1);
            }
            p.src = srcs[rnd() % 5];
            p.hop = hops[rnd() % 4];
            if rnd() % 12 == 0 {
                p.ver = 4;
            }
            if rnd() % 12 == 0 {
                p.plen = Some(0);
            }
            if rnd() % 15 == 0 {
                p.src_mac |= 0x0100_0000_0000;
            }
            let f = p.frame();
            if let Some(r) = ref_rx(&f, gv) {
                exp.push(r);
            }
            frames.push(f);
        }
        let o = run_rx(&frames, 0, gv, |_, _| true);
        let got_i = clean(&o, 0);
        let got_u = clean(&o, 1);
        let want_i: Vec<Vec<u8>> = exp.iter().filter(|r| r.0 == 0).map(|r| r.1.clone()).collect();
        let want_u: Vec<Vec<u8>> = exp.iter().filter(|r| r.0 == 1).map(|r| r.1.clone()).collect();
        assert_eq!(got_i, want_i, "gua_valid {gv}: icmp");
        assert_eq!(got_u, want_u, "gua_valid {gv}: udp");
        assert_eq!(o.meta[0].len(), want_i.len());
        assert_eq!(o.meta[1].len(), want_u.len());
        assert!(want_i.len() > 20 && want_u.len() > 20, "weak sample");
        want[0].extend(exp);
    }
}

// ------------------------------------------------------------ chain: eth_rx -> ipv6_rx

struct RxChain {
    icmp: Vec<Vec<u8>>,
    udp: Vec<Vec<u8>>,
    icmp_metas: Vec<M>,
    udp_metas: Vec<M>,
}

fn rx_chain(frames: &[Vec<u8>], gap: usize, take: impl Fn(usize) -> bool) -> RxChain {
    let (mac_o, mac) = signal::<U<48>, DefaultClock>();
    let (sn_o, sn1) = signal::<U<24>, DefaultClock>();
    let sn2 = sn1.clone();
    let (ll_o, ll) = signal::<U<128>, DefaultClock>();
    let (gua_o, gua) = signal::<U<128>, DefaultClock>();
    let (gv_o, gv) = signal::<Bit, DefaultClock>();
    mac_o.set(U::<48>::from(MAC));
    sn_o.set(U::<24>::from(SN_LOW));
    ll_o.set(U::<128>::from(LL));
    gua_o.set(U::<128>::from(GUA));
    gv_o.set(Bit::from_bool(true));
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
    let (mut d_eth, mut d_rx) = (EthRx::default(), Ipv6Rx::default());
    let mut sim = Running::new(join2(
        d_eth.run((in_r, mac, sn1), (am_t, ap_t, v4m_t, v4p_t, v6m_t, v6p_t)),
        d_rx.run((v6m, v6p, ll, gua, gv, sn2), (im_t, i_t, um_t, u_t)),
    ));
    let mut q: Vec<Option<FrameByte>> = vec![];
    for f in frames {
        for (k, b) in f.iter().enumerate() {
            q.push(Some(FrameByte::new(*b, k + 1 == f.len(), false)));
            for _ in 0..gap {
                q.push(None);
            }
        }
    }
    let mut out = RxChain {
        icmp: vec![],
        udp: vec![],
        icmp_metas: vec![],
        udp_metas: vec![],
    };
    let (mut io, mut uo) = (vec![], vec![]);
    let mut next = 0;
    for t in 0..(q.len() * 6 + 2000) {
        if next < q.len() {
            match q[next] {
                None => next += 1,
                Some(b) => {
                    if in_t.ready().to_bool() {
                        in_t.send(b);
                        next += 1;
                    }
                }
            }
        }
        let _ = (am.recv(), ap.recv(), v4m.recv(), v4p.recv());
        if take(t) {
            if let Some(m) = im.recv() {
                out.icmp_metas.push(grab(m));
            }
            if let Some(m) = um.recv() {
                out.udp_metas.push(grab(m));
            }
            if let Some(b) = i.recv() {
                io.push(b.data.raw() as u8);
                if b.last.to_bool() {
                    assert!(!b.err.to_bool());
                    out.icmp.push(std::mem::take(&mut io));
                }
            }
            if let Some(b) = u.recv() {
                uo.push(b.data.raw() as u8);
                if b.last.to_bool() {
                    assert!(!b.err.to_bool());
                    out.udp.push(std::mem::take(&mut uo));
                }
            }
        }
        sim.cycle();
    }
    assert!(io.is_empty() && uo.is_empty(), "open frame");
    out
}

/// What the Ethernet filter and IPv6 rules let through for a whole frame.
fn ref_chain(f: &[u8]) -> Option<(usize, Vec<u8>)> {
    if f.len() < 15 || f[12] != 0x86 || f[13] != 0xDD {
        return None;
    }
    let dst = mac_of(f);
    if dst != MAC && dst != sn_mac(SN_LOW) && dst != 0x3333_0000_0001 {
        return None;
    }
    ref_rx(
        &RxFr {
            dst_mac: dst,
            src_mac: mac_of(&f[6..]),
            bytes: f[14..].to_vec(),
            err: false,
        },
        true,
    )
}

#[test]
fn chain_rx_vectors() {
    let mut total_icmp = 0;
    let mut total_udp = 0;
    for name in [
        "ping6.hex",
        "ndp_ns_na.hex",
        "ntp6_basic.hex",
        "neg_udp6_checksum.hex",
        "neg_ndp_hoplimit.hex",
        "neg_foreign_mac.hex",
        "neg_runt.hex",
        "ntp4_basic.hex",
        "ping4.hex",
        "arp_request_reply.hex",
        "ntp_unsynchronised.hex",
        "ntp_version3.hex",
    ] {
        let rx: Vec<Vec<u8>> = vector(name)
            .into_iter()
            .filter(|(r, _)| *r)
            .map(|(_, f)| f)
            .collect();
        let want: Vec<(usize, Vec<u8>)> = rx.iter().filter_map(|f| ref_chain(f)).collect();
        for (gap, tk) in [(0usize, false), (1, true)] {
            let o = rx_chain(&rx, gap, |t| !tk || t % 4 != 0);
            let wi: Vec<Vec<u8>> = want.iter().filter(|w| w.0 == 0).map(|w| w.1.clone()).collect();
            let wu: Vec<Vec<u8>> = want.iter().filter(|w| w.0 == 1).map(|w| w.1.clone()).collect();
            assert_eq!(o.icmp, wi, "{name}: icmp");
            assert_eq!(o.udp, wu, "{name}: udp");
            assert_eq!(o.icmp_metas.len(), wi.len(), "{name}");
            assert_eq!(o.udp_metas.len(), wu.len(), "{name}");
        }
        total_icmp += want.iter().filter(|w| w.0 == 0).count();
        total_udp += want.iter().filter(|w| w.0 == 1).count();
    }
    // Hard-coded sanity: the vectors do reach the outputs (ping6: 2 echo
    // requests, ndp: NS frames, ntp6: 1 UDP, neg_udp6_checksum: 2 UDP).
    assert!(total_icmp >= 4, "icmp frames: {total_icmp}");
    assert!(total_udp >= 3, "udp frames: {total_udp}");
}

#[test]
fn chain_rx_ping6_and_ntp6_exact() {
    let v = vector("ping6.hex");
    let rx: Vec<Vec<u8>> = v.iter().filter(|(r, _)| *r).map(|(_, f)| f.clone()).collect();
    let o = rx_chain(&rx, 0, |_| true);
    assert_eq!(o.icmp.len(), 2);
    assert!(o.udp.is_empty());
    for (f, got) in rx.iter().zip(&o.icmp) {
        let plen = ((f[18] as usize) << 8) | f[19] as usize;
        assert_eq!(&got[..], &f[54..54 + plen]);
        assert_eq!(got[0], 128); // echo request
    }
    assert_eq!(o.icmp_metas[0].src, PEER);
    assert_eq!(o.icmp_metas[0].dst, GUA);
    assert_eq!(o.icmp_metas[0].next, 58);
    assert_eq!(o.icmp_metas[0].hop, 64);
    assert_eq!(o.icmp_metas[0].eth_src, PEER_MAC);
    assert!(!o.icmp_metas[0].multicast);
    let v = vector("ntp6_basic.hex");
    let rx: Vec<Vec<u8>> = v.iter().filter(|(r, _)| *r).map(|(_, f)| f.clone()).collect();
    let o = rx_chain(&rx, 0, |_| true);
    assert_eq!(o.udp.len(), 1);
    assert_eq!(o.udp[0].len(), 56); // UDP header 8 + NTP 48
    assert_eq!(o.udp_metas[0].next, 17);
    assert_eq!(o.udp_metas[0].len, 56);
    // NDP: the NS to the solicited-node group arrives with multicast = 1.
    let v = vector("ndp_ns_na.hex");
    let rx: Vec<Vec<u8>> = v.iter().filter(|(r, _)| *r).map(|(_, f)| f.clone()).collect();
    let o = rx_chain(&rx, 0, |_| true);
    assert_eq!(o.icmp.len(), rx.len());
    assert!(o.icmp_metas[0].multicast && o.icmp_metas[0].hop == 255);
    assert!(!o.icmp_metas[1].multicast);
    for g in &o.icmp {
        assert_eq!(g[0], 135); // neighbor solicitation
    }
}

// ------------------------------------------------------------ chain: ipv6_tx -> tx_arb source 3

#[test]
fn chain_tx_arb_source3() {
    let jobs = vector_jobs(&["ping6.hex", "ntp6_basic.hex", "ndp_ns_na.hex"]);
    let mut all: Vec<Job> = jobs.iter().map(|(j, _)| j.clone()).collect();
    // An aborted and a bad-length packet in the middle: the arbiter must still
    // see complete frames.
    let mut bad = job(PEER, GUA, 17, 64, 4);
    bad.len = 0;
    let mut ab = job(PEER, GUA, 17, 64, 9);
    ab.err = true;
    all.insert(1, bad);
    all.insert(3, ab);
    let (rq_t, rq) = chan::<Ip6TxReq, DefaultClock>();
    let (op_t, op) = chan::<FrameByte, DefaultClock>();
    let (h3_t, h3) = chan::<EthHdr, DefaultClock>();
    let (p3_t, p3) = chan::<FrameByte, DefaultClock>();
    let (h0_t, h0) = chan::<EthHdr, DefaultClock>();
    let (p0_t, p0) = chan::<FrameByte, DefaultClock>();
    let (h1_t, h1) = chan::<EthHdr, DefaultClock>();
    let (p1_t, p1) = chan::<FrameByte, DefaultClock>();
    let (h2_t, h2) = chan::<EthHdr, DefaultClock>();
    let (p2_t, p2) = chan::<FrameByte, DefaultClock>();
    let (oh_t, oh) = chan::<EthHdr, DefaultClock>();
    let (opl_t, opl) = chan::<FrameByte, DefaultClock>();
    let (mut d_tx, mut d_arb) = (Ipv6Tx::default(), TxArb::default());
    let mut sim = Running::new(join2(
        d_tx.run((rq, op), (h3_t, p3_t)),
        d_arb.run((h0, p0, h1, p1, h2, p2, h3, p3), (oh_t, opl_t)),
    ));
    let _keep = (&h0_t, &p0_t, &h1_t, &p1_t, &h2_t, &p2_t);
    let mut nreq = 0;
    let (mut pf, mut pi) = (0, 0);
    let mut hdrs: Vec<(u64, u16)> = vec![];
    let mut frames: Vec<(u64, Vec<FrameByte>)> = vec![];
    let mut open = vec![];
    for t in 0..6000 {
        if nreq < all.len() && rq_t.ready().to_bool() {
            let j = &all[nreq];
            rq_t.send(Ip6TxReq {
                dst_mac: U::<48>::from(j.dst_mac),
                dst_ip: U::<128>::from(j.dst_ip),
                src_ip: U::<128>::from(j.src_ip),
                next: U::<8>::from(j.next),
                len: U::<16>::from(j.len),
                hop: U::<8>::from(j.hop),
            });
            nreq += 1;
        }
        if pf < all.len() && t % 2 == 0 && op_t.ready().to_bool() {
            let j = &all[pf];
            let last = pi + 1 == j.payload.len();
            op_t.send(FrameByte::new(j.payload[pi], last, last && j.err));
            if last {
                pf += 1;
                pi = 0;
            } else {
                pi += 1;
            }
        }
        if t % 5 != 3 {
            if let Some(h) = oh.recv() {
                hdrs.push((h.dst.raw() as u64, h.ethertype.raw() as u16));
            }
            if let Some(b) = opl.recv() {
                assert!(hdrs.len() > frames.len(), "payload before hdr");
                open.push(b);
                if b.last.to_bool() {
                    frames.push((hdrs[frames.len()].0, std::mem::take(&mut open)));
                    assert_eq!(hdrs[frames.len() - 1].1, 0x86DD);
                }
            }
        }
        sim.cycle();
    }
    assert!(open.is_empty());
    assert_eq!(hdrs.len(), frames.len());
    assert_eq!(frames.len(), all.len());
    let mut k = 0;
    for (idx, j) in all.iter().enumerate() {
        let (dm, f) = &frames[idx];
        assert_eq!(*dm, j.dst_mac);
        let d: Vec<u8> = f.iter().map(|b| b.data.raw() as u8).collect();
        if idx == 1 {
            assert!(f.last().unwrap().err.to_bool());
            assert_eq!(f.len(), 40);
        } else if idx == 3 {
            assert!(f.last().unwrap().err.to_bool());
        } else {
            assert!(!f.last().unwrap().err.to_bool());
            assert_eq!(d, jobs[k].1, "vector packet {k}");
            k += 1;
        }
    }
}

// ------------------------------------------------------------ Verilog lowering

#[test]
fn lowers_to_verilog() {
    let v = Ipv6Rx::verilog("ipv6_rx");
    assert!(v.contains("module ipv6_rx"), "ipv6_rx");
    let v = Ipv6Tx::verilog("ipv6_tx");
    assert!(v.contains("module ipv6_tx"), "ipv6_tx");
}
