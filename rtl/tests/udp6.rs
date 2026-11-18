// SPDX-License-Identifier: Apache-2.0
//! Tests of task T3.4: `udp6_tx` (byte exact against the Python vector and an
//! independent Rust checksum), `udp6_rx` (table of good and bad datagrams),
//! the whole chain eth_rx -> ipv6_rx -> udp6_rx -> (echo stand-in) ->
//! udp6_tx -> ipv6_tx -> tx_arb source 3 -> eth_tx against `ntp6_basic.hex`
//! and the negative IPv6/UDP vectors, and the lowering to Verilog.
use ntp_fpga::eth_rx::{EthMeta, EthRx};
use ntp_fpga::eth_tx::{EthHdr, EthTx};
use ntp_fpga::ipv6_rx::{Ip6Meta, Ipv6Rx};
use ntp_fpga::ipv6_tx::{Ip6TxReq, Ipv6Tx};
use ntp_fpga::stream::{FrameByte, Meta, Ts};
use ntp_fpga::tx_arb::TxArb;
use ntp_fpga::udp6_rx::Udp6Rx;
use ntp_fpga::udp6_tx::{Udp6Tx, Udp6TxReq};
use txhdl::comp::{chan, join2, signal, DefaultClock, Running, Unit};
use txhdl::types::{Bit, U};

const MAC: u64 = 0x0200_0000_0001;
const GUA: u128 = 0x2001_0db8_0000_0000_0000_0000_0000_0001;
const LL: u128 = 0xfe80_0000_0000_0000_0000_00ff_fe00_0001;
const PEER: u128 = 0x2001_0db8_0000_0000_0000_0000_0000_0002;
const SN_LOW: u32 = 1;

// ------------------------------------------------------------ references

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
        v.push((d == "rx", from_hex(h)));
    }
    v
}

/// RFC 1071 sum of `b` (zero padded to even length) added to `s`, folded
/// with a loop: independent of the `csum.rs` functions.
fn sum16(mut s: u32, b: &[u8]) -> u32 {
    for c in b.chunks(2) {
        s += ((c[0] as u32) << 8) | *c.get(1).unwrap_or(&0) as u32;
        while s >> 16 != 0 {
            s = (s & 0xFFFF) + (s >> 16);
        }
    }
    s
}

/// Computed UDP checksum (complement of the sum), before the 0 -> 0xFFFF rule,
/// of a segment whose checksum field is zero (RFC 8200 section 8.1).
fn udp_ck_raw(src: u128, dst: u128, seg: &[u8]) -> u16 {
    let mut ps = vec![];
    ps.extend_from_slice(&src.to_be_bytes());
    ps.extend_from_slice(&dst.to_be_bytes());
    ps.extend_from_slice(&(seg.len() as u32).to_be_bytes());
    ps.extend_from_slice(&[0, 0, 0, 17]);
    let s = sum16(sum16(0, &ps), seg);
    !(s as u16)
}

/// UDP datagram; `ck`: None = correct checksum (0 -> 0xFFFF), Some(v) = v.
fn dgram(src: u128, dst: u128, sport: u16, dport: u16, pay: &[u8], ck: Option<u16>) -> Vec<u8> {
    let mut d = vec![];
    d.extend_from_slice(&sport.to_be_bytes());
    d.extend_from_slice(&dport.to_be_bytes());
    d.extend_from_slice(&((8 + pay.len()) as u16).to_be_bytes());
    d.extend_from_slice(&[0, 0]);
    d.extend_from_slice(pay);
    let c = match ck {
        Some(v) => v,
        None => match udp_ck_raw(src, dst, &d) {
            0 => 0xFFFF,
            c => c,
        },
    };
    d[6] = (c >> 8) as u8;
    d[7] = c as u8;
    d
}

fn pay(n: usize, seed: usize) -> Vec<u8> {
    (0..n)
        .map(|i| ((i * 29 + seed * 7 + (i >> 3) * 5) ^ (seed >> 2)) as u8)
        .collect()
}

fn split(bytes: &[FrameByte]) -> (Vec<u8>, bool) {
    (
        bytes.iter().map(|b| b.data.raw() as u8).collect(),
        bytes.last().map_or(false, |b| b.err.to_bool()),
    )
}

/// Stream contract: `last` once at the end, `err` only with `last`.
fn assert_contract(bytes: &[FrameByte]) {
    for (i, b) in bytes.iter().enumerate() {
        assert_eq!(b.last.to_bool(), i + 1 == bytes.len(), "last position");
        assert!(!b.err.to_bool() || b.last.to_bool(), "err without last");
    }
}

// ------------------------------------------------------------ udp6_tx harness

#[derive(Clone)]
struct TxJob {
    mac: u64,
    dip: u128,
    sip: u128,
    sport: u16,
    dport: u16,
    /// `len` field of the request.
    len: u16,
    payload: Vec<u8>,
    err_last: bool,
}

fn job(mac: u64, dip: u128, sport: u16, dport: u16, payload: Vec<u8>) -> TxJob {
    TxJob {
        mac,
        dip,
        sip: GUA,
        sport,
        dport,
        len: payload.len() as u16,
        payload,
        err_last: false,
    }
}

#[derive(Debug)]
struct TxOut {
    mac: u64,
    dip: u128,
    sip: u128,
    next: u8,
    hop: u8,
    len: u16,
    bytes: Vec<FrameByte>,
    /// Cycle (sink side) the ip_req / first byte / last byte were seen.
    t_req: usize,
    t_first: usize,
    t_last: usize,
}

/// Cycle (driver side) at which the last payload byte of each job was sent.
fn tx_run(jobs: &[TxJob], gap: usize, take: impl Fn(usize) -> bool) -> (Vec<TxOut>, Vec<usize>) {
    let (rq_t, rq) = chan::<Udp6TxReq, DefaultClock>();
    let (py_t, py) = chan::<FrameByte, DefaultClock>();
    let (oq_t, oq) = chan::<Ip6TxReq, DefaultClock>();
    let (op_t, op) = chan::<FrameByte, DefaultClock>();
    let mut d = Udp6Tx::default();
    let mut sim = Running::new(d.run((rq, py), (oq_t, op_t)));
    // Event queue: request first, then the payload bytes.
    enum Ev {
        Req(usize),
        Byte(FrameByte, bool),
        Gap,
    }
    let mut q = vec![];
    for (i, j) in jobs.iter().enumerate() {
        q.push(Ev::Req(i));
        let n = j.payload.len();
        for (k, &b) in j.payload.iter().enumerate() {
            let last = k + 1 == n;
            q.push(Ev::Byte(FrameByte::new(b, last, last && j.err_last), last));
            for _ in 0..gap {
                q.push(Ev::Gap);
            }
        }
    }
    let mut next = 0;
    let mut job_i = 0;
    let mut last_sent = vec![];
    let mut out: Vec<TxOut> = vec![];
    let mut reqs: Vec<(u64, u128, u128, u8, u8, u16, usize)> = vec![];
    let mut open: Vec<FrameByte> = vec![];
    let mut first_t = 0;
    let total: usize = jobs
        .iter()
        .map(|j| j.payload.len() * (gap + 1) + 40)
        .sum::<usize>()
        * 4
        + 500;
    for t in 0..total {
        if next < q.len() {
            match &q[next] {
                Ev::Gap => next += 1,
                Ev::Req(i) => {
                    let j = &jobs[*i];
                    if rq_t.ready().to_bool() {
                        rq_t.send(Udp6TxReq {
                            dst_mac: U::<48>::from(j.mac),
                            dst_ip: U::<128>::from(j.dip),
                            src_ip: U::<128>::from(j.sip),
                            sport: U::<16>::from(j.sport),
                            dport: U::<16>::from(j.dport),
                            len: U::<16>::from(j.len),
                        });
                        next += 1;
                    }
                }
                Ev::Byte(b, last) => {
                    if py_t.ready().to_bool() {
                        py_t.send(*b);
                        if *last {
                            last_sent.push(t);
                            job_i += 1;
                        }
                        next += 1;
                    }
                }
            }
        }
        if take(t) {
            if let Some(r) = oq.recv() {
                reqs.push((
                    r.dst_mac.raw() as u64,
                    r.dst_ip.raw(),
                    r.src_ip.raw(),
                    r.next.raw() as u8,
                    r.hop.raw() as u8,
                    r.len.raw() as u16,
                    t,
                ));
            }
            if let Some(b) = op.recv() {
                assert!(reqs.len() > out.len(), "ip_payload byte before its ip_req");
                if open.is_empty() {
                    first_t = t;
                }
                open.push(b);
                if b.last.to_bool() {
                    let r = reqs[out.len()];
                    out.push(TxOut {
                        mac: r.0,
                        dip: r.1,
                        sip: r.2,
                        next: r.3,
                        hop: r.4,
                        len: r.5,
                        bytes: std::mem::take(&mut open),
                        t_req: r.6,
                        t_first: first_t,
                        t_last: t,
                    });
                }
            }
        }
        sim.cycle();
    }
    let _ = job_i;
    assert!(open.is_empty(), "a datagram was left open");
    assert_eq!(reqs.len(), out.len(), "an ip_req without its datagram");
    (out, last_sent)
}

/// The datagram a clean job must produce.
fn want_seg(j: &TxJob) -> Vec<u8> {
    dgram(j.sip, j.dip, j.sport, j.dport, &j.payload, None)
}

fn check_job(o: &TxOut, j: &TxJob, what: &str) {
    assert_eq!(o.mac, j.mac, "{what}: mac");
    assert_eq!(o.dip, j.dip, "{what}: dip");
    assert_eq!(o.sip, j.sip, "{what}: sip");
    assert_eq!(o.next, 17, "{what}: next header");
    assert_eq!(o.hop, 64, "{what}: hop limit");
    assert_eq!(o.len as usize, 8 + j.payload.len(), "{what}: ip_req len");
    assert_contract(&o.bytes);
    let (d, err) = split(&o.bytes);
    assert!(!err, "{what}: clean datagram flagged err");
    assert_eq!(d, want_seg(j), "{what}: datagram bytes");
}

// ------------------------------------------------------------ udp6_tx tests

#[test]
fn tx_vector_ntp6_basic() {
    let v = vector("ntp6_basic.hex");
    let txs: Vec<&Vec<u8>> = v.iter().filter(|(r, _)| !*r).map(|(_, f)| f).collect();
    assert_eq!(txs.len(), 2);
    for tx in txs {
        let pkt = &tx[14..];
        let seg = &pkt[40..];
        let sip = u128::from_be_bytes(pkt[8..24].try_into().unwrap());
        let dip = u128::from_be_bytes(pkt[24..40].try_into().unwrap());
        let mut j = job(
            0x0200_0000_0002,
            dip,
            u16::from_be_bytes([seg[0], seg[1]]),
            u16::from_be_bytes([seg[2], seg[3]]),
            seg[8..].to_vec(),
        );
        j.sip = sip;
        assert_eq!((j.sport, j.dport, j.payload.len()), (123, 40000, 48));
        for gap in [0, 1, 3] {
            let (o, _) = tx_run(&[j.clone()], gap, |_| true);
            assert_eq!(o.len(), 1);
            check_job(&o[0], &j, "vector");
            // The ip_req carries what the IPv6 header of the vector says.
            assert_eq!(o[0].len, u16::from_be_bytes([pkt[4], pkt[5]]));
            assert_eq!((o[0].next, o[0].hop), (pkt[6], pkt[7]));
            let (d, _) = split(&o[0].bytes);
            assert_eq!(&d[..], seg, "UDP header (with checksum) and payload");
        }
    }
}

#[test]
fn tx_bitwise_all_lengths() {
    let mut jobs = vec![];
    let mut s: u64 = 0x1234_5678_9abc_def1;
    let mut rnd = move || {
        s = s
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (s >> 32) as u32
    };
    for n in 1..=128usize {
        let sport = rnd() as u16;
        let dport = rnd() as u16;
        let dip = ((rnd() as u128) << 96) | ((rnd() as u128) << 64) | ((rnd() as u128) << 32) | rnd() as u128;
        let sip = ((rnd() as u128) << 96) | ((rnd() as u128) << 64) | ((rnd() as u128) << 32) | rnd() as u128;
        let mac = 0x0200_0000_0000u64 | rnd() as u64;
        let mut j = job(mac, dip, sport, dport, pay(n, n));
        j.sip = sip;
        jobs.push(j);
    }
    // Extreme words: all ones (carry chains) and all zeros.
    let mut mk = |mac, dip, sip, sp, dp, p: Vec<u8>| {
        let mut j = job(mac, dip, sp, dp, p);
        j.sip = sip;
        jobs.push(j);
    };
    mk(1, u128::MAX, u128::MAX, 0xFFFF, 0xFFFF, vec![0xFF; 128]);
    mk(2, 0, 0, 0, 0, vec![0; 128]);
    mk(3, u128::MAX, u128::MAX, 0xFFFF, 0xFFFF, vec![0xFF; 127]);
    mk(4, 1, 1, 1, 1, vec![0]);
    mk(5, u128::MAX, 0, 0xFFFF, 0xFFFF, vec![0xFF; 1]);
    for (gap, tk) in [(0usize, 0usize), (1, 0), (0, 1)] {
        let (o, _) = tx_run(&jobs, gap, |t| tk == 0 || t % 3 != 0);
        assert_eq!(o.len(), jobs.len(), "gap {gap}");
        for (k, (r, j)) in o.iter().zip(&jobs).enumerate() {
            check_job(r, j, &format!("job {k} len {}", j.payload.len()));
        }
    }
}

#[test]
fn tx_zero_checksum_sent_as_ffff() {
    let (dip, sport, dport) = (PEER, 123u16, 40000u16);
    let mut p = vec![1u8, 2, 0, 0];
    // Choose the last word so that the sum is 0xFFFF (computed checksum 0).
    let seg0 = dgram(GUA, dip, sport, dport, &p, Some(0));
    let mut ps = vec![];
    ps.extend_from_slice(&GUA.to_be_bytes());
    ps.extend_from_slice(&dip.to_be_bytes());
    ps.extend_from_slice(&(seg0.len() as u32).to_be_bytes());
    ps.extend_from_slice(&[0, 0, 0, 17]);
    let s = sum16(sum16(0, &ps), &seg0);
    let w = 0xFFFFu32 - s;
    p[2] = (w >> 8) as u8;
    p[3] = w as u8;
    let seg = dgram(GUA, dip, sport, dport, &p, Some(0));
    assert_eq!(udp_ck_raw(GUA, dip, &seg), 0, "crafted: computed checksum 0");
    let j = job(0x0200_0000_0002, dip, sport, dport, p);
    let (o, _) = tx_run(&[j.clone()], 0, |_| true);
    let (d, err) = split(&o[0].bytes);
    assert!(!err);
    assert_eq!(&d[6..8], &[0xFF, 0xFF], "0x0000 is sent as 0xFFFF");
    assert_eq!(d, want_seg(&j));
}

#[test]
fn tx_overlong_and_bad_lengths_abort() {
    let good = job(5, PEER, 10, 20, pay(48, 1));
    // Longer than 128 (the request says so too, and also lies about it).
    let mut over = job(6, PEER, 10, 20, pay(129, 2));
    let mut over2 = job(7, PEER, 10, 20, pay(300, 3));
    over2.len = 128;
    over.len = 129;
    // Short or long compared to `len`.
    let mut short = job(8, PEER, 10, 20, pay(10, 4));
    short.len = 12;
    let mut long = job(9, PEER, 10, 20, pay(12, 5));
    long.len = 10;
    let mut zero = job(10, PEER, 10, 20, pay(3, 6));
    zero.len = 0;
    // Exactly 128 with err on the input last.
    let mut e = job(11, PEER, 10, 20, pay(128, 7));
    e.err_last = true;
    let mut e1 = job(12, PEER, 10, 20, pay(1, 8));
    e1.err_last = true;
    let jobs = vec![
        good.clone(),
        over.clone(),
        good.clone(),
        over2.clone(),
        short.clone(),
        long.clone(),
        zero.clone(),
        e.clone(),
        e1.clone(),
        good.clone(),
    ];
    for gap in [0usize, 1] {
        let (o, _) = tx_run(&jobs, gap, |_| true);
        assert_eq!(o.len(), jobs.len(), "one datagram per request");
        check_job(&o[0], &good, "good 0");
        check_job(&o[2], &good, "good 2");
        check_job(&o[9], &good, "good 9");
        // Aborted ones: complete in length, `err` on the last byte only.
        let aborted = |r: &TxOut, n: usize, what: &str| {
            assert_contract(&r.bytes);
            assert_eq!(r.next, 17);
            assert_eq!(r.len as usize, 8 + n, "{what}: ip_req len");
            assert_eq!(r.bytes.len(), 8 + n, "{what}: datagram length");
            assert!(split(&r.bytes).1, "{what}: last byte carries err");
            assert!(
                r.bytes[..r.bytes.len() - 1]
                    .iter()
                    .all(|b| !b.err.to_bool()),
                "{what}: err only on last"
            );
        };
        aborted(&o[1], 128, "129 bytes");
        aborted(&o[3], 128, "300 bytes");
        aborted(&o[4], 10, "shorter than len");
        aborted(&o[5], 12, "longer than len");
        aborted(&o[6], 3, "len 0");
        aborted(&o[7], 128, "err on input last");
        aborted(&o[8], 1, "err on input last, 1 byte");
        // The first 128 payload bytes are the ones kept.
        let (d, _) = split(&o[1].bytes);
        assert_eq!(&d[8..], &pay(129, 2)[..128]);
    }
}

#[test]
fn tx_latency_is_constant() {
    // Cycles between the last payload byte sent by the driver and the first
    // `ip_payload` byte seen by the sink do not depend on the length.
    let mut lat = vec![];
    for n in [1usize, 2, 3, 47, 48, 64, 127, 128] {
        let j = job(1, 2, 3, 4, pay(n, n));
        let (o, ls) = tx_run(&[j.clone()], 0, |_| true);
        check_job(&o[0], &j, "latency");
        let first = o[0].t_first - ls[0];
        let req = o[0].t_req - ls[0];
        let end = o[0].t_last - ls[0];
        lat.push((n, req, first, end));
    }
    for &(n, req, first, end) in &lat {
        // driver send -> unit recv (+1): documented t; ip_req at t+1 -> seen
        // at t+2; first byte sent t+2 -> seen t+3; last byte t+9+n -> seen
        // t+10+n.
        assert_eq!(req, 3, "ip_req latency, n={n}");
        assert_eq!(first, 4, "first byte latency, n={n}");
        assert_eq!(end, 11 + n, "last byte, n={n}");
    }
}

#[test]
fn tx_back_to_back_and_stalls() {
    let jobs: Vec<TxJob> = (0..20)
        .map(|k| {
            job(
                k as u64 + 1,
                (0x2001_0db8u128 << 96) + (k as u128) * 0x0001_0000_0000_0000_0001,
                1000 + k as u16,
                123,
                pay(1 + k as usize * 6, k as usize),
            )
        })
        .collect();
    for (gap, m) in [(0usize, 2usize), (2, 3), (0, 5), (1, 7)] {
        let (o, _) = tx_run(&jobs, gap, |t| t % m != 1 && t % 11 != 4);
        assert_eq!(o.len(), jobs.len());
        for (k, (r, j)) in o.iter().zip(&jobs).enumerate() {
            check_job(r, j, &format!("stalled job {k}"));
        }
    }
}

// ------------------------------------------------------------ udp6_rx harness

#[derive(Clone)]
struct RxIn {
    src: u128,
    dst: u128,
    /// `Ip6Meta.len`.
    len: u16,
    eth_src: u64,
    /// `Ip6Meta.rx_ts`.
    ts: u64,
    bytes: Vec<u8>,
    err_last: bool,
}

fn rx_in(src: u128, dst: u128, d: Vec<u8>) -> RxIn {
    RxIn {
        src,
        dst,
        len: d.len() as u16,
        eth_src: 0x0200_0000_0002,
        ts: 0x1122_3344_5566_7788,
        bytes: d,
        err_last: false,
    }
}

struct RxOut {
    metas: Vec<Meta>,
    /// Payloads, with the cycle of the meta and of the first byte (order).
    pays: Vec<Vec<FrameByte>>,
    meta_t: Vec<usize>,
    first_t: Vec<usize>,
}

fn rx_run(inp: &[RxIn], gap: usize, take: impl Fn(usize) -> bool) -> RxOut {
    let (m_t, m_r) = chan::<Ip6Meta, DefaultClock>();
    let (p_t, p_r) = chan::<FrameByte, DefaultClock>();
    let (nm_t, nm) = chan::<Meta, DefaultClock>();
    let (np_t, np) = chan::<FrameByte, DefaultClock>();
    let mut d = Udp6Rx::default();
    let mut sim = Running::new(d.run((m_r, p_r), (nm_t, np_t)));
    // (meta to send with the first byte of the datagram) / byte / gap.
    let mut q: Vec<(Option<usize>, Option<FrameByte>)> = vec![];
    for (i, x) in inp.iter().enumerate() {
        let n = x.bytes.len();
        for (k, &b) in x.bytes.iter().enumerate() {
            let last = k + 1 == n;
            q.push((
                if k == 0 { Some(i) } else { None },
                Some(FrameByte::new(b, last, last && x.err_last)),
            ));
            for _ in 0..gap {
                q.push((None, None));
            }
        }
    }
    let mut out = RxOut {
        metas: vec![],
        pays: vec![],
        meta_t: vec![],
        first_t: vec![],
    };
    let mut open: Vec<FrameByte> = vec![];
    let mut next = 0;
    let mut meta_sent = vec![false; inp.len()];
    let total = q.len() * 4 + 600;
    for t in 0..total {
        if next < q.len() {
            let (mi, b) = q[next];
            match b {
                None => next += 1,
                Some(b) => {
                    // The meta goes first (a cycle earlier or together).
                    let mut ok = true;
                    if let Some(i) = mi {
                        if !meta_sent[i] {
                            if m_t.ready().to_bool() {
                                let x = &inp[i];
                                m_t.send(Ip6Meta {
                                    src: U::<128>::from(x.src),
                                    dst: U::<128>::from(x.dst),
                                    next: U::<8>::from(17u8),
                                    len: U::<16>::from(x.len),
                                    hop: U::<8>::from(64u8),
                                    eth_src: U::<48>::from(x.eth_src),
                                    multicast: Bit::Zero,
                                    rx_ts: U::<64>::from(x.ts),
                                });
                                meta_sent[i] = true;
                            } else {
                                ok = false;
                            }
                        }
                    }
                    if ok && p_t.ready().to_bool() {
                        p_t.send(b);
                        next += 1;
                    }
                }
            }
        }
        if take(t) {
            if let Some(m) = nm.recv() {
                out.metas.push(m);
                out.meta_t.push(t);
            }
            if let Some(b) = np.recv() {
                assert!(
                    out.metas.len() > out.pays.len(),
                    "payload byte before its meta"
                );
                if open.is_empty() {
                    out.first_t.push(t);
                }
                open.push(b);
                if b.last.to_bool() {
                    out.pays.push(std::mem::take(&mut open));
                }
            }
        }
        sim.cycle();
    }
    assert!(open.is_empty(), "a payload was left open");
    assert_eq!(out.metas.len(), out.pays.len(), "meta without payload");
    out
}

const C_IP: u128 = PEER;
const LOCAL_IP: u128 = GUA;

fn good_in(sport: u16, dport: u16, p: &[u8]) -> RxIn {
    rx_in(C_IP, LOCAL_IP, dgram(C_IP, LOCAL_IP, sport, dport, p, None))
}

fn check_meta(m: &Meta, x: &RxIn, sport: u16, dport: u16, plen: usize) {
    assert!(m.family.to_bool(), "family 1 = IPv6");
    assert_eq!(m.src.raw(), x.src);
    assert_eq!(m.dst.raw(), x.dst);
    assert_eq!(m.sport.raw() as u16, sport);
    assert_eq!(m.dport.raw() as u16, dport);
    assert_eq!(m.len.raw() as usize, plen);
    assert_eq!(m.eth_src.raw() as u64, x.eth_src);
    assert_eq!(m.rx_ts.raw() as u64, x.ts, "rx_ts copied");
}

// ------------------------------------------------------------ udp4_rx tests

#[test]
fn rx_good_datagrams() {
    for gap in [0usize, 1, 2] {
        for n in [1usize, 2, 3, 47, 48, 49, 100, 127, 128, 129, 1452] {
            let p = pay(n, n);
            let a = good_in(40000, 123, &p);
            let b = good_in(5, 123, &pay(n, n + 1));
            let o = rx_run(&[a.clone(), b.clone()], gap, |_| true);
            assert_eq!(o.metas.len(), 2, "n={n}");
            check_meta(&o.metas[0], &a, 40000, 123, n);
            check_meta(&o.metas[1], &b, 5, 123, n);
            for (k, want) in [p.clone(), pay(n, n + 1)].iter().enumerate() {
                assert_contract(&o.pays[k]);
                let (d, err) = split(&o.pays[k]);
                assert!(!err, "n={n} k={k}");
                assert_eq!(&d, want);
                assert!(o.meta_t[k] <= o.first_t[k], "meta before the first byte");
            }
        }
    }
    // Checksum words with all ones (carries) and a computed 0 sent as 0xFFFF.
    for p in [vec![0xFFu8; 40], vec![0xFF; 41], vec![0u8; 1]] {
        let x = good_in(0xFFFF, 123, &p);
        let o = rx_run(&[x.clone()], 0, |_| true);
        assert_eq!(split(&o.pays[0]), (p, false));
    }
}

#[test]
fn rx_checksum_zero_is_dropped() {
    // Mandatory over IPv6 (RFC 8200 8.1): a zero field is dropped at byte 7,
    // before any meta or payload byte, whatever the payload.
    let ok = good_in(40000, 123, &pay(20, 3));
    for n in [1usize, 2, 20, 48, 1452] {
        let z = rx_in(
            C_IP,
            LOCAL_IP,
            dgram(C_IP, LOCAL_IP, 5, 123, &pay(n, n), Some(0)),
        );
        assert_eq!(&z.bytes[6..8], &[0, 0]);
        for gap in [0usize, 1] {
            let o = rx_run(&[ok.clone(), z.clone(), ok.clone()], gap, |_| true);
            assert_eq!(o.metas.len(), 2, "n={n}");
            assert_eq!(o.pays.len(), 2);
            for k in 0..2 {
                check_meta(&o.metas[k], &ok, 40000, 123, 20);
                assert_eq!(split(&o.pays[k]), (pay(20, 3), false));
            }
        }
    }
}

#[test]
fn rx_vector_ntp6_basic() {
    let v = vector("ntp6_basic.hex");
    for (k, (_, rx)) in v.iter().filter(|(r, _)| *r).enumerate() {
        let src = u128::from_be_bytes(rx[22..38].try_into().unwrap());
        let dst = u128::from_be_bytes(rx[38..54].try_into().unwrap());
        let seg = rx[54..54 + 56].to_vec();
        let x = rx_in(src, dst, seg.clone());
        let o = rx_run(&[x.clone()], 0, |_| true);
        assert_eq!(o.metas.len(), 1, "request {k}");
        check_meta(&o.metas[0], &x, 40000, 123, 48);
        assert_eq!(split(&o.pays[0]), (seg[8..].to_vec(), false));
    }
}

#[test]
fn rx_bad_checksum_ends_with_err() {
    let p = pay(48, 9);
    let good = dgram(C_IP, LOCAL_IP, 40000, 123, &p, None);
    // Corrupt every position (data, header checksum field; not the length or
    // ports, which drop or change the meaning).
    for at in (6..good.len()).chain(0..2) {
        for flip in [0x01u8, 0x80] {
            let mut bad = good.clone();
            bad[at] ^= flip;
            if bad[6] == 0 && bad[7] == 0 {
                continue; // became "no checksum": dropped, tested elsewhere
            }
            let o = rx_run(
                &[
                    rx_in(C_IP, LOCAL_IP, bad),
                    rx_in(C_IP, LOCAL_IP, good.clone()),
                ],
                0,
                |_| true,
            );
            assert_eq!(o.pays.len(), 2, "byte {at}");
            let (d, err) = split(&o.pays[0]);
            assert!(err, "byte {at} flip {flip:x}: bad checksum must end with err");
            assert_contract(&o.pays[0]);
            assert_eq!(d.len(), 48);
            let (d, err) = split(&o.pays[1]);
            assert!(!err, "the next datagram is clean");
            assert_eq!(d, p);
        }
    }
    // Wrong pseudo-header: any bit of any address word (the sum is done in
    // three pieces, so every byte lane is tried).
    for byte in 0..16 {
        for (which, name) in [(0, "source"), (1, "destination")] {
            let flip = 1u128 << (8 * byte + (byte % 8));
            let (s, d) = if which == 0 {
                (C_IP ^ flip, LOCAL_IP)
            } else {
                (C_IP, LOCAL_IP ^ flip)
            };
            let o = rx_run(&[rx_in(s, d, good.clone())], 0, |_| true);
            assert!(
                split(&o.pays[0]).1,
                "wrong {name} address in the pseudo-header, byte {byte}"
            );
        }
    }
    // Wrong length in the pseudo-header only: IpMeta.len differs from the
    // UDP field -> dropped at byte 5 (nothing emitted), see the drop table.
}

#[test]
fn rx_drops_other_ports_and_bad_lengths() {
    let p = pay(20, 3);
    let ok = good_in(40000, 123, &p);
    let mut cases: Vec<(&str, RxIn)> = vec![];
    cases.push(("port 9999", good_in(40000, 9999, &p)));
    cases.push(("port 122", good_in(40000, 122, &p)));
    cases.push(("port 124", good_in(40000, 124, &p)));
    cases.push(("port 0", good_in(40000, 0, &p)));
    cases.push(("sport 123 dport 40000", good_in(123, 40000, &p)));
    // UDP length field not equal to IpMeta.len.
    let d = dgram(C_IP, LOCAL_IP, 40000, 123, &p, None);
    for (name, field) in [
        ("length shorter", 27u16),
        ("length longer", 29),
        ("length 0", 0),
        ("length 7", 7),
        ("length 8", 8),
        ("length 9 vs 28", 9),
        ("length 0xFFFF", 0xFFFF),
    ] {
        let mut x = rx_in(C_IP, LOCAL_IP, d.clone());
        x.bytes[4] = (field >> 8) as u8;
        x.bytes[5] = field as u8;
        cases.push((name, x));
    }
    // Empty payload: length field 8 and IpMeta.len 8.
    cases.push((
        "empty payload",
        rx_in(C_IP, LOCAL_IP, dgram(C_IP, LOCAL_IP, 1, 123, &[], None)),
    ));
    cases.push((
        "empty payload, checksum 0",
        rx_in(C_IP, LOCAL_IP, dgram(C_IP, LOCAL_IP, 1, 123, &[], Some(0))),
    ));
    // IpMeta.len says one more / fewer than the bytes (and the field).
    let mut x = rx_in(C_IP, LOCAL_IP, d.clone());
    x.len += 1;
    cases.push(("Ip6Meta.len longer", x));
    let mut x = rx_in(C_IP, LOCAL_IP, d.clone());
    x.len -= 1;
    cases.push(("Ip6Meta.len shorter", x));
    // Truncated inside the header, at every position.
    for cut in 1..8usize {
        let mut x = rx_in(C_IP, LOCAL_IP, d[..cut].to_vec());
        x.len = d.len() as u16;
        let mut y = x.clone();
        cases.push(("truncated header", x));
        y.err_last = true;
        cases.push(("truncated header with err", y));
    }
    for gap in [0usize, 2] {
        for (name, c) in &cases {
            // The dropped datagram between two good ones: nothing of it shows
            // and the neighbours are unharmed.
            let o = rx_run(&[ok.clone(), c.clone(), ok.clone()], gap, |_| true);
            assert_eq!(o.metas.len(), 2, "{name}: dropped, nothing emitted");
            assert_eq!(o.pays.len(), 2, "{name}");
            for k in 0..2 {
                check_meta(&o.metas[k], &ok, 40000, 123, 20);
                assert_eq!(split(&o.pays[k]), (p.clone(), false), "{name}");
            }
        }
    }
    // Alone: nothing at all.
    for (name, c) in &cases {
        let o = rx_run(&[c.clone()], 0, |_| true);
        assert!(o.metas.is_empty() && o.pays.is_empty(), "{name}");
    }
}

#[test]
fn rx_strips_padding_and_flags_err() {
    let p = pay(5, 1);
    let mut x = good_in(40000, 123, &p);
    // Padding does not reach the unit from ipv4_rx; if it did, the datagram
    // ends at its length with err on the byte and the rest is swallowed.
    let mut padded = x.clone();
    padded.bytes.extend_from_slice(&[0; 20]);
    padded.len = x.len; // IpMeta.len = datagram length only
    let o = rx_run(&[padded, x.clone()], 0, |_| true);
    assert_eq!(o.pays.len(), 2);
    let (d, err) = split(&o.pays[0]);
    assert_eq!(d, p);
    assert!(err, "bytes past the length: last byte aborted");
    assert_eq!(split(&o.pays[1]), (p.clone(), false));
    // Input err on last is propagated (good checksum otherwise).
    x.err_last = true;
    let o = rx_run(&[x.clone()], 0, |_| true);
    assert_eq!(o.pays.len(), 1);
    assert_eq!(split(&o.pays[0]), (p.clone(), true));
    // Input ends early after the meta went out: last byte with err.
    let g = good_in(40000, 123, &pay(30, 2));
    for cut in [9usize, 10, 20, 37] {
        let mut x = g.clone();
        x.bytes.truncate(cut);
        let o = rx_run(&[x, g.clone()], 0, |_| true);
        assert_eq!(o.pays.len(), 2, "cut {cut}");
        let (d, err) = split(&o.pays[0]);
        assert!(err, "cut {cut}");
        assert_eq!(d.len(), cut - 8);
        assert_eq!(split(&o.pays[1]), (pay(30, 2), false));
    }
    // Input ends exactly at byte 8 (header only, truncated): nothing.
    let mut x = g.clone();
    x.bytes.truncate(8);
    let o = rx_run(&[x], 0, |_| true);
    assert!(o.metas.is_empty());
}

#[test]
fn rx_back_to_back_with_stalls() {
    let mut inp = vec![];
    let mut want = vec![];
    for k in 0..16usize {
        let p = pay(1 + k * 3, k);
        let x = good_in(1000 + k as u16, 123, &p);
        inp.push(x.clone());
        want.push((x, 1000 + k as u16, p));
        // A dropped one in between.
        inp.push(good_in(1, 9, &pay(k + 1, 0)));
    }
    for (gap, m) in [(0usize, 1usize), (0, 2), (1, 3), (0, 5)] {
        let o = rx_run(&inp, gap, |t| m == 1 || (t % m != 1 && t % 7 != 2));
        assert_eq!(o.pays.len(), want.len());
        for (k, (x, sp, p)) in want.iter().enumerate() {
            check_meta(&o.metas[k], x, *sp, 123, p.len());
            assert_eq!(split(&o.pays[k]), (p.clone(), false));
        }
    }
}

// ------------------------------------------------------------ chain

fn crc32_ref(bytes: &[u8]) -> u32 {
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

#[derive(Default)]
struct Chain {
    /// Complete frames out of eth_tx (padded, FCS included).
    frames: Vec<Vec<u8>>,
    /// NTP datagrams the stand-in saw: (meta, payload, err).
    seen: Vec<(Meta, Vec<u8>, bool)>,
}

/// Stamp fed with the `k`-th frame of [`chain`] (distinct per frame).
fn chain_stamp(k: usize) -> u64 {
    0x4E54_5000_0000_0000 | ((k as u64) << 20) | 0x123
}

/// Feed `frames` (no FCS); the stand-in answers every clean NTP payload with
/// the next entry of `replies` (payload bytes of the answer, ports swapped,
/// to the sender's MAC and address).
fn chain(
    frames: &[Vec<u8>],
    replies: &[Vec<u8>],
    gap: usize,
    take: impl Fn(usize) -> bool,
) -> Chain {
    let (mac_o, mac) = signal::<U<48>, DefaultClock>();
    let mac2 = mac.clone();
    let (ll_o, ll) = signal::<U<128>, DefaultClock>();
    let (gua_o, gua) = signal::<U<128>, DefaultClock>();
    let (gv_o, gv) = signal::<Bit, DefaultClock>();
    mac_o.set(U::<48>::from(MAC));
    ll_o.set(U::<128>::from(LL));
    gua_o.set(U::<128>::from(GUA));
    gv_o.set(Bit::One);
    let (sn_o, sn) = signal::<U<24>, DefaultClock>();
    let sn2 = sn.clone();
    sn_o.set(U::<24>::from(SN_LOW));
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
    let (nm_t, nm) = chan::<Meta, DefaultClock>();
    let (np_t, np) = chan::<FrameByte, DefaultClock>();
    let (rq_t, rq) = chan::<Udp6TxReq, DefaultClock>();
    let (py_t, py) = chan::<FrameByte, DefaultClock>();
    let (oq_t, oq) = chan::<Ip6TxReq, DefaultClock>();
    let (op_t, op) = chan::<FrameByte, DefaultClock>();
    let (h2_t, h2) = chan::<EthHdr, DefaultClock>();
    let (p2_t, p2) = chan::<FrameByte, DefaultClock>();
    let (h0_t, h0) = chan::<EthHdr, DefaultClock>();
    let (p0_t, p0) = chan::<FrameByte, DefaultClock>();
    let (h1_t, h1) = chan::<EthHdr, DefaultClock>();
    let (p1_t, p1) = chan::<FrameByte, DefaultClock>();
    let (h3_t, h3) = chan::<EthHdr, DefaultClock>();
    let (p3_t, p3) = chan::<FrameByte, DefaultClock>();
    let (oh_t, oh) = chan::<EthHdr, DefaultClock>();
    let (opl_t, opl) = chan::<FrameByte, DefaultClock>();
    let (f_t, f_r) = chan::<FrameByte, DefaultClock>();
    let (st_t, st_r) = chan::<Ts, DefaultClock>();
    let (mut d_eth, mut d_rx, mut d_urx, mut d_utx, mut d_itx, mut d_arb, mut d_etx) = (
        EthRx::default(),
        Ipv6Rx::default(),
        Udp6Rx::default(),
        Udp6Tx::default(),
        Ipv6Tx::default(),
        TxArb::default(),
        EthTx::default(),
    );
    let mut sim = Running::new(join2(
        join2(
            join2(
                d_eth.run((in_r, mac, sn, st_r), (am_t, ap_t, v4m_t, v4p_t, v6m_t, v6p_t)),
                d_rx.run((v6m, v6p, ll, gua, gv, sn2), (im_t, i_t, um_t, u_t)),
            ),
            join2(
                d_urx.run((um, u), (nm_t, np_t)),
                d_utx.run((rq, py), (oq_t, op_t)),
            ),
        ),
        join2(
            d_itx.run((oq, op), (h3_t, p3_t)),
            join2(
                d_arb.run((h0, p0, h1, p1, h2, p2, h3, p3), (oh_t, opl_t)),
                d_etx.run((oh, opl, mac2), f_t),
            ),
        ),
    ));
    let _keep = (&h0_t, &p0_t, &h1_t, &p1_t, &h2_t, &p2_t);
    let mut q: Vec<Option<FrameByte>> = vec![];
    for f in frames {
        for (k, b) in f.iter().enumerate() {
            q.push(Some(FrameByte::new(*b, k + 1 == f.len(), false)));
            for _ in 0..gap {
                q.push(None);
            }
        }
    }
    let mut out = Chain::default();
    let mut next = 0;
    let mut next_st = 0;
    let mut cur_meta: Option<Meta> = None;
    let mut cur_pay: Vec<u8> = vec![];
    let mut metas: Vec<Meta> = vec![];
    // The stand-in's pending answers: (req, bytes).
    let mut answers: std::collections::VecDeque<(Udp6TxReq, Vec<u8>)> = Default::default();
    let mut req_sent = false;
    let mut byte_i = 0;
    let mut open: Vec<u8> = vec![];
    let mut n_ans = 0;
    let total = q.len() * 6 + 6000 + replies.len() * 400;
    for t in 0..total {
        // One receive stamp per frame, in order (T1.13).
        if next_st < frames.len() && st_t.ready().to_bool() {
            st_t.send(Ts::from(chain_stamp(next_st)));
            next_st += 1;
        }
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
        // Sinks: ARP / IPv6 / ICMP outputs.
        let _ = (
            am.recv(),
            ap.recv(),
            v4m.recv(),
            v4p.recv(),
            im.recv(),
            i.recv(),
        );
        // Echo stand-in: collect the NTP datagram, then answer.
        if let Some(m) = nm.recv() {
            metas.push(m);
        }
        if let Some(b) = np.recv() {
            cur_pay.push(b.data.raw() as u8);
            if b.last.to_bool() {
                let m = if cur_meta.is_none() {
                    metas.remove(0)
                } else {
                    cur_meta.take().unwrap()
                };
                let err = b.err.to_bool();
                assert_eq!(
                    m.len.raw() as usize,
                    cur_pay.len(),
                    "meta len = payload length"
                );
                out.seen.push((m, std::mem::take(&mut cur_pay), err));
                if !err {
                    let rep = replies[n_ans].clone();
                    n_ans += 1;
                    answers.push_back((
                        Udp6TxReq {
                            dst_mac: m.eth_src,
                            dst_ip: m.src,
                            // Answer from the address that was targeted.
                            src_ip: m.dst,
                            sport: m.dport,
                            dport: m.sport,
                            len: U::<16>::from(rep.len() as u16),
                        },
                        rep,
                    ));
                }
            }
        }
        if let Some((rq0, bytes)) = answers.front() {
            if !req_sent {
                if rq_t.ready().to_bool() {
                    rq_t.send(*rq0);
                    req_sent = true;
                    byte_i = 0;
                }
            } else if py_t.ready().to_bool() {
                let last = byte_i + 1 == bytes.len();
                py_t.send(FrameByte::new(bytes[byte_i], last, false));
                byte_i += 1;
                if last {
                    answers.pop_front();
                    req_sent = false;
                }
            }
        }
        if take(t) {
            if let Some(b) = f_r.recv() {
                open.push(b.data.raw() as u8);
                assert!(!b.err.to_bool() || b.last.to_bool());
                if b.last.to_bool() {
                    out.frames.push(std::mem::take(&mut open));
                }
            }
        }
        sim.cycle();
    }
    assert!(open.is_empty(), "a frame was left open");
    out
}

fn check_frame(f: &[u8], want: &[u8], what: &str) {
    let n = f.len();
    assert!(n >= 64, "{what}: padded frame with FCS");
    let (body, fcs) = f.split_at(n - 4);
    assert_eq!(
        u32::from_le_bytes([fcs[0], fcs[1], fcs[2], fcs[3]]),
        crc32_ref(body),
        "{what}: FCS"
    );
    assert_eq!(&body[..want.len()], want, "{what}: frame bytes");
    assert!(
        body[want.len()..].iter().all(|b| *b == 0),
        "{what}: padding"
    );
}

/// Request frame and reply frame of the `k`-th exchange of `ntp6_basic.hex`.
fn ntp6() -> (Vec<Vec<u8>>, Vec<Vec<u8>>) {
    let v = vector("ntp6_basic.hex");
    let rx = v.iter().filter(|(r, _)| *r).map(|(_, f)| f.clone()).collect();
    let tx = v.iter().filter(|(r, _)| !*r).map(|(_, f)| f.clone()).collect();
    (rx, tx)
}

/// An Ethernet frame (no FCS, padded to 60) carrying an IPv6 datagram.
fn frame6(src: u128, dst: u128, seg: &[u8], plen: u16) -> Vec<u8> {
    let mut f = vec![];
    f.extend_from_slice(&MAC.to_be_bytes()[2..]);
    f.extend_from_slice(&0x0200_0000_0002u64.to_be_bytes()[2..]);
    f.extend_from_slice(&[0x86, 0xDD, 0x60, 0, 0, 0]);
    f.extend_from_slice(&plen.to_be_bytes());
    f.extend_from_slice(&[17, 64]);
    f.extend_from_slice(&src.to_be_bytes());
    f.extend_from_slice(&dst.to_be_bytes());
    f.extend_from_slice(seg);
    while f.len() < 60 {
        f.push(0);
    }
    f
}

#[test]
fn chain_ntp6_basic() {
    let (rx, tx) = ntp6();
    assert_eq!(rx.len(), 2);
    // The NTP content of each reply is the vector's own.
    let replies: Vec<Vec<u8>> = tx.iter().map(|t| t[14 + 40 + 8..].to_vec()).collect();
    for (gap, tk) in [(0usize, 0usize), (1, 0), (0, 1), (2, 1)] {
        let o = chain(&rx, &replies, gap, |t| tk == 0 || t % 3 != 0);
        assert_eq!(o.seen.len(), 2, "both NTP datagrams reach the stand-in");
        for (k, (m, p, err)) in o.seen.iter().enumerate() {
            assert!(!err);
            assert_eq!(p[..], rx[k][62..62 + 48]);
            assert_eq!((m.sport.raw(), m.dport.raw()), (40000, 123));
            assert!(m.family.to_bool());
        }
        assert_eq!(o.frames.len(), 2, "gap {gap} tk {tk}");
        // Reply 0 answers from the global address, reply 1 from the
        // link-local one: src_ip = the address that was targeted.
        check_frame(&o.frames[0], &tx[0], "ntp6_basic reply (global)");
        check_frame(&o.frames[1], &tx[1], "ntp6_basic reply (link-local)");
    }
    // Several requests back to back.
    let o = chain(
        &[rx[0].clone(), rx[1].clone(), rx[0].clone()],
        &[replies[0].clone(), replies[1].clone(), replies[0].clone()],
        0,
        |_| true,
    );
    assert_eq!(o.frames.len(), 3);
    for (f, k) in o.frames.iter().zip([0usize, 1, 0]) {
        check_frame(f, &tx[k], "ntp6_basic reply, back to back");
    }
}

#[test]
fn chain_negative_vectors() {
    let (_, tx) = ntp6();
    let reply = tx[0][14 + 48..].to_vec();
    // IPv6/UDP negatives, plus scenarios that must not reach udp6_rx at all.
    for name in [
        "neg_udp6_checksum.hex",
        "neg_ndp_hoplimit.hex",
        "neg_foreign_mac.hex",
        "neg_runt.hex",
        "neg_udp_port.hex",
    ] {
        let nv = vector(name);
        let rxs: Vec<Vec<u8>> = nv.iter().filter(|(r, _)| *r).map(|(_, f)| f.clone()).collect();
        let want: Vec<&Vec<u8>> = nv.iter().filter(|(r, _)| !*r).map(|(_, f)| f).collect();
        let o = chain(&rxs, &vec![reply.clone(); rxs.len()], 0, |_| true);
        // Only a valid IPv6 NTP request gets an answer: only the first frame
        // of neg_udp6_checksum (the other files carry no such frame, their
        // answers belong to other units, which this chain does not hold).
        let n_ans = if name == "neg_udp6_checksum.hex" { 1 } else { 0 };
        assert_eq!(o.frames.len(), n_ans, "{name}");
        if n_ans == 1 {
            assert_eq!(want.len(), 1);
            check_frame(&o.frames[0], want[0], name);
        }
        // A bad checksum is only known at the last byte: that datagram
        // reaches the stand-in, ending with err.
        let clean = o.seen.iter().filter(|s| !s.2).count();
        assert_eq!(clean, n_ans, "{name}: clean datagrams seen");
        let late = if name == "neg_udp6_checksum.hex" { 1 } else { 0 };
        assert_eq!(o.seen.len() - clean, late, "{name}: late-err datagrams");
    }
}

/// Crafted IPv6/UDP negatives between two good requests.
#[test]
fn chain_crafted_negatives() {
    let (rx, tx) = ntp6();
    let reply = tx[0][14 + 48..].to_vec();
    let good = rx[0].clone();
    let seg = good[54..54 + 56].to_vec();
    let with = |f: &dyn Fn(&mut Vec<u8>)| {
        let mut s = seg.clone();
        f(&mut s);
        frame6(PEER, GUA, &s, 56)
    };
    let mut bad: Vec<(&str, Vec<u8>)> = vec![];
    // Checksum 0.
    bad.push(("checksum 0", with(&|s| s[6..8].copy_from_slice(&[0, 0]))));
    // Zero checksum even when the "right" one would be 0xFFFF is still a
    // different datagram: only the literal 0 field is illegal.
    bad.push(("checksum flipped", with(&|s| s[6] ^= 0x10)));
    bad.push(("payload flipped", with(&|s| s[30] ^= 0x01)));
    bad.push(("port 9999", {
        let mut s = seg.clone();
        s[2..4].copy_from_slice(&9999u16.to_be_bytes());
        s[6..8].copy_from_slice(&[0, 0]);
        let c = udp_ck_raw(PEER, GUA, &s);
        s[6..8].copy_from_slice(&c.to_be_bytes());
        frame6(PEER, GUA, &s, 56)
    }));
    // UDP length shorter / longer than the IPv6 payload length.
    bad.push(("udp length 55", with(&|s| s[4..6].copy_from_slice(&55u16.to_be_bytes()))));
    bad.push(("udp length 57", with(&|s| s[4..6].copy_from_slice(&57u16.to_be_bytes()))));
    bad.push(("udp length 8", with(&|s| s[4..6].copy_from_slice(&8u16.to_be_bytes()))));
    let frames: Vec<Vec<u8>> = {
        let mut v = vec![good.clone()];
        for (_, f) in &bad {
            v.push(f.clone());
            v.push(good.clone());
        }
        v
    };
    let o = chain(&frames, &vec![reply.clone(); frames.len()], 0, |_| true);
    // The late-err datagrams reach the stand-in with err and get no answer.
    let clean = o.seen.iter().filter(|s| !s.2).count();
    assert_eq!(clean, 1 + bad.len(), "one clean datagram per good frame");
    assert_eq!(o.frames.len(), 1 + bad.len());
    for f in &o.frames {
        check_frame(f, &tx[0], "good frame between negatives");
    }
}

/// Interleaved ARP, ICMP, NTP, IPv4 and foreign-MAC frames, each with its
/// own stamp: the `Meta.rx_ts` of every NTP datagram is the stamp of the
/// frame that carried it.
#[test]
fn chain_rx_ts_follows_its_frame() {
    let first = |name: &str| {
        vector(name)
            .into_iter()
            .filter(|(r, _)| *r)
            .map(|(_, f)| f)
            .collect::<Vec<_>>()
    };
    let (rx, tx) = ntp6();
    let ntp = rx[0].clone();
    let arp = first("arp_request_reply.hex")[0].clone();
    let icmp = first("ping6.hex")[0].clone();
    let v4 = first("ntp4_basic.hex")[0].clone();
    let foreign = first("neg_foreign_mac.hex")[1].clone();
    let reply = tx[0][14 + 48..].to_vec();
    let frames = vec![
        arp.clone(),     // 0
        ntp.clone(),     // 1 NTP
        icmp.clone(),    // 2
        v4.clone(),      // 3
        foreign.clone(), // 4
        ntp.clone(),     // 5 NTP
        v4.clone(),      // 6
        arp.clone(),     // 7
        icmp.clone(),    // 8
        ntp.clone(),     // 9 NTP
    ];
    let want: Vec<u64> = [1usize, 5, 9].iter().map(|&k| chain_stamp(k)).collect();
    for (gap, tk) in [(0usize, 0usize), (1, 1)] {
        let o = chain(&frames, &vec![reply.clone(); 3], gap, |t| tk == 0 || t % 3 != 0);
        let got: Vec<u64> = o.seen.iter().map(|s| s.0.rx_ts.raw() as u64).collect();
        assert_eq!(got, want, "gap {gap}");
    }
}

// ------------------------------------------------------------ Verilog

#[test]
fn lowers_to_verilog() {
    let v = Udp6Rx::verilog("udp6_rx");
    assert!(v.contains("module udp6_rx"), "udp6_rx lowers");
    let v = Udp6Tx::verilog("udp6_tx");
    assert!(v.contains("module udp6_tx"), "udp6_tx lowers");
}
