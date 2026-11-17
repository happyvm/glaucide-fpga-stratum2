// SPDX-License-Identifier: Apache-2.0
//! Tests of tasks T2.3/T2.4/T2.7: `ntp_server` byte exact against the Python
//! vectors (IPv4, IPv6, NTPv3, unsynchronised), the policy table (modes,
//! versions, lengths, `err`), timestamp semantics, the rate limiter and its
//! KoD, back-pressure, throughput, and the chains
//! udp4_rx -> ntp_server -> (IPv4 adapter) -> udp4_tx -> ipv4_tx -> tx_arb ->
//! eth_tx (byte exact for `ntp4_basic.hex`, and the measurement behind the
//! default `tx_latency`), plus the IPv6 latency chain through `ipv6_tx`.
use ntp_fpga::clock::INCR;
use ntp_fpga::eth_tx::{EthHdr, EthTx};
use ntp_fpga::ipv4_rx::IpMeta;
use ntp_fpga::ipv4_tx::{IpTxReq, Ipv4Tx};
use ntp_fpga::ipv6_tx::{Ip6TxReq, Ipv6Tx};
use ntp_fpga::ntp_server::{
    src_hash, NtpReplyMeta, NtpServer, DEFAULT_PRECISION, KOD_RATE, TX_CYCLES_V4, TX_CYCLES_V6,
    TX_LATENCY_V4, TX_LATENCY_V6,
};
use ntp_fpga::stream::{FrameByte, Meta};
use ntp_fpga::tx_arb::TxArb;
use ntp_fpga::udp4_rx::Udp4Rx;
use ntp_fpga::udp4_tx::{Udp4Tx, UdpTxReq};
use std::collections::VecDeque;
use txhdl::comp::{chan, join2, signal, tie, DefaultClock, In, Running, Unit};
use txhdl::types::{Bit, U};

// ------------------------------------------------------------ vectors

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

fn be(b: &[u8]) -> u128 {
    b.iter().fold(0u128, |a, &x| (a << 8) | x as u128)
}

/// Configuration of `tools/golden/vectors.py` (`CONFIG`), see vectors/README.md.
const REF_ID4: u32 = 0xC000_0263;
const REF_ID6: u32 = 0xB70A_6091;
const ROOT_DELAY: u32 = 0x0000_0333;
const ROOT_DISP: u32 = 0x0000_0666;
const T_REF: u64 = 0xECC5_8CC0_0000_0000;
const T_RECV: u64 = 0xECC5_8CC0_4000_0000;
const T_XMIT: u64 = 0xECC5_8CC0_4001_4F8B;

/// A request as the UDP layer delivers it, from a vector's rx frame.
#[derive(Clone, Debug)]
struct Req {
    v6: bool,
    src: u128,
    dst: u128,
    sport: u16,
    esrc: u64,
    rx_ts: u64,
    pay: Vec<u8>,
    err: bool,
}

fn req_from_frame(f: &[u8], rx_ts: u64) -> Req {
    let et = u16::from_be_bytes([f[12], f[13]]);
    let esrc = be(&f[6..12]) as u64;
    let (v6, src, dst, udp) = if et == 0x0800 {
        (false, be(&f[26..30]), be(&f[30..34]), 34)
    } else {
        assert_eq!(et, 0x86DD);
        (true, be(&f[22..38]), be(&f[38..54]), 54)
    };
    let ulen = u16::from_be_bytes([f[udp + 4], f[udp + 5]]) as usize;
    Req {
        v6,
        src,
        dst,
        sport: u16::from_be_bytes([f[udp], f[udp + 1]]),
        esrc,
        rx_ts,
        pay: f[udp + 8..udp + ulen].to_vec(),
        err: false,
    }
}

// ------------------------------------------------------------ harness

#[derive(Clone, Debug)]
struct Cfg {
    synced: bool,
    stratum: u8,
    ref_id: u32,
    ref_id6: u32,
    ref_ts: u64,
    root_delay: u32,
    root_disp: u32,
    precision: u8,
    tx_latency: u64,
    rate_limit: u16,
    kod: bool,
}

fn vcfg() -> Cfg {
    Cfg {
        synced: true,
        stratum: 2,
        ref_id: REF_ID4,
        ref_id6: REF_ID6,
        ref_ts: T_REF,
        root_delay: ROOT_DELAY,
        root_disp: ROOT_DISP,
        precision: DEFAULT_PRECISION,
        tx_latency: T_XMIT - T_RECV,
        rate_limit: 0,
        kod: false,
    }
}

#[derive(Debug, Clone)]
struct Out {
    meta: NtpReplyMeta,
    bytes: Vec<FrameByte>,
    /// Cycles (sink side) of the meta, of byte 40 and of the last byte.
    t_meta: usize,
    t_first: usize,
    t40: usize,
    t_last: usize,
}

impl Out {
    fn data(&self) -> Vec<u8> {
        self.bytes.iter().map(|b| b.data.raw() as u8).collect()
    }
    fn t3(&self) -> u64 {
        be(&self.data()[40..48]) as u64
    }
}

struct RunOut {
    outs: Vec<Out>,
    /// Cycle at which each request (index into the input) was fully sent.
    sent_t: Vec<usize>,
}

type Wires = (
    In<Bit>,
    In<U<8>>,
    In<U<32>>,
    In<U<32>>,
    In<U<64>>,
    In<U<32>>,
    In<U<32>>,
    In<U<8>>,
    In<U<64>>,
    In<U<16>>,
    In<Bit>,
);

/// The constant wire inputs of the unit, `synced` to `kod_enable`.
fn cfg_wires(c: &Cfg) -> Wires {
    (
        tie(Bit::from_bool(c.synced)),
        tie(U::<8>::from(c.stratum)),
        tie(U::<32>::from(c.ref_id)),
        tie(U::<32>::from(c.ref_id6)),
        tie(U::<64>::from(c.ref_ts)),
        tie(U::<32>::from(c.root_delay)),
        tie(U::<32>::from(c.root_disp)),
        tie(U::<8>::from(c.precision)),
        tie(U::<64>::from(c.tx_latency)),
        tie(U::<16>::from(c.rate_limit)),
        tie(Bit::from_bool(c.kod)),
    )
}

/// Plays `reqs` (IPv4 ones on meta4/req4, IPv6 ones on meta6/req6, each
/// family in input order; `gap` idle cycles after every byte), `time(t)`
/// on the clock port, collects the outputs when `take_m(t)` / `take_b(t)`.
fn run(
    reqs: &[Req],
    cfg: &Cfg,
    gap: usize,
    time: impl Fn(usize) -> u128,
    take_m: impl Fn(usize) -> bool,
    take_b: impl Fn(usize) -> bool,
    cycles: usize,
) -> RunOut {
    let (m4_t, m4) = chan::<Meta, DefaultClock>();
    let (r4_t, r4) = chan::<FrameByte, DefaultClock>();
    let (m6_t, m6) = chan::<Meta, DefaultClock>();
    let (r6_t, r6) = chan::<FrameByte, DefaultClock>();
    let (time_o, time_i) = signal::<U<96>, DefaultClock>();
    let (sy, st, ri, ri6, rt, rd, rp, pr, tl, rl, kd) = cfg_wires(cfg);
    let (om_t, om) = chan::<NtpReplyMeta, DefaultClock>();
    let (ob_t, ob) = chan::<FrameByte, DefaultClock>();
    let mut d = NtpServer::default();
    let mut sim = Running::new(d.run(
        (
            m4, r4, m6, r6, time_i, sy, st, ri, ri6, rt, rd, rp, pr, tl, rl, kd,
        ),
        (om_t, ob_t),
    ));
    // Per family: queue of (request index, byte index) steps; byte index
    // usize::MAX = the meta, usize::MAX - 1 = a gap.
    let mut qs: [VecDeque<(usize, usize)>; 2] = [VecDeque::new(), VecDeque::new()];
    for (i, r) in reqs.iter().enumerate() {
        let q = &mut qs[r.v6 as usize];
        q.push_back((i, usize::MAX));
        for k in 0..r.pay.len() {
            q.push_back((i, k));
            for _ in 0..gap {
                q.push_back((i, usize::MAX - 1));
            }
        }
    }
    let mut sent_t = vec![0; reqs.len()];
    let mut outs: Vec<Out> = vec![];
    let mut metas: VecDeque<(NtpReplyMeta, usize)> = VecDeque::new();
    let mut open: Vec<FrameByte> = vec![];
    let mut t40 = 0;
    let mut t_first = 0;
    let mut done: VecDeque<(Vec<FrameByte>, usize, usize, usize)> = VecDeque::new();
    for t in 0..cycles {
        time_o.set(U::<96>::new(time(t) & ((1u128 << 96) - 1)));
        #[allow(clippy::needless_range_loop)]
        for f in 0..2 {
            if let Some(&(i, k)) = qs[f].front() {
                let r = &reqs[i];
                if k == usize::MAX {
                    let mt = if f == 0 { &m4_t } else { &m6_t };
                    if mt.ready().to_bool() {
                        mt.send(Meta {
                            family: Bit::from_bool(r.v6),
                            src: U::<128>::new(r.src),
                            dst: U::<128>::new(r.dst),
                            sport: U::<16>::from(r.sport),
                            dport: U::<16>::from(123u16),
                            len: U::<16>::from(r.pay.len() as u16),
                            eth_src: U::<48>::from(r.esrc),
                            rx_ts: U::<64>::from(r.rx_ts),
                        });
                        qs[f].pop_front();
                    }
                } else if k == usize::MAX - 1 {
                    qs[f].pop_front();
                } else {
                    let bt = if f == 0 { &r4_t } else { &r6_t };
                    if bt.ready().to_bool() {
                        let last = k + 1 == r.pay.len();
                        bt.send(FrameByte::new(r.pay[k], last, last && r.err));
                        if last {
                            sent_t[i] = t;
                        }
                        qs[f].pop_front();
                    }
                }
            }
        }
        if take_m(t) {
            if let Some(m) = om.recv() {
                metas.push_back((m, t));
            }
        }
        if take_b(t) {
            if let Some(b) = ob.recv() {
                if open.len() == 40 {
                    t40 = t;
                }
                open.push(b);
                if open.len() == 1 {
                    t_first = t;
                }
                if b.last.to_bool() {
                    done.push_back((std::mem::take(&mut open), t_first, t40, t));
                }
            }
        }
        // Pair metas and replies in order (separate channels: the sink may
        // read a reply before its meta when it stalls `reply_meta` only).
        while !metas.is_empty() && !done.is_empty() {
            let (m, tm) = metas.pop_front().unwrap();
            let (bytes, tf, t40, tl) = done.pop_front().unwrap();
            outs.push(Out {
                meta: m,
                bytes,
                t_meta: tm,
                t_first: tf,
                t40,
                t_last: tl,
            });
        }
        sim.cycle();
    }
    assert!(open.is_empty(), "a reply was left open");
    assert!(
        metas.is_empty() && done.is_empty(),
        "a reply_meta without its reply or back"
    );
    assert!(qs.iter().all(|q| q.is_empty()), "requests not all taken");
    RunOut { outs, sent_t }
}

fn cycles_for(reqs: &[Req], gap: usize, slow: usize) -> usize {
    reqs.iter()
        .map(|r| (r.pay.len() * (gap + 1) + 120) * slow)
        .sum::<usize>()
        + 400
}

/// Simple run: frozen clock at `T_RECV`, all outputs ready.
fn run_simple(reqs: &[Req], cfg: &Cfg) -> Vec<Out> {
    let n = cycles_for(reqs, 0, 1);
    run(
        reqs,
        cfg,
        0,
        |_| (T_RECV as u128) << 32,
        |_| true,
        |_| true,
        n,
    )
    .outs
}

/// Stream contract and length of every reply.
fn assert_reply_shape(o: &Out) {
    assert_eq!(o.bytes.len(), 48, "48-byte reply");
    for (i, b) in o.bytes.iter().enumerate() {
        assert_eq!(b.last.to_bool(), i == 47, "last only on byte 47");
        assert!(!b.err.to_bool(), "a reply never carries err");
    }
    assert_eq!(o.meta.len.raw(), 48);
    assert_eq!(o.meta.sport.raw(), 123);
}

// ------------------------------------------------------------ reference

/// Independent reference of the reply (rules N4-N10 of responder.py, plus
/// the KoD of the module header). `t3` is the Transmit Timestamp expected.
fn expect(r: &Req, c: &Cfg, kod: bool, t3: u64) -> Vec<u8> {
    let p = &r.pay;
    let sync = c.synced && c.stratum != 0 && c.stratum < 16;
    let vn = (p[0] >> 3) & 7;
    let li = if kod || !sync { 3 } else { 0 };
    let mut v = vec![(li << 6) | (vn << 3) | 4];
    v.push(if kod {
        0
    } else if sync {
        c.stratum
    } else {
        16
    });
    v.push(p[2]);
    v.push(c.precision);
    v.extend_from_slice(&c.root_delay.to_be_bytes());
    v.extend_from_slice(&c.root_disp.to_be_bytes());
    let rid = if kod {
        KOD_RATE
    } else if r.v6 {
        c.ref_id6
    } else {
        c.ref_id
    };
    v.extend_from_slice(&rid.to_be_bytes());
    v.extend_from_slice(&(if kod { 0 } else { c.ref_ts }).to_be_bytes());
    let org = &p[40..48];
    v.extend_from_slice(org);
    if kod {
        v.extend_from_slice(org);
        v.extend_from_slice(org);
    } else {
        v.extend_from_slice(&r.rx_ts.to_be_bytes());
        v.extend_from_slice(&t3.to_be_bytes());
    }
    v
}

fn check_meta(o: &Out, r: &Req) {
    assert_reply_shape(o);
    assert_eq!(o.meta.family.to_bool(), r.v6, "family");
    assert_eq!(o.meta.dst_ip.raw(), r.src, "reply goes to the client");
    assert_eq!(o.meta.src_ip.raw(), r.dst, "from the address asked");
    assert_eq!(o.meta.dst_mac.raw() as u64, r.esrc, "client MAC");
    assert_eq!(o.meta.dport.raw() as u16, r.sport, "client port");
}

/// NTP request payload (48 bytes + `ext` extension bytes).
fn ntp_req(li_vn_mode: u8, poll: u8, xmit: u64, ext: usize) -> Vec<u8> {
    let mut p = vec![0u8; 48 + ext];
    p[0] = li_vn_mode;
    p[2] = poll;
    // Some junk in the fields a server must not copy.
    p[1] = 0x55;
    p[3] = 0xAA;
    for (i, x) in p.iter_mut().enumerate().skip(4).take(36) {
        *x = (i as u8).wrapping_mul(37) ^ 0x5A;
    }
    p[40..48].copy_from_slice(&xmit.to_be_bytes());
    for (i, x) in p.iter_mut().enumerate().skip(48) {
        *x = (i as u8).wrapping_mul(13) ^ 0xC3;
    }
    p
}

const CLI4: u128 = 0xC000_0202;
const SRV4: u128 = 0xC000_0201;
const CLI6: u128 = 0x2001_0db8_0000_0000_0000_0000_0000_0002;
const SRV6: u128 = 0x2001_0db8_0000_0000_0000_0000_0000_0001;
const CMAC: u64 = 0x0200_0000_0002;

fn req4(pay: Vec<u8>, sport: u16) -> Req {
    Req {
        v6: false,
        src: CLI4,
        dst: SRV4,
        sport,
        esrc: CMAC,
        rx_ts: T_RECV,
        pay,
        err: false,
    }
}

fn req6(pay: Vec<u8>, sport: u16) -> Req {
    Req {
        v6: true,
        src: CLI6,
        dst: SRV6,
        sport,
        esrc: CMAC,
        rx_ts: T_RECV,
        pay,
        err: false,
    }
}

// ------------------------------------------------------------ vectors

#[test]
fn vectors_byte_exact() {
    for (name, synced) in [
        ("ntp4_basic.hex", true),
        ("ntp6_basic.hex", true),
        ("ntp_version3.hex", true),
        ("ntp_unsynchronised.hex", false),
        ("neg_ntp_mode4.hex", true),
    ] {
        let v = vector(name);
        let mut reqs = vec![];
        let mut want: Vec<Option<Vec<u8>>> = vec![];
        for (rx, f) in &v {
            if *rx {
                reqs.push(req_from_frame(f, T_RECV));
                want.push(None);
            } else {
                *want.last_mut().unwrap() = Some(f.clone());
            }
        }
        let cfg = Cfg { synced, ..vcfg() };
        let outs = run_simple(&reqs, &cfg);
        let answered: Vec<(usize, &Vec<u8>)> = want
            .iter()
            .enumerate()
            .filter_map(|(i, w)| w.as_ref().map(|w| (i, w)))
            .collect();
        assert_eq!(outs.len(), answered.len(), "{name}: number of replies");
        // Requests of one family are answered in order; match by origin.
        for (i, tx) in answered {
            let r = &reqs[i];
            let o = outs
                .iter()
                .find(|o| o.data()[24..32] == r.pay[40..48])
                .unwrap_or_else(|| panic!("{name}: reply to request {i}"));
            check_meta(o, r);
            // The vector's own frame: addresses and NTP payload.
            let ntp = &tx[tx.len() - 48..];
            assert_eq!(o.data(), ntp, "{name}: request {i}, NTP reply bytes");
            assert_eq!(o.meta.dst_mac.raw() as u64, be(&tx[0..6]) as u64);
            if r.v6 {
                assert_eq!(o.meta.src_ip.raw(), be(&tx[22..38]));
                assert_eq!(o.meta.dst_ip.raw(), be(&tx[38..54]));
            } else {
                assert_eq!(o.meta.src_ip.raw(), be(&tx[26..30]));
                assert_eq!(o.meta.dst_ip.raw(), be(&tx[30..34]));
            }
            assert_eq!(
                o.data(),
                expect(r, &cfg, false, T_XMIT),
                "{name}: reference"
            );
        }
    }
}

// ------------------------------------------------------------ policy

#[test]
fn modes_and_versions() {
    let mut reqs = vec![];
    for vn in 0..8u8 {
        for mode in 0..8u8 {
            let sport = 1000 + (vn as u16) * 8 + mode as u16;
            let p = ntp_req((vn << 3) | mode, 6, 0x1111_0000_0000_0000 | sport as u64, 0);
            reqs.push(req4(p.clone(), sport));
            reqs.push(req6(p, sport));
        }
    }
    let cfg = vcfg();
    let outs = run_simple(&reqs, &cfg);
    let ok: Vec<&Req> = reqs
        .iter()
        .filter(|r| r.pay[0] & 7 == 3 && matches!(r.pay[0] >> 3, 3 | 4))
        .collect();
    assert_eq!(ok.len(), 4, "VN 3 and 4, mode 3, both families");
    assert_eq!(outs.len(), ok.len(), "only mode 3 with VN 3/4 is answered");
    for r in ok {
        let o = outs
            .iter()
            .find(|o| o.meta.dport.raw() as u16 == r.sport && o.meta.family.to_bool() == r.v6)
            .unwrap();
        check_meta(o, r);
        assert_eq!(o.data(), expect(r, &cfg, false, T_XMIT));
        assert_eq!(o.data()[0] >> 3 & 7, r.pay[0] >> 3 & 7, "VN = min(VN, 4)");
        assert_eq!(o.data()[0] & 7, 4, "mode 4");
    }
    // LI bits of the request do not matter.
    let reqs: Vec<Req> = (0..4u8)
        .map(|li| req4(ntp_req((li << 6) | 0x23, 4, li as u64, 0), 2000 + li as u16))
        .collect();
    let outs = run_simple(&reqs, &cfg);
    assert_eq!(outs.len(), 4);
    for o in &outs {
        assert_eq!(o.data()[0], 0x24, "LI 0, VN 4, mode 4");
    }
}

#[test]
fn lengths_and_extension_fields() {
    let cfg = vcfg();
    let mut reqs = vec![];
    // Too short: never answered.
    for n in [1usize, 2, 3, 8, 40, 46, 47] {
        let mut p = ntp_req(0x23, 6, n as u64, 0);
        p.truncate(n);
        reqs.push(req4(p, 3000 + n as u16));
    }
    // 48 and longer (extension fields, MAC): answered with 48 bytes, the
    // Origin is request bytes 40-47, whatever follows.
    let long = [0usize, 1, 4, 12, 20, 52, 200, 1424];
    for &ext in &long {
        reqs.push(req4(
            ntp_req(0x23, 6, 0xABCD_0000 + ext as u64, ext),
            4000 + ext as u16,
        ));
        reqs.push(req6(
            ntp_req(0x1B, 7, 0xDCBA_0000 + ext as u64, ext),
            4000 + ext as u16,
        ));
    }
    let outs = run_simple(&reqs, &cfg);
    assert_eq!(outs.len(), 2 * long.len());
    for o in &outs {
        let r = reqs
            .iter()
            .find(|r| r.sport == o.meta.dport.raw() as u16 && r.v6 == o.meta.family.to_bool())
            .unwrap();
        assert!(r.pay.len() >= 48);
        check_meta(o, r);
        assert_eq!(o.data(), expect(r, &cfg, false, T_XMIT));
        assert!(o.bytes.len() <= r.pay.len(), "no amplification");
    }
}

#[test]
fn err_on_last_byte_drops_the_request() {
    let cfg = Cfg {
        rate_limit: 1,
        kod: true,
        ..vcfg()
    };
    let mut reqs = vec![];
    for (k, ext) in [0usize, 1, 60].iter().enumerate() {
        let mut r = req4(ntp_req(0x23, 6, 10 + k as u64, *ext), 5000 + k as u16);
        r.err = true;
        reqs.push(r);
        let mut r = req6(ntp_req(0x23, 6, 20 + k as u64, *ext), 5000 + k as u16);
        r.err = true;
        reqs.push(r);
    }
    // A short request with err too.
    let mut r = req4(ntp_req(0x23, 6, 30, 0)[..20].to_vec(), 5100);
    r.err = true;
    reqs.push(r);
    // Then a good one per family: answered normally, and the rate limiter
    // (limit 1) was not consumed by the bad ones (no KoD).
    reqs.push(req4(ntp_req(0x23, 6, 40, 0), 5200));
    reqs.push(req6(ntp_req(0x23, 6, 41, 0), 5200));
    let outs = run_simple(&reqs, &cfg);
    assert_eq!(outs.len(), 2, "only the two clean requests are answered");
    for o in &outs {
        let r = reqs
            .iter()
            .find(|r| r.sport == 5200 && r.v6 == o.meta.family.to_bool())
            .unwrap();
        check_meta(o, r);
        assert_eq!(o.data(), expect(r, &cfg, false, T_XMIT));
    }
}

#[test]
fn families_interleaved_round_robin() {
    let cfg = vcfg();
    let mut reqs = vec![];
    for i in 0..6u16 {
        reqs.push(req4(
            ntp_req(0x23, 6, 0x4000 + i as u64, (i as usize) * 3),
            100 + i,
        ));
        reqs.push(req6(
            ntp_req(0x1B, 5, 0x6000 + i as u64, (i as usize) * 5),
            200 + i,
        ));
    }
    // Extra IPv4 ones at the end: served back to back once IPv6 is empty.
    for i in 6..9u16 {
        reqs.push(req4(ntp_req(0x23, 6, 0x4000 + i as u64, 0), 100 + i));
    }
    let outs = run_simple(&reqs, &cfg);
    assert_eq!(outs.len(), reqs.len());
    let fams: Vec<bool> = outs.iter().map(|o| o.meta.family.to_bool()).collect();
    for k in 1..12 {
        assert_ne!(
            fams[k],
            fams[k - 1],
            "alternation while both are pending: {fams:?}"
        );
    }
    assert!(fams[12..].iter().all(|f| !f));
    // Each family in its own order, every reply correct.
    let mut n4 = 0;
    let mut n6 = 0;
    for o in &outs {
        let r = if o.meta.family.to_bool() {
            n6 += 1;
            &reqs[(n6 - 1) * 2 + 1]
        } else {
            n4 += 1;
            if n4 <= 6 {
                &reqs[(n4 - 1) * 2]
            } else {
                &reqs[12 + n4 - 7]
            }
        };
        check_meta(o, r);
        assert_eq!(o.data(), expect(r, &cfg, false, T_XMIT));
    }
    // Reference ID per family.
    for o in &outs {
        let rid = be(&o.data()[12..16]) as u32;
        assert_eq!(
            rid,
            if o.meta.family.to_bool() {
                REF_ID6
            } else {
                REF_ID4
            }
        );
    }
}

#[test]
fn unsynchronised_and_stratum_range() {
    let r4 = req4(ntp_req(0x23, 6, 77, 0), 6000);
    let r6 = req6(ntp_req(0x23, 6, 78, 0), 6000);
    for (synced, stratum, li, st_out) in [
        (false, 2u8, 3u8, 16u8),
        (true, 2, 0, 2),
        (true, 1, 0, 1),
        (true, 15, 0, 15),
        (true, 0, 3, 16),
        (true, 16, 3, 16),
        (true, 200, 3, 16),
        (false, 0, 3, 16),
    ] {
        let cfg = Cfg {
            synced,
            stratum,
            ..vcfg()
        };
        let outs = run_simple(&[r4.clone(), r6.clone()], &cfg);
        assert_eq!(outs.len(), 2);
        for o in &outs {
            let d = o.data();
            assert_eq!(d[0] >> 6, li, "LI, synced {synced} stratum {stratum}");
            assert_eq!(d[1], st_out, "stratum, synced {synced} stratum {stratum}");
            assert_ne!(d[1], 0, "a server reply never has stratum 0");
            let r = if o.meta.family.to_bool() { &r6 } else { &r4 };
            assert_eq!(d, expect(r, &cfg, false, T_XMIT));
            // Reference ID kept when not synchronised (as the model).
            assert_eq!(be(&d[12..16]) as u32, if r.v6 { REF_ID6 } else { REF_ID4 });
        }
    }
}

#[test]
fn precision_field() {
    let r = req4(ntp_req(0x23, 6, 5, 0), 7000);
    for p in [DEFAULT_PRECISION, 0xEC, 0xE6, 0x00, 0x7F] {
        let cfg = Cfg {
            precision: p,
            ..vcfg()
        };
        let outs = run_simple(std::slice::from_ref(&r), &cfg);
        assert_eq!(outs[0].data()[3], p);
    }
    assert_eq!(DEFAULT_PRECISION as i8, -24);
}

// ------------------------------------------------------------ timestamps

/// NTP accumulator at cycle t of a running clock that starts at `t0` (NTP).
fn running(t0: u64) -> impl Fn(usize) -> u128 {
    move |t| ((t0 as u128) << 32) + t as u128 * INCR
}

#[test]
fn origin_receive_transmit() {
    let t0: u64 = 0xECC5_8CC1_0000_0000;
    let k: u64 = 0x0000_0000_0000_1234;
    let cfg = Cfg {
        tx_latency: k,
        ..vcfg()
    };
    // Distinct origin (client's transmit, unrelated to our clock), receive
    // stamps a little before the clock.
    let mut reqs = vec![];
    for i in 0..8u64 {
        let mut r = req4(
            ntp_req(0x23, 6, 0x0123_4567_89AB_CDEF ^ (i << 40), 0),
            8000 + i as u16,
        );
        r.rx_ts = t0 - 1000 * (i + 1);
        reqs.push(r.clone());
        r.v6 = true;
        r.src = CLI6 + i as u128;
        r.dst = SRV6;
        reqs.push(r);
    }
    let clk = running(t0);
    for (gap, pat) in [(0usize, 0usize), (1, 3), (2, 5), (0, 7)] {
        let take = move |t: usize| pat == 0 || t % pat != 1;
        let n = cycles_for(&reqs, gap, 3);
        let ro = run(&reqs, &cfg, gap, &clk, take, take, n);
        assert_eq!(ro.outs.len(), reqs.len());
        let mut lag = None;
        for o in &ro.outs {
            let d = o.data();
            let r = reqs
                .iter()
                .find(|r| r.sport == o.meta.dport.raw() as u16 && r.v6 == o.meta.family.to_bool())
                .unwrap();
            check_meta(o, r);
            assert_eq!(&d[24..32], &r.pay[40..48], "Origin = client's Transmit");
            assert_eq!(be(&d[32..40]) as u64, r.rx_ts, "Receive = Meta.rx_ts");
            let t3 = o.t3();
            assert!(t3 >= r.rx_ts, "Transmit >= Receive");
            // T3 - K is the clock of exactly one cycle, at a fixed distance
            // from the cycle byte 40 was seen, whatever the stalls.
            let ts = (0..n)
                .find(|&t| (clk(t) >> 32) as u64 == t3 - k)
                .expect("T3 - K is a sampled clock value");
            // The clock is that of cycle S - 1 (byte 40 sent in S, the
            // testbench's iteration numbers are the unit's cycles); byte 40
            // reaches the sink in S + 1, or later when the sink skips that
            // cycle: 2, or 3 with these one-cycle stalls, whatever the
            // stalls before byte 40 (up to ~16 cycles here): T3 is sampled
            // at byte 40, not at the request.
            let l = o.t40 as i64 - ts as i64;
            assert!(
                (2..=3).contains(&l),
                "sampling cycle relative to byte 40: {l}"
            );
            lag = Some(lag.map_or(l, |m: i64| m.max(l)));
            assert_eq!(d, expect(r, &cfg, false, t3));
        }
        assert!(lag.is_some());
    }
}

#[test]
fn transmit_never_before_receive() {
    // A receive stamp in the future of the clock (backward step of the
    // clock, or a wrong K): T3 is clamped to the receive stamp.
    let t0: u64 = 0xECC5_8CC1_0000_0000;
    let cfg = Cfg {
        tx_latency: 0,
        ..vcfg()
    };
    let mut r = req4(ntp_req(0x23, 6, 9, 0), 9000);
    r.rx_ts = t0 + 0x1_0000_0000; // one second ahead
    let mut r2 = req4(ntp_req(0x23, 6, 10, 0), 9001);
    r2.rx_ts = u64::MAX;
    let n = cycles_for(&[r.clone(), r2.clone()], 0, 2);
    let outs = run(
        &[r.clone(), r2.clone()],
        &cfg,
        0,
        running(t0),
        |_| true,
        |_| true,
        n,
    )
    .outs;
    assert_eq!(outs[0].t3(), r.rx_ts);
    assert_eq!(outs[1].t3(), u64::MAX);
    // tx_latency wraps modulo 2^64 like NTP time; still clamped.
    let cfg = Cfg {
        tx_latency: u64::MAX - 5,
        ..vcfg()
    };
    let outs = run(&[r.clone()], &cfg, 0, running(t0), |_| true, |_| true, n).outs;
    assert_eq!(outs[0].t3(), r.rx_ts);
}

// ------------------------------------------------------------ rate limiter

fn src_of(o: &Out) -> u128 {
    o.meta.dst_ip.raw()
}

fn hash4(a: u128) -> u32 {
    src_hash(U::<64>::new(a & 0xFFFF_FFFF)).raw() as u32
}

#[test]
fn rate_limiter() {
    let a = 0xC000_0202u128;
    // A source in another bucket, and one in the same bucket as `a`.
    let b = (1..1000u128)
        .map(|i| 0xC000_0200 + i)
        .find(|&x| hash4(x) != hash4(a))
        .unwrap();
    let c = (1..100_000u128)
        .map(|i| 0xC633_6400 + i)
        .find(|&x| hash4(x) == hash4(a))
        .unwrap();
    let mk = |src: u128, n: usize, base: u16| -> Vec<Req> {
        (0..n)
            .map(|i| {
                let mut r = req4(
                    ntp_req(0x23, 6, (base as u64) << 16 | i as u64, 0),
                    base + i as u16,
                );
                r.src = src;
                r
            })
            .collect()
    };
    let t0 = 0xECC5_8CC1_0000_0000u64;
    // Fixed second (frozen clock).
    let frozen = |_| (t0 as u128) << 32;
    // 1. Burst of 10 from a, interleaved with 10 from b: 3 each.
    let mut reqs = vec![];
    let (ra, rb) = (mk(a, 10, 100), mk(b, 10, 200));
    for i in 0..10 {
        reqs.push(ra[i].clone());
        reqs.push(rb[i].clone());
    }
    let base = Cfg {
        rate_limit: 3,
        ..vcfg()
    };
    let n = cycles_for(&reqs, 0, 1);
    let outs = run(&reqs, &base, 0, frozen, |_| true, |_| true, n).outs;
    let na = outs.iter().filter(|o| src_of(o) == a).count();
    let nb = outs.iter().filter(|o| src_of(o) == b).count();
    assert_eq!((na, nb), (3, 3), "burst limited per source");
    // The first three of each source are the ones answered.
    for o in &outs {
        let d = o.data();
        assert!(d[31] < 3, "first three answered");
    }
    // 2. KoD variant: 3 normal replies then KoD for the rest.
    let kcfg = Cfg {
        kod: true,
        ..base.clone()
    };
    let outs = run(&ra, &kcfg, 0, frozen, |_| true, |_| true, n).outs;
    assert_eq!(outs.len(), 10, "every request answered, 7 by a KoD");
    for (i, o) in outs.iter().enumerate() {
        check_meta(o, &ra[i]);
        let kod = i >= 3;
        let t3 = if kod { 0 } else { o.t3() };
        assert_eq!(o.data(), expect(&ra[i], &kcfg, kod, t3), "reply {i}");
        if kod {
            let d = o.data();
            assert_eq!(d[0] >> 6, 3, "KoD LI 3");
            assert_eq!(d[1], 0, "KoD stratum 0");
            assert_eq!(&d[12..16], b"RATE");
            assert_eq!(&d[24..32], &ra[i].pay[40..48], "KoD Origin");
            assert_eq!(d[0] & 7, 4, "KoD mode 4");
        }
    }
    // 3. Limit 0 = unlimited.
    let ucfg = Cfg {
        rate_limit: 0,
        ..vcfg()
    };
    let many = mk(a, 40, 300);
    let n2 = cycles_for(&many, 0, 1);
    let outs = run(&many, &ucfg, 0, frozen, |_| true, |_| true, n2).outs;
    assert_eq!(outs.len(), 40);
    // 4. Window rollover: the clock crosses a second boundary in the middle
    // of the burst; the counter starts again.
    let cut = 400; // cycle at which the second changes
    let roll = move |t: usize| {
        let s = if t < cut { t0 } else { t0 + (1u64 << 32) };
        (s as u128) << 32
    };
    let ro = run(&mk(a, 10, 400), &base, 0, roll, |_| true, |_| true, n);
    let before = ro.outs.iter().filter(|o| o.t_last < cut).count();
    let after = ro.outs.iter().filter(|o| o.t_last >= cut).count();
    assert!(ro.sent_t[9] > cut + 200, "burst spans the boundary");
    assert!(ro.sent_t[3] < cut, "a request limited in the first second");
    assert_eq!(before, 3, "3 in the first second");
    assert_eq!(after, 3, "3 again in the next second");
    assert_eq!(ro.outs.len(), 6);
    // 5. Collision: a and c share a bucket: together at most 3 (c limited
    // early), never more than 3 for a.
    let mut reqs = mk(a, 2, 500);
    reqs.extend(mk(c, 5, 600));
    reqs.extend(mk(a, 3, 700));
    let outs = run(&reqs, &base, 0, frozen, |_| true, |_| true, n).outs;
    let na = outs.iter().filter(|o| src_of(o) == a).count();
    let nc = outs.iter().filter(|o| src_of(o) == c).count();
    assert_eq!(
        (na, nc),
        (2, 1),
        "shared bucket: limited early, not bypassed"
    );
    // 6. IPv6: per /64 prefix. Two addresses of one /64 share a counter;
    // another /64 does not.
    let p1 = 0x2001_0db8_0001_0000u128 << 64;
    let p2 = (1..1000u128)
        .map(|i| (0x2001_0db8_0002_0000u128 + i) << 64)
        .find(|p| src_hash(U::<64>::new(p >> 64)).raw() != src_hash(U::<64>::new(p1 >> 64)).raw())
        .unwrap();
    let mut reqs = vec![];
    for i in 0..4u16 {
        for (src, base) in [(p1 | 1, 800u16), (p1 | 0xFFFF, 900), (p2 | 1, 1000)] {
            let mut r = req6(ntp_req(0x23, 6, base as u64 + i as u64, 0), base + i);
            r.src = src;
            reqs.push(r);
        }
    }
    let outs = run(&reqs, &base, 0, frozen, |_| true, |_| true, n).outs;
    let in1 = outs.iter().filter(|o| src_of(o) >> 64 == p1 >> 64).count();
    let in2 = outs.iter().filter(|o| src_of(o) >> 64 == p2 >> 64).count();
    assert_eq!((in1, in2), (3, 3), "IPv6 limited per /64");
}

// ------------------------------------------------------------ back-pressure

#[test]
fn backpressure_on_outputs() {
    let cfg = vcfg();
    let mut reqs = vec![];
    for i in 0..10u16 {
        reqs.push(req4(
            ntp_req(0x23, 6, i as u64, (i as usize * 7) % 30),
            100 + i,
        ));
        reqs.push(req6(
            ntp_req(0x1B, 6, 100 + i as u64, (i as usize * 3) % 17),
            200 + i,
        ));
    }
    for (gap, pm, pb) in [(0usize, 2usize, 3usize), (1, 5, 2), (0, 7, 11), (3, 1, 4)] {
        let n = cycles_for(&reqs, gap, 4) + 2000;
        let ro = run(
            &reqs,
            &cfg,
            gap,
            |_| (T_RECV as u128) << 32,
            move |t| t % pm == 0 || pm == 1,
            move |t| t % pb != 0 && t % 13 != 5,
            n,
        );
        assert_eq!(ro.outs.len(), reqs.len(), "gap {gap} pm {pm} pb {pb}");
        for o in &ro.outs {
            let r = reqs
                .iter()
                .find(|r| r.sport == o.meta.dport.raw() as u16 && r.v6 == o.meta.family.to_bool())
                .unwrap();
            check_meta(o, r);
            assert_eq!(o.data(), expect(r, &cfg, false, T_XMIT));
            assert!(o.t_meta <= o.t_last);
            if pm == 1 {
                assert!(o.t_meta < o.t_first, "reply_meta before the first byte");
            }
        }
    }
}

// ------------------------------------------------------------ throughput

#[test]
fn throughput_back_to_back() {
    let cfg = vcfg();
    // Cycles per request: one request alone, from its meta offered (cycle 0)
    // to the last reply byte seen.
    let one = vec![req4(ntp_req(0x23, 6, 1, 0), 1)];
    let ro = run(
        &one,
        &cfg,
        0,
        |_| (T_RECV as u128) << 32,
        |_| true,
        |_| true,
        300,
    );
    assert_eq!(ro.outs.len(), 1);
    let single = ro.outs[0].t_last;
    // 1000 back to back (IPv4), then the steady-state period.
    let reqs: Vec<Req> = (0..1000u32)
        .map(|i| req4(ntp_req(0x23, 6, i as u64, 0), i as u16))
        .collect();
    let n = 1000 * 110 + 500;
    let ro = run(
        &reqs,
        &cfg,
        0,
        |_| (T_RECV as u128) << 32,
        |_| true,
        |_| true,
        n,
    );
    assert_eq!(ro.outs.len(), 1000, "every request answered");
    for (i, o) in ro.outs.iter().enumerate() {
        assert_eq!(o.data(), expect(&reqs[i], &cfg, false, T_XMIT), "reply {i}");
    }
    let periods: Vec<usize> = ro
        .outs
        .windows(2)
        .map(|w| w[1].t_last - w[0].t_last)
        .collect();
    let per = *periods.iter().max().unwrap();
    assert!(periods.iter().all(|&p| p == per), "constant period");
    let total = ro.outs[999].t_last;
    println!(
        "ntp_server: single request {single} cycles (meta offered -> last reply byte seen), \
         back to back {per} cycles per request, 1000 requests in {total} cycles"
    );
    // n + 50 cycles for an n-byte request (module header): 98 for 48 bytes.
    assert_eq!(per, 98, "cycles per 48-byte request");
    // Line rate: 109 000 IPv4 requests/s = one per 917 cycles at 100 MHz.
    assert!(per * 109_000 < 100_000_000);
    // Extension fields: n + 50.
    let reqs: Vec<Req> = (0..20u32)
        .map(|i| req4(ntp_req(0x23, 6, i as u64, 20), i as u16))
        .collect();
    let ro = run(
        &reqs,
        &cfg,
        0,
        |_| (T_RECV as u128) << 32,
        |_| true,
        |_| true,
        20 * 130 + 300,
    );
    assert_eq!(ro.outs.len(), 20);
    for w in ro.outs.windows(2) {
        assert_eq!(w[1].t_last - w[0].t_last, 68 + 50);
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

const MAC: u64 = 0x0200_0000_0001;
const LOCAL_IP: u32 = 0xC000_0201;

/// Frames out of eth_tx (padded, FCS included) with the cycle their last
/// byte was taken (the cycle `tx_path` would accept it: s0).
type Frames = Vec<(Vec<u8>, usize)>;

fn collect_frame(f_r: &txhdl::comp::Rx<FrameByte>, t: usize, open: &mut Vec<u8>, out: &mut Frames) {
    if let Some(b) = f_r.recv() {
        open.push(b.data.raw() as u8);
        assert!(!b.err.to_bool(), "no aborted frame");
        if b.last.to_bool() {
            out.push((std::mem::take(open), t));
        }
    }
}

/// udp4_rx -> ntp_server -> IPv4 adapter -> udp4_tx -> ipv4_tx -> tx_arb
/// (source 2) -> eth_tx, fed with UDP datagrams (`IpMeta` + bytes). The
/// testbench (1) threads `rx_ts` into the `Meta` between udp4_rx and the
/// server (stand-in for T1.13; udp4_rx still writes 0) and (2) turns
/// `NtpReplyMeta` into `UdpTxReq` (the integration adapter). Payload bytes
/// go through registered channels only, unit to unit.
fn chain4(
    dgrams: &[(IpMeta, Vec<u8>)],
    cfg: &Cfg,
    rx_ts: u64,
    time: impl Fn(usize) -> u128,
    cycles: usize,
) -> Frames {
    let (time_o, time_i) = signal::<U<96>, DefaultClock>();
    let (sy, st, ri, ri6, rt, rd, rp, pr, tl, rl, kd) = cfg_wires(cfg);
    let (um_t, um) = chan::<IpMeta, DefaultClock>();
    let (u_t, u) = chan::<FrameByte, DefaultClock>();
    let (nm_t, nm) = chan::<Meta, DefaultClock>();
    let (np_t, np) = chan::<FrameByte, DefaultClock>();
    let (m4_t, m4) = chan::<Meta, DefaultClock>();
    let (_m6_t, m6) = chan::<Meta, DefaultClock>();
    let (_r6_t, r6) = chan::<FrameByte, DefaultClock>();
    let (om_t, om) = chan::<NtpReplyMeta, DefaultClock>();
    let (ob_t, ob) = chan::<FrameByte, DefaultClock>();
    let (rq_t, rq) = chan::<UdpTxReq, DefaultClock>();
    let (oq_t, oq) = chan::<IpTxReq, DefaultClock>();
    let (op_t, op) = chan::<FrameByte, DefaultClock>();
    let (h2_t, h2) = chan::<EthHdr, DefaultClock>();
    let (p2_t, p2) = chan::<FrameByte, DefaultClock>();
    let (_h0_t, h0) = chan::<EthHdr, DefaultClock>();
    let (_p0_t, p0) = chan::<FrameByte, DefaultClock>();
    let (_h1_t, h1) = chan::<EthHdr, DefaultClock>();
    let (_p1_t, p1) = chan::<FrameByte, DefaultClock>();
    let (_h3_t, h3) = chan::<EthHdr, DefaultClock>();
    let (_p3_t, p3) = chan::<FrameByte, DefaultClock>();
    let (oh_t, oh) = chan::<EthHdr, DefaultClock>();
    let (opl_t, opl) = chan::<FrameByte, DefaultClock>();
    let (f_t, f_r) = chan::<FrameByte, DefaultClock>();
    let (mut d_urx, mut d_ntp, mut d_utx, mut d_itx, mut d_arb, mut d_etx) = (
        Udp4Rx::default(),
        NtpServer::default(),
        Udp4Tx::default(),
        Ipv4Tx::default(),
        TxArb::default(),
        EthTx::default(),
    );
    let mut sim = Running::new(join2(
        join2(
            d_urx.run((um, u), (nm_t, np_t)),
            d_ntp.run(
                (
                    m4, np, m6, r6, time_i, sy, st, ri, ri6, rt, rd, rp, pr, tl, rl, kd,
                ),
                (om_t, ob_t),
            ),
        ),
        join2(
            join2(
                d_utx.run((rq, ob, tie(U::<32>::from(LOCAL_IP))), (oq_t, op_t)),
                d_itx.run((oq, op, tie(U::<32>::from(LOCAL_IP))), (h2_t, p2_t)),
            ),
            join2(
                d_arb.run((h0, p0, h1, p1, h2, p2, h3, p3), (oh_t, opl_t)),
                d_etx.run((oh, opl, tie(U::<48>::from(MAC))), f_t),
            ),
        ),
    ));
    // Input queue: meta with byte 0, then bytes.
    let mut q: VecDeque<(Option<IpMeta>, FrameByte)> = VecDeque::new();
    for (m, d) in dgrams {
        for (k, &b) in d.iter().enumerate() {
            q.push_back((
                if k == 0 { Some(*m) } else { None },
                FrameByte::new(b, k + 1 == d.len(), false),
            ));
        }
    }
    let mut meta_sent = false;
    let mut frames = vec![];
    let mut open = vec![];
    for t in 0..cycles {
        time_o.set(U::<96>::new(time(t) & ((1u128 << 96) - 1)));
        if let Some((m, b)) = q.front() {
            let mut ok = true;
            if let Some(m) = m {
                if !meta_sent {
                    if um_t.ready().to_bool() {
                        um_t.send(*m);
                        meta_sent = true;
                    } else {
                        ok = false;
                    }
                }
            }
            if ok && u_t.ready().to_bool() {
                u_t.send(*b);
                q.pop_front();
                meta_sent = false;
            }
        }
        // Stand-in for T1.13: the receive stamp rides in the Meta.
        if m4_t.ready().to_bool() {
            if let Some(mut m) = nm.recv() {
                m.rx_ts = U::<64>::from(rx_ts);
                m4_t.send(m);
            }
        }
        // IPv4 adapter: NtpReplyMeta -> UdpTxReq.
        if rq_t.ready().to_bool() {
            if let Some(r) = om.recv() {
                assert!(!r.family.to_bool());
                rq_t.send(UdpTxReq {
                    dst_mac: r.dst_mac,
                    dst_ip: r.dst_ip.slice::<0, 32>(),
                    sport: r.sport,
                    dport: r.dport,
                    len: r.len,
                });
            }
        }
        collect_frame(&f_r, t, &mut open, &mut frames);
        sim.cycle();
    }
    assert!(q.is_empty() && open.is_empty());
    frames
}

/// IpMeta + UDP datagram of an IPv4 vector frame.
fn dgram_of(f: &[u8]) -> (IpMeta, Vec<u8>) {
    let tot = u16::from_be_bytes([f[16], f[17]]) as usize;
    let seg = f[34..14 + tot].to_vec();
    (
        IpMeta {
            src: U::<32>::from(be(&f[26..30]) as u32),
            dst: U::<32>::from(be(&f[30..34]) as u32),
            proto: U::<8>::from(17u8),
            len: U::<16>::from(seg.len() as u16),
            eth_src: U::<48>::from(be(&f[6..12]) as u64),
            ttl: U::<8>::from(f[22]),
            // The chain test patches the receive stamp into `Meta` itself.
            rx_ts: U::<64>::from(0u64),
        },
        seg,
    )
}

#[test]
fn chain_ntp4_basic() {
    let v = vector("ntp4_basic.hex");
    let rx = v.iter().find(|(r, _)| *r).unwrap().1.clone();
    let tx = v.iter().find(|(r, _)| !*r).unwrap().1.clone();
    let d = dgram_of(&rx);
    // Frozen clock at T_RECV, K = T_XMIT - T_RECV: the vector's timestamps.
    let cfg = vcfg();
    let frames = chain4(
        std::slice::from_ref(&d),
        &cfg,
        T_RECV,
        |_| (T_RECV as u128) << 32,
        800,
    );
    assert_eq!(frames.len(), 1);
    check_frame(&frames[0].0, &tx, "ntp4_basic reply frame");
    // Back to back.
    let frames = chain4(
        &[d.clone(), d.clone(), d.clone()],
        &cfg,
        T_RECV,
        |_| (T_RECV as u128) << 32,
        1500,
    );
    assert_eq!(frames.len(), 3);
    for (f, _) in &frames {
        check_frame(f, &tx, "ntp4_basic, back to back");
    }
    // Latency: running clock, K = 0, so T3 is the clock of the sampling
    // cycle; s0 is the cycle the last FCS byte is taken.
    let t0 = T_RECV + 0x100;
    let clk = running(t0);
    let cfg0 = Cfg {
        tx_latency: 0,
        ..vcfg()
    };
    let frames = chain4(&[d], &cfg0, T_RECV, &clk, 800);
    let (f, s0) = &frames[0];
    let t3 = be(&f[14 + 20 + 8 + 40..14 + 20 + 8 + 48]) as u64;
    let ts = (0..800).find(|&t| (clk(t) >> 32) as u64 == t3).unwrap();
    // `ts` is the cycle whose clock was sampled (S - 1; the testbench's
    // iteration numbers are the units' cycles: a wire set in iteration i is
    // read in cycle i, a send in cycle i is taken by the testbench in
    // i + 1 at the earliest); s0 is the cycle the last FCS byte is taken
    // from eth_tx's registered output, as tx_path would take it.
    let cyc = (*s0 - ts) as u64;
    println!(
        "ntp_server IPv4: {cyc} cycles from the sampling of time to the last FCS byte; \
         default tx_latency {TX_LATENCY_V4} (2^-32 s)"
    );
    assert_eq!(cyc, TX_CYCLES_V4, "documented TX_CYCLES_V4");
    // Rest of the frame as the vector (except T3 and the UDP checksum).
    assert_eq!(f.len(), 90 + 4);
    assert_eq!(&f[..40], &tx[..40]);
    assert_eq!(&f[42..82], &tx[42..82]);
}

/// IPv6 latency: ntp_server (IPv6 request) -> adapter -> udp4_tx (stand-in
/// for the future udp6_tx, same store-and-forward structure) -> adapter
/// IpTxReq -> Ip6TxReq -> ipv6_tx -> tx_arb (source 3) -> eth_tx.
#[test]
fn chain_latency_v6() {
    let v = vector("ntp6_basic.hex");
    let rx = v.iter().find(|(r, _)| *r).unwrap().1.clone();
    let tx = v.iter().find(|(r, _)| !*r).unwrap().1.clone();
    let r = req_from_frame(&rx, T_RECV);
    let t0 = T_RECV + 0x100;
    let clk = running(t0);
    let cfg = Cfg {
        tx_latency: 0,
        ..vcfg()
    };
    let (time_o, time_i) = signal::<U<96>, DefaultClock>();
    let (sy, st, ri, ri6, rt, rd, rp, pr, tl, rl, kd) = cfg_wires(&cfg);
    let (_m4_t, m4) = chan::<Meta, DefaultClock>();
    let (_r4_t, r4) = chan::<FrameByte, DefaultClock>();
    let (m6_t, m6) = chan::<Meta, DefaultClock>();
    let (r6_t, r6) = chan::<FrameByte, DefaultClock>();
    let (om_t, om) = chan::<NtpReplyMeta, DefaultClock>();
    let (ob_t, ob) = chan::<FrameByte, DefaultClock>();
    let (rq_t, rq) = chan::<UdpTxReq, DefaultClock>();
    let (oq_t, oq) = chan::<IpTxReq, DefaultClock>();
    let (op_t, op) = chan::<FrameByte, DefaultClock>();
    let (q6_t, q6) = chan::<Ip6TxReq, DefaultClock>();
    let (h3_t, h3) = chan::<EthHdr, DefaultClock>();
    let (p3_t, p3) = chan::<FrameByte, DefaultClock>();
    let (_h0_t, h0) = chan::<EthHdr, DefaultClock>();
    let (_p0_t, p0) = chan::<FrameByte, DefaultClock>();
    let (_h1_t, h1) = chan::<EthHdr, DefaultClock>();
    let (_p1_t, p1) = chan::<FrameByte, DefaultClock>();
    let (_h2_t, h2) = chan::<EthHdr, DefaultClock>();
    let (_p2_t, p2) = chan::<FrameByte, DefaultClock>();
    let (oh_t, oh) = chan::<EthHdr, DefaultClock>();
    let (opl_t, opl) = chan::<FrameByte, DefaultClock>();
    let (f_t, f_r) = chan::<FrameByte, DefaultClock>();
    let (mut d_ntp, mut d_utx, mut d_i6, mut d_arb, mut d_etx) = (
        NtpServer::default(),
        Udp4Tx::default(),
        Ipv6Tx::default(),
        TxArb::default(),
        EthTx::default(),
    );
    let mut sim = Running::new(join2(
        join2(
            d_ntp.run(
                (
                    m4, r4, m6, r6, time_i, sy, st, ri, ri6, rt, rd, rp, pr, tl, rl, kd,
                ),
                (om_t, ob_t),
            ),
            d_utx.run((rq, ob, tie(U::<32>::from(LOCAL_IP))), (oq_t, op_t)),
        ),
        join2(
            d_i6.run((q6, op), (h3_t, p3_t)),
            join2(
                d_arb.run((h0, p0, h1, p1, h2, p2, h3, p3), (oh_t, opl_t)),
                d_etx.run((oh, opl, tie(U::<48>::from(MAC))), f_t),
            ),
        ),
    ));
    let mut meta_sent = false;
    let mut k = 0;
    let mut cur: Option<NtpReplyMeta> = None;
    let mut frames = vec![];
    let mut open = vec![];
    for t in 0..800 {
        time_o.set(U::<96>::new(clk(t)));
        if !meta_sent {
            if m6_t.ready().to_bool() {
                m6_t.send(Meta {
                    family: Bit::One,
                    src: U::<128>::new(r.src),
                    dst: U::<128>::new(r.dst),
                    sport: U::<16>::from(r.sport),
                    dport: U::<16>::from(123u16),
                    len: U::<16>::from(48u16),
                    eth_src: U::<48>::from(r.esrc),
                    rx_ts: U::<64>::from(r.rx_ts),
                });
                meta_sent = true;
            }
        } else if k < r.pay.len() && r6_t.ready().to_bool() {
            r6_t.send(FrameByte::new(r.pay[k], k + 1 == r.pay.len(), false));
            k += 1;
        }
        if rq_t.ready().to_bool() {
            if let Some(m) = om.recv() {
                assert!(m.family.to_bool());
                cur = Some(m);
                rq_t.send(UdpTxReq {
                    dst_mac: m.dst_mac,
                    dst_ip: m.dst_ip.slice::<0, 32>(),
                    sport: m.sport,
                    dport: m.dport,
                    len: m.len,
                });
            }
        }
        if q6_t.ready().to_bool() {
            if let Some(q) = oq.recv() {
                let m = cur.unwrap();
                q6_t.send(Ip6TxReq {
                    dst_mac: q.dst_mac,
                    dst_ip: m.dst_ip,
                    src_ip: m.src_ip,
                    next: U::<8>::from(17u8),
                    len: q.len,
                    hop: U::<8>::from(64u8),
                });
            }
        }
        collect_frame(&f_r, t, &mut open, &mut frames);
        sim.cycle();
    }
    assert_eq!(frames.len(), 1);
    let (f, s0) = &frames[0];
    assert_eq!(f.len(), 14 + 40 + 8 + 48 + 4);
    // Everything but the UDP checksum (stand-in) and T3 as the vector.
    assert_eq!(
        &f[..54 + 6],
        &tx[..54 + 6],
        "Ethernet, IPv6 header, ports, length"
    );
    assert_eq!(&f[62..62 + 40], &tx[62..62 + 40], "NTP bytes 0-39");
    let t3 = be(&f[62 + 40..62 + 48]) as u64;
    let ts = (0..800).find(|&t| (clk(t) >> 32) as u64 == t3).unwrap();
    // As for IPv4, minus the stand-in relay IpTxReq -> Ip6TxReq of this
    // testbench, which delays the IPv6 header by one cycle.
    let cyc = (*s0 - ts - 1) as u64;
    println!(
        "ntp_server IPv6: {cyc} cycles from the sampling of time to the last FCS byte; \
         default tx_latency {TX_LATENCY_V6} (2^-32 s)"
    );
    assert_eq!(cyc, TX_CYCLES_V6, "documented TX_CYCLES_V6");
}

#[test]
fn default_tx_latency_values() {
    // K = cycles x 10 ns + 790 ns, in 2^-32 s.
    let k4 = ((TX_CYCLES_V4 * 10 + 790) as f64) * 1e-9 * 4294967296.0;
    let k6 = ((TX_CYCLES_V6 * 10 + 790) as f64) * 1e-9 * 4294967296.0;
    assert!((TX_LATENCY_V4 as f64 - k4).abs() <= 0.5);
    assert!((TX_LATENCY_V6 as f64 - k6).abs() <= 0.5);
    // The values written in the module header.
    assert_eq!((TX_CYCLES_V4, TX_CYCLES_V6), (108, 128));
    assert_eq!((TX_LATENCY_V4, TX_LATENCY_V6), (8032, 8891));
}

#[test]
fn lowers_to_verilog() {
    let v = NtpServer::verilog("ntp_server");
    assert!(v.contains("module ntp_server"), "{v}");
}
