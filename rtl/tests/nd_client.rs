// SPDX-License-Identifier: Apache-2.0
//! Task T3.6: IPv6 neighbor discovery as an initiator. Chain eth_rx ->
//! ipv6_rx -> icmpv6 -> ipv6_tx -> tx_arb (source 3) with `nbr_cache` fed by
//! the `learn` output of icmpv6; references are built here by an independent
//! byte-level implementation and by literals computed with the Python model
//! (`tools/golden/responder.py::solicit`, cross-checked with scapy).
use ntp_fpga::arp::ArpEntry;
use ntp_fpga::eth_rx::{EthMeta, EthRx};
use ntp_fpga::eth_tx::EthHdr;
use ntp_fpga::icmpv6::{Icmpv6, NdLearn};
use ntp_fpga::ipv6_rx::{Ip6Meta, Ipv6Rx};
use ntp_fpga::ipv6_tx::{Ip6TxReq, Ipv6Tx};
use ntp_fpga::nbr_cache::NbrCache;
use ntp_fpga::stream::{FrameByte, Ts};
use ntp_fpga::tx_arb::TxArb;
use txhdl::comp::{chan, join2, signal, DefaultClock, Running, Unit};
use txhdl::types::{Bit, U};

const MAC: u64 = 0x0200_0000_0001;
const PEER_MAC: u64 = 0x0200_0000_0002;
const AA_MAC: u64 = 0x0200_0000_00aa;
const LL: u128 = 0xfe80_0000_0000_0000_0000_00ff_fe00_0001;
const GUA: u128 = 0x2001_0db8_0000_0000_0000_0000_0000_0001;
const PEER: u128 = 0x2001_0db8_0000_0000_0000_0000_0000_0002;
const PEER_LL: u128 = 0xfe80_0000_0000_0000_0000_00ff_fe00_0002;
const FAR: u128 = 0x2001_0db8_1234_5678_9abc_def0_1234_5678;
const ALL_NODES: u128 = 0xff02_0000_0000_0000_0000_0000_0000_0001;

fn sn_of(a: u128) -> u128 {
    0xff02_0000_0000_0000_0000_0001_ff00_0000 | (a & 0xFF_FFFF)
}
fn sn_mac(a: u128) -> u64 {
    0x3333_ff00_0000 | (a & 0xFF_FFFF) as u64
}

// ------------------------------------------------------------ references

fn cksum_sum(b: &[u8]) -> u16 {
    let mut s: u32 = 0;
    for c in b.chunks(2) {
        s += ((c[0] as u32) << 8) | *c.get(1).unwrap_or(&0) as u32;
    }
    while s >> 16 != 0 {
        s = (s & 0xFFFF) + (s >> 16);
    }
    s as u16
}

fn icmp6(ty: u8, code: u8, rest: &[u8], src: u128, dst: u128) -> Vec<u8> {
    let mut m = vec![ty, code, 0, 0];
    m.extend_from_slice(rest);
    let mut p = src.to_be_bytes().to_vec();
    p.extend_from_slice(&dst.to_be_bytes());
    p.extend_from_slice(&(m.len() as u32).to_be_bytes());
    p.extend_from_slice(&[0, 0, 0, 58]);
    p.extend_from_slice(&m);
    let c = !cksum_sum(&p);
    m[2] = (c >> 8) as u8;
    m[3] = c as u8;
    m
}

fn ip6_header(src: u128, dst: u128, hop: u8, len: usize) -> Vec<u8> {
    let mut h = vec![0x60, 0, 0, 0, (len >> 8) as u8, len as u8, 58, hop];
    h.extend_from_slice(&src.to_be_bytes());
    h.extend_from_slice(&dst.to_be_bytes());
    h
}

fn eth6(dst_mac: u64, src: u128, dst: u128, hop: u8, msg: &[u8]) -> Vec<u8> {
    let mut f = dst_mac.to_be_bytes()[2..].to_vec();
    f.extend_from_slice(&PEER_MAC.to_be_bytes()[2..]);
    f.extend_from_slice(&[0x86, 0xdd]);
    f.extend_from_slice(&ip6_header(src, dst, hop, msg.len()));
    f.extend_from_slice(msg);
    while f.len() < 60 {
        f.push(0);
    }
    f
}

fn opt(t: u8, mac: u64) -> Vec<u8> {
    let mut o = vec![t, 1];
    o.extend_from_slice(&mac.to_be_bytes()[2..]);
    o
}

fn ns_msg(target: u128, opts: &[u8], src: u128, dst: u128) -> Vec<u8> {
    let mut rest = vec![0, 0, 0, 0];
    rest.extend_from_slice(&target.to_be_bytes());
    rest.extend_from_slice(opts);
    icmp6(135, 0, &rest, src, dst)
}

fn na_msg(target: u128, flags: u8, opts: &[u8], src: u128, dst: u128) -> Vec<u8> {
    let mut rest = vec![flags, 0, 0, 0];
    rest.extend_from_slice(&target.to_be_bytes());
    rest.extend_from_slice(opts);
    icmp6(136, 0, &rest, src, dst)
}

/// NA frame from the peer to us, as a reply to a solicitation.
fn na_frame(target: u128, opts: &[u8], dst: u128) -> Vec<u8> {
    let m = na_msg(target, 0x60, opts, target, dst);
    eth6(MAC, target, dst, 255, &m)
}

/// Expected solicitation: (dst mac, IPv6 packet).
fn exp_ns(target: u128, src: u128) -> (u64, Vec<u8>) {
    let dst = sn_of(target);
    let m = ns_msg(target, &opt(1, MAC), src, dst);
    assert_eq!(m.len(), 32);
    let mut p = ip6_header(src, dst, 255, 32);
    p.extend_from_slice(&m);
    (sn_mac(target), p)
}

/// NA the unit answers a solicitation with (target = us).
fn exp_na(target: u128, dst: u128, dst_mac: u64) -> (u64, Vec<u8>) {
    let m = na_msg(target, 0x60, &opt(2, MAC), target, dst);
    let mut p = ip6_header(target, dst, 255, m.len());
    p.extend_from_slice(&m);
    (dst_mac, p)
}

fn from_hex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

// ------------------------------------------------------------ bench

#[derive(Debug)]
struct Sent {
    dst_mac: u64,
    ethertype: u16,
    data: Vec<u8>,
    err: bool,
}

#[derive(Default, Clone)]
struct Bench {
    frames: Vec<(usize, Vec<u8>)>,
    resolves: Vec<(usize, u128)>,
    lookups: Vec<(usize, u128)>,
    flush: Vec<usize>,
    gua_off: bool,
    cycles: usize,
}

#[derive(Debug, Default)]
struct Out {
    sent: Vec<Sent>,
    /// (cycle, ip, mac) seen on icmpv6's `learn` output.
    learns: Vec<(usize, u128, u64)>,
    /// (cycle, found, mac) of the cache.
    rsp: Vec<(usize, bool, u64)>,
}

fn run(b: &Bench) -> Out {
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
    mac_o.set(U::<48>::from(MAC));
    sn_o.set(U::<24>::from(1u32));
    ll_o.set(U::<128>::from(LL));
    gua_o.set(U::<128>::from(GUA));
    gv_o.set(Bit::from_bool(!b.gua_off));
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
    let (st_t, st_r) = chan::<Ts, DefaultClock>();
    let (rs_t, rs_r) = chan::<U<128>, DefaultClock>();
    let (ln_t, ln_r) = chan::<NdLearn, DefaultClock>();
    let (cl_t, cl_r) = chan::<NdLearn, DefaultClock>();
    let (lk_t, lk_r) = chan::<U<128>, DefaultClock>();
    let (rsp_t, rsp_r) = chan::<ArpEntry, DefaultClock>();
    let (mut d_eth, mut d_rx, mut d_icmp, mut d_tx, mut d_arb, mut d_cache) = (
        EthRx::default(),
        Ipv6Rx::default(),
        Icmpv6::default(),
        Ipv6Tx::default(),
        TxArb::default(),
        NbrCache::default(),
    );
    let mut sim = Running::new(join2(
        join2(
            d_eth.run((in_r, mac1, sn1, st_r), (am_t, ap_t, v4m_t, v4p_t, v6m_t, v6p_t)),
            d_rx.run((v6m, v6p, ll1, gua1, gv1, sn2), (im_t, i_t, um_t, u_t)),
        ),
        join2(
            join2(
                d_icmp.run((im, i, ll2, gua2, gv2, mac2, rs_r), (rq_t, op_t, ln_t)),
                join2(
                    d_tx.run((rq, op), (h3_t, p3_t)),
                    d_cache.run((cl_r, lk_r, fl), (rsp_t,)),
                ),
            ),
            d_arb.run((h0, p0, h1, p1, h2, p2, h3, p3), (oh_t, opl_t)),
        ),
    ));
    let _keep = (&h0_t, &p0_t, &h1_t, &p1_t, &h2_t, &p2_t);
    let mut out = Out::default();
    let mut hdrs: Vec<(u64, u16)> = vec![];
    let mut open: Vec<FrameByte> = vec![];
    let (mut fi, mut bi, mut ri, mut li, mut stamps) = (0usize, 0usize, 0usize, 0usize, 0u64);
    let mut fwd: Option<NdLearn> = None;
    for t in 0..b.cycles {
        fl_o.set(Bit::from_bool(b.flush.contains(&t)));
        if stamps < b.frames.len() as u64 && st_t.ready().to_bool() {
            stamps += 1;
            st_t.send(Ts::from(stamps));
        }
        if fi < b.frames.len() && t >= b.frames[fi].0 && in_t.ready().to_bool() {
            let f = &b.frames[fi].1;
            let last = bi + 1 == f.len();
            in_t.send(FrameByte::new(f[bi], last, false));
            bi += 1;
            if last {
                fi += 1;
                bi = 0;
            }
        }
        if ri < b.resolves.len() && t >= b.resolves[ri].0 && rs_t.ready().to_bool() {
            rs_t.send(U::<128>::from(b.resolves[ri].1));
            ri += 1;
        }
        if li < b.lookups.len() && t >= b.lookups[li].0 && lk_t.ready().to_bool() {
            lk_t.send(U::<128>::from(b.lookups[li].1));
            li += 1;
        }
        let _ = (am.recv(), ap.recv(), v4m.recv(), v4p.recv(), um.recv(), u.recv());
        // icmpv6.learn -> cache.learn (recorded on the way).
        if fwd.is_none() {
            if let Some(l) = ln_r.recv() {
                out.learns.push((t, l.ip.raw(), l.mac.raw() as u64));
                fwd = Some(l);
            }
        }
        if let Some(l) = fwd {
            if cl_t.ready().to_bool() {
                cl_t.send(l);
                fwd = None;
            }
        }
        if let Some(r) = rsp_r.recv() {
            out.rsp.push((t, r.found.to_bool(), r.mac.raw() as u64));
        }
        if let Some(h) = oh.recv() {
            hdrs.push((h.dst.raw() as u64, h.ethertype.raw() as u16));
        }
        if let Some(x) = opl.recv() {
            assert!(hdrs.len() > out.sent.len(), "payload before hdr");
            open.push(x);
            if x.last.to_bool() {
                let (d, e) = hdrs[out.sent.len()];
                let data = open.iter().map(|b| b.data.raw() as u8).collect();
                out.sent.push(Sent { dst_mac: d, ethertype: e, data, err: x.err.to_bool() });
                open.clear();
            }
        }
        sim.cycle();
    }
    assert!(open.is_empty(), "a frame was left open");
    assert_eq!(hdrs.len(), out.sent.len(), "a hdr without its frame");
    out
}

fn check_frames(what: &str, got: &[Sent], want: &[(u64, Vec<u8>)]) {
    assert_eq!(got.len(), want.len(), "{what}: number of frames");
    for (k, (s, (mac, pkt))) in got.iter().zip(want).enumerate() {
        assert_eq!(s.ethertype, 0x86dd, "{what} #{k}");
        assert_eq!(s.dst_mac, *mac, "{what} #{k}: dst mac");
        assert!(!s.err, "{what} #{k}: err");
        assert_eq!(&s.data, pkt, "{what} #{k}: packet");
    }
}

fn bench(frames: Vec<(usize, Vec<u8>)>, resolves: Vec<(usize, u128)>, cycles: usize) -> Bench {
    Bench { frames, resolves, cycles, ..Default::default() }
}

// ------------------------------------------------------------ NS generation

#[test]
fn ns_literals_from_python_model() {
    // tools/golden/responder.py::solicit (checked against scapy), full frames.
    let lit = [
        (PEER, "3333ff00000202000000000186dd6000000000203aff20010db8000000000000000000000001ff0200000000000000000001ff00000287001c270000000020010db80000000000000000000000020101020000000001"),
        (PEER_LL, "3333ff00000202000000000186dd6000000000203afffe80000000000000000000fffe000001ff0200000000000000000001ff00000287007c9700000000fe80000000000000000000fffe0000020101020000000001"),
        (FAR, "3333ff34567802000000000186dd6000000000203aff20010db8000000000000000000000001ff0200000000000000000001ff34567887007a780000000020010db8123456789abcdef0123456780101020000000001"),
    ];
    let o = run(&bench(vec![], lit.iter().map(|(t, _)| (0usize, *t)).collect(), 400));
    let want: Vec<(u64, Vec<u8>)> = lit
        .iter()
        .map(|(_, h)| {
            let f = from_hex(h);
            (u64::from_be_bytes([0, 0, f[0], f[1], f[2], f[3], f[4], f[5]]), f[14..].to_vec())
        })
        .collect();
    check_frames("python literals", &o.sent, &want);
    // The independent Rust reference agrees with the literals.
    assert_eq!(want[0], exp_ns(PEER, GUA));
    assert_eq!(want[1], exp_ns(PEER_LL, LL));
    assert_eq!(want[2], exp_ns(FAR, GUA));
}

#[test]
fn ns_source_selection() {
    // Global target: global source when valid; link-local target: link-local;
    // global target without a valid global address: link-local.
    let o = run(&bench(vec![], vec![(0, PEER), (0, PEER_LL)], 300));
    check_frames("gua", &o.sent, &[exp_ns(PEER, GUA), exp_ns(PEER_LL, LL)]);
    let mut b = bench(vec![], vec![(0, PEER)], 200);
    b.gua_off = true;
    check_frames("no gua", &run(&b).sent, &[exp_ns(PEER, LL)]);
}

#[test]
fn ns_all_solicited_node_bytes() {
    // Several targets with different low 24 bits (multicast address and MAC).
    let ts = [0x2001_0db8u128 << 96 | 0x00ab_cdef, 0x2001_0db8u128 << 96 | 0xff00_0001, 0xfe80u128 << 112 | 0x0102_0304];
    let o = run(&bench(vec![], ts.iter().map(|t| (0usize, *t)).collect(), 500));
    let want: Vec<_> = ts.iter().map(|t| exp_ns(*t, if *t >> 112 == 0xfe80 { LL } else { GUA })).collect();
    check_frames("targets", &o.sent, &want);
}

#[test]
fn resolve_of_own_address_is_dropped() {
    let o = run(&bench(vec![], vec![(0, LL), (0, GUA), (0, PEER)], 300));
    check_frames("own dropped", &o.sent, &[exp_ns(PEER, GUA)]);
    // GUA is not "ours" while gua_valid is clear: it is resolved.
    let mut b = bench(vec![], vec![(0, GUA)], 200);
    b.gua_off = true;
    check_frames("gua invalid", &run(&b).sent, &[exp_ns(GUA, LL)]);
}

fn echo_req_frame() -> Vec<u8> {
    let mut rest = vec![0x12, 0x34, 0, 7];
    rest.extend_from_slice(&[0xa5; 40]);
    let m = icmp6(128, 0, &rest, PEER, GUA);
    eth6(MAC, PEER, GUA, 64, &m)
}

fn echo_reply_packet() -> Vec<u8> {
    let mut rest = vec![0x12, 0x34, 0, 7];
    rest.extend_from_slice(&[0xa5; 40]);
    let m = icmp6(129, 0, &rest, GUA, PEER);
    let mut p = ip6_header(GUA, PEER, 64, m.len());
    p.extend_from_slice(&m);
    p
}

#[test]
fn resolve_waits_while_busy() {
    // A resolve already waiting when the frame reaches the unit goes first and
    // the frame (held in the channels) is answered afterwards.
    let o = run(&bench(vec![(0, echo_req_frame())], vec![(0, PEER_LL)], 600));
    check_frames("idle", &o.sent, &[exp_ns(PEER_LL, LL), (PEER_MAC, echo_reply_packet())]);
    // Sweep the resolve over the whole reception and reply: frames are always
    // whole and complete; once the unit has started the message the echo reply
    // comes first (the resolve waits), both orders occur.
    let (mut echo_first, mut ns_first) = (0, 0);
    for at in (0..160).step_by(4) {
        let o = run(&bench(vec![(0, echo_req_frame())], vec![(at, PEER_LL)], 600));
        let (a, b) = (exp_ns(PEER_LL, LL), (PEER_MAC, echo_reply_packet()));
        if o.sent.first().map(|s| s.data == b.1) == Some(true) {
            echo_first += 1;
            check_frames("echo first", &o.sent, &[b, a]);
        } else {
            ns_first += 1;
            check_frames("ns first", &o.sent, &[a, b]);
        }
    }
    assert!(echo_first > 0 && ns_first > 0, "{echo_first} {ns_first}");
}

#[test]
fn resolve_waits_for_an_na_in_progress() {
    // NS for our global address from the peer, then a resolve in the middle of
    // the answer: the NA is complete and intact, then the solicitation.
    let ns = eth6(
        sn_mac(GUA),
        PEER,
        sn_of(GUA),
        255,
        &ns_msg(GUA, &opt(1, AA_MAC), PEER, sn_of(GUA)),
    );
    for at in [10usize, 25, 35, 45, 55] {
        let o = run(&bench(vec![(0, ns.clone())], vec![(at, PEER_LL)], 500));
        assert_eq!(o.sent.len(), 2, "resolve at {at}");
        let (m, p) = exp_na(GUA, PEER, AA_MAC);
        let na_first = o.sent[0].data == p;
        let (m2, p2) = exp_ns(PEER_LL, LL);
        if na_first {
            check_frames("na then ns", &o.sent, &[(m, p), (m2, p2)]);
        } else {
            // The resolve got in before the NS reached the unit: whole frames.
            check_frames("ns then na", &o.sent, &[(m2, p2), (m, p)]);
        }
    }
}

// ------------------------------------------------------------ learning from NA

/// Frames fed 120 cycles apart, starting at 200; `resolve` (a target) is
/// solicited at cycle 0 (F5: a Neighbor Advertisement is learnt only as the
/// answer to our own solicitation).
fn learns_for(resolve: Option<u128>, frames: Vec<Vec<u8>>) -> Out {
    let n = frames.len();
    run(&bench(
        frames.into_iter().enumerate().map(|(k, f)| (200 + k * 120, f)).collect(),
        resolve.into_iter().map(|t| (0, t)).collect(),
        200 + n * 120 + 300,
    ))
}

fn learns(frames: Vec<Vec<u8>>) -> Out {
    learns_for(None, frames)
}

/// Source address of the solicitation for `t`.
fn ns_src(t: u128) -> u128 {
    if t >> 118 == 0x3fa {
        LL
    } else {
        GUA
    }
}

#[test]
fn na_learning_table() {
    let tlla = opt(2, AA_MAC);
    let ok_msg = |t: u128, o: &[u8], d: u128| na_msg(t, 0x60, o, if d == LL { PEER_LL } else { PEER }, d);
    // (what, frame, target being resolved, expected learn)
    let mut bad_ck = eth6(MAC, PEER, GUA, 255, &ok_msg(PEER, &tlla, GUA));
    bad_ck[14 + 40 + 2] ^= 0x10;
    let mut bad_ck_last = eth6(MAC, PEER, GUA, 255, &ok_msg(PEER, &tlla, GUA));
    bad_ck_last[14 + 40 + 31] ^= 0x01;
    let mut code1 = ok_msg(PEER, &tlla, GUA);
    code1[1] = 1;
    // Short NA: 20 bytes (no complete target), valid checksum.
    let mut short_rest = vec![0x60, 0, 0, 0];
    short_rest.extend_from_slice(&PEER.to_be_bytes()[..12]);
    let short = icmp6(136, 0, &short_rest, PEER, GUA);
    // Bad options: length 0, dup TLLA, TLLA of length 2.
    let zero_len = ok_msg(PEER, &[2, 0, 0, 0, 0, 0, 0, 0], GUA);
    let mut dup = tlla.clone();
    dup.extend_from_slice(&opt(2, PEER_MAC));
    let mut long = vec![2, 2];
    long.extend_from_slice(&[0, 0, 0, 0, 0, 0xaa]);
    long.extend_from_slice(&[0; 8]);
    let cases: Vec<(&str, Vec<u8>, u128, Option<(u128, u64)>)> = vec![
        ("tlla to gua", eth6(MAC, PEER, GUA, 255, &ok_msg(PEER, &tlla, GUA)), PEER, Some((PEER, AA_MAC))),
        ("no tlla: eth src", eth6(MAC, PEER, GUA, 255, &ok_msg(PEER, &[], GUA)), PEER, Some((PEER, PEER_MAC))),
        ("tlla to ll", eth6(MAC, PEER_LL, LL, 255, &ok_msg(PEER_LL, &tlla, LL)), PEER_LL, Some((PEER_LL, AA_MAC))),
        ("S=1 R=1 O=0", eth6(MAC, PEER, GUA, 255, &na_msg(PEER, 0xc0, &[], PEER, GUA)), PEER, Some((PEER, PEER_MAC))),
        ("SLLA ignored in NA", eth6(MAC, PEER, GUA, 255, &ok_msg(PEER, &opt(1, AA_MAC), GUA)), PEER, Some((PEER, PEER_MAC))),
        ("unknown option skipped", eth6(MAC, PEER, GUA, 255, &ok_msg(PEER, &[9, 1, 0, 0, 0, 0, 0, 0], GUA)), PEER, Some((PEER, PEER_MAC))),
        // F5: not solicited.
        ("S=0 O=0", eth6(MAC, PEER, GUA, 255, &na_msg(PEER, 0, &tlla, PEER, GUA)), PEER, None),
        ("S=0 O=1", eth6(MAC, PEER, GUA, 255, &na_msg(PEER, 0x20, &tlla, PEER, GUA)), PEER, None),
        ("R flag only", eth6(MAC, PEER, GUA, 255, &na_msg(PEER, 0x80, &[], PEER, GUA)), PEER, None),
        ("target differs from source", eth6(MAC, PEER, GUA, 255, &ok_msg(FAR, &tlla, GUA)), FAR, None),
        ("NA for another target than the one resolved", eth6(MAC, PEER, GUA, 255, &ok_msg(PEER, &tlla, GUA)), PEER_LL, None),
        ("bad checksum (type word)", bad_ck, PEER, None),
        ("bad checksum (last byte)", bad_ck_last, PEER, None),
        ("hop limit 254", eth6(MAC, PEER, GUA, 254, &ok_msg(PEER, &tlla, GUA)), PEER, None),
        ("hop limit 64", eth6(MAC, PEER, GUA, 64, &ok_msg(PEER, &tlla, GUA)), PEER, None),
        ("multicast target", eth6(MAC, PEER, GUA, 255, &ok_msg(ALL_NODES, &tlla, GUA)), ALL_NODES, None),
        ("solicited-node target", eth6(MAC, PEER, GUA, 255, &ok_msg(sn_of(PEER), &[], GUA)), sn_of(PEER), None),
        ("code 1", eth6(MAC, PEER, GUA, 255, &code1), PEER, None),
        ("short NA", eth6(MAC, PEER, GUA, 255, &short), PEER, None),
        ("option length 0", eth6(MAC, PEER, GUA, 255, &zero_len), PEER, None),
        ("duplicate TLLA", eth6(MAC, PEER, GUA, 255, &ok_msg(PEER, &dup, GUA)), PEER, None),
        ("TLLA of length 2", eth6(MAC, PEER, GUA, 255, &ok_msg(PEER, &long, GUA)), PEER, None),
    ];
    for (what, f, resolving, want) in cases {
        // Each case alone, then the good NA for the same target right after
        // (the unit recovers; after a learn the second NA is ignored: one learn
        // per solicitation).
        let good = na_frame(resolving, &tlla, GUA);
        let o = learns_for(Some(resolving), vec![f, good]);
        check_frames(what, &o.sent, &[exp_ns(resolving, ns_src(resolving))]);
        let w = want.unwrap_or((resolving, AA_MAC));
        let got: Vec<(u128, u64)> = o.learns.iter().map(|(_, i, m)| (*i, *m)).collect();
        // A multicast or malformed case still ends with the valid NA: learnt
        // only if the target is not multicast.
        let exp = if what == "multicast target" || what == "solicited-node target" { vec![] } else { vec![w] };
        assert_eq!(got, exp, "{what}");
    }
}

#[test]
fn na_unsolicited_is_never_learnt() {
    // F5: no solicitation outstanding: a perfect NA teaches nothing, before, during
    // or after another resolution.
    let tlla = opt(2, AA_MAC);
    let o = learns(vec![na_frame(PEER, &tlla, GUA), na_frame(PEER_LL, &tlla, LL)]);
    assert!(o.learns.is_empty() && o.sent.is_empty());
    // A solicitation for PEER does not let an NA for FAR through.
    let o = learns_for(Some(PEER), vec![na_frame(FAR, &tlla, GUA), na_frame(PEER, &tlla, GUA)]);
    let got: Vec<(u128, u64)> = o.learns.iter().map(|(_, i, m)| (*i, *m)).collect();
    assert_eq!(got, vec![(PEER, AA_MAC)]);
}

#[test]
fn na_after_a_learn_is_ignored() {
    // One learn per solicitation: the entry is not rebound by a second solicited-looking NA.
    let o = learns_for(
        Some(PEER),
        vec![na_frame(PEER, &opt(2, AA_MAC), GUA), na_frame(PEER, &opt(2, PEER_MAC), GUA)],
    );
    let got: Vec<(u128, u64)> = o.learns.iter().map(|(_, i, m)| (*i, *m)).collect();
    assert_eq!(got, vec![(PEER, AA_MAC)]);
    // A new solicitation re-arms the unit.
    let b = Bench {
        frames: vec![(200, na_frame(PEER, &opt(2, AA_MAC), GUA)), (500, na_frame(PEER, &opt(2, PEER_MAC), GUA))],
        resolves: vec![(0, PEER), (400, PEER)],
        cycles: 900,
        ..Default::default()
    };
    let o = run(&b);
    let got: Vec<(u128, u64)> = o.learns.iter().map(|(_, i, m)| (*i, *m)).collect();
    assert_eq!(got, vec![(PEER, AA_MAC), (PEER, PEER_MAC)]);
}

#[test]
fn na_back_to_back_learning() {
    // Six NAs for different targets 0 cycles apart; only the one for the
    // target being resolved (the third) is learnt.
    let fr: Vec<(usize, Vec<u8>)> = (0..6u128)
        .map(|k| (200usize, na_frame(PEER + 16 * k, &opt(2, AA_MAC + k as u64), GUA)))
        .collect();
    let o = run(&bench(fr, vec![(0, PEER + 32)], 1700));
    let got: Vec<(u128, u64)> = o.learns.iter().map(|(_, i, m)| (*i, *m)).collect();
    assert_eq!(got, vec![(PEER + 32, AA_MAC + 2)]);
}

// ------------------------------------------------------------ NS never teaches (F5)

fn ns_frame(src: u128, dst: u128, target: u128, opts: &[u8], hop: u8) -> Vec<u8> {
    let dmac = if dst >> 120 == 0xff { sn_mac(dst) } else { MAC };
    eth6(dmac, src, dst, hop, &ns_msg(target, opts, src, dst))
}

#[test]
fn ns_learning_table() {
    let slla = opt(1, AA_MAC);
    // (what, frame, expected NA): F5, no case learns, answers are unchanged.
    let na = |dst, mac| Some(exp_na(GUA, dst, mac));
    let cases: Vec<(&str, Vec<u8>, Option<(u64, Vec<u8>)>)> = vec![
        ("multicast NS with SLLA", ns_frame(PEER, sn_of(GUA), GUA, &slla, 255), na(PEER, AA_MAC)),
        ("unicast NS with SLLA", ns_frame(PEER, GUA, GUA, &slla, 255), na(PEER, AA_MAC)),
        ("NS without SLLA", ns_frame(PEER, sn_of(GUA), GUA, &[], 255), na(PEER, PEER_MAC)),
        ("NS with TLLA only", ns_frame(PEER, sn_of(GUA), GUA, &opt(2, AA_MAC), 255), na(PEER, PEER_MAC)),
        ("DAD probe", ns_frame(0, sn_of(GUA), GUA, &[], 255), Some(exp_dad_na())),
        ("hop limit 254", ns_frame(PEER, sn_of(GUA), GUA, &slla, 254), None),
        ("not our target", ns_frame(PEER, sn_of(GUA), FAR, &slla, 255), None),
        ("duplicate SLLA", ns_frame(PEER, sn_of(GUA), GUA, &[slla.clone(), slla.clone()].concat(), 255), None),
    ];
    for (what, f, na) in cases {
        // Without and with an outstanding solicitation for the NS's own source.
        let o = learns(vec![f.clone()]);
        assert!(o.learns.is_empty(), "{what}: an NS teaches nothing");
        check_frames(what, &o.sent, &na.clone().into_iter().collect::<Vec<_>>());
        let o = learns_for(Some(PEER), vec![f]);
        assert!(o.learns.is_empty(), "{what}: an NS teaches nothing (resolving its source)");
        let mut want = vec![exp_ns(PEER, GUA)];
        want.extend(na);
        check_frames(what, &o.sent, &want);
    }
    // Bad checksum: neither answered nor learnt.
    let mut f = ns_frame(PEER, sn_of(GUA), GUA, &slla, 255);
    f[14 + 40 + 3] ^= 0x01;
    let o = learns(vec![f]);
    assert!(o.learns.is_empty() && o.sent.is_empty());
}

fn exp_dad_na() -> (u64, Vec<u8>) {
    let m = na_msg(GUA, 0x20, &opt(2, MAC), GUA, ALL_NODES);
    let mut p = ip6_header(GUA, ALL_NODES, 255, m.len());
    p.extend_from_slice(&m);
    (0x3333_0000_0001, p)
}

#[test]
fn ns_learning_through_link_local_target() {
    // Still answered, to the SLLA, but nothing is learnt.
    let f = ns_frame(PEER_LL, sn_of(LL), LL, &opt(1, AA_MAC), 255);
    let o = learns(vec![f]);
    assert!(o.learns.is_empty());
    check_frames("na", &o.sent, &[exp_na(LL, PEER_LL, AA_MAC)]);
}

// ------------------------------------------------------------ cache

#[derive(Default)]
struct CacheScript {
    learns: Vec<(usize, u128, u64)>,
    lookups: Vec<(usize, u128)>,
    flush: Vec<usize>,
}

/// Returns the responses (found, mac) in order.
fn cache_run(s: &CacheScript, cycles: usize, rsp_gap: usize) -> Vec<(bool, u64)> {
    let (fl_o, fl) = signal::<Bit, DefaultClock>();
    let (lt, lr) = chan::<NdLearn, DefaultClock>();
    let (kt, kr) = chan::<U<128>, DefaultClock>();
    let (st, sr) = chan::<ArpEntry, DefaultClock>();
    let mut dut = NbrCache::default();
    let mut sim = Running::new(dut.run((lr, kr, fl), (st,)));
    let (mut li, mut ki) = (0, 0);
    let mut out = vec![];
    for t in 0..cycles {
        fl_o.set(Bit::from_bool(s.flush.contains(&t)));
        if li < s.learns.len() && t >= s.learns[li].0 && lt.ready().to_bool() {
            let (_, ip, mac) = s.learns[li];
            lt.send(NdLearn { ip: U::<128>::from(ip), mac: U::<48>::from(mac) });
            li += 1;
        }
        if ki < s.lookups.len() && t >= s.lookups[ki].0 && kt.ready().to_bool() {
            kt.send(U::<128>::from(s.lookups[ki].1));
            ki += 1;
        }
        if rsp_gap == 0 || t % (rsp_gap + 1) == 0 {
            if let Some(r) = sr.recv() {
                out.push((r.found.to_bool(), r.mac.raw() as u64));
            }
        }
        sim.cycle();
    }
    out
}

fn ip(n: u128) -> u128 {
    0x2001_0db8u128 << 96 | n
}

/// Sequence of (learn at 10*k) then lookups of all given addresses.
fn probe(learns: &[(u128, u64)], flush_after: Option<usize>, looks: &[u128]) -> Vec<(bool, u64)> {
    let mut s = CacheScript::default();
    for (k, (i, m)) in learns.iter().enumerate() {
        s.learns.push((10 + 10 * k, *i, *m));
    }
    let t0 = 40 + 10 * learns.len();
    if let Some(f) = flush_after {
        s.flush.push(10 + 10 * f + 5);
    }
    for (k, a) in looks.iter().enumerate() {
        s.lookups.push((t0 + 4 * k, *a));
    }
    cache_run(&s, t0 + 4 * looks.len() + 40, 0)
}

#[test]
fn cache_hit_and_miss() {
    let r = probe(&[(ip(1), 0x11), (ip(2), 0x22)], None, &[ip(1), ip(2), ip(3), ip(1)]);
    assert_eq!(r, vec![(true, 0x11), (true, 0x22), (false, 0), (true, 0x11)]);
    // Empty cache misses with mac 0.
    assert_eq!(probe(&[], None, &[ip(1)]), vec![(false, 0)]);
}

#[test]
fn cache_update_in_place() {
    // Re-learning an address changes its MAC and takes no extra entry: after
    // 1, 2, 1', 3, 4 the four addresses are all present (no eviction).
    let l = [(ip(1), 0x11), (ip(2), 0x22), (ip(1), 0x99), (ip(3), 0x33), (ip(4), 0x44)];
    let r = probe(&l, None, &[ip(1), ip(2), ip(3), ip(4)]);
    assert_eq!(r, vec![(true, 0x99), (true, 0x22), (true, 0x33), (true, 0x44)]);
}

#[test]
fn cache_round_robin_eviction() {
    // The fifth distinct address evicts the first, the sixth the second.
    let l: Vec<_> = (1..=5u128).map(|k| (ip(k), 0x10 * k as u64)).collect();
    let r = probe(&l, None, &[ip(1), ip(2), ip(3), ip(4), ip(5)]);
    assert_eq!(r, vec![(false, 0), (true, 0x20), (true, 0x30), (true, 0x40), (true, 0x50)]);
    let l: Vec<_> = (1..=6u128).map(|k| (ip(k), 0x10 * k as u64)).collect();
    let r = probe(&l, None, &[ip(1), ip(2), ip(3), ip(5), ip(6)]);
    assert_eq!(r, vec![(false, 0), (false, 0), (true, 0x30), (true, 0x50), (true, 0x60)]);
    // An update in place does not advance the pointer: 1,2,3,2',4,5 -> 5 evicts 1.
    let l = [(ip(1), 1), (ip(2), 2), (ip(3), 3), (ip(2), 0x22), (ip(4), 4), (ip(5), 5)];
    let r = probe(&l, None, &[ip(1), ip(2), ip(3), ip(4), ip(5)]);
    assert_eq!(r, vec![(false, 0), (true, 0x22), (true, 3), (true, 4), (true, 5)]);
}

#[test]
fn cache_flush() {
    let l = [(ip(1), 0x11), (ip(2), 0x22)];
    // Flush after the second learn: everything is gone; learning works again
    // and starts at entry 0 again.
    assert_eq!(probe(&l, Some(1), &[ip(1), ip(2)]), vec![(false, 0), (false, 0)]);
    let mut s = CacheScript::default();
    s.learns = vec![(5, ip(1), 1), (10, ip(2), 2), (40, ip(3), 3), (50, ip(4), 4), (60, ip(5), 5), (70, ip(6), 6)];
    s.flush = vec![20];
    s.lookups = (1..=6u128).map(|k| (100 + 5 * k as usize, ip(k))).collect();
    let r = cache_run(&s, 200, 0);
    // After the flush 3,4,5,6 fill entries 0..3 (rr restarts at 0): all present.
    assert_eq!(r, vec![(false, 0), (false, 0), (true, 3), (true, 4), (true, 5), (true, 6)]);
}

#[test]
fn cache_same_cycle_learn_and_lookup() {
    // Learn and lookup of the same address sent together: the lookup sees the
    // contents before the learn (documented), the next lookup hits.
    let s = CacheScript {
        learns: vec![(10, ip(7), 0x77)],
        lookups: vec![(10, ip(7)), (30, ip(7))],
        flush: vec![],
    };
    assert_eq!(cache_run(&s, 80, 0), vec![(false, 0), (true, 0x77)]);
}

#[test]
fn cache_flush_beats_learn_same_cycle() {
    // The flush cycle is chosen against the cycle the learn is processed in:
    // sweep it, the entry is either flushed (learn processed before or at the
    // flush) or absent (learn lost): never present.
    for f in 8..16 {
        let s = CacheScript {
            learns: vec![(10, ip(7), 0x77)],
            lookups: vec![(60, ip(7))],
            flush: vec![f],
        };
        let r = cache_run(&s, 100, 0);
        if f >= 11 {
            assert_eq!(r, vec![(false, 0)], "flush at {f}");
        }
    }
}

#[test]
fn cache_lookup_backpressure_keeps_order() {
    let mut s = CacheScript::default();
    s.learns = vec![(2, ip(1), 0x11), (3, ip(2), 0x22)];
    s.lookups = (0..8).map(|k| (10 + k, if k % 2 == 0 { ip(1) } else { ip(3) })).collect();
    let r = cache_run(&s, 200, 3);
    let want: Vec<(bool, u64)> = (0..8).map(|k| if k % 2 == 0 { (true, 0x11) } else { (false, 0) }).collect();
    assert_eq!(r, want);
}

// ------------------------------------------------------------ end to end

#[test]
fn resolve_na_learn_lookup() {
    // lookup (miss) -> resolve -> NS on the wire -> NA from the peer (built as
    // the reply to that NS) -> learn -> lookup (hit).
    let na = eth6(
        MAC,
        PEER,
        GUA,
        255,
        &na_msg(PEER, 0x60, &opt(2, AA_MAC), PEER, GUA),
    );
    let b = Bench {
        frames: vec![(300, na)],
        resolves: vec![(20, PEER)],
        lookups: vec![(0, PEER), (200, PEER), (700, PEER), (710, FAR)],
        cycles: 900,
        ..Default::default()
    };
    let o = run(&b);
    check_frames("ns", &o.sent, &[exp_ns(PEER, GUA)]);
    assert_eq!(o.learns.len(), 1);
    assert_eq!((o.learns[0].1, o.learns[0].2), (PEER, AA_MAC));
    let r: Vec<(bool, u64)> = o.rsp.iter().map(|(_, f, m)| (*f, *m)).collect();
    assert_eq!(r, vec![(false, 0), (false, 0), (true, AA_MAC), (false, 0)]);
    // The NS precedes the learn.
    assert!(o.learns[0].0 > 300);
}

#[test]
fn resolve_with_flush() {
    let na = na_frame(PEER, &opt(2, AA_MAC), GUA);
    let b = Bench {
        frames: vec![(100, na)],
        resolves: vec![(0, PEER)],
        lookups: vec![(400, PEER), (600, PEER)],
        flush: vec![500],
        cycles: 800,
        ..Default::default()
    };
    let r: Vec<(bool, u64)> = run(&b).rsp.iter().map(|(_, f, m)| (*f, *m)).collect();
    assert_eq!(r, vec![(true, AA_MAC), (false, 0)]);
}

// ------------------------------------------------------------ lowering

#[test]
fn lowers_to_verilog() {
    assert!(Icmpv6::verilog("icmpv6").contains("module icmpv6"));
    assert!(NbrCache::verilog("nbr_cache").contains("module nbr_cache"));
}
