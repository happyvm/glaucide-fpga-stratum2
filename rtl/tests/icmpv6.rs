// SPDX-License-Identifier: Apache-2.0
//! Whole-chain test of task T3.3: eth_rx (IPv6 channels) -> ipv6_rx ->
//! icmpv6 -> ipv6_tx -> tx_arb source 3, registered channels between all
//! units, against `ping6.hex`, `ndp_ns_na.hex`, `neg_ndp_hoplimit.hex`, plus
//! echo / NDP cases built here (references written independently of the RTL).
use ntp_fpga::eth_rx::{EthMeta, EthRx};
use ntp_fpga::eth_tx::EthHdr;
use ntp_fpga::icmpv6::{Icmpv6, NdLearn};
use ntp_fpga::ipv6_rx::{Ip6Meta, Ipv6Rx};
use ntp_fpga::ipv6_tx::{Ip6TxReq, Ipv6Tx};
use ntp_fpga::stream::{FrameByte, Ts};
use ntp_fpga::tx_arb::TxArb;
use txhdl::comp::{chan, join2, signal, DefaultClock, Running, Unit};
use txhdl::types::{Bit, U};

const MAC: u64 = 0x0200_0000_0001;
const PEER_MAC: u64 = 0x0200_0000_0002;
const LL: u128 = 0xfe80_0000_0000_0000_0000_00ff_fe00_0001;
const GUA: u128 = 0x2001_0db8_0000_0000_0000_0000_0000_0001;
const PEER: u128 = 0x2001_0db8_0000_0000_0000_0000_0000_0002;
const PEER_LL: u128 = 0xfe80_0000_0000_0000_0000_00ff_fe00_0002;
const OTHER: u128 = 0x2001_0db8_0000_0000_0000_0000_0000_0077;
const ALL_NODES: u128 = 0xff02_0000_0000_0000_0000_0000_0000_0001;

fn sn_of(a: u128) -> u128 {
    0xff02_0000_0000_0000_0000_0001_ff00_0000 | (a & 0xFF_FFFF)
}
fn sn_mac(a: u128) -> u64 {
    0x3333_ff00_0000 | (a & 0xFF_FFFF) as u64
}

#[derive(Debug)]
struct Sent {
    dst_mac: u64,
    ethertype: u16,
    bytes: Vec<FrameByte>,
}

impl Sent {
    fn data(&self) -> Vec<u8> {
        self.bytes.iter().map(|b| b.data.raw() as u8).collect()
    }
    fn err(&self) -> bool {
        self.bytes.last().unwrap().err.to_bool()
    }
}

/// Feed `frames` (full Ethernet frames, no FCS, optional err on last), `gap`
/// idle cycles between bytes; `take(t)` is the accept of the final sink.
fn chain(
    frames: &[(Vec<u8>, bool)],
    gap: usize,
    gua_valid: bool,
    take: impl Fn(usize) -> bool,
) -> Vec<Sent> {
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
    mac_o.set(U::<48>::from(MAC));
    sn_o.set(U::<24>::from(1u32));
    ll_o.set(U::<128>::from(LL));
    gua_o.set(U::<128>::from(GUA));
    gv_o.set(Bit::from_bool(gua_valid));
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
    // Neighbor learning is tied off here (drained below); no resolve requests.
    let (rs_t, rs_r) = chan::<U<128>, DefaultClock>();
    let (ln_t, ln_r) = chan::<NdLearn, DefaultClock>();
    // One receive stamp per input frame (T1.13); the values are not used here.
    let (st_t, st_r) = chan::<Ts, DefaultClock>();
    let (mut d_eth, mut d_rx, mut d_icmp, mut d_tx, mut d_arb) = (
        EthRx::default(),
        Ipv6Rx::default(),
        Icmpv6::default(),
        Ipv6Tx::default(),
        TxArb::default(),
    );
    let mut sim = Running::new(join2(
        join2(
            d_eth.run((in_r, mac1, sn1, st_r), (am_t, ap_t, v4m_t, v4p_t, v6m_t, v6p_t)),
            d_rx.run((v6m, v6p, ll1, gua1, gv1, sn2), (im_t, i_t, um_t, u_t)),
        ),
        join2(
            join2(
                d_icmp.run((im, i, ll2, gua2, gv2, mac2, rs_r), (rq_t, op_t, ln_t)),
                d_tx.run((rq, op), (h3_t, p3_t)),
            ),
            d_arb.run((h0, p0, h1, p1, h2, p2, h3, p3), (oh_t, opl_t)),
        ),
    ));
    let _keep = (&rs_t, &h0_t, &p0_t, &h1_t, &p1_t, &h2_t, &p2_t);
    let mut q: Vec<Option<FrameByte>> = vec![];
    for (f, err) in frames {
        for (k, b) in f.iter().enumerate() {
            let last = k + 1 == f.len();
            q.push(Some(FrameByte::new(*b, last, last && *err)));
            for _ in 0..gap {
                q.push(None);
            }
        }
    }
    let mut sent: Vec<Sent> = vec![];
    let mut hdrs: Vec<(u64, u16)> = vec![];
    let mut open: Vec<FrameByte> = vec![];
    let mut next = 0;
    let mut next_st = 0;
    let total = q.len() * 6 + 3000;
    for t in 0..total {
        if next_st < frames.len() && st_t.ready().to_bool() {
            st_t.send(Ts::from(next_st as u64 + 1));
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
        let _ = (am.recv(), ap.recv(), v4m.recv(), v4p.recv());
        let _ = (um.recv(), u.recv(), ln_r.recv());
        if take(t) {
            if let Some(h) = oh.recv() {
                hdrs.push((h.dst.raw() as u64, h.ethertype.raw() as u16));
            }
            if let Some(b) = opl.recv() {
                assert!(hdrs.len() > sent.len(), "payload before hdr");
                open.push(b);
                if b.last.to_bool() {
                    let (d, e) = hdrs[sent.len()];
                    sent.push(Sent {
                        dst_mac: d,
                        ethertype: e,
                        bytes: std::mem::take(&mut open),
                    });
                }
            }
        }
        sim.cycle();
    }
    assert!(open.is_empty(), "a frame was left open");
    assert_eq!(hdrs.len(), sent.len(), "a hdr without its frame");
    sent
}

// ------------------------------------------------------------ builders

fn from_hex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

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

/// ICMPv6 message: type, code, 4 bytes after the checksum, body; checksum over
/// the pseudo-header of (src, dst).
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

/// Ethernet frame (padded to 60) carrying an ICMPv6 message.
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

fn echo_msg(ty: u8, id: u16, seq: u16, data: &[u8], src: u128, dst: u128) -> Vec<u8> {
    let mut rest = vec![(id >> 8) as u8, id as u8, (seq >> 8) as u8, seq as u8];
    rest.extend_from_slice(data);
    icmp6(ty, 0, &rest, src, dst)
}

fn ns_msg(target: u128, opts: &[u8], src: u128, dst: u128) -> Vec<u8> {
    let mut rest = vec![0, 0, 0, 0];
    rest.extend_from_slice(&target.to_be_bytes());
    rest.extend_from_slice(opts);
    icmp6(135, 0, &rest, src, dst)
}

fn slla(mac: u64) -> Vec<u8> {
    let mut o = vec![1, 1];
    o.extend_from_slice(&mac.to_be_bytes()[2..]);
    o
}

/// The reply packet (IPv6 header + message) the model builds.
fn echo_reply(src: u128, dst: u128, id: u16, seq: u16, data: &[u8]) -> Vec<u8> {
    let m = echo_msg(129, id, seq, data, dst, src);
    let mut p = ip6_header(dst, src, 64, m.len());
    p.extend_from_slice(&m);
    p
}

fn na_packet(target: u128, dst: u128, solicited: bool) -> Vec<u8> {
    let flags: u8 = if solicited { 0x60 } else { 0x20 };
    let mut rest = vec![flags, 0, 0, 0];
    rest.extend_from_slice(&target.to_be_bytes());
    rest.extend_from_slice(&[2, 1]);
    rest.extend_from_slice(&MAC.to_be_bytes()[2..]);
    let m = icmp6(136, 0, &rest, target, dst);
    let mut p = ip6_header(target, dst, 255, m.len());
    p.extend_from_slice(&m);
    p
}

/// Expected frame: (dst mac, packet).
type Want = (u64, Vec<u8>);

fn run_check(
    what: &str,
    frames: &[Vec<u8>],
    want: &[Want],
    gua_valid: bool,
    gap: usize,
    take: impl Fn(usize) -> bool,
) {
    let fr: Vec<(Vec<u8>, bool)> = frames.iter().map(|f| (f.clone(), false)).collect();
    let o = chain(&fr, gap, gua_valid, take);
    assert_eq!(o.len(), want.len(), "{what}: number of replies");
    for (k, (s, (mac, pkt))) in o.iter().zip(want).enumerate() {
        assert_eq!(s.ethertype, 0x86dd, "{what} #{k}");
        assert_eq!(s.dst_mac, *mac, "{what} #{k}: dst mac");
        assert!(!s.err(), "{what} #{k}: err");
        assert_eq!(&s.data(), pkt, "{what} #{k}: packet");
    }
}

fn mac_of(f: &[u8]) -> u64 {
    u64::from_be_bytes([0, 0, f[0], f[1], f[2], f[3], f[4], f[5]])
}

// ------------------------------------------------------------ vectors

fn replay(name: &str, gap: usize, take: impl Fn(usize) -> bool) {
    let v = vector(name);
    let rx: Vec<Vec<u8>> = v.iter().filter(|(r, _)| *r).map(|(_, f)| f.clone()).collect();
    let want: Vec<Want> = v
        .iter()
        .filter(|(r, _)| !*r)
        .map(|(_, f)| (mac_of(f), f[14..].to_vec()))
        .collect();
    run_check(name, &rx, &want, true, gap, take);
}

#[test]
fn vectors_exact() {
    for name in ["ping6.hex", "ndp_ns_na.hex", "neg_ndp_hoplimit.hex"] {
        for gap in [0usize, 1] {
            replay(name, gap, |_| true);
        }
    }
}

#[test]
fn vectors_backpressure() {
    for name in ["ping6.hex", "ndp_ns_na.hex", "neg_ndp_hoplimit.hex"] {
        replay(name, 0, |t| t % 4 == 0 || t % 9 == 1);
        replay(name, 2, |t| t % 3 == 0);
    }
}

// ------------------------------------------------------------ echo

fn echo_case(dst: u128, dst_mac: u64, n: usize) {
    let data: Vec<u8> = (0..n).map(|i| (i * 13 + n) as u8).collect();
    let m = echo_msg(128, 0x1234, 7, &data, PEER, dst);
    let f = eth6(dst_mac, PEER, dst, 64, &m);
    run_check(
        &format!("echo {n}"),
        &[f],
        &[(PEER_MAC, echo_reply(PEER, dst, 0x1234, 7, &data))],
        true,
        0,
        |_| true,
    );
}

#[test]
fn echo_sizes() {
    for n in [0usize, 1, 2, 3, 32, 55, 1432] {
        echo_case(GUA, MAC, n);
    }
    echo_case(LL, MAC, 5);
}

#[test]
fn echo_checksum_sweep() {
    // Many ids/seqs: carries of the +0x0100 adjustment, 0x0000 and 0xFFFF.
    let mut frames = vec![];
    let mut want = vec![];
    for id in (0..=0xFFFFu32).step_by(0x0377) {
        for seq in [0u16, 0xfeff, 0xffff, 0x0100] {
            let m = echo_msg(128, id as u16, seq, &[1, 2, 3], PEER, GUA);
            frames.push(eth6(MAC, PEER, GUA, 64, &m));
            want.push((PEER_MAC, echo_reply(PEER, GUA, id as u16, seq, &[1, 2, 3])));
        }
    }
    run_check("sweep", &frames, &want, true, 0, |_| true);
}

#[test]
fn echo_bad_checksum_ends_with_err() {
    let data: Vec<u8> = (0..40u8).collect();
    let good = echo_msg(128, 7, 9, &data, PEER, GUA);
    for bad_at in [2usize, 3, 4, 8, 20, good.len() - 1] {
        let mut m = good.clone();
        m[bad_at] ^= 0x10;
        let fr = [
            (eth6(MAC, PEER, GUA, 64, &m), false),
            (eth6(MAC, PEER, GUA, 64, &good), false),
        ];
        let o = chain(&fr, 0, true, |_| true);
        assert_eq!(o.len(), 2, "byte {bad_at}");
        assert!(o[0].err(), "byte {bad_at}: aborted reply");
        let d = &o[0].bytes;
        assert!(d[..d.len() - 1]
            .iter()
            .all(|b| !b.err.to_bool() && !b.last.to_bool()));
        assert_eq!(d.len(), 40 + good.len());
        assert!(!o[1].err());
        assert_eq!(o[1].data(), echo_reply(PEER, GUA, 7, 9, &data));
    }
    // FCS error on the frame: the reply is flagged.
    let fr = [(eth6(MAC, PEER, GUA, 64, &good), true)];
    let o = chain(&fr, 0, true, |_| true);
    assert!(o.is_empty() || o[0].err());
}

#[test]
fn echo_ignored_cases() {
    let data = [1u8, 2, 3, 4];
    let good = eth6(MAC, PEER, GUA, 64, &echo_msg(128, 5, 5, &data, PEER, GUA));
    let mut frames = vec![];
    // Echo reply received, code 1, to the solicited-node group, too short (4
    // and 1 bytes), unknown type, bad type.
    frames.push(eth6(MAC, PEER, GUA, 64, &echo_msg(129, 1, 1, &data, PEER, GUA)));
    frames.push(eth6(
        MAC,
        PEER,
        GUA,
        64,
        &icmp6(128, 1, &[0, 1, 0, 1, 9], PEER, GUA),
    ));
    frames.push(eth6(
        sn_mac(GUA),
        PEER,
        sn_of(GUA),
        64,
        &echo_msg(128, 1, 1, &data, PEER, sn_of(GUA)),
    ));
    frames.push(eth6(MAC, PEER, GUA, 64, &icmp6(128, 0, &[], PEER, GUA)));
    frames.push(eth6(MAC, PEER, GUA, 64, &icmp6(1, 0, &[0, 0, 0, 0, 1, 2], PEER, GUA)));
    frames.push(good);
    run_check(
        "echo ignored",
        &frames,
        &[(PEER_MAC, echo_reply(PEER, GUA, 5, 5, &data))],
        true,
        0,
        |_| true,
    );
}

// ------------------------------------------------------------ NDP

/// One NS scenario: frame and the expected reply (None = nothing).
struct Ns {
    what: &'static str,
    frame: Vec<u8>,
    want: Option<Want>,
    gua_valid: bool,
}

fn ns_ll_mc(src: u128) -> Vec<u8> {
    eth6(
        sn_mac(LL),
        src,
        sn_of(LL),
        255,
        &ns_msg(LL, &slla(PEER_MAC), src, sn_of(LL)),
    )
}

fn ns_cases() -> Vec<Ns> {
    let mut v = vec![];
    let na_ll = (PEER_MAC, na_packet(LL, PEER_LL, true));
    let na_gua = (PEER_MAC, na_packet(GUA, PEER, true));
    let mut add = |what, frame, want: Option<Want>, gua_valid| {
        v.push(Ns {
            what,
            frame,
            want,
            gua_valid,
        })
    };
    add("ll via sn multicast", ns_ll_mc(PEER_LL), Some(na_ll.clone()), true);
    add(
        "ll unicast",
        eth6(MAC, PEER_LL, LL, 255, &ns_msg(LL, &slla(PEER_MAC), PEER_LL, LL)),
        Some(na_ll.clone()),
        false,
    );
    for gv in [true, false] {
        let w = if gv { Some(na_gua.clone()) } else { None };
        add(
            "gua unicast",
            eth6(MAC, PEER, GUA, 255, &ns_msg(GUA, &slla(PEER_MAC), PEER, GUA)),
            w.clone(),
            gv,
        );
        add(
            "gua via sn multicast",
            eth6(
                sn_mac(GUA),
                PEER,
                sn_of(GUA),
                255,
                &ns_msg(GUA, &slla(PEER_MAC), PEER, sn_of(GUA)),
            ),
            w,
            gv,
        );
    }
    add(
        "hop limit 254",
        eth6(
            sn_mac(LL),
            PEER_LL,
            sn_of(LL),
            254,
            &ns_msg(LL, &slla(PEER_MAC), PEER_LL, sn_of(LL)),
        ),
        None,
        true,
    );
    add(
        "hop limit 64",
        eth6(MAC, PEER_LL, LL, 64, &ns_msg(LL, &[], PEER_LL, LL)),
        None,
        true,
    );
    // Target not ours (but destination is ours / sn of ours).
    add(
        "target of someone else",
        eth6(MAC, PEER_LL, LL, 255, &ns_msg(OTHER, &[], PEER_LL, LL)),
        None,
        true,
    );
    add(
        "target multicast",
        eth6(MAC, PEER_LL, LL, 255, &ns_msg(ALL_NODES, &[], PEER_LL, LL)),
        None,
        true,
    );
    // Destination unicast to our other address than the target.
    add(
        "dst is not target nor its sn (ll target sent to gua)",
        eth6(MAC, PEER, GUA, 255, &ns_msg(LL, &[], PEER, GUA)),
        None,
        true,
    );
    // No options: the reply goes to the Ethernet source.
    add(
        "no options",
        eth6(MAC, PEER_LL, LL, 255, &ns_msg(LL, &[], PEER_LL, LL)),
        Some(na_ll.clone()),
        true,
    );
    // SLLA different from the Ethernet source: the model replies there.
    add(
        "slla differs",
        eth6(
            MAC,
            PEER_LL,
            LL,
            255,
            &ns_msg(LL, &slla(0x0200_0000_00aa), PEER_LL, LL),
        ),
        Some((0x0200_0000_00aa, na_packet(LL, PEER_LL, true))),
        true,
    );
    // Other option before SLLA (MTU, type 5, 1 unit), and a long unknown one.
    let mut o = vec![5, 1, 0, 0, 0, 0, 5, 0xdc];
    o.extend_from_slice(&slla(PEER_MAC));
    add(
        "mtu option then slla",
        eth6(MAC, PEER_LL, LL, 255, &ns_msg(LL, &o, PEER_LL, LL)),
        Some(na_ll.clone()),
        true,
    );
    let mut o = vec![0x99, 3];
    o.extend_from_slice(&[7u8; 22]);
    o.extend_from_slice(&slla(0x0200_0000_00bb));
    add(
        "unknown 24-byte option then slla",
        eth6(MAC, PEER_LL, LL, 255, &ns_msg(LL, &o, PEER_LL, LL)),
        Some((0x0200_0000_00bb, na_packet(LL, PEER_LL, true))),
        true,
    );
    // Malformed options.
    add(
        "option length 0",
        eth6(MAC, PEER_LL, LL, 255, &ns_msg(LL, &[1, 0, 2, 0, 0, 0, 0, 2], PEER_LL, LL)),
        None,
        true,
    );
    add(
        "option overruns",
        eth6(MAC, PEER_LL, LL, 255, &ns_msg(LL, &[1, 2, 2, 0, 0, 0, 0, 2], PEER_LL, LL)),
        None,
        true,
    );
    add(
        "one stray byte",
        eth6(MAC, PEER_LL, LL, 255, &ns_msg(LL, &[0], PEER_LL, LL)),
        None,
        true,
    );
    add(
        "stray type+len, no body",
        eth6(MAC, PEER_LL, LL, 255, &ns_msg(LL, &[1, 1], PEER_LL, LL)),
        None,
        true,
    );
    let mut o = slla(PEER_MAC);
    o.extend_from_slice(&slla(PEER_MAC));
    add(
        "two slla",
        eth6(MAC, PEER_LL, LL, 255, &ns_msg(LL, &o, PEER_LL, LL)),
        None,
        true,
    );
    let mut o = vec![1, 2];
    o.extend_from_slice(&[2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]);
    add(
        "slla of length 2",
        eth6(MAC, PEER_LL, LL, 255, &ns_msg(LL, &o, PEER_LL, LL)),
        None,
        true,
    );
    // Short NS: 20 bytes (target cut), and 8 bytes.
    let mut short = icmp6(135, 0, &[0, 0, 0, 0, 0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], PEER_LL, LL);
    add(
        "20 bytes",
        eth6(MAC, PEER_LL, LL, 255, &short),
        None,
        true,
    );
    short = icmp6(135, 0, &[0, 0, 0, 0], PEER_LL, LL);
    add("8 bytes", eth6(MAC, PEER_LL, LL, 255, &short), None, true);
    // Code != 0.
    let mut rest = vec![0, 0, 0, 0];
    rest.extend_from_slice(&LL.to_be_bytes());
    add(
        "code 1",
        eth6(MAC, PEER_LL, LL, 255, &icmp6(135, 1, &rest, PEER_LL, LL)),
        None,
        true,
    );
    // Bad checksum, at several places.
    for at in [2usize, 3, 9, 23] {
        let mut m = ns_msg(LL, &slla(PEER_MAC), PEER_LL, LL);
        m[at] ^= 0x04;
        add(
            "bad checksum",
            eth6(MAC, PEER_LL, LL, 255, &m),
            None,
            true,
        );
    }
    // DAD: source ::, to the solicited-node address of the target.
    let dad_ll = (0x3333_0000_0001u64, na_packet(LL, ALL_NODES, false));
    add(
        "DAD ll",
        eth6(sn_mac(LL), 0, sn_of(LL), 255, &ns_msg(LL, &[], 0, sn_of(LL))),
        Some(dad_ll.clone()),
        true,
    );
    add(
        "DAD gua",
        eth6(sn_mac(GUA), 0, sn_of(GUA), 255, &ns_msg(GUA, &[], 0, sn_of(GUA))),
        Some((0x3333_0000_0001u64, na_packet(GUA, ALL_NODES, false))),
        true,
    );
    add(
        "DAD gua, gua not valid",
        eth6(sn_mac(GUA), 0, sn_of(GUA), 255, &ns_msg(GUA, &[], 0, sn_of(GUA))),
        None,
        false,
    );
    add(
        "DAD with slla",
        eth6(
            sn_mac(LL),
            0,
            sn_of(LL),
            255,
            &ns_msg(LL, &slla(PEER_MAC), 0, sn_of(LL)),
        ),
        None,
        true,
    );
    add(
        "DAD to the unicast target",
        eth6(MAC, 0, LL, 255, &ns_msg(LL, &[], 0, LL)),
        None,
        true,
    );
    add(
        "DAD hop limit 1",
        eth6(sn_mac(LL), 0, sn_of(LL), 1, &ns_msg(LL, &[], 0, sn_of(LL))),
        None,
        true,
    );
    // NA / RA / Redirect / other received.
    let mut na_rest = vec![0x60, 0, 0, 0];
    na_rest.extend_from_slice(&PEER_LL.to_be_bytes());
    let ra_rest = [64u8, 0, 0x07, 0x08, 0, 0, 0, 0, 0, 0, 0, 0];
    let mut redir = vec![0, 0, 0, 0];
    redir.extend_from_slice(&PEER_LL.to_be_bytes());
    redir.extend_from_slice(&PEER_LL.to_be_bytes());
    add(
        "NA received",
        eth6(MAC, PEER_LL, LL, 255, &icmp6(136, 0, &na_rest, PEER_LL, LL)),
        None,
        true,
    );
    add(
        "RA received (unicast)",
        eth6(MAC, PEER_LL, LL, 255, &icmp6(134, 0, &ra_rest, PEER_LL, LL)),
        None,
        true,
    );
    add(
        "Redirect received",
        eth6(MAC, PEER_LL, LL, 255, &icmp6(137, 0, &redir, PEER_LL, LL)),
        None,
        true,
    );
    add(
        "dest unreachable",
        eth6(MAC, PEER_LL, LL, 255, &icmp6(1, 3, &[0, 0, 0, 0, 1, 2, 3], PEER_LL, LL)),
        None,
        true,
    );
    v
}

#[test]
fn ns_table() {
    // Each case alone, then followed by a good sentinel NS (the unit must
    // recover and answer it).
    let sentinel = ns_ll_mc(PEER_LL);
    let sent_want = (PEER_MAC, na_packet(LL, PEER_LL, true));
    for c in ns_cases() {
        let mut want = vec![];
        if let Some(w) = &c.want {
            want.push(w.clone());
        }
        run_check(c.what, &[c.frame.clone()], &want, c.gua_valid, 0, |_| true);
        want.push(sent_want.clone());
        run_check(
            c.what,
            &[c.frame.clone(), sentinel.clone()],
            &want,
            c.gua_valid,
            0,
            |_| true,
        );
    }
}

#[test]
fn ns_back_to_back_gaps_backpressure() {
    // All cases with gua_valid = true in a row (the ones that need it false
    // are skipped), with gaps and a stalling sink.
    let mut frames = vec![];
    let mut want = vec![];
    for c in ns_cases().into_iter().filter(|c| c.gua_valid) {
        frames.push(c.frame);
        if let Some(w) = c.want {
            want.push(w);
        }
    }
    for k in 0..3 {
        let m = echo_msg(128, k, k, &[k as u8; 9], PEER, GUA);
        frames.push(eth6(MAC, PEER, GUA, 64, &m));
        want.push((PEER_MAC, echo_reply(PEER, GUA, k, k, &[k as u8; 9])));
        frames.push(ns_ll_mc(PEER_LL));
        want.push((PEER_MAC, na_packet(LL, PEER_LL, true)));
    }
    run_check("all, gap 0", &frames, &want, true, 0, |_| true);
    run_check("all, gap 1", &frames, &want, true, 1, |_| true);
    run_check("all, stall", &frames, &want, true, 0, |t| t % 3 != 0);
    run_check("all, hard stall", &frames, &want, true, 0, |t| t % 7 == 0);
}

#[test]
fn ns_frame_err_gives_nothing() {
    let fr = [(ns_ll_mc(PEER_LL), true), (ns_ll_mc(PEER_LL), false)];
    let o = chain(&fr, 0, true, |_| true);
    assert_eq!(o.len(), 1);
    assert!(!o[0].err());
}

// ------------------------------------------------------------ Verilog lowering

#[test]
fn lowers_to_verilog() {
    let v = Icmpv6::verilog("icmpv6");
    assert!(v.contains("module icmpv6"), "icmpv6");
}
