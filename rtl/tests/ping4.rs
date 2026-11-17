// SPDX-License-Identifier: Apache-2.0
//! Whole-chain test of task T1.10: eth_rx (IPv4 channels) -> ipv4_rx ->
//! icmp_echo -> ipv4_tx -> tx_arb source 1, with registered channels between
//! all units, against `tools/golden/vectors/ping4.hex` and the IPv4 negative
//! vectors, plus ICMP corner cases built here.
use ntp_fpga::eth_rx::{EthMeta, EthRx};
use ntp_fpga::eth_tx::EthHdr;
use ntp_fpga::icmp_echo::IcmpEcho;
use ntp_fpga::ipv4_rx::{IpMeta, Ipv4Rx};
use ntp_fpga::ipv4_tx::{IpTxReq, Ipv4Tx};
use ntp_fpga::stream::FrameByte;
use ntp_fpga::tx_arb::TxArb;
use txhdl::comp::{chan, join2, signal, DefaultClock, Running, Unit};
use txhdl::types::U;

const MAC: u64 = 0x0200_0000_0001;
const CLIENT_MAC: [u8; 6] = [2, 0, 0, 0, 0, 2];
const LOCAL_IP: u32 = 0xC000_0201;

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

#[derive(Default)]
struct Chain {
    /// Frames out of tx_arb (hdr + payload), in order.
    sent: Vec<Sent>,
    /// UDP packets forwarded by ipv4_rx (payload bytes, clean ones only).
    udp: Vec<Vec<u8>>,
    udp_metas: usize,
}

/// Feed `frames` (full Ethernet frames without FCS, optionally with err on the
/// last byte), `gap` idle cycles between bytes; `take(t)` is the accept of the
/// final sink (hdr and payload).
fn chain(frames: &[(Vec<u8>, bool)], gap: usize, take: impl Fn(usize) -> bool) -> Chain {
    let (mac_o, mac) = signal::<U<48>, DefaultClock>();
    let (sn_o, sn) = signal::<U<24>, DefaultClock>();
    let (ip_o, ip1) = signal::<U<32>, DefaultClock>();
    let ip2 = ip1.clone();
    mac_o.set(U::<48>::from(MAC));
    sn_o.set(U::<24>::from(1u32));
    ip_o.set(U::<32>::from(LOCAL_IP));
    let (in_t, in_r) = chan::<FrameByte, DefaultClock>();
    let (am_t, am) = chan::<EthMeta, DefaultClock>();
    let (ap_t, ap) = chan::<FrameByte, DefaultClock>();
    let (v4m_t, v4m) = chan::<EthMeta, DefaultClock>();
    let (v4p_t, v4p) = chan::<FrameByte, DefaultClock>();
    let (v6m_t, v6m) = chan::<EthMeta, DefaultClock>();
    let (v6p_t, v6p) = chan::<FrameByte, DefaultClock>();
    let (im_t, im) = chan::<IpMeta, DefaultClock>();
    let (i_t, i) = chan::<FrameByte, DefaultClock>();
    let (um_t, um) = chan::<IpMeta, DefaultClock>();
    let (u_t, u) = chan::<FrameByte, DefaultClock>();
    let (rq_t, rq) = chan::<IpTxReq, DefaultClock>();
    let (op_t, op) = chan::<FrameByte, DefaultClock>();
    let (h1_t, h1) = chan::<EthHdr, DefaultClock>();
    let (p1_t, p1) = chan::<FrameByte, DefaultClock>();
    let (h0_t, h0) = chan::<EthHdr, DefaultClock>();
    let (p0_t, p0) = chan::<FrameByte, DefaultClock>();
    let (h2_t, h2) = chan::<EthHdr, DefaultClock>();
    let (p2_t, p2) = chan::<FrameByte, DefaultClock>();
    let (h3_t, h3) = chan::<EthHdr, DefaultClock>();
    let (p3_t, p3) = chan::<FrameByte, DefaultClock>();
    let (oh_t, oh) = chan::<EthHdr, DefaultClock>();
    let (opl_t, opl) = chan::<FrameByte, DefaultClock>();
    let (mut d_eth, mut d_rx, mut d_icmp, mut d_tx, mut d_arb) = (
        EthRx::default(),
        Ipv4Rx::default(),
        IcmpEcho::default(),
        Ipv4Tx::default(),
        TxArb::default(),
    );
    let mut sim = Running::new(join2(
        join2(
            d_eth.run((in_r, mac, sn), (am_t, ap_t, v4m_t, v4p_t, v6m_t, v6p_t)),
            d_rx.run((v4m, v4p, ip1), (im_t, i_t, um_t, u_t)),
        ),
        join2(
            join2(
                d_icmp.run((im, i), (rq_t, op_t)),
                d_tx.run((rq, op, ip2), (h1_t, p1_t)),
            ),
            d_arb.run((h0, p0, h1, p1, h2, p2, h3, p3), (oh_t, opl_t)),
        ),
    ));
    let _keep = (&h0_t, &p0_t, &h2_t, &p2_t, &h3_t, &p3_t);
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
    let mut out = Chain::default();
    let mut hdrs: Vec<(u64, u16)> = vec![];
    let mut open: Vec<FrameByte> = vec![];
    let mut udp_open: Vec<u8> = vec![];
    let mut next = 0;
    let total = q.len() * 6 + 3000;
    for t in 0..total {
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
        // Sinks: ARP / IPv6 outputs of eth_rx, UDP output of ipv4_rx.
        let _ = (am.recv(), ap.recv(), v6m.recv(), v6p.recv());
        if um.recv().is_some() {
            out.udp_metas += 1;
        }
        if let Some(b) = u.recv() {
            udp_open.push(b.data.raw() as u8);
            if b.last.to_bool() {
                assert!(!b.err.to_bool());
                out.udp.push(std::mem::take(&mut udp_open));
            }
        }
        if take(t) {
            if let Some(h) = oh.recv() {
                hdrs.push((h.dst.raw() as u64, h.ethertype.raw() as u16));
            }
            if let Some(b) = opl.recv() {
                assert!(hdrs.len() > out.sent.len(), "payload before hdr");
                open.push(b);
                if b.last.to_bool() {
                    let (d, e) = hdrs[out.sent.len()];
                    out.sent.push(Sent {
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
    assert_eq!(hdrs.len(), out.sent.len(), "a hdr without its frame");
    out
}

// ------------------------------------------------------------ builders

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

fn cksum(b: &[u8]) -> u16 {
    let mut s: u32 = 0;
    for c in b.chunks(2) {
        s += ((c[0] as u32) << 8) | *c.get(1).unwrap_or(&0) as u32;
    }
    while s >> 16 != 0 {
        s = (s & 0xFFFF) + (s >> 16);
    }
    !(s as u16)
}

/// ICMP message with a correct checksum.
fn icmp(ty: u8, code: u8, id: u16, seq: u16, data: &[u8]) -> Vec<u8> {
    let mut m = vec![
        ty,
        code,
        0,
        0,
        (id >> 8) as u8,
        id as u8,
        (seq >> 8) as u8,
        seq as u8,
    ];
    m.extend_from_slice(data);
    let c = cksum(&m);
    m[2] = (c >> 8) as u8;
    m[3] = c as u8;
    m
}

/// Ethernet frame (no FCS, padded to 60) carrying an IPv4 packet.
fn eth_ip(payload: &[u8], proto: u8) -> Vec<u8> {
    let total = 20 + payload.len();
    let mut h = vec![
        0x45,
        0,
        (total >> 8) as u8,
        total as u8,
        0x3d,
        0xab,
        0x40,
        0,
        64,
        proto,
        0,
        0,
        192,
        0,
        2,
        2,
        192,
        0,
        2,
        1,
    ];
    let c = cksum(&h);
    h[10] = (c >> 8) as u8;
    h[11] = c as u8;
    let mut f = vec![0x02, 0, 0, 0, 0, 1];
    f.extend_from_slice(&CLIENT_MAC);
    f.extend_from_slice(&[8, 0]);
    f.extend_from_slice(&h);
    f.extend_from_slice(payload);
    while f.len() < 60 {
        f.push(0);
    }
    f
}

/// The reply the model builds for an echo request of `data`.
fn expected_reply(id: u16, seq: u16, data: &[u8]) -> Vec<u8> {
    let m = icmp(0, 0, id, seq, data);
    let total = 20 + m.len();
    let mut h = vec![
        0x45,
        0,
        (total >> 8) as u8,
        total as u8,
        0,
        0,
        0x40,
        0,
        64,
        1,
        0,
        0,
        192,
        0,
        2,
        1,
        192,
        0,
        2,
        2,
    ];
    let c = cksum(&h);
    h[10] = (c >> 8) as u8;
    h[11] = c as u8;
    h.extend_from_slice(&m);
    h
}

// ------------------------------------------------------------ tests

/// Compare a run to the vector file: every `tx` line follows its `rx`, the
/// frames the chain must emit. Returns the chain output.
fn replay(name: &str, gap: usize, take: impl Fn(usize) -> bool) -> (Chain, Vec<(bool, Vec<u8>)>) {
    let v = vector(name);
    let rx: Vec<(Vec<u8>, bool)> = v
        .iter()
        .filter(|(r, _)| *r)
        .map(|(_, f)| (f.clone(), false))
        .collect();
    (chain(&rx, gap, take), v)
}

fn check_replies(name: &str, o: &Chain, v: &[(bool, Vec<u8>)]) {
    let want: Vec<&Vec<u8>> = v.iter().filter(|(r, _)| !*r).map(|(_, f)| f).collect();
    assert_eq!(o.sent.len(), want.len(), "{name}: number of replies");
    for (s, w) in o.sent.iter().zip(want) {
        assert_eq!(s.ethertype, 0x0800);
        assert_eq!(
            s.dst_mac,
            u64::from_be_bytes([0, 0, w[0], w[1], w[2], w[3], w[4], w[5]])
        );
        assert!(!s.err(), "{name}: reply flagged err");
        assert_eq!(
            s.data(),
            w[14..].to_vec(),
            "{name}: reply bytes (frame minus Ethernet header)"
        );
    }
}

#[test]
fn ping4_vector() {
    for gap in [0usize, 1] {
        let (o, v) = replay("ping4.hex", gap, |_| true);
        check_replies("ping4", &o, &v);
        assert_eq!(o.sent.len(), 2);
    }
}

#[test]
fn ping4_backpressure() {
    let (o, v) = replay("ping4.hex", 0, |t| t % 4 == 0 || t % 9 == 1);
    check_replies("ping4", &o, &v);
}

#[test]
fn negative_vectors_ipv4() {
    // These vectors carry UDP/NTP: the chain forwards the valid first frame
    // to its UDP output and answers nothing (no ICMP); the altered second
    // frame must not appear anywhere.
    for name in [
        "neg_ipv4_checksum.hex",
        "neg_ipv4_fragment.hex",
        "neg_ipv4_ttl0.hex",
    ] {
        let v = vector(name);
        let rx: Vec<(Vec<u8>, bool)> = v
            .iter()
            .filter(|(r, _)| *r)
            .map(|(_, f)| (f.clone(), false))
            .collect();
        assert_eq!(rx.len(), 2, "{name}");
        let o = chain(&rx, 0, |_| true);
        assert!(o.sent.is_empty(), "{name}: nothing to send");
        assert_eq!(o.udp_metas, 1, "{name}: only the valid frame is forwarded");
        assert_eq!(o.udp.len(), 1, "{name}");
        // UDP payload = IP payload of the valid frame (total length 76 - 20).
        assert_eq!(o.udp[0], rx[0].0[34..14 + 76].to_vec(), "{name}");
    }
    for name in ["neg_foreign_mac.hex", "neg_runt.hex"] {
        let v = vector(name);
        let rx: Vec<(Vec<u8>, bool)> = v
            .iter()
            .filter(|(r, _)| *r)
            .map(|(_, f)| (f.clone(), false))
            .collect();
        let o = chain(&rx, 0, |_| true);
        let want: Vec<&Vec<u8>> = v.iter().filter(|(r, _)| !*r).map(|(_, f)| f).collect();
        // The valid first frame is an NTP query (UDP): no ICMP reply either way.
        assert!(o.sent.is_empty() && want.len() <= 1, "{name}");
    }
}

fn echo_case(id: u16, seq: u16, n: usize) {
    let data: Vec<u8> = (0..n).map(|i| (i * 13 + id as usize) as u8).collect();
    let req = eth_ip(&icmp(8, 0, id, seq, &data), 1);
    let o = chain(&[(req, false)], 0, |_| true);
    assert_eq!(o.sent.len(), 1, "data {n}");
    assert_eq!(o.sent[0].dst_mac, 0x0200_0000_0002);
    assert!(!o.sent[0].err());
    assert_eq!(o.sent[0].data(), expected_reply(id, seq, &data), "data {n}");
}

#[test]
fn echo_sizes() {
    echo_case(0x1234, 1, 0);
    echo_case(0x1234, 2, 1);
    echo_case(0xffff, 0xffff, 2);
    echo_case(0, 0, 0);
    echo_case(0x00ff, 0xff00, 55);
    echo_case(0xabcd, 77, 1472);
}

#[test]
fn echo_checksum_carry_and_wrap() {
    // Sweep ids and data so that the request checksum covers the end-around
    // carry of the +0x0800 adjustment (high byte >= 0xF8) and 0xFFFF.
    let mut n = 0;
    for id in (0..=0xFFFFu32).step_by(0x0501) {
        for seq in [0u16, 0x07ff, 0xf7ff, 0xffff] {
            let req = eth_ip(&icmp(8, 0, id as u16, seq, &[1, 2, 3]), 1);
            let o = chain(&[(req, false)], 0, |_| true);
            assert_eq!(o.sent.len(), 1);
            assert_eq!(
                o.sent[0].data(),
                expected_reply(id as u16, seq, &[1, 2, 3]),
                "id {id:x} seq {seq:x}"
            );
            n += 1;
        }
    }
    assert!(n > 100);
}

#[test]
fn echo_back_to_back_and_gaps() {
    let mut frames = vec![];
    let mut want = vec![];
    for k in 0..12u16 {
        let data: Vec<u8> = (0..(k * 9 + 1)).map(|i| (i + k) as u8).collect();
        frames.push((eth_ip(&icmp(8, 0, 0x4242, k, &data), 1), false));
        want.push(expected_reply(0x4242, k, &data));
    }
    for (gap, take) in [(0usize, 0usize), (0, 1), (2, 2)] {
        let o = chain(&frames, gap, |t| take == 0 || t % 3 != 0);
        assert_eq!(o.sent.len(), want.len(), "gap {gap}");
        for (s, w) in o.sent.iter().zip(&want) {
            assert_eq!(&s.data(), w);
            assert!(!s.err());
        }
    }
}

#[test]
fn bad_icmp_checksum_ends_with_err() {
    let data: Vec<u8> = (0..40u8).collect();
    let good = icmp(8, 0, 7, 9, &data);
    for bad_at in [2usize, 3, 4, 8, 20, good.len() - 1] {
        let mut m = good.clone();
        m[bad_at] ^= 0x10;
        let frames = [(eth_ip(&m, 1), false), (eth_ip(&good, 1), false)];
        let o = chain(&frames, 0, |_| true);
        assert_eq!(o.sent.len(), 2, "byte {bad_at}");
        assert!(
            o.sent[0].err(),
            "byte {bad_at}: bad request gives an aborted reply"
        );
        let d = &o.sent[0].bytes;
        assert!(d[..d.len() - 1]
            .iter()
            .all(|b| !b.err.to_bool() && !b.last.to_bool()));
        assert_eq!(d.len(), 20 + good.len());
        assert!(!o.sent[1].err());
        assert_eq!(o.sent[1].data(), expected_reply(7, 9, &data));
    }
}

#[test]
fn other_icmp_gives_nothing() {
    let data = [1u8, 2, 3, 4];
    let frames: Vec<(Vec<u8>, bool)> = vec![
        (eth_ip(&icmp(0, 0, 1, 1, &data), 1), false), // echo reply
        (eth_ip(&icmp(8, 1, 1, 1, &data), 1), false), // code 1
        (eth_ip(&icmp(3, 0, 1, 1, &data), 1), false), // unreachable
        (eth_ip(&icmp(13, 0, 1, 1, &data), 1), false), // timestamp
        (eth_ip(&[8, 0, 0xf7, 0xff], 1), false),      // 4 bytes: shorter than 8
        (eth_ip(&[8], 1), false),                     // 1 byte
        (eth_ip(&icmp(8, 0, 5, 5, &data), 1), false), // good one at the end
    ];
    let o = chain(&frames, 0, |_| true);
    assert_eq!(o.sent.len(), 1);
    assert_eq!(o.sent[0].data(), expected_reply(5, 5, &data));
}

#[test]
fn frame_err_and_group_destination_give_nothing_clean() {
    // FCS error on the request (err on the input last): the reply, if any,
    // must be flagged err; a request to a group address gets no reply.
    let data = [9u8; 20];
    let o = chain(&[(eth_ip(&icmp(8, 0, 3, 3, &data), 1), true)], 0, |_| true);
    assert_eq!(o.sent.len(), 1);
    assert!(o.sent[0].err());
    let mut f = eth_ip(&icmp(8, 0, 3, 3, &data), 1);
    f[0..6].copy_from_slice(&[0xff; 6]);
    let o = chain(&[(f, false)], 0, |_| true);
    assert!(o.sent.is_empty());
    let mut f = eth_ip(&icmp(8, 0, 3, 3, &data), 1);
    f[0..6].copy_from_slice(&[0x01, 0, 0x5e, 0, 0, 1]);
    let o = chain(&[(f, false)], 0, |_| true);
    assert!(o.sent.is_empty());
}
