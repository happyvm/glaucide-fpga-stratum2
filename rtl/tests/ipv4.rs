// SPDX-License-Identifier: Apache-2.0
//! Cycle-by-cycle tests of `ipv4_tx` (header bytes, checksum, length abort)
//! and `ipv4_rx` (rules I0-I6, truncation, err) for task T1.10.
use ntp_fpga::eth_rx::EthMeta;
use ntp_fpga::eth_tx::EthHdr;
use ntp_fpga::ipv4_rx::{IpMeta, Ipv4Rx};
use ntp_fpga::ipv4_tx::{IpTxReq, Ipv4Tx};
use ntp_fpga::stream::FrameByte;
use txhdl::comp::{chan, signal, DefaultClock, Running, Unit};
use txhdl::types::{Bit, U};

const LOCAL_IP: u32 = 0xC000_0201; // 192.0.2.1
const PEER_IP: u32 = 0xC000_0202; // 192.0.2.2
const PEER_MAC: u64 = 0x0200_0000_0002;

/// Independent reference of the RFC 1071 checksum (does not use csum.rs):
/// 16-bit end-around-carry addition, one word at a time.
fn ref_cksum(words: &[u16]) -> u16 {
    let mut s: u16 = 0;
    for &w in words {
        let (r, c) = s.overflowing_add(w);
        s = r.wrapping_add(c as u16);
    }
    !s
}

/// The 20-byte header ipv4_tx must send.
fn ref_header(dst: u32, proto: u8, len: u16) -> Vec<u8> {
    let total = len.wrapping_add(20);
    let mut h = vec![0x45, 0, (total >> 8) as u8, total as u8, 0, 0, 0x40, 0, 64, proto, 0, 0];
    h.extend_from_slice(&LOCAL_IP.to_be_bytes());
    h.extend_from_slice(&dst.to_be_bytes());
    let words: Vec<u16> = h.chunks(2).map(|c| ((c[0] as u16) << 8) | c[1] as u16).collect();
    let c = ref_cksum(&words);
    h[10] = (c >> 8) as u8;
    h[11] = c as u8;
    h
}

// ------------------------------------------------------------ ipv4_tx

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
    dst_ip: u32,
    proto: u8,
    len: u16,
    payload: Vec<u8>,
    err: bool,
}

fn run_tx(jobs: &[Job], gap: usize, take: impl Fn(usize) -> bool) -> Vec<TxFrame> {
    let (ip_o, ip) = signal::<U<32>, DefaultClock>();
    let (rq_t, rq) = chan::<IpTxReq, DefaultClock>();
    let (pl_t, pl) = chan::<FrameByte, DefaultClock>();
    let (h_t, h) = chan::<EthHdr, DefaultClock>();
    let (o_t, o) = chan::<FrameByte, DefaultClock>();
    ip_o.set(U::<32>::from(LOCAL_IP));
    let mut dut = Ipv4Tx::default();
    let mut sim = Running::new(dut.run((rq, pl, ip), (h_t, o_t)));
    let mut next_req = 0;
    let mut pf = 0;
    let mut pi = 0;
    let total: usize = jobs.iter().map(|j| j.payload.len() + 30).sum();
    let mut frames: Vec<TxFrame> = vec![];
    let mut open: Vec<FrameByte> = vec![];
    let mut hdrs: Vec<(u64, u16)> = vec![];
    for t in 0..(total * 4 + 400) {
        if next_req < jobs.len() && rq_t.ready().to_bool() {
            let j = &jobs[next_req];
            rq_t.send(IpTxReq {
                dst_mac: U::<48>::from(j.dst_mac),
                dst_ip: U::<32>::from(j.dst_ip),
                proto: U::<8>::from(j.proto),
                len: U::<16>::from(j.len),
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
                assert!(!hdrs.is_empty() && hdrs.len() > frames.len(), "payload byte before its hdr");
                assert!(!b.err.to_bool() || b.last.to_bool(), "err without last");
                open.push(b);
                if b.last.to_bool() {
                    let (d, e) = hdrs[frames.len()];
                    frames.push(TxFrame { dst_mac: d, ethertype: e, bytes: std::mem::take(&mut open) });
                }
            }
        }
        sim.cycle();
    }
    assert!(open.is_empty(), "frame left open");
    assert_eq!(hdrs.len(), frames.len(), "a hdr without a complete frame");
    frames
}

fn job(dst_ip: u32, proto: u8, n: usize) -> Job {
    Job {
        dst_mac: PEER_MAC,
        dst_ip,
        proto,
        len: n as u16,
        payload: (0..n).map(|i| (i * 7 + 3) as u8).collect(),
        err: false,
    }
}

#[test]
fn tx_header_vectors_from_python_model() {
    // tools/golden/vectors/ping4.hex: replies of 60 and 128 bytes of IP.
    let h = ref_header(PEER_IP, 1, 40);
    assert_eq!(&h[10..12], &[0xb6, 0xbd]);
    let h = ref_header(PEER_IP, 1, 108);
    assert_eq!(&h[10..12], &[0xb6, 0x79]);
    let f = &run_tx(&[job(PEER_IP, 1, 40)], 0, |_| true)[0];
    assert_eq!(f.dst_mac, PEER_MAC);
    assert_eq!(f.ethertype, 0x0800);
    assert_eq!(&f.data()[..20], &[0x45, 0, 0, 60, 0, 0, 0x40, 0, 0x40, 1, 0xb6, 0xbd, 0xc0, 0, 2, 1, 0xc0, 0, 2, 2]);
    assert!(!f.err());
}

#[test]
fn tx_checksum_many_combinations() {
    let dsts = [0, 1, 0xC000_0202, 0xFFFF_FFFF, 0x0A00_00FF, 0x7F00_0001, 0xFFFF_0000, 0x0000_FFFF];
    let protos = [0u8, 1, 6, 17, 255];
    let lens = [1usize, 2, 8, 41, 255, 256, 1480];
    let mut jobs = vec![];
    for &d in &dsts {
        for &p in &protos {
            for &l in &lens {
                jobs.push(job(d, p, l));
            }
        }
    }
    // A pseudo-random sweep of the destination.
    let mut x = 0x1234_5678u32;
    for i in 0..60 {
        x = x.wrapping_mul(1664525).wrapping_add(1013904223);
        jobs.push(job(x, (x >> 24) as u8, 1 + (i * 23) % 1480));
    }
    let frames = run_tx(&jobs, 0, |_| true);
    assert_eq!(frames.len(), jobs.len());
    for (j, f) in jobs.iter().zip(&frames) {
        let d = f.data();
        assert_eq!(d.len(), 20 + j.len as usize);
        assert_eq!(&d[..20], &ref_header(j.dst_ip, j.proto, j.len)[..], "{:x} {} {}", j.dst_ip, j.proto, j.len);
        assert_eq!(&d[20..], &j.payload[..]);
        assert!(!f.err());
        assert_eq!(f.dst_mac, j.dst_mac);
    }
}

#[test]
fn tx_backpressure_and_gaps() {
    let jobs: Vec<Job> = (1..6).map(|i| job(PEER_IP + i, 17, 10 * i as usize)).collect();
    let frames = run_tx(&jobs, 3, |t| t % 5 != 0 && t % 7 != 1);
    assert_eq!(frames.len(), jobs.len());
    for (j, f) in jobs.iter().zip(&frames) {
        let mut exp = ref_header(j.dst_ip, j.proto, j.len);
        exp.extend_from_slice(&j.payload);
        assert_eq!(f.data(), exp);
        assert!(!f.err());
    }
}

#[test]
fn tx_length_mismatch_and_upstream_abort() {
    // Payload shorter than len: last byte carries err.
    let mut short = job(PEER_IP, 1, 10);
    short.len = 20;
    // Payload longer than len: the len-th byte carries err, the rest is swallowed.
    let mut long = job(PEER_IP, 1, 30);
    long.len = 12;
    // Upstream abort.
    let mut ab = job(PEER_IP, 1, 10);
    ab.err = true;
    // len 0: header aborted, payload swallowed.
    let mut zero = job(PEER_IP, 1, 3);
    zero.len = 0;
    // Good packet after each, to see the unit recover.
    let good = job(PEER_IP, 17, 16);
    let jobs = vec![short, good.clone(), long, good.clone(), ab, good.clone(), zero, good.clone()];
    let frames = run_tx(&jobs, 0, |_| true);
    assert_eq!(frames.len(), 8);
    assert!(frames[0].err());
    assert_eq!(frames[0].bytes.len(), 30); // 20 + 10 bytes then end
    assert!(!frames[1].err());
    assert!(frames[2].err());
    assert_eq!(frames[2].bytes.len(), 32);
    assert!(!frames[3].err());
    assert!(frames[4].err());
    assert_eq!(frames[4].bytes.len(), 30);
    assert!(!frames[5].err());
    assert!(frames[6].err());
    assert_eq!(frames[6].bytes.len(), 20);
    for k in [1usize, 3, 5, 7] {
        let mut exp = ref_header(PEER_IP, 17, 16);
        exp.extend_from_slice(&good.payload);
        assert_eq!(frames[k].data(), exp);
        assert!(!frames[k].err());
    }
    // err only on the last byte.
    for f in &frames {
        for b in &f.bytes[..f.bytes.len() - 1] {
            assert!(!b.err.to_bool() && !b.last.to_bool());
        }
    }
}

// ------------------------------------------------------------ ipv4_rx

/// A frame offered to ipv4_rx: the Ethernet meta and the payload of the frame.
#[derive(Clone)]
struct RxFr {
    dst_mc: bool,
    src_mac: u64,
    bytes: Vec<u8>,
    err: bool,
}

#[derive(Default, Debug)]
struct RxOut {
    /// (proto channel 0 = icmp, 1 = udp): metas and the byte streams.
    meta: [Vec<(u32, u32, u8, u16, u64, u8)>; 2],
    bytes: [Vec<FrameByte>; 2],
}

fn run_rx(frames: &[RxFr], gap: usize, take: impl Fn(usize, usize) -> bool) -> RxOut {
    let (ip_o, ip) = signal::<U<32>, DefaultClock>();
    let (m_t, m) = chan::<EthMeta, DefaultClock>();
    let (p_t, p) = chan::<FrameByte, DefaultClock>();
    let (im_t, im) = chan::<IpMeta, DefaultClock>();
    let (i_t, i) = chan::<FrameByte, DefaultClock>();
    let (um_t, um) = chan::<IpMeta, DefaultClock>();
    let (u_t, u) = chan::<FrameByte, DefaultClock>();
    ip_o.set(U::<32>::from(LOCAL_IP));
    let mut dut = Ipv4Rx::default();
    let mut sim = Running::new(dut.run((m, p, ip), (im_t, i_t, um_t, u_t)));
    let mut out = RxOut::default();
    let (mut fi, mut bi) = (0usize, 0usize);
    let mut meta_sent = false;
    let total: usize = frames.iter().map(|f| f.bytes.len() + gap + 4).sum();
    for t in 0..(total * 3 + 300) {
        if fi < frames.len() {
            let f = &frames[fi];
            // eth_rx sends the meta strictly before the first payload byte.
            if !meta_sent {
                if m_t.ready().to_bool() {
                    m_t.send(EthMeta {
                        dst: U::<48>::from(0x0200_0000_0001u64),
                        src: U::<48>::from(f.src_mac),
                        ethertype: U::<16>::from(0x0800u16),
                        multicast: Bit::from_bool(f.dst_mc),
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
                    out.meta[c].push((
                        x.src.raw() as u32,
                        x.dst.raw() as u32,
                        x.proto.raw() as u8,
                        x.len.raw() as u16,
                        x.eth_src.raw() as u64,
                        x.ttl.raw() as u8,
                    ));
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
struct Pkt {
    vihl: u8,
    ttl: u8,
    flags: u16,
    proto: u8,
    src: u32,
    dst: u32,
    /// None: the correct total length.
    total: Option<u16>,
    bad_csum: bool,
    data: Vec<u8>,
}

impl Pkt {
    fn new(proto: u8, n: usize) -> Pkt {
        Pkt {
            vihl: 0x45,
            ttl: 64,
            flags: 0x4000,
            proto,
            src: PEER_IP,
            dst: LOCAL_IP,
            total: None,
            bad_csum: false,
            data: (0..n).map(|i| (i * 5 + 1) as u8).collect(),
        }
    }
    fn build(&self) -> Vec<u8> {
        let total = self.total.unwrap_or(20 + self.data.len() as u16);
        let mut h = vec![self.vihl, 0, (total >> 8) as u8, total as u8, 0x12, 0x34];
        h.extend_from_slice(&self.flags.to_be_bytes());
        h.extend_from_slice(&[self.ttl, self.proto, 0, 0]);
        h.extend_from_slice(&self.src.to_be_bytes());
        h.extend_from_slice(&self.dst.to_be_bytes());
        let words: Vec<u16> = h.chunks(2).map(|c| ((c[0] as u16) << 8) | c[1] as u16).collect();
        let mut c = ref_cksum(&words);
        if self.bad_csum {
            c ^= 0x0100;
        }
        h[10] = (c >> 8) as u8;
        h[11] = c as u8;
        h.extend_from_slice(&self.data);
        h
    }
    /// Ethernet payload: padded to 46 bytes like a minimum frame.
    fn frame(&self) -> RxFr {
        let mut b = self.build();
        while b.len() < 46 {
            b.push(0);
        }
        RxFr { dst_mc: false, src_mac: PEER_MAC, bytes: b, err: false }
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

#[test]
fn rx_table() {
    let mut cases: Vec<(&str, RxFr, Option<(usize, Vec<u8>)>)> = vec![];
    let ok_i = Pkt::new(1, 12);
    let ok_u = Pkt::new(17, 30);
    cases.push(("icmp", ok_i.frame(), Some((0, ok_i.data.clone()))));
    cases.push(("udp", ok_u.frame(), Some((1, ok_u.data.clone()))));
    let tcp = Pkt::new(6, 20);
    cases.push(("proto 6", tcp.frame(), None));
    let mut p = Pkt::new(1, 12);
    p.vihl = 0x65;
    cases.push(("bad version", p.frame(), None));
    p.vihl = 0x46;
    cases.push(("IHL 6", p.frame(), None));
    p.vihl = 0x44;
    cases.push(("IHL 4", p.frame(), None));
    let mut p = Pkt::new(1, 12);
    p.bad_csum = true;
    cases.push(("bad checksum", p.frame(), None));
    let mut p = Pkt::new(1, 12);
    p.flags = 0x2000;
    cases.push(("MF", p.frame(), None));
    p.flags = 0x0008;
    cases.push(("offset", p.frame(), None));
    p.flags = 0x4001;
    cases.push(("DF + offset", p.frame(), None));
    p.flags = 0x8000;
    cases.push(("reserved bit only (model ignores it)", p.frame(), Some((0, p.data.clone()))));
    let mut p = Pkt::new(1, 12);
    p.ttl = 0;
    cases.push(("ttl 0", p.frame(), None));
    p.ttl = 1;
    cases.push(("ttl 1", p.frame(), Some((0, p.data.clone()))));
    let mut p = Pkt::new(1, 12);
    p.dst = LOCAL_IP + 1;
    cases.push(("wrong dst", p.frame(), None));
    p.dst = 0xFFFF_FFFF;
    cases.push(("dst broadcast", p.frame(), None));
    for (n, s) in [("src 0.0.0.0", 0u32), ("src broadcast", 0xFFFF_FFFF), ("src multicast", 0xE000_0001), ("src multicast hi", 0xEFFF_FFFF)] {
        let mut p = Pkt::new(1, 12);
        p.src = s;
        cases.push((n, p.frame(), None));
    }
    let mut p = Pkt::new(1, 12);
    p.src = 0xF000_0001; // reserved class E, not rejected by the model
    cases.push(("src class E", p.frame(), Some((0, p.data.clone()))));
    p.src = 0xDFFF_FFFF;
    cases.push(("src just below multicast", p.frame(), Some((0, p.data.clone()))));
    let mut f = Pkt::new(1, 12).frame();
    f.dst_mc = true;
    cases.push(("group-addressed dst", f, None));
    let mut f = Pkt::new(1, 12).frame();
    f.src_mac = 0x0300_0000_0002;
    cases.push(("group-addressed src", f, None));
    // Total length shorter than the frame: padding stripped.
    let p = Pkt::new(1, 8);
    cases.push(("padding stripped", p.frame(), Some((0, p.data.clone()))));
    // Total length 20 (no data) and 21.
    let mut p = Pkt::new(1, 8);
    p.total = Some(20);
    cases.push(("total 20", p.frame(), None));
    p.total = Some(21);
    cases.push(("total 21", p.frame(), Some((0, p.data[..1].to_vec()))));
    p.total = Some(19);
    cases.push(("total 19", p.frame(), None));
    // A 1480-byte payload, no padding.
    let p = Pkt::new(17, 1480);
    cases.push(("full size", p.frame(), Some((1, p.data.clone()))));

    for gap in [0usize, 2] {
        for (name, f, exp) in &cases {
            // Each case alone, preceded and followed by a good packet so that
            // state leaking between frames would show.
            let g = Pkt::new(17, 9);
            let frames = vec![g.frame(), f.clone(), g.frame()];
            let o = run_rx(&frames, gap, |_, _| true);
            let mut want_u = vec![g.data.clone(), g.data.clone()];
            let mut want_i: Vec<Vec<u8>> = vec![];
            let mut metas = [0usize, 2];
            if let Some((c, d)) = exp {
                if *c == 0 {
                    want_i.push(d.clone());
                    metas = [1, 2];
                } else {
                    want_u.insert(1, d.clone());
                    metas = [0, 3];
                }
            }
            assert_eq!(clean(&o, 0), want_i, "{name} (gap {gap}): icmp");
            assert_eq!(clean(&o, 1), want_u, "{name} (gap {gap}): udp");
            assert_eq!(o.meta[0].len(), metas[0], "{name}: icmp metas");
            assert_eq!(o.meta[1].len(), metas[1], "{name}: udp metas");
            if let Some((c, d)) = exp {
                let m = o.meta[*c][if *c == 0 { 0 } else { 1 }];
                assert_eq!(m.2, if *c == 0 { 1 } else { 17 }, "{name}: proto");
                assert_eq!(m.3 as usize, d.len(), "{name}: meta len");
                assert_eq!(m.1, LOCAL_IP);
                assert_eq!(m.4, f.src_mac);
                let _ = m.0;
            }
        }
    }
}

#[test]
fn rx_meta_fields() {
    let mut p = Pkt::new(1, 12);
    p.ttl = 33;
    p.src = 0xC633_6407;
    let o = run_rx(&[p.frame()], 0, |_, _| true);
    assert_eq!(o.meta[0], vec![(0xC633_6407, LOCAL_IP, 1, 12, PEER_MAC, 33)]);
}

#[test]
fn rx_truncated_and_err() {
    // Total length longer than the frame: forwarded then aborted with err.
    let mut p = Pkt::new(1, 8);
    p.total = Some(20 + 60);
    let o = run_rx(&[p.frame()], 0, |_, _| true);
    // frame = 46 bytes: 26 payload bytes arrive, last with err.
    assert_eq!(o.bytes[0].len(), 26);
    let l = o.bytes[0].last().unwrap();
    assert!(l.last.to_bool() && l.err.to_bool());
    assert!(o.bytes[0][..25].iter().all(|b| !b.last.to_bool() && !b.err.to_bool()));
    // Frame ending inside the header: nothing.
    let b = Pkt::new(1, 8).build();
    for n in [1usize, 5, 19, 20] {
        let f = RxFr { dst_mc: false, src_mac: PEER_MAC, bytes: b[..n].to_vec(), err: false };
        let o = run_rx(&[f], 0, |_, _| true);
        assert!(o.meta[0].is_empty() && o.bytes[0].is_empty(), "len {n}");
    }
    // err on the input last: copied to the output last, with and without
    // padding after the packet.
    let mut f = Pkt::new(17, 8).frame();
    f.err = true;
    let o = run_rx(&[f], 0, |_, _| true);
    assert_eq!(o.bytes[1].len(), 8);
    assert!(o.bytes[1][7].last.to_bool() && o.bytes[1][7].err.to_bool());
    assert!(o.bytes[1][..7].iter().all(|b| !b.last.to_bool() && !b.err.to_bool()));
    let mut f = Pkt::new(17, 26).frame(); // no padding
    f.err = true;
    let o = run_rx(&[f], 0, |_, _| true);
    assert_eq!(o.bytes[1].len(), 26);
    assert!(o.bytes[1][25].last.to_bool() && o.bytes[1][25].err.to_bool());
    // err on a dropped frame: nothing.
    let mut p = Pkt::new(1, 8);
    p.ttl = 0;
    let mut f = p.frame();
    f.err = true;
    let o = run_rx(&[f], 0, |_, _| true);
    assert!(o.bytes[0].is_empty() && o.meta[0].is_empty());
}

#[test]
fn rx_backpressure_and_meta_before_payload() {
    let frames: Vec<RxFr> = (0..6).map(|i| Pkt::new(if i % 2 == 0 { 1 } else { 17 }, 5 + 7 * i).frame()).collect();
    let o = run_rx(&frames, 0, |t, c| t % 3 == c || t % 11 == 0);
    assert_eq!(clean(&o, 0).len(), 3);
    assert_eq!(clean(&o, 1).len(), 3);
    for (k, d) in clean(&o, 0).iter().enumerate() {
        assert_eq!(d, &Pkt::new(1, 5 + 14 * k).data);
    }
    assert_eq!(o.meta[0].len(), 3);
}
