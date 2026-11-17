// SPDX-License-Identifier: Apache-2.0
//! Table-driven tests of the Ethernet filter and classifier (T1.5).
use ntp_fpga::eth_rx::{EthMeta, EthRx};
use ntp_fpga::stream::{FrameByte, Ts};
use txhdl::comp::{chan, signal, DefaultClock, Running, Unit};
use txhdl::types::U;

const MAC: u64 = 0x0223_4567_89ab;
const SN: u32 = 0xabcdef;
const BCAST: u64 = 0xffff_ffff_ffff;
const SOLNODE: u64 = 0x3333_ff00_0000 | SN as u64;
const ALLNODES: u64 = 0x3333_0000_0001;
const SRC: u64 = 0x0200_0000_0042;
const ARP: u16 = 0x0806;
const V4: u16 = 0x0800;
const V6: u16 = 0x86dd;

/// Stamp of the `k`-th input frame (distinct per frame).
fn stamp(k: usize) -> u64 {
    0x5EED_0000_0000_0000 | ((k as u64) << 16) | 0xbeef
}

/// One input frame (no FCS) and the err flag of its `last`.
#[derive(Clone)]
struct Fr {
    bytes: Vec<u8>,
    err: bool,
}

fn frame(dst: u64, et: u16, payload: &[u8]) -> Vec<u8> {
    let mut v = dst.to_be_bytes()[2..].to_vec();
    v.extend_from_slice(&SRC.to_be_bytes()[2..]);
    v.extend_from_slice(&et.to_be_bytes());
    v.extend_from_slice(payload);
    v
}

fn pl(n: usize, seed: usize) -> Vec<u8> {
    (0..n).map(|i| (i * 5 + seed) as u8).collect()
}

fn ok(bytes: Vec<u8>) -> Fr {
    Fr { bytes, err: false }
}

/// Channel index: 0 ARP, 1 IPv4, 2 IPv6.
#[derive(Debug, Clone, PartialEq)]
struct Meta {
    t: usize,
    dst: u64,
    src: u64,
    et: u16,
    mc: bool,
    ts: u64,
}

#[derive(Debug, Clone, PartialEq)]
struct Pb {
    t: usize,
    data: u8,
    last: bool,
    err: bool,
}

#[derive(Default)]
struct Out {
    meta: [Vec<Meta>; 3],
    pay: [Vec<Pb>; 3],
}

/// Feed the frames back to back (or `gap` idle cycles apart); `take(t, ch)`
/// says whether payload channel `ch` accepts at cycle `t`, `mtake` likewise
/// for the meta channels.
fn run(
    frames: &[Fr],
    gap: usize,
    mtake: impl Fn(usize, usize) -> bool,
    take: impl Fn(usize, usize) -> bool,
) -> Out {
    let (mac_o, mac) = signal::<U<48>, DefaultClock>();
    let (sn_o, sn) = signal::<U<24>, DefaultClock>();
    let (in_tx, in_rx) = chan::<FrameByte, DefaultClock>();
    let (am_t, am) = chan::<EthMeta, DefaultClock>();
    let (ap_t, ap) = chan::<FrameByte, DefaultClock>();
    let (v4m_t, v4m) = chan::<EthMeta, DefaultClock>();
    let (v4p_t, v4p) = chan::<FrameByte, DefaultClock>();
    let (v6m_t, v6m) = chan::<EthMeta, DefaultClock>();
    let (v6p_t, v6p) = chan::<FrameByte, DefaultClock>();
    let (st_tx, st_rx) = chan::<Ts, DefaultClock>();
    mac_o.set(U::<48>::from(MAC));
    sn_o.set(U::<24>::from(SN));
    let mut dut = EthRx::default();
    let mut sim = Running::new(dut.run((in_rx, mac, sn, st_rx), (am_t, ap_t, v4m_t, v4p_t, v6m_t, v6p_t)));
    // The input as a stream of (byte, idle-before) items.
    let mut q: Vec<Option<FrameByte>> = vec![];
    for f in frames {
        for (i, b) in f.bytes.iter().enumerate() {
            let last = i + 1 == f.bytes.len();
            q.push(Some(FrameByte::new(*b, last, last && f.err)));
        }
        for _ in 0..gap {
            q.push(None);
        }
    }
    let mut out = Out::default();
    let mut next = 0;
    let mut next_st = 0;
    let total = q.len() * 40 + 400;
    for t in 0..total {
        // One stamp per input frame, in order, delivered ahead of its frame.
        if next_st < frames.len() && st_tx.ready().to_bool() {
            st_tx.send(Ts::from(stamp(next_st)));
            next_st += 1;
        }
        if next < q.len() {
            match q[next] {
                None => next += 1,
                Some(b) => {
                    if in_tx.ready().to_bool() {
                        in_tx.send(b);
                        next += 1;
                    }
                }
            }
        }
        let ms = [&am, &v4m, &v6m];
        let ps = [&ap, &v4p, &v6p];
        for c in 0..3 {
            if mtake(t, c) {
                if let Some(m) = ms[c].recv() {
                    out.meta[c].push(Meta {
                        t,
                        dst: m.dst.raw() as u64,
                        src: m.src.raw() as u64,
                        et: m.ethertype.raw() as u16,
                        mc: m.multicast.to_bool(),
                        ts: m.ts.raw() as u64,
                    });
                }
            }
            if take(t, c) {
                if let Some(b) = ps[c].recv() {
                    out.pay[c].push(Pb {
                        t,
                        data: b.data.raw() as u8,
                        last: b.last.to_bool(),
                        err: b.err.to_bool(),
                    });
                }
            }
        }
        sim.cycle();
    }
    out
}

fn all(_: usize, _: usize) -> bool {
    true
}

/// Split the payload bytes of channel `c` into frames.
fn pframes(o: &Out, c: usize) -> Vec<Vec<Pb>> {
    let mut v = vec![];
    let mut cur = vec![];
    for b in &o.pay[c] {
        assert!(!b.err || b.last, "err without last");
        cur.push(b.clone());
        if b.last {
            v.push(std::mem::take(&mut cur));
        }
    }
    assert!(cur.is_empty(), "payload without last");
    v
}

/// Expected output: (channel, dst, et, payload, err).
type Exp = (usize, u64, u16, Vec<u8>, bool);

fn check(
    frames: &[Fr],
    gap: usize,
    expect: &[Exp],
    mtake: impl Fn(usize, usize) -> bool,
    take: impl Fn(usize, usize) -> bool,
) {
    // The meta is sent before the payload; a consumer that pops its meta
    // channel late still sees it late, so the order is only asserted on the
    // consumer side when the meta channel is always popped.
    let order = (0..200).all(|t| (0..3).all(|c| mtake(t, c)));
    let o = run(frames, gap, mtake, take);
    for c in 0..3 {
        let pf = pframes(&o, c);
        let want: Vec<&Exp> = expect.iter().filter(|e| e.0 == c).collect();
        assert_eq!(o.meta[c].len(), want.len(), "meta count on channel {c}");
        assert_eq!(pf.len(), want.len(), "frame count on channel {c}");
        for (i, e) in want.iter().enumerate() {
            let m = &o.meta[c][i];
            assert_eq!((m.dst, m.src, m.et), (e.1, SRC, e.2), "meta {c}/{i}");
            assert_eq!(m.mc, (e.1 >> 40) & 1 == 1, "multicast flag {c}/{i}");
            let d: Vec<u8> = pf[i].iter().map(|b| b.data).collect();
            assert_eq!(d, e.3, "payload {c}/{i}");
            assert_eq!(pf[i].last().unwrap().err, e.4, "err {c}/{i}");
            if order {
                assert!(m.t < pf[i][0].t, "meta {c}/{i} precedes its payload");
            }
        }
    }
}

/// Which channel and acceptance for (destination, ethertype).
fn route(dst: u64, et: u16) -> Option<usize> {
    let uni = dst == MAC;
    match et {
        ARP if uni || dst == BCAST => Some(0),
        V4 if uni || dst == BCAST => Some(1),
        V6 if uni || dst == SOLNODE || dst == ALLNODES => Some(2),
        _ => None,
    }
}

#[test]
fn class_table() {
    let dsts = [
        MAC,
        BCAST,
        SOLNODE,
        ALLNODES,
        0x0223_4567_89ac,            // foreign unicast
        0x3333_ff00_0000 | 0xabcdee, // other solicited-node group
        0x0100_5e00_0001,            // IPv4 multicast
        0x3333_0000_0002,
    ];
    let ets = [ARP, V4, V6, 0x8100, 0x88f7, 0x0000];
    for &d in &dsts {
        for &e in &ets {
            let p = pl(46, d as usize ^ e as usize);
            let exp: Vec<Exp> = route(d, e)
                .map(|c| (c, d, e, p.clone(), false))
                .into_iter()
                .collect();
            check(&[ok(frame(d, e, &p))], 0, &exp, all, all);
        }
    }
}

#[test]
fn spec_rows() {
    // The rows of the task, spelled out.
    assert_eq!(route(MAC, ARP), Some(0));
    assert_eq!(route(MAC, V4), Some(1));
    assert_eq!(route(MAC, V6), Some(2));
    assert_eq!(route(BCAST, ARP), Some(0));
    assert_eq!(route(BCAST, V4), Some(1));
    assert_eq!(route(BCAST, V6), None, "broadcast IPv6 is dropped");
    assert_eq!(route(SOLNODE, V6), Some(2));
    assert_eq!(route(ALLNODES, V6), Some(2));
    assert_eq!(route(SOLNODE, V4), None, "multicast to IPv4 is dropped");
}

#[test]
fn meta_content_and_multicast_flag() {
    let p = pl(30, 1);
    let o = run(
        &[ok(frame(SOLNODE, V6, &p)), ok(frame(MAC, V4, &p))],
        0,
        all,
        all,
    );
    assert_eq!(o.meta[2][0].dst, SOLNODE);
    assert!(o.meta[2][0].mc);
    assert_eq!(o.meta[2][0].src, SRC);
    assert_eq!(o.meta[2][0].et, V6);
    assert!(!o.meta[1][0].mc);
    let o = run(&[ok(frame(BCAST, ARP, &p))], 0, all, all);
    assert!(o.meta[0][0].mc, "broadcast has the group bit");
}

#[test]
fn runt_frames_emit_nothing() {
    // Every length up to the header and one byte more (empty payload: 14).
    let full = frame(MAC, V4, &pl(10, 3));
    for n in 1..=14 {
        let o = run(&[ok(full[..n].to_vec())], 0, all, all);
        for c in 0..3 {
            assert!(o.meta[c].is_empty() && o.pay[c].is_empty(), "length {n}");
        }
    }
    // 15 bytes: one payload byte.
    check(
        &[ok(full[..15].to_vec())],
        0,
        &[(1, MAC, V4, vec![full[14]], false)],
        all,
        all,
    );
    // A runt does not disturb the next frame.
    let p = pl(20, 9);
    check(
        &[ok(full[..9].to_vec()), ok(frame(MAC, ARP, &p))],
        0,
        &[(0, MAC, ARP, p, false)],
        all,
        all,
    );
}

#[test]
fn err_on_last() {
    let p = pl(50, 2);
    // Forwarded with err on the last byte only.
    let f = Fr {
        bytes: frame(MAC, V6, &p),
        err: true,
    };
    check(&[f.clone()], 0, &[(2, MAC, V6, p.clone(), true)], all, all);
    // Dropped frames with err emit nothing; the error does not leak.
    let q = pl(30, 4);
    let g = Fr {
        bytes: frame(0x0223_4567_89ac, V4, &q),
        err: true,
    };
    let r = Fr {
        bytes: frame(MAC, 0x1234, &q)[..10].to_vec(),
        err: true,
    };
    check(
        &[g, r, f, ok(frame(MAC, V4, &q))],
        0,
        &[(2, MAC, V6, p, true), (1, MAC, V4, q, false)],
        all,
        all,
    );
    // Header-only frame with err.
    let h = Fr {
        bytes: frame(MAC, V4, &[]),
        err: true,
    };
    check(&[h], 0, &[], all, all);
}

#[test]
fn back_to_back_and_mixed() {
    let a = pl(46, 1);
    let b = pl(60, 2);
    let c = pl(100, 3);
    let d = pl(1, 4);
    let frames = vec![
        ok(frame(MAC, ARP, &a)),
        ok(frame(BCAST, V6, &b)), // dropped
        ok(frame(MAC, V4, &b)),
        ok(frame(ALLNODES, V6, &c)),
        ok(frame(MAC, 0x8100, &c)), // dropped
        ok(frame(BCAST, ARP, &d)),
        ok(frame(SOLNODE, V6, &a)),
    ];
    let exp: Vec<Exp> = vec![
        (0, MAC, ARP, a.clone(), false),
        (1, MAC, V4, b, false),
        (2, ALLNODES, V6, c, false),
        (0, BCAST, ARP, d, false),
        (2, SOLNODE, V6, a, false),
    ];
    check(&frames, 0, &exp, all, all);
    check(&frames, 3, &exp, all, all);
}

#[test]
fn backpressure_on_outputs() {
    let a = pl(80, 1);
    let b = pl(70, 2);
    let frames = vec![
        ok(frame(MAC, ARP, &a)),
        ok(frame(MAC, V4, &b)),
        ok(frame(SOLNODE, V6, &a)),
        ok(frame(BCAST, V6, &b)),
        ok(frame(MAC, V6, &b)),
    ];
    let exp: Vec<Exp> = vec![
        (0, MAC, ARP, a.clone(), false),
        (1, MAC, V4, b.clone(), false),
        (2, SOLNODE, V6, a, false),
        (2, MAC, V6, b, false),
    ];
    // Payload accepted one cycle in three, meta one in five, per channel.
    check(
        &frames,
        0,
        &exp,
        |t, c| (t + c) % 5 == 0,
        |t, c| (t + c) % 3 == 0,
    );
    // Nothing accepted at all for 300 cycles, then everything.
    check(&frames, 0, &exp, |t, _| t > 300, |t, _| t > 400);
    // Pseudo-random stalls.
    let rnd = |t: usize, c: usize| ((t * 2654435761 + c * 97) >> 7) % 3 != 0;
    check(&frames, 2, &exp, rnd, |t, c| rnd(t + 11, c + 1));
}

#[test]
fn stalled_output_does_not_leak_into_other_classes() {
    // ARP payload channel stalled for a long time: an IPv4 frame behind it
    // waits (in-order) and nothing is emitted on the IPv4 channel early.
    let a = pl(40, 1);
    let b = pl(40, 2);
    let frames = vec![ok(frame(MAC, ARP, &a)), ok(frame(MAC, V4, &b))];
    let o = run(&frames, 0, all, |t, c| c != 0 || t > 200);
    let arp_end = o.pay[0].last().unwrap().t;
    let v4_start = o.pay[1][0].t;
    assert!(
        v4_start > arp_end,
        "no interleaving: {v4_start} > {arp_end}"
    );
}

#[test]
fn one_byte_per_cycle_when_unstalled() {
    let a = pl(100, 1);
    let o = run(&[ok(frame(MAC, V4, &a))], 0, all, all);
    let t: Vec<usize> = o.pay[1].iter().map(|b| b.t).collect();
    assert_eq!(t.len(), 100);
    for w in t.windows(2) {
        assert_eq!(w[1], w[0] + 1, "cut-through at one byte per cycle");
    }
}

#[test]
fn other_solicited_node_group_is_dropped() {
    // The group follows `sn_low`: another group's frame is not ours.
    let p = pl(20, 6);
    check(
        &[ok(frame(0x3333_ff00_0000 | 0xabcdee, V6, &p))],
        0,
        &[],
        all,
        all,
    );
}

/// Each forwarded frame carries the stamp of its own frame, whatever was
/// dropped before it (wrong MAC, unknown EtherType, runts, empty payload).
#[test]
fn stamp_follows_its_frame_across_drops() {
    let frames = vec![
        ok(frame(0x0223_0000_0001, V4, &pl(20, 1))), // 0: wrong MAC, dropped
        ok(frame(MAC, V4, &pl(20, 2))),              // 1: IPv4
        ok(frame(MAC, 0x1234, &pl(20, 3))),          // 2: unknown EtherType
        ok(pl(5, 4)),                                // 3: runt (ends before byte 14)
        ok(frame(BCAST, ARP, &pl(28, 5))),           // 4: ARP
        ok(frame(MAC, V6, &[])),                     // 5: empty payload
        ok(pl(1, 6)),                                // 6: one byte
        ok(frame(SOLNODE, V6, &pl(40, 7))),          // 7: IPv6
        ok(frame(BCAST, V6, &pl(40, 8))),            // 8: IPv6 to broadcast, dropped
        ok(frame(MAC, V4, &pl(30, 9))),              // 9: IPv4
        ok(pl(14, 10)),                              // 10: header only
        ok(frame(ALLNODES, V6, &pl(41, 11))),        // 11: IPv6
    ];
    let want: [(usize, usize); 5] = [(1, 1), (0, 4), (2, 7), (1, 9), (2, 11)];
    let pats: [&dyn Fn(usize, usize) -> bool; 2] = [&all, &|t, c| (t + c) % 3 != 0];
    for gap in [0, 3] {
        for p in pats {
            let o = run(&frames, gap, p, p);
            for c in 0..3 {
                let w: Vec<usize> = want.iter().filter(|e| e.0 == c).map(|e| e.1).collect();
                let got: Vec<u64> = o.meta[c].iter().map(|m| m.ts).collect();
                let exp: Vec<u64> = w.iter().map(|&k| stamp(k)).collect();
                assert_eq!(got, exp, "stamps on channel {c}, gap {gap}");
            }
        }
    }
}
