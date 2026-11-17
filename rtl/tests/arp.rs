// SPDX-License-Identifier: Apache-2.0
//! Cycle-by-cycle tests of the ARP unit (T1.9): reference vector, cache,
//! resolution, priority, backpressure.
use ntp_fpga::arp::{Arp, ArpEntry, ArpTiming, ArpWith, Board, Sim};
use ntp_fpga::eth_rx::EthMeta;
use ntp_fpga::eth_tx::EthHdr;
use ntp_fpga::stream::FrameByte;
use txhdl::comp::{chan, signal, DefaultClock, Running, Unit};
use txhdl::types::{Bit, U};

const MAC: u64 = 0x0200_0000_0001;
const IP: u32 = 0xC000_0201; // 192.0.2.1
const BCAST: u64 = 0xffff_ffff_ffff;

fn mac_bytes(m: u64) -> [u8; 6] {
    let b = m.to_be_bytes();
    [b[2], b[3], b[4], b[5], b[6], b[7]]
}

/// ARP packet (28 bytes), Ethernet/IPv4.
fn arp(op: u16, sha: u64, spa: u32, tha: u64, tpa: u32) -> Vec<u8> {
    let mut v = vec![0, 1, 8, 0, 6, 4];
    v.extend_from_slice(&op.to_be_bytes());
    v.extend_from_slice(&mac_bytes(sha));
    v.extend_from_slice(&spa.to_be_bytes());
    v.extend_from_slice(&mac_bytes(tha));
    v.extend_from_slice(&tpa.to_be_bytes());
    v
}

/// Ethernet frame padded to 60 bytes.
fn eth(dst: u64, src: u64, payload: &[u8]) -> Vec<u8> {
    let mut v = mac_bytes(dst).to_vec();
    v.extend_from_slice(&mac_bytes(src));
    v.extend_from_slice(&[0x08, 0x06]);
    v.extend_from_slice(payload);
    while v.len() < 60 {
        v.push(0);
    }
    v
}

#[derive(Clone)]
struct Rx {
    start: usize,
    frame: Vec<u8>,
    err: bool,
}

fn rxf(start: usize, frame: Vec<u8>) -> Rx {
    Rx { start, frame, err: false }
}

#[derive(Default)]
struct Script {
    frames: Vec<Rx>,
    resolves: Vec<(usize, u32)>,
    lookups: Vec<(usize, u32)>,
    /// Cycles during which `flush` is high.
    flush: Vec<usize>,
}

#[derive(Debug, Default)]
struct Out {
    /// (dst, ethertype, payload, hdr cycle)
    frames: Vec<(u64, u16, Vec<u8>, usize)>,
    /// (cycle, found, mac)
    rsp: Vec<(usize, bool, u64)>,
    resolve_taken: Vec<usize>,
}

fn run(s: &Script, cycles: usize, take: impl Fn(usize) -> bool) -> Out {
    run_t::<Board>(s, cycles, take)
}

fn run_t<T: ArpTiming>(s: &Script, cycles: usize, take: impl Fn(usize) -> bool) -> Out {
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
    mac_o.set(U::<48>::from(MAC));
    ip_o.set(U::<32>::from(IP));
    fl_o.set(Bit::Zero);
    let mut dut = ArpWith::<T>::default();
    let mut sim = Running::new(dut.run((mr, pr, mac, ip, rr, lr, fl), (ht, ot, st)));
    let mut out = Out::default();
    let mut fi = 0usize; // current frame
    let mut bi = 0usize; // 0 = meta not sent; k = k-1 payload bytes sent
    let mut ri = 0usize;
    let mut li = 0usize;
    let mut hdrs: std::collections::VecDeque<(u64, u16, usize)> = Default::default();
    let mut cur: Vec<u8> = vec![];
    for t in 0..cycles {
        fl_o.set(if s.flush.contains(&t) { Bit::One } else { Bit::Zero });
        if fi < s.frames.len() && t >= s.frames[fi].start {
            let f = &s.frames[fi];
            let pl = &f.frame[14.min(f.frame.len())..];
            if bi == 0 {
                if mt.ready().to_bool() {
                    let d = u64::from_be_bytes([0, 0, f.frame[0], f.frame[1], f.frame[2], f.frame[3], f.frame[4], f.frame[5]]);
                    let sr_ = u64::from_be_bytes([0, 0, f.frame[6], f.frame[7], f.frame[8], f.frame[9], f.frame[10], f.frame[11]]);
                    mt.send(EthMeta {
                        dst: U::<48>::from(d),
                        src: U::<48>::from(sr_),
                        ethertype: U::<16>::from(0x0806u16),
                        multicast: Bit::from_bool((d >> 40) & 1 == 1),
                        ts: U::<64>::from(0u64),
                    });
                    bi = 1;
                }
            } else if pt.ready().to_bool() {
                let k = bi - 1;
                let last = k + 1 == pl.len();
                pt.send(FrameByte::new(pl[k], last, last && f.err));
                bi += 1;
                if last {
                    fi += 1;
                    bi = 0;
                }
            }
        }
        if ri < s.resolves.len() && t >= s.resolves[ri].0 && rt.ready().to_bool() {
            rt.send(U::<32>::from(s.resolves[ri].1));
            ri += 1;
        }
        if li < s.lookups.len() && t >= s.lookups[li].0 && lt.ready().to_bool() {
            lt.send(U::<32>::from(s.lookups[li].1));
            li += 1;
        }
        if let Some(r) = sr.recv() {
            out.rsp.push((t, r.found.to_bool(), r.mac.raw() as u64));
        }
        if take(t) {
            if let Some(h) = hr.recv() {
                hdrs.push_back((h.dst.raw() as u64, h.ethertype.raw() as u16, t));
            }
            if let Some(b) = or.recv() {
                assert!(!hdrs.is_empty(), "payload byte without hdr");
                assert!(!b.err.to_bool(), "err on ARP output");
                cur.push(b.data.raw() as u8);
                if b.last.to_bool() {
                    let (d, e, c) = hdrs.pop_front().unwrap();
                    out.frames.push((d, e, std::mem::take(&mut cur), c));
                }
            }
        }
        sim.cycle();
    }
    assert!(hdrs.is_empty() && cur.is_empty(), "a hdr without its complete payload");
    let _ = &mut out.resolve_taken;
    out
}

fn always(_: usize) -> bool {
    true
}

const REMOTE: u64 = 0x0200_0000_0050;
const RIP: u32 = 0xC000_0232; // 192.0.2.50

fn request_for(target: u32) -> Vec<u8> {
    eth(BCAST, REMOTE, &arp(1, REMOTE, RIP, 0, target))
}
fn reply_to_us() -> Vec<u8> {
    eth(MAC, REMOTE, &arp(2, REMOTE, RIP, MAC, IP))
}
fn expected_reply() -> (u64, u16, Vec<u8>) {
    (REMOTE, 0x0806, arp(2, MAC, IP, REMOTE, RIP))
}
fn request_frame(tpa: u32) -> (u64, u16, Vec<u8>) {
    (BCAST, 0x0806, arp(1, MAC, IP, 0, tpa))
}
fn strip(o: &Out) -> Vec<(u64, u16, Vec<u8>)> {
    o.frames.iter().map(|f| (f.0, f.1, f.2.clone())).collect()
}

fn hex(s: &str) -> Vec<u8> {
    (0..s.len() / 2).map(|i| u8::from_str_radix(&s[2 * i..2 * i + 2], 16).unwrap()).collect()
}

#[test]
fn reference_vector() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../tools/golden/vectors/arp_request_reply.hex");
    let text = std::fs::read_to_string(path).unwrap();
    let mut s = Script::default();
    let mut expect: Vec<(u64, u16, Vec<u8>)> = vec![];
    let mut n = 0;
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let (d, h) = line.split_once(' ').unwrap();
        let b = hex(h);
        match d {
            "rx" => {
                s.frames.push(rxf(n * 300, b));
                n += 1;
            }
            "tx" => {
                let dst = u64::from_be_bytes([0, 0, b[0], b[1], b[2], b[3], b[4], b[5]]);
                assert_eq!(&b[6..12], &mac_bytes(MAC)[..], "model source MAC");
                expect.push((dst, u16::from_be_bytes([b[12], b[13]]), b[14..].to_vec()));
            }
            _ => panic!("bad vector line"),
        }
    }
    assert!(n >= 2 && !expect.is_empty());
    let o = run(&s, n * 300 + 100, always);
    assert_eq!(strip(&o), expect);
}

#[test]
fn request_for_another_address_gets_nothing() {
    let s = Script { frames: vec![rxf(0, request_for(0xC000_024D))], lookups: vec![(150, RIP)], ..Default::default() };
    let o = run(&s, 300, always);
    assert!(o.frames.is_empty());
    // The sender of a packet not for us is not learned.
    assert_eq!(o.rsp.len(), 1);
    assert!(!o.rsp[0].1);
}

#[test]
fn request_for_us_replies_and_is_not_learned() {
    // F5: a request addressed to us is answered but its sender is not learned.
    let s = Script { frames: vec![rxf(0, request_for(IP))], lookups: vec![(150, RIP)], ..Default::default() };
    let o = run(&s, 300, always);
    assert_eq!(strip(&o), vec![expected_reply()]);
    assert_eq!(o.rsp.len(), 1);
    assert_eq!((o.rsp[0].1, o.rsp[0].2), (false, 0));
}

#[test]
fn reply_is_learned_then_lookup_hits() {
    let s = Script { resolves: vec![(0, RIP)], frames: vec![rxf(100, reply_to_us())], lookups: vec![(0, RIP), (250, RIP), (260, RIP + 1)], ..Default::default() };
    let o = run(&s, 400, always);
    assert_eq!(strip(&o), vec![request_frame(RIP)], "a reply is never answered");
    assert_eq!(o.rsp.len(), 3);
    assert!(!o.rsp[0].1, "miss before learning");
    assert_eq!((o.rsp[1].1, o.rsp[1].2), (true, REMOTE));
    assert_eq!((o.rsp[2].1, o.rsp[2].2), (false, 0));
}

#[test]
fn lookup_miss_on_empty_cache() {
    let s = Script { lookups: vec![(0, IP), (3, 0)], ..Default::default() };
    let o = run(&s, 50, always);
    assert_eq!(o.rsp.len(), 2);
    assert!(o.rsp.iter().all(|r| !r.1 && r.2 == 0));
}

#[test]
fn cache_replacement_is_round_robin() {
    // Learn 5 addresses: the first one is evicted, the others stay.
    let mut s = Script::default();
    for k in 0..5u32 {
        let m = 0x0200_0000_0100 + k as u64;
        s.resolves.push((k as usize * 120, 0xC000_0300 + k));
        s.frames.push(rxf(k as usize * 120 + 50, eth(MAC, m, &arp(2, m, 0xC000_0300 + k, MAC, IP))));
    }
    for k in 0..5u32 {
        s.lookups.push((700 + 3 * k as usize, 0xC000_0300 + k));
    }
    // An update of an existing entry does not consume a slot.
    let o = run(&s, 900, always);
    let r: Vec<bool> = o.rsp.iter().map(|x| x.1).collect();
    assert_eq!(r, vec![false, true, true, true, true]);
    assert_eq!(o.rsp[4].2, 0x0200_0000_0104);
}

#[test]
fn second_reply_does_not_rebind_the_entry() {
    // The first reply answers the resolution; a later one for the same IP is
    // unsolicited and must not change the entry (F5).
    let mut s = Script { resolves: vec![(0, RIP)], ..Default::default() };
    for (k, m) in [0x0200_0000_0101u64, 0x0200_0000_0102].iter().enumerate() {
        s.frames.push(rxf(100 + k * 120, eth(MAC, *m, &arp(2, *m, RIP, MAC, IP))));
    }
    s.lookups.push((400, RIP));
    let o = run(&s, 500, always);
    assert_eq!((o.rsp[0].1, o.rsp[0].2), (true, 0x0200_0000_0101));
}

#[test]
fn resolve_emits_one_broadcast_request() {
    let s = Script { resolves: vec![(0, RIP)], ..Default::default() };
    let o = run(&s, 300, always);
    assert_eq!(strip(&o), vec![request_frame(RIP)]);
}

#[test]
fn resolve_of_cached_ip_emits_nothing() {
    let s = Script { resolves: vec![(0, RIP), (200, RIP)], frames: vec![rxf(100, reply_to_us())], lookups: vec![(350, RIP)], ..Default::default() };
    let o = run(&s, 500, always);
    assert_eq!(strip(&o), vec![request_frame(RIP)], "only the first resolve sends a request");
    assert!(o.rsp[0].1);
}

#[test]
fn second_resolve_waits_for_the_first_and_retransmit_is_allowed() {
    let other = 0xC000_0240u32;
    let s = Script {
        // RIP, then another address, then RIP again (retransmission), the reply
        // for RIP arrives at 600: only then the other one goes out.
        resolves: vec![(0, RIP), (1, other)],
        frames: vec![rxf(600, reply_to_us())],
        ..Default::default()
    };
    let o = run(&s, 1000, always);
    assert_eq!(strip(&o), vec![request_frame(RIP), request_frame(other)]);
    assert!(o.frames[1].3 > 600, "second request only after the answer");

    let s = Script { resolves: vec![(0, RIP), (200, RIP)], ..Default::default() };
    let o = run(&s, 500, always);
    assert_eq!(strip(&o), vec![request_frame(RIP), request_frame(RIP)]);
}

#[test]
fn flush_clears_the_cache_and_the_resolution() {
    let s = Script {
        resolves: vec![(0, RIP)],
        frames: vec![rxf(100, reply_to_us())],
        lookups: vec![(250, RIP), (350, RIP)],
        flush: vec![300],
        ..Default::default()
    };
    let o = run(&s, 450, always);
    assert!(o.rsp[0].1);
    assert!(!o.rsp[1].1);
    // After a flush a new resolution can start.
    let other = 0xC000_0240u32;
    let s = Script { resolves: vec![(0, RIP), (1, other)], flush: vec![250], ..Default::default() };
    let o = run(&s, 600, always);
    assert_eq!(strip(&o), vec![request_frame(RIP), request_frame(other)]);
}

#[test]
fn err_frame_is_ignored() {
    let mut f = rxf(0, request_for(IP));
    f.err = true;
    let mut g = rxf(120, reply_to_us());
    g.err = true;
    let s = Script { resolves: vec![(0, RIP)], frames: vec![f, g], lookups: vec![(300, RIP)], ..Default::default() };
    let o = run(&s, 400, always);
    assert_eq!(strip(&o), vec![request_frame(RIP)]);
    assert!(!o.rsp[0].1);
}

#[test]
fn short_and_garbage_arp_is_ignored() {
    let good = arp(1, REMOTE, RIP, 0, IP);
    let mut cases: Vec<Vec<u8>> = vec![];
    // Short payload (27 bytes: the frame is not padded by the test here).
    cases.push(eth(BCAST, REMOTE, &good)[..14 + 27].to_vec());
    cases.push(eth(BCAST, REMOTE, &good)[..14 + 1].to_vec());
    for (i, v) in [1usize, 3, 4, 5, 6].iter().map(|i| (*i, 0)).collect::<Vec<_>>() {
        let _ = v;
        let mut p = good.clone();
        p[i] ^= 0x10; // htype, ptype, hlen, plen, op high byte
        cases.push(eth(BCAST, REMOTE, &p));
    }
    for op in [0u16, 3, 8] {
        cases.push(eth(BCAST, REMOTE, &arp(op, REMOTE, RIP, 0, IP)));
    }
    // Sender MAC with the group bit (A4) and Ethernet source with the group bit (E1).
    cases.push(eth(BCAST, REMOTE, &arp(1, 0x0300_0000_0050, RIP, 0, IP)));
    cases.push(eth(BCAST, 0x0300_0000_0050, &good));
    // Valid packets for another address / sent to another MAC.
    cases.push(eth(BCAST, REMOTE, &arp(1, REMOTE, RIP, 0, IP + 1)));
    cases.push(eth(0x0200_0000_0099, REMOTE, &good));
    let n = cases.len();
    let s = Script {
        resolves: vec![(0, RIP)],
        frames: cases.into_iter().enumerate().map(|(i, f)| rxf(i * 100, f)).collect(),
        lookups: vec![(n * 100 + 50, RIP)],
        ..Default::default()
    };
    let o = run(&s, n * 100 + 200, always);
    assert_eq!(strip(&o), vec![request_frame(RIP)], "only our own request goes out");
    assert!(!o.rsp[0].1, "nothing learned");
}

#[test]
fn back_to_back_requests() {
    let mut s = Script::default();
    let mut exp = vec![];
    for k in 0..4u32 {
        let m = 0x0200_0000_0200 + k as u64;
        let sp = 0xC000_0300 + k;
        s.frames.push(rxf(0, eth(BCAST, m, &arp(1, m, sp, 0, IP))));
        exp.push((m, 0x0806u16, arp(2, MAC, IP, m, sp)));
    }
    let o = run(&s, 800, always);
    assert_eq!(strip(&o), exp);
}

#[test]
fn output_backpressure() {
    let s = Script {
        frames: vec![rxf(0, request_for(IP)), rxf(0, request_for(IP))],
        resolves: vec![(5, 0xC000_0240)],
        ..Default::default()
    };
    for p in [2usize, 3, 7] {
        let o = run(&s, 1500, |t| t % p == 0);
        // The resolve (t = 5) is decided before the replies: only frame integrity
        // and count are checked here, the order is covered by the priority test.
        let mut got = strip(&o);
        got.sort_by_key(|f| f.0);
        assert_eq!(got, vec![expected_reply(), expected_reply(), request_frame(0xC000_0240)], "period {p}");
    }
}

#[test]
fn reply_has_priority_over_pending_resolve() {
    // The output is blocked until 400: frame 1's reply fills the output, then a
    // resolve and a second request queue up; the reply goes first.
    let s = Script {
        frames: vec![rxf(0, request_for(IP)), rxf(100, request_for(IP))],
        resolves: vec![(110, 0xC000_0240)],
        ..Default::default()
    };
    let o = run(&s, 900, |t| t >= 400);
    assert_eq!(strip(&o), vec![expected_reply(), expected_reply(), request_frame(0xC000_0240)]);
}

// ---- pending timeout (T4.2c); `Sim::TIMEOUT` = 300 cycles ----

const SILENT: u32 = 0xC000_0240; // a next hop that never answers

#[test]
fn board_name_is_the_board_timing() {
    // `Arp` is `ArpWith<Board>` and behaves as before inside the timeout.
    let _ = Arp::default();
    let s = Script { resolves: vec![(0, RIP), (1, SILENT)], ..Default::default() };
    let o = run(&s, 2000, always);
    assert_eq!(strip(&o), vec![request_frame(RIP)], "no timeout on the board within 2000 cycles");
}

#[test]
fn pending_resolution_times_out_and_unblocks_the_next() {
    // Silent next hop first, then another address: the second request goes
    // out once the first resolution has been abandoned (about TIMEOUT later),
    // and the second resolves normally.
    let s = Script {
        resolves: vec![(0, SILENT), (1, RIP)],
        frames: vec![rxf(500, reply_to_us())],
        lookups: vec![(1200, RIP), (1210, SILENT)],
        ..Default::default()
    };
    let o = run_t::<Sim>(&s, 1500, always);
    assert_eq!(strip(&o), vec![request_frame(SILENT), request_frame(RIP)]);
    let t1 = o.frames[1].3;
    assert!(t1 >= 300 && t1 < 320, "second request just after the timeout, got {}", t1);
    assert_eq!(o.rsp.iter().map(|r| (r.1, r.2)).collect::<Vec<_>>(), vec![(true, REMOTE), (false, 0)]);
}

#[test]
fn nothing_is_abandoned_before_the_timeout() {
    let s = Script { resolves: vec![(0, SILENT), (1, RIP)], ..Default::default() };
    let o = run_t::<Sim>(&s, 290, always);
    assert_eq!(strip(&o), vec![request_frame(SILENT)]);
}

#[test]
fn reply_after_the_timeout_is_ignored_and_matches_no_pending_request() {
    // SILENT times out; RIP is then resolved (pending). A late reply from
    // SILENT arrives while RIP waits: it is NOT learned (F5: lookup misses)
    // and does not complete RIP's resolution (a third address stays blocked
    // until RIP's own timeout).
    let late = eth(MAC, 0x0200_0000_0060, &arp(2, 0x0200_0000_0060, SILENT, MAC, IP));
    let s = Script {
        resolves: vec![(0, SILENT), (1, RIP), (450, 0xC000_0250)],
        frames: vec![rxf(400, late)],
        lookups: vec![(500, SILENT), (510, RIP)],
        ..Default::default()
    };
    let o = run_t::<Sim>(&s, 1200, always);
    assert_eq!(o.rsp.iter().map(|r| (r.1, r.2)).collect::<Vec<_>>(), vec![(false, 0), (false, 0)]);
    // RIP's request at ~300, then RIP times out at ~600: the third address
    // (offered at 450, after the late reply) goes out only then.
    let f = strip(&o);
    assert_eq!(f[..2], [request_frame(SILENT), request_frame(RIP)]);
    assert_eq!(f[2], request_frame(0xC000_0250));
    assert!(o.frames[2].3 >= 600 && o.frames[2].3 < 620, "got {}", o.frames[2].3);
}

#[test]
fn retransmission_restarts_the_timer() {
    // Retrying the same IP every 200 cycles keeps the resolution alive past
    // the 300-cycle timeout: another address stays blocked.
    let s = Script {
        resolves: vec![(0, SILENT), (200, SILENT), (400, SILENT), (450, RIP)],
        ..Default::default()
    };
    let o = run_t::<Sim>(&s, 650, always);
    assert_eq!(strip(&o), vec![request_frame(SILENT); 3]);
    let s = Script { resolves: vec![(0, SILENT), (200, SILENT), (400, SILENT), (450, RIP)], ..Default::default() };
    let o = run_t::<Sim>(&s, 900, always);
    assert_eq!(strip(&o).last().unwrap(), &request_frame(RIP));
    assert!(o.frames[3].3 >= 700, "RIP only after the last restart + timeout");
}

// ---- F5/F6: solicited-only learning ----

const GW: u32 = 0xC000_0201 + 0x100; // 192.0.2.1 + 256 = 192.0.3.1 (the gateway)
const GW_MAC: u64 = 0x0200_0000_00aa;
const EVIL: u64 = 0x0200_0000_00ee;

fn gw_reply(sha: u64, spa: u32, tha: u64, tpa: u32) -> Vec<u8> {
    eth(MAC, sha, &arp(2, sha, spa, tha, tpa))
}

fn rsp(o: &Out) -> Vec<(bool, u64)> {
    o.rsp.iter().map(|r| (r.1, r.2)).collect()
}

#[test]
fn arp_poisoning_attempts_never_change_the_cache() {
    // Phase 1 (no resolution outstanding): every attack is ignored.
    // Phase 2: the gateway is resolved and a valid reply is learned.
    // Phase 3: the same attacks (and unsolicited replies for the learned
    // address) never rebind it.
    let attacks = |t0: usize| -> Vec<Rx> {
        let fr = vec![
            // Unsolicited unicast reply for the gateway.
            gw_reply(EVIL, GW, MAC, IP),
            // Gratuitous ARP (broadcast request and reply forms, spa = tpa).
            eth(BCAST, EVIL, &arp(1, EVIL, GW, 0, GW)),
            eth(BCAST, EVIL, &arp(2, EVIL, GW, EVIL, GW)),
            // A request for us carrying the gateway as sender.
            eth(BCAST, EVIL, &arp(1, EVIL, GW, 0, IP)),
            // Reply with the wrong target IP, the wrong target MAC, both.
            gw_reply(EVIL, GW, MAC, IP + 5),
            gw_reply(EVIL, GW, EVIL, IP),
            // Reply sent to broadcast with the right target fields.
            eth(BCAST, EVIL, &arp(2, EVIL, GW, MAC, IP)),
        ];
        fr.into_iter().enumerate().map(|(i, f)| rxf(t0 + i * 90, f)).collect()
    };
    let mut s = Script::default();
    s.frames.extend(attacks(0)); // phase 1: 0..630
    s.lookups.push((700, GW));
    s.resolves.push((710, GW));
    s.frames.push(rxf(1000, gw_reply(GW_MAC, GW, MAC, IP)));
    s.lookups.push((1200, GW));
    s.frames.extend(attacks(1300)); // phase 3: 1300..1930
    s.lookups.push((2100, GW));
    let o = run(&s, 2200, always);
    assert_eq!(rsp(&o), vec![(false, 0), (true, GW_MAC), (true, GW_MAC)]);
    // The request for us (carrying the gateway as sender) is answered twice (we are a server).
    let replies: Vec<_> = strip(&o).into_iter().filter(|f| f.2[7] == 2).collect();
    assert_eq!(replies.len(), 2);
    assert!(replies.iter().all(|f| f.0 == EVIL));
}

#[test]
fn arp_attacks_while_resolving_do_not_win_but_a_valid_reply_does() {
    // The gateway is being resolved; the attacker's malformed or off-target
    // replies arrive first, the genuine one afterwards.
    let s = Script {
        resolves: vec![(0, GW)],
        frames: vec![
            rxf(100, gw_reply(EVIL, GW, MAC, IP + 5)),
            rxf(200, gw_reply(EVIL, GW, EVIL, IP)),
            rxf(300, gw_reply(EVIL, GW + 1, MAC, IP)), // reply for another address
            rxf(400, eth(BCAST, EVIL, &arp(1, EVIL, GW, 0, IP))), // request from "the gateway"
            rxf(500, gw_reply(GW_MAC, GW, MAC, IP)),
        ],
        lookups: vec![(450, GW), (450, GW + 1), (700, GW), (710, GW + 1)],
        ..Default::default()
    };
    let o = run(&s, 800, always);
    assert_eq!(rsp(&o), vec![(false, 0), (false, 0), (true, GW_MAC), (false, 0)]);
}

#[test]
fn arp_flooding_cannot_evict_the_gateway() {
    // F6: 8 distinct hosts send requests and unsolicited replies: the
    // gateway entry survives.
    let mut s = Script { resolves: vec![(0, GW)], ..Default::default() };
    s.frames.push(rxf(100, gw_reply(GW_MAC, GW, MAC, IP)));
    for k in 0..8u32 {
        let m = 0x0200_0000_0300 + k as u64;
        let t = 300 + 100 * k as usize;
        if k % 2 == 0 {
            s.frames.push(rxf(t, eth(BCAST, m, &arp(1, m, 0xC000_0400 + k, 0, IP))));
        } else {
            s.frames.push(rxf(t, gw_reply(m, 0xC000_0400 + k, MAC, IP)));
        }
    }
    s.lookups.push((1500, GW));
    s.lookups.push((1510, 0xC000_0400));
    let o = run(&s, 1600, always);
    assert_eq!(rsp(&o), vec![(true, GW_MAC), (false, 0)]);
}

#[test]
fn arp_reply_after_the_timeout_is_not_learned() {
    let s = Script {
        resolves: vec![(0, GW)],
        frames: vec![rxf(400, gw_reply(GW_MAC, GW, MAC, IP))],
        lookups: vec![(600, GW)],
        ..Default::default()
    };
    let o = run_t::<Sim>(&s, 700, always);
    assert_eq!(rsp(&o), vec![(false, 0)]);
    // On time (before the 300-cycle timeout) the same reply is learned.
    let s = Script {
        resolves: vec![(0, GW)],
        frames: vec![rxf(150, gw_reply(GW_MAC, GW, MAC, IP))],
        lookups: vec![(600, GW)],
        ..Default::default()
    };
    let o = run_t::<Sim>(&s, 700, always);
    assert_eq!(rsp(&o), vec![(true, GW_MAC)]);
}
