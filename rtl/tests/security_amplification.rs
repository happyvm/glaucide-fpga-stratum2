// SPDX-License-Identifier: Apache-2.0
//! Security review (task T5.5), reflection and amplification: what an
//! attacker who sends spoofed requests can make the server send to a third
//! party, and what a spoofed flood does to the service of a real client.
//! Findings and context: `docs/security.md`.
//!
//! Two benches:
//!
//! * the whole server top on its MII pins (`security_common::PinBench`,
//!   board configuration `Test`, rate limit 8 per second and per source,
//!   KoD off, as on the board): request sizes, modes and versions, the rate
//!   limiter seen from the wire;
//! * `NtpServer` alone (cheap to run for hundreds of requests): the
//!   per-bucket and global capacity of the rate limiter, KoD sizes.
mod security_common;

use ntp_fpga::ntp_server::{NtpReplyMeta, NtpServer, KOD_RATE, RL_ENTRIES};
use ntp_fpga::stream::{FrameByte, Meta};
use ntp_fpga::top::FixedSync;
use security_common::*;
use std::collections::VecDeque;
use txhdl::comp::{chan, signal, tie, DefaultClock, Running, Unit};
use txhdl::types::{Bit, U};

/// The attacker's (or client's) MAC, as seen by us: the router's MAC for
/// off-link sources.
const PEER_MAC: u64 = 0x0200_0000_0002;

/// The 6-bit rate-limiter index of an IPv4 source (`ntp_server::src_hash`,
/// recomputed from its header: XOR fold of the zero-extended address).
fn bucket4(ip: u32) -> u32 {
    let k = ip as u64;
    let mut h = 0;
    for i in (0..64).step_by(6) {
        h ^= (k >> i) & 0x3F;
    }
    h as u32
}

/// IPv4 sources 198.51.100.k, k in 0..64: 64 distinct buckets.
fn src_k(k: u32) -> u32 {
    0xC633_6400 | k
}

/// Wire bytes of an Ethernet frame as sent (padded to 60, plus FCS).
fn wire_len(f: &[u8]) -> usize {
    f.len().max(60) + 4
}

/// NTP replies among the transmitted frames, parsed.
fn ntp_replies(sent: &[(Vec<u8>, u64)]) -> Vec<Sent> {
    sent.iter()
        .map(|(b, t)| parse_sent(b, *t))
        .filter(|s| matches!(&s.udp, Some(u) if u.3 == 123))
        .collect()
}

/// Xmit timestamp of request `k` (unique, so a reply names its request
/// through its Origin field).
fn xmit(k: u64) -> u64 {
    0xDEAD_0000_0000_0000 | k
}

fn origin(s: &Sent) -> u64 {
    let p = &s.udp.as_ref().unwrap().5;
    u64::from_be_bytes(p[24..32].try_into().unwrap())
}

// ------------------------------------------------------------ sizes

/// For a sweep of request sizes (UDP payload 48..=1472 over IPv4,
/// 48..=1452 over IPv6), every reply is a single 48-byte NTP packet and
/// its frame is never longer on the wire than the request's frame: the
/// byte amplification factor is at most 1 (exactly 1 for a 48-byte
/// request: 90 + 4 bytes each way over IPv4, 110 + 4 over IPv6).
#[test]
fn reply_never_larger_than_request() {
    let sizes4 = [48usize, 49, 56, 68, 100, 256, 576, 1000, 1400, 1471, 1472];
    let sizes6 = [48usize, 49, 64, 300, 1000, 1451, 1452];
    let mut b = PinBench::new();
    let mut reqs: Vec<(u64, usize)> = vec![]; // (xmit, wire bytes)
    let mut k = 0u64;
    for &n in &sizes4 {
        let f = ntp4_frame(PEER_MAC, src_k(k as u32), 40000, &ntp_payload(0x23, n, xmit(k)));
        assert!(f.len() <= 1514);
        reqs.push((xmit(k), wire_len(&f)));
        b.frame(&f, 400, false);
        k += 1;
    }
    for &n in &sizes6 {
        let src = 0x2001_0DB8_0000_0000_0000_0000_0000_0002u128 | ((k as u128) << 64);
        let f = ntp6_frame(PEER_MAC, src, 40000, &ntp_payload(0x23, n, xmit(k)));
        assert!(f.len() <= 1514);
        reqs.push((xmit(k), wire_len(&f)));
        b.frame(&f, 400, false);
        k += 1;
    }
    let sent = b.run::<FixedSync>(3000);
    assert_eq!(sent.len(), reqs.len(), "one reply per request, nothing else");
    let mut total_in = 0;
    let mut total_out = 0;
    for (raw, t) in &sent {
        let s = parse_sent(raw, *t);
        let u = s.udp.as_ref().expect("an NTP reply");
        assert_eq!(u.3, 123, "source port 123");
        assert_eq!(u.5.len(), 48, "a 48-byte reply whatever the request size");
        assert_eq!(u.5[0] & 7, 4, "mode 4");
        let (_, req_len) = reqs.iter().find(|r| r.0 == origin(&s)).expect("reply to a request");
        assert!(raw.len() <= *req_len, "reply {} bytes > request {}", raw.len(), req_len);
        total_in += req_len;
        total_out += raw.len();
    }
    println!(
        "{} requests, {} bytes in, {} bytes out (FCS included): factor {:.3}",
        reqs.len(),
        total_in,
        total_out,
        total_out as f64 / total_in as f64
    );
}

// ------------------------------------------------------------ modes

/// Only mode 3 with VN 3 or 4 and at least 48 bytes is answered. In
/// particular modes 6 (control: `ntpq`, `readvar`) and 7 (private: `ntpdc`,
/// `monlist`, the 2014 amplification vector with a factor up to ~550) never
/// get a reply, whatever their size; neither do short mode-3 packets, mode
/// 4 (no reply loop with another server or with an echo service), mode 5
/// (broadcast) or symmetric modes 1/2.
#[test]
fn only_client_mode_is_answered() {
    let mut b = PinBench::new();
    let mut k = 0u32;
    let mut expect = vec![];
    let mut push = |b: &mut PinBench, pay: Vec<u8>, answer: bool| {
        let f = ntp4_frame(PEER_MAC, src_k(k % 64), 40000 + k as u16, &pay);
        b.frame(&f, 200, false);
        if answer {
            expect.push(40000 + k as u16);
        }
        k += 1;
    };
    for vn in 0..8u8 {
        for mode in 0..8u8 {
            let ok = mode == 3 && (vn == 3 || vn == 4);
            push(&mut b, ntp_payload((vn << 3) | mode, 48, 1), ok);
        }
    }
    // Control (mode 6) READVAR, 12-byte header; private (mode 7) MON_GETLIST_1
    // (implementation 3, request code 42) of 8, 48 and 192 bytes.
    push(&mut b, vec![0x16, 0x02, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0], false);
    for n in [8usize, 48, 192] {
        let mut p = vec![0u8; n];
        p[0] = 0x17;
        p[2] = 3;
        p[3] = 42;
        push(&mut b, p, false);
    }
    // Short client requests (1, 12, 47 bytes) and a request ending inside
    // the Transmit field.
    for n in [1usize, 12, 44, 47] {
        push(&mut b, ntp_payload(0x23, n, 1), false);
    }
    // Our own reply sent back to us (an echo service reflecting it, or a
    // spoofed request from another NTP server's port 123): mode 4, dropped.
    let sent0 = {
        let mut b0 = PinBench::new();
        b0.frame(&ntp4_frame(PEER_MAC, src_k(1), 7, &ntp_payload(0x23, 48, 5)), 100, false);
        b0.run::<FixedSync>(1500)
    };
    let echoed = ntp_replies(&sent0)[0].udp.as_ref().unwrap().5.clone();
    push(&mut b, echoed, false);
    let sent = b.run::<FixedSync>(2000);
    let got: Vec<u16> = ntp_replies(&sent)
        .iter()
        .map(|s| s.udp.as_ref().unwrap().4)
        .collect();
    assert_eq!(sent.len(), got.len(), "only NTP replies are transmitted");
    assert_eq!(got, expect, "replies (by client port)");
}

// ------------------------------------------------------------ rate limiter, wire view

/// The rate limiter seen from the wire (8 replies per second and per
/// bucket): a source over the limit gets silence (KoD off), other buckets
/// are not affected.
///
/// Victim scenario (finding F2 of `docs/security.md`): the limiter is keyed
/// on the claimed source address, which an attacker forges. 8 spoofed
/// requests "from" the victim V (they arrive from the attacker's MAC, or
/// from the router for an off-link attacker) use up V's allowance: the 8
/// replies go to V (or back to the attacker's MAC on the link), and V's own
/// request in the same second gets no answer. A source W that merely shares
/// V's 6-bit bucket is denied as well. A source in another bucket is served.
#[test]
fn rate_limiter_and_spoofed_victim() {
    const ATTACKER_MAC: u64 = 0x0200_0000_0bad;
    const VICTIM_MAC: u64 = 0x0200_0000_0003;
    let v = 0xC000_0203u32; // 192.0.2.3, the victim (on link here)
    // W: another address with V's bucket (differs in two 6-bit chunks by
    // the same pattern, so the fold cancels).
    let w = v ^ (0x15 << 6) ^ 0x15;
    assert_ne!(w, v);
    assert_eq!(bucket4(w), bucket4(v));
    let other = src_k(bucket4(v) ^ 1);
    assert_ne!(bucket4(other), bucket4(v));
    let mut b = PinBench::new();
    let mut tag = 0u64;
    let mut frames: Vec<(&str, u64, u64)> = vec![]; // (who, xmit, eth src)
    let mut send = |b: &mut PinBench, who: &'static str, mac: u64, ip: u32| {
        b.frame(&ntp4_frame(mac, ip, 40000, &ntp_payload(0x23, 48, xmit(tag))), 300, false);
        frames.push((who, xmit(tag), mac));
        tag += 1;
    };
    // A single source over the limit, interleaved with another bucket.
    for i in 0..12 {
        send(&mut b, "flood", PEER_MAC, src_k(5));
        if i % 3 == 0 {
            send(&mut b, "bystander", PEER_MAC, src_k(6));
        }
    }
    // The spoofed victim: 8 forged requests, then the real ones.
    for _ in 0..8 {
        send(&mut b, "spoofed-as-victim", ATTACKER_MAC, v);
    }
    send(&mut b, "victim", VICTIM_MAC, v);
    send(&mut b, "same-bucket", PEER_MAC, w);
    send(&mut b, "other-bucket", PEER_MAC, other);
    let sent = b.run::<FixedSync>(2000);
    let replies = ntp_replies(&sent);
    assert_eq!(sent.len(), replies.len());
    let count = |who: &str| {
        replies
            .iter()
            .filter(|s| frames.iter().any(|f| f.0 == who && f.1 == origin(s)))
            .count()
    };
    assert_eq!(count("flood"), 8, "a source gets at most 8 replies per second");
    assert_eq!(count("bystander"), 4, "another bucket is not affected");
    assert_eq!(count("spoofed-as-victim"), 8, "the forged requests are answered");
    assert_eq!(count("victim"), 0, "the real victim is denied (F2)");
    assert_eq!(count("same-bucket"), 0, "a source of the same bucket is denied (F2)");
    assert_eq!(count("other-bucket"), 1);
    // Where the 8 replies to the forged requests went: IP destination V,
    // Ethernet destination the MAC the forged frames came from.
    for s in replies.iter().filter(|s| frames.iter().any(|f| f.0 == "spoofed-as-victim" && f.1 == origin(s))) {
        let u = s.udp.as_ref().unwrap();
        assert_eq!(u.2, v as u128, "reflected to the victim's address");
        assert_eq!(s.dst_mac, ATTACKER_MAC, "on the link, to the sender's MAC");
    }
}

// ------------------------------------------------------------ NtpServer alone

/// A request for the unit bench: (v6, src, payload).
type UReq = (bool, u128, Vec<u8>);

/// Plays requests into `NtpServer` (all outputs always ready), with the
/// given rate limit and KoD setting, the clock frozen inside one second;
/// returns (meta, 48 bytes) per reply, in order.
fn run_server(reqs: &[UReq], rate: u16, kod: bool) -> Vec<(NtpReplyMeta, Vec<u8>)> {
    let (m4_t, m4) = chan::<Meta, DefaultClock>();
    let (r4_t, r4) = chan::<FrameByte, DefaultClock>();
    let (m6_t, m6) = chan::<Meta, DefaultClock>();
    let (r6_t, r6) = chan::<FrameByte, DefaultClock>();
    let (time_o, time_i) = signal::<U<96>, DefaultClock>();
    let (om_t, om) = chan::<NtpReplyMeta, DefaultClock>();
    let (ob_t, ob) = chan::<FrameByte, DefaultClock>();
    let mut d = NtpServer::default();
    let mut sim = Running::new(d.run(
        (
            m4,
            r4,
            m6,
            r6,
            time_i,
            tie(Bit::One),
            tie(U::<8>::from(2u8)),
            tie(U::<32>::from(0xC000_0263u32)),
            tie(U::<32>::from(0xB70A_6091u32)),
            tie(U::<64>::from(0xECC5_8CC0_0000_0000u64)),
            tie(U::<32>::from(0x333u32)),
            tie(U::<32>::from(0x666u32)),
            tie(U::<8>::from(0xE8u8)),
            tie(U::<64>::from(0u64)),
            tie(U::<16>::from(rate)),
            tie(Bit::from_bool(kod)),
        ),
        (om_t, ob_t),
    ));
    time_o.set(U::<96>::new((0xECC5_8CC0_4000_0000u128) << 32));
    // One family at a time, in input order (meta, then bytes).
    let mut q: VecDeque<(usize, usize)> = VecDeque::new();
    for (i, r) in reqs.iter().enumerate() {
        q.push_back((i, usize::MAX));
        for k in 0..r.2.len() {
            q.push_back((i, k));
        }
    }
    let mut metas = VecDeque::new();
    let mut outs = vec![];
    let mut open = vec![];
    let cycles = reqs.iter().map(|r| r.2.len() + 60).sum::<usize>() + 200;
    for _ in 0..cycles {
        if let Some(&(i, k)) = q.front() {
            let (v6, src, pay) = &reqs[i];
            let (mt, bt) = if *v6 { (&m6_t, &r6_t) } else { (&m4_t, &r4_t) };
            if k == usize::MAX {
                if mt.ready().to_bool() {
                    mt.send(Meta {
                        family: Bit::from_bool(*v6),
                        src: U::<128>::new(*src),
                        dst: U::<128>::new(if *v6 { OUR_GUA } else { OUR_IP4 as u128 }),
                        sport: U::<16>::from(40000u16),
                        dport: U::<16>::from(123u16),
                        len: U::<16>::from(pay.len() as u16),
                        eth_src: U::<48>::from(PEER_MAC),
                        rx_ts: U::<64>::from(0xECC5_8CC0_4000_0000u64),
                    });
                    q.pop_front();
                }
            } else if bt.ready().to_bool() {
                let last = k + 1 == pay.len();
                bt.send(FrameByte::new(pay[k], last, false));
                q.pop_front();
            }
        }
        if let Some(m) = om.recv() {
            metas.push_back(m);
        }
        if let Some(x) = ob.recv() {
            open.push(x.data.raw() as u8);
            if x.last.to_bool() {
                assert_eq!(open.len(), 48, "every reply is 48 bytes");
                outs.push((metas.pop_front().expect("meta first"), std::mem::take(&mut open)));
            }
        }
        sim.cycle();
    }
    assert!(q.is_empty(), "requests not all taken");
    assert!(open.is_empty() && metas.is_empty());
    outs
}

/// Global capacity (finding F1): the table has 64 buckets, so at most
/// 64 x 8 = 512 normal replies leave the server per second, whatever the
/// number of clients; and an attacker who forges 9 requests for each of the
/// 64 buckets (576 small packets, about 60 kB) silences every client for
/// the rest of that second. The test shows both: 64 x 9 forged requests get
/// 512 replies, then 64 requests from fresh, distinct sources get none.
#[test]
fn limiter_capacity_and_global_denial() {
    assert_eq!(RL_ENTRIES, 64);
    let mut reqs: Vec<UReq> = vec![];
    for round in 0..9u32 {
        for k in 0..64u32 {
            // A different forged address each time, bucket k.
            let ip = (0x0A00_0000 | (round << 12)) ^ k ^ bucket4(0x0A00_0000 | (round << 12));
            assert_eq!(bucket4(ip), k);
            reqs.push((false, ip as u128, ntp_payload(0x23, 48, 1)));
        }
    }
    // Then legitimate clients, one request each, all distinct addresses.
    for k in 0..64u32 {
        reqs.push((false, 0xC633_6400 | k as u128, ntp_payload(0x23, 48, 2)));
    }
    let outs = run_server(&reqs, 8, false);
    let forged = outs.iter().filter(|o| o.1[24..32] == 1u64.to_be_bytes()).count();
    let legit = outs.iter().filter(|o| o.1[24..32] == 2u64.to_be_bytes()).count();
    assert_eq!(forged, 64 * 8, "capacity: 8 replies per bucket per second");
    assert_eq!(legit, 0, "every legitimate client is denied for the second (F1)");
}

/// IPv6 sources are keyed on their /64 (finding F2): two addresses of one
/// /64 share a counter (an attacker inside a client's /64, or spoofing it,
/// denies the whole /64); different /64s with the same fold collide too.
#[test]
fn ipv6_limiter_keyed_on_slash64() {
    let a = 0x2001_0DB8_0000_0001_0000_0000_0000_0001u128;
    let b = 0x2001_0DB8_0000_0001_DEAD_BEEF_0000_0002u128; // same /64
    let c = 0x2001_0DB8_0000_0002_0000_0000_0000_0001u128; // other /64
    let mut reqs: Vec<UReq> = (0..8).map(|_| (true, a, ntp_payload(0x23, 48, 1))).collect();
    reqs.push((true, b, ntp_payload(0x23, 48, 2)));
    reqs.push((true, c, ntp_payload(0x23, 48, 3)));
    let outs = run_server(&reqs, 8, false);
    let org: Vec<u8> = outs.iter().map(|o| o.1[31]).collect();
    assert_eq!(org, vec![1, 1, 1, 1, 1, 1, 1, 1, 3], "b shares a's /64 counter");
}

/// With KoD enabled (not the board's setting) every limited request gets a
/// 48-byte KoD RATE: never larger than the request, one per request (no
/// amplification, but no reduction of the reflected volume either), with
/// Receive = Transmit = the request's Transmit (no time leaked).
#[test]
fn kod_is_never_larger_than_the_request() {
    let mut reqs: Vec<UReq> = vec![];
    for (i, n) in [48usize, 48, 200, 1472].iter().enumerate() {
        reqs.push((false, 0xC000_0202, ntp_payload(0x23, *n, 0x1111_0000 + i as u64)));
    }
    let outs = run_server(&reqs, 1, true);
    assert_eq!(outs.len(), 4, "one normal reply, then one KoD per request");
    for (i, (m, p)) in outs.iter().enumerate() {
        assert_eq!(m.len.raw(), 48);
        let kod = i > 0;
        assert_eq!(p[1] == 0, kod, "stratum 0 = KoD");
        if kod {
            assert_eq!(u32::from_be_bytes(p[12..16].try_into().unwrap()), KOD_RATE);
            assert_eq!(p[0] >> 6, 3, "LI 3");
            assert_eq!(&p[32..40], &p[24..32], "Receive = Origin");
            assert_eq!(&p[40..48], &p[24..32], "Transmit = Origin");
        }
    }
}

// ------------------------------------------------------------ source addresses

/// Requests whose source address cannot be a real client (RFC 1122
/// 3.2.1.3, RFC 4291 2.5.2/2.5.3, RFC 6890): loopback, our own address,
/// the subnet broadcast, class E; over IPv6 the unspecified address, the
/// loopback, our own address and IPv4-mapped addresses. A reply to them is
/// useless at best and, for the subnet broadcast, a reflection towards a
/// whole link. Desired behaviour: no reply. Finding F8: they are answered
/// (only 0.0.0.0, 224/4, 255.255.255.255 and ff00::/8 are filtered today).
#[test]
#[ignore = "FINDING F8: requests from martian source addresses are answered (docs/security.md)"]
fn no_reply_to_martian_sources() {
    let v4 = [
        0x7F00_0001u32, // 127.0.0.1
        OUR_IP4,        // ourselves
        0xC000_02FF,    // 192.0.2.255, our subnet's broadcast
        0xF000_0001,    // 240.0.0.1, class E
    ];
    let v6 = [
        0u128,                                     // ::
        1,                                         // ::1
        OUR_GUA,                                   // ourselves
        0x0000_0000_0000_0000_0000_FFFF_C000_0202, // ::ffff:192.0.2.2
    ];
    let mut b = PinBench::new();
    for (k, &ip) in v4.iter().enumerate() {
        b.frame(&ntp4_frame(PEER_MAC, ip, 40000, &ntp_payload(0x23, 48, xmit(k as u64))), 300, false);
    }
    for (k, &ip) in v6.iter().enumerate() {
        b.frame(&ntp6_frame(PEER_MAC, ip, 40000, &ntp_payload(0x23, 48, xmit(10 + k as u64))), 300, false);
    }
    let sent = b.run::<FixedSync>(1500);
    let answered: Vec<u128> = ntp_replies(&sent).iter().map(|s| s.udp.as_ref().unwrap().2).collect();
    assert!(answered.is_empty(), "replies sent to {answered:x?}");
}

/// What happens today with those sources (the measured behaviour behind
/// F8): every one of them is answered.
#[test]
fn martian_sources_today() {
    let v4 = [0x7F00_0001u32, OUR_IP4, 0xC000_02FF, 0xF000_0001];
    let v6 = [0u128, 1, OUR_GUA, 0x0000_0000_0000_0000_0000_FFFF_C000_0202];
    let mut b = PinBench::new();
    for (k, &ip) in v4.iter().enumerate() {
        b.frame(&ntp4_frame(PEER_MAC, ip, 40000, &ntp_payload(0x23, 48, xmit(k as u64))), 300, false);
    }
    for (k, &ip) in v6.iter().enumerate() {
        b.frame(&ntp6_frame(PEER_MAC, ip, 40000, &ntp_payload(0x23, 48, xmit(10 + k as u64))), 300, false);
    }
    let sent = b.run::<FixedSync>(1500);
    let mut answered: Vec<u128> = ntp_replies(&sent).iter().map(|s| s.udp.as_ref().unwrap().2).collect();
    answered.sort();
    println!("answered: {answered:x?}");
    let mut want: Vec<u128> = v4.iter().map(|&a| a as u128).chain(v6).collect();
    want.sort();
    assert_eq!(answered, want);
}

/// Reflection towards UDP services (finding F8): a request forged with the
/// source port of an echo (7), daytime (13), quote (17), chargen (19) or
/// time (37) service, or port 0, is answered. Echo sends our 48-byte mode-4
/// reply back: dropped (no loop, see `only_client_mode_is_answered`). But
/// a text service answers with ASCII lines, and a line starting with `c`
/// (0x63: LI 1, VN 4, mode 3) or `[` (0x5b: VN 3, mode 3) of 48 bytes or
/// more is a valid request: the server answers it, to the service again,
/// which answers again: a loop between us and the service, slowed only by
/// the rate limiter (8 per second). The test shows both halves.
#[test]
fn service_ports_and_text_loops_today() {
    let mut b = PinBench::new();
    let mut k = 0;
    for port in [0u16, 7, 13, 17, 19, 37] {
        b.frame(&ntp4_frame(PEER_MAC, src_k(20 + k), port, &ntp_payload(0x23, 48, xmit(k as u64))), 300, false);
        k += 1;
    }
    // A chargen line (RFC 864: 72 printable characters and CR LF) starting
    // with 'c', arriving from port 19.
    let line: Vec<u8> = (0..72u8).map(|i| b'c' + (i % 30)).chain([b'\r', b'\n']).collect();
    b.frame(&ntp4_frame(PEER_MAC, src_k(40), 19, &line), 300, false);
    let sent = b.run::<FixedSync>(1500);
    let ports: Vec<u16> = ntp_replies(&sent).iter().map(|s| s.udp.as_ref().unwrap().4).collect();
    assert_eq!(ports, vec![0, 7, 13, 17, 19, 37, 19], "every one answered, the chargen line too");
}

/// Desired behaviour behind F8: no reply to source port 0 or to the ports
/// of the small UDP services (7, 13, 17, 19, 37), as for martian addresses.
#[test]
#[ignore = "FINDING F8: requests from UDP service ports are answered (docs/security.md)"]
fn no_reply_to_service_ports() {
    let mut b = PinBench::new();
    for (k, port) in [0u16, 7, 13, 17, 19, 37].into_iter().enumerate() {
        b.frame(&ntp4_frame(PEER_MAC, src_k(20 + k as u32), port, &ntp_payload(0x23, 48, xmit(k as u64))), 300, false);
    }
    let sent = b.run::<FixedSync>(1500);
    assert!(ntp_replies(&sent).is_empty());
}
