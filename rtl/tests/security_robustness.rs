// SPDX-License-Identifier: Apache-2.0
//! Security review (task T5.5): robustness of the whole receive/transmit
//! pipeline against malformed, truncated, oversized and hostile frames,
//! injected on the MII pins of the server top (`security_common::PinBench`,
//! real `mii_rx`, `rx_path`, `fcs_check`, `eth_rx`, IPv4/IPv6/ARP/ICMP/UDP
//! parsers, `ntp_server`, the transmit path and `mii_tx`).
//!
//! Each test plays a fixed-seed random campaign in batches; after every
//! batch, and an idle time for the queues to drain, a known-good NTP
//! request (the *probe*, unique Transmit timestamp) is sent. Checks:
//!
//! * every probe is answered (no unit is stuck, no channel blocked forever,
//!   no stamp lost: the pipeline is alive after every batch);
//! * every transmitted frame has a good FCS (and good IPv4/UDP checksums)
//!   and is *explained* by a received frame: an ARP reply to an ARP request
//!   for our address, an echo reply carrying the request's data, an NA for
//!   our address answering an NS, or a 48-byte mode-4 NTP reply whose
//!   Origin is the Transmit of a received mode-3 request from that address
//!   and port. Nothing else may leave the board;
//! * in bytes, the transmitted volume never exceeds the received volume.
//!
//! Debug build: each test is about 150 000 to 250 000 simulated cycles
//! (about 1 300 frames in all, under a minute each).
mod security_common;

use ntp_fpga::top::FixedSync;
use security_common::*;

const PEER_MAC: u64 = 0x0200_0000_0002;
const BCAST: u64 = 0xFFFF_FFFF_FFFF;

fn be(b: &[u8]) -> u128 {
    b.iter().fold(0u128, |a, &x| (a << 8) | x as u128)
}

// ------------------------------------------------------------ oracle

/// Offset of the IP payload and (family 6?, src, dst, proto) of an
/// Ethernet frame without FCS, when it parses as IPv4 (IHL from the
/// header) or IPv6 (no extension header).
fn ip_of(f: &[u8]) -> Option<(usize, bool, u128, u128, u8)> {
    if f.len() < 14 {
        return None;
    }
    match u16::from_be_bytes([f[12], f[13]]) {
        0x0800 if f.len() >= 34 => {
            let ihl = (f[14] & 15) as usize * 4;
            Some((14 + ihl, false, be(&f[26..30]), be(&f[30..34]), f[23]))
        }
        0x86dd if f.len() >= 54 => Some((54, true, be(&f[22..38]), be(&f[38..54]), f[20])),
        _ => None,
    }
}

/// Is the transmitted frame `s` a legitimate answer to the received `inp`?
fn explains(inp: &[u8], s: &Sent) -> bool {
    let o = &s.bytes;
    match s.ethertype {
        0x0806 => {
            // ARP reply from us to the sender of a request for our address.
            inp.len() >= 42
                && inp[12..14] == [0x08, 0x06]
                && inp[20..22] == [0, 1]
                && be(&inp[38..42]) == OUR_IP4 as u128
                && o[20..22] == [0, 2]
                && be(&o[22..28]) == OUR_MAC as u128
                && be(&o[28..32]) == OUR_IP4 as u128
                && o[32..42] == inp[22..32]
                && s.dst_mac as u128 == be(&inp[22..28])
        }
        0x0800 | 0x86dd => {
            let Some((po, v6, src, dst, proto)) = ip_of(o) else { return false };
            let Some((pi, iv6, isrc, idst, iproto)) = ip_of(inp) else { return false };
            if v6 != iv6 || proto != iproto {
                return false;
            }
            let ours = if v6 { src == OUR_GUA || src == OUR_LL } else { src == OUR_IP4 as u128 };
            if !ours {
                return false;
            }
            if let Some(u) = &s.udp {
                // NTP reply: 48 bytes, mode 4, Origin = the request's Transmit.
                let ip_len = if v6 {
                    u16::from_be_bytes([inp[18], inp[19]]) as usize + 40
                } else {
                    u16::from_be_bytes([inp[16], inp[17]]) as usize
                };
                let ui = pi;
                if inp.len() < ui + 8 + 48 || ip_len < (ui - 14) + 8 + 48 {
                    return false;
                }
                let ipay = &inp[ui + 8..];
                return u.3 == 123
                    && u.5.len() == 48
                    && u.5[0] & 7 == 4
                    && isrc == dst
                    && idst == src
                    && u16::from_be_bytes([inp[ui], inp[ui + 1]]) == u.4
                    && u16::from_be_bytes([inp[ui + 2], inp[ui + 3]]) == 123
                    && ipay[0] & 7 == 3
                    && u.5[24..32] == ipay[40..48];
            }
            if !v6 && proto == 1 {
                // ICMP echo reply: same identifier, sequence and data.
                let ilen = u16::from_be_bytes([inp[16], inp[17]]) as usize;
                let olen = u16::from_be_bytes([o[16], o[17]]) as usize;
                return o[po] == 0
                    && inp.get(pi) == Some(&8)
                    && ilen == olen
                    && isrc == dst
                    && inp.len() >= 14 + ilen
                    && o[po + 4..14 + olen] == inp[pi + 4..14 + ilen];
            }
            if v6 && proto == 58 {
                let olen = u16::from_be_bytes([o[18], o[19]]) as usize;
                let ilen = u16::from_be_bytes([inp[18], inp[19]]) as usize;
                if inp.len() < 54 + ilen.min(inp.len() - 54) {
                    return false;
                }
                return match o[po] {
                    129 => {
                        inp.get(pi) == Some(&128)
                            && ilen == olen
                            && inp.len() >= 54 + ilen
                            && isrc == dst
                            && o[po + 4..54 + olen] == inp[pi + 4..54 + ilen]
                    }
                    136 => {
                        let target = be(&o[po + 8..po + 24]);
                        inp.get(pi) == Some(&135)
                            && inp.len() >= pi + 24
                            && be(&inp[pi + 8..pi + 24]) == target
                            && (target == OUR_GUA || target == OUR_LL)
                            && (dst == isrc || dst == 0xff02_0000_0000_0000_0000_0000_0000_0001)
                    }
                    _ => false,
                };
            }
            false
        }
        _ => false,
    }
}

// ------------------------------------------------------------ campaign

fn templates() -> Vec<Vec<u8>> {
    all_rx_frames().into_iter().filter(|f| f.len() >= 14).collect()
}

/// A valid NTP request (IPv4 or IPv6) from a random source, 48..=400 bytes.
fn random_request(r: &mut Rng) -> Vec<u8> {
    let n = 48 + r.below(353);
    let pay = ntp_payload(if r.below(2) == 0 { 0x23 } else { 0x1b }, n, r.next());
    if r.below(2) == 0 {
        // 10.x.y.z: random buckets, a few per bucket at most in a run.
        let ip = 0x0A00_0000 | (r.next() as u32 & 0x00FF_FFC0) | r.below(16) as u32;
        ntp4_frame(PEER_MAC, ip, r.next() as u16, &pay)
    } else {
        let src = 0x2001_0DB8_00AA_0000_0000_0000_0000_0000u128 | (r.next() as u128);
        ntp6_frame(PEER_MAC, src, r.next() as u16, &pay)
    }
}

/// Recomputes the IPv4 header checksum of a frame (IHL from the header).
fn fix_ip4(f: &mut [u8]) {
    let ihl = (f[14] & 15) as usize * 4;
    if f.len() < 14 + ihl.max(20) {
        return;
    }
    f[24] = 0;
    f[25] = 0;
    let c = !fold(sum16(0, &f[14..14 + ihl.max(20)]));
    f[24] = (c >> 8) as u8;
    f[25] = c as u8;
}

/// A structurally malformed IP/UDP/ICMP frame, built from valid ones so
/// that it passes the checks before the one under test.
fn malformed(r: &mut Rng) -> Vec<u8> {
    let pay = ntp_payload(0x23, 48, r.next());
    let v4 = ntp4_frame(PEER_MAC, 0xC000_0202, 40000, &pay);
    let v6 = ntp6_frame(PEER_MAC, 0x2001_0DB8_0000_0000_0000_0000_0000_0002, 40000, &pay);
    match r.below(16) {
        0 => {
            // IPv4 options (IHL 6..15), checksum fixed.
            let n = 1 + r.below(10);
            let mut f = v4[..34].to_vec();
            f[14] = 0x45 + n as u8;
            f.extend(r.bytes(4 * n));
            f.extend_from_slice(&v4[34..]);
            let tl = u16::from_be_bytes([f[16], f[17]]) as usize + 4 * n;
            f[16..18].copy_from_slice(&(tl as u16).to_be_bytes());
            fix_ip4(&mut f);
            f
        }
        1 => {
            // Fragment: MF and/or offset.
            let mut f = v4.clone();
            let fo = r.below(0x2000) as u16 | if r.below(2) == 0 { 0x2000 } else { 0 };
            f[20..22].copy_from_slice(&fo.to_be_bytes());
            fix_ip4(&mut f);
            f
        }
        2 => {
            // IPv4 total length larger or smaller than the frame.
            let mut f = v4.clone();
            let tl = r.below(0x10000) as u16;
            f[16..18].copy_from_slice(&tl.to_be_bytes());
            fix_ip4(&mut f);
            f
        }
        3 => {
            // UDP length field mismatch (IPv4, checksum 0 = not checked).
            let mut f = v4.clone();
            let ul = r.below(0x10000) as u16;
            f[38..40].copy_from_slice(&ul.to_be_bytes());
            f[40] = 0;
            f[41] = 0;
            f
        }
        4 => {
            // TTL 0, odd protocols, odd sources/destinations.
            let mut f = v4.clone();
            match r.below(5) {
                0 => f[22] = 0,
                1 => f[23] = r.byte(),
                2 => f[26..30].copy_from_slice(&[255, 255, 255, 255]),
                3 => f[26..30].copy_from_slice(&[224 + r.below(16) as u8, 0, 0, 1]),
                _ => f[30..34].copy_from_slice(&[192, 0, 2, 255]),
            }
            fix_ip4(&mut f);
            f
        }
        5 => {
            // IPv6 extension header chain before UDP (hop-by-hop, routing,
            // fragment, destination options, ESP, no next header).
            let mut f = v6[..54].to_vec();
            let kinds = [0u8, 43, 44, 60, 50, 59, 51];
            let n = 1 + r.below(4);
            let mut nh = Vec::new();
            for _ in 0..n {
                nh.push(kinds[r.below(kinds.len())]);
            }
            f[20] = nh[0];
            let mut ext = vec![];
            for i in 0..n {
                let next = if i + 1 < n { nh[i + 1] } else { 17 };
                ext.push(next);
                ext.push(0);
                ext.extend(r.bytes(6));
            }
            f.extend(&ext);
            f.extend_from_slice(&v6[54..]);
            let pl = u16::from_be_bytes([f[18], f[19]]) as usize + ext.len();
            f[18..20].copy_from_slice(&(pl as u16).to_be_bytes());
            f
        }
        6 => {
            // IPv6 payload length mismatch, hop limit 0, multicast source,
            // all-nodes destination, UDP to the solicited-node group.
            let mut f = v6.clone();
            match r.below(5) {
                0 => f[18..20].copy_from_slice(&(r.below(0x10000) as u16).to_be_bytes()),
                1 => f[21] = 0,
                2 => f[22] = 0xff,
                3 => {
                    f[0..6].copy_from_slice(&[0x33, 0x33, 0, 0, 0, 1]);
                    f[38..54].copy_from_slice(&0xff02_0000_0000_0000_0000_0000_0000_0001u128.to_be_bytes());
                }
                _ => {
                    f[0..6].copy_from_slice(&[0x33, 0x33, 0xff, 0, 0, 1]);
                    f[38..54].copy_from_slice(&0xff02_0000_0000_0000_0000_0001_ff00_0001u128.to_be_bytes());
                }
            }
            f
        }
        7 => {
            // IPv6 UDP with checksum 0 (illegal) or a bad one.
            let mut f = v6.clone();
            if r.below(2) == 0 {
                f[60] = 0;
                f[61] = 0;
            } else {
                f[61] ^= 1 << r.below(8);
            }
            f
        }
        8 => {
            // 802.1Q tag in front of a valid request.
            let mut f = v4[..12].to_vec();
            f.extend_from_slice(&[0x81, 0x00, 0, r.byte()]);
            f.extend_from_slice(&v4[12..]);
            f
        }
        9 => {
            // NS for our address with malformed or many options.
            let mut f = vector("ndp_ns_na.hex")[0].1.clone();
            let opts: Vec<u8> = match r.below(4) {
                0 => vec![1, 0, 0, 0, 0, 0, 0, 0],
                1 => vec![1, 3, 1, 2, 3, 4, 5, 6],
                2 => (0..r.below(20)).flat_map(|_| vec![9, 1, 0, 0, 0, 0, 0, 0]).collect(),
                _ => {
                    let n = 8 * r.below(4);
                    r.bytes(n)
                }
            };
            f.truncate(14 + 40 + 24);
            f.extend(&opts);
            let pl = 24 + opts.len();
            f[18..20].copy_from_slice(&(pl as u16).to_be_bytes());
            // fix the ICMPv6 checksum so that only the options are wrong
            f[56] = 0;
            f[57] = 0;
            let mut a = sum16(0, &f[22..54]);
            a = sum16(a, &(pl as u32).to_be_bytes());
            a += 58;
            a = sum16(a, &f[54..54 + pl]);
            let c = !fold(a);
            f[56] = (c >> 8) as u8;
            f[57] = c as u8;
            f
        }
        10 => {
            // ICMP echo of a random size (up to a full frame), checksum good
            // or not: answered at the same size, or aborted.
            let n = if r.below(4) == 0 { 8 + r.below(1472) } else { 8 + r.below(200) };
            let mut icmp = vec![8, 0, 0, 0];
            icmp.extend(r.bytes(n - 4));
            let c = !fold(sum16(0, &icmp));
            icmp[2] = (c >> 8) as u8;
            icmp[3] = c as u8;
            if r.below(3) == 0 {
                icmp[n - 1] ^= 0x40;
            }
            let mut ip = ipv4_hdr(0xC000_0202, OUR_IP4, 1, 64, icmp.len());
            ip.extend(icmp);
            eth(OUR_MAC, PEER_MAC, 0x0800, &ip)
        }
        11 => {
            // ARP with odd fields (hlen, plen, op, htype), for us.
            let mut f = vector("arp_request_reply.hex")[0].1.clone();
            let i = [14usize, 15, 16, 17, 18, 19, 20, 21][r.below(8)];
            f[i] = r.byte();
            f
        }
        12 => {
            // Runt: 1..=17 bytes (shorter than a header).
            let n = 1 + r.below(17);
            v4[..n].to_vec()
        }
        13 => {
            // Jumbo: a valid request followed by garbage up to 1519..2000
            // bytes (mii_rx cuts after 1518 and flags err).
            let mut f = v4.clone();
            let n = 1519 + r.below(100);
            f.extend(r.bytes(n - f.len()));
            f
        }
        14 => {
            // Valid request with an NTP extension field / MAC (ignored).
            let mut p = ntp_payload(0x23, 48, r.next());
            let n = 4 * (1 + r.below(100));
            p.extend(r.bytes(n));
            ntp4_frame(PEER_MAC, 0xC000_0202, 40001, &p)
        }
        _ => {
            // Random EtherType, random destination class.
            let dst = [OUR_MAC, BCAST, 0x3333_ff00_0001, 0x3333_0000_0001, r.next() & 0xFFFF_FFFF_FFFF][r.below(5)];
            let et = [0x0800u16, 0x86dd, 0x0806, 0x8100, 0x88cc, 0x8863, r.next() as u16][r.below(7)];
            let n = r.below(200);
            eth(dst, PEER_MAC, et, &r.bytes(n))
        }
    }
}

/// One fuzz frame: (frame without FCS, whether to corrupt its FCS, gap,
/// whether to raise rx_er inside it).
fn fuzz_frame(r: &mut Rng, tpl: &[Vec<u8>], kind_mask: u32) -> (Vec<u8>, bool, usize, bool) {
    let kinds: Vec<u32> = (0..7).filter(|k| kind_mask & (1 << k) != 0).collect();
    let kind = kinds[r.below(kinds.len())];
    let mut fcs_bad = false;
    let mut er = false;
    let f = match kind {
        0 => {
            // Random bytes, random length (runts included), usually with an
            // address and EtherType that some parser accepts.
            let n = 1 + r.below(120);
            let mut f = r.bytes(n);
            if n >= 14 && r.below(4) != 0 {
                let dst = [OUR_MAC, BCAST, 0x3333_ff00_0001][r.below(3)];
                f[0..6].copy_from_slice(&mac_bytes(dst));
                let et = [0x0800u16, 0x86dd, 0x0806][r.below(3)];
                f[12..14].copy_from_slice(&et.to_be_bytes());
            }
            f
        }
        1 => {
            // A golden frame with 1..=4 random byte corruptions, FCS good.
            let mut f = tpl[r.below(tpl.len())].clone();
            for _ in 0..1 + r.below(4) {
                let i = if r.below(4) != 0 { r.below(f.len().min(80)) } else { r.below(f.len()) };
                f[i] ^= 1 << r.below(8) as u8;
            }
            f
        }
        2 => {
            // A golden frame truncated, or extended with random bytes.
            let mut f = tpl[r.below(tpl.len())].clone();
            if r.below(2) == 0 {
                f.truncate(1 + r.below(f.len()));
            } else {
                let n = r.below(200);
                f.extend(r.bytes(n));
            }
            f
        }
        3 => {
            // A golden frame with a bad FCS or an rx_er.
            let f = tpl[r.below(tpl.len())].clone();
            if r.below(2) == 0 {
                fcs_bad = true;
            } else {
                er = true;
            }
            f
        }
        4 => malformed(r),
        5 => random_request(r),
        _ => {
            // Short: a golden frame cut to at most 64 bytes, or up to 64
            // random bytes behind a header some parser accepts.
            if r.below(2) == 0 {
                let mut f = tpl[r.below(tpl.len())].clone();
                f.truncate(1 + r.below(64));
                f
            } else {
                let n = r.below(51);
                let dst = [OUR_MAC, BCAST, 0x3333_ff00_0001][r.below(3)];
                let et = [0x0800u16, 0x86dd, 0x0806][r.below(3)];
                let mut f = eth(dst, PEER_MAC, et, &r.bytes(n));
                f.truncate(14 + n);
                f
            }
        }
    };
    // Mostly line rate (minimum gap: 12 bytes = 24 nibbles), sometimes a
    // gap shorter than the standard allows, sometimes idle.
    let gap = match r.below(8) {
        0 => 2,
        1 => 24 + r.below(400),
        _ => 24,
    };
    (f, fcs_bad, gap, er)
}

/// Probe `k`: a valid IPv4 request from 203.0.113.(16 + k) with a unique
/// Transmit timestamp. The probes use distinct rate-limiter buckets; a
/// random request may share one, far below the limit of 8 per second (the
/// whole run lasts a few ms of simulated time). At most 48 probes per run.
fn probe(seed: u64, k: usize) -> (Vec<u8>, u64) {
    assert!(k < 48);
    let x = 0xFEED_0000_0000_0000 | (seed << 16) | k as u64;
    let ip = 0xCB00_7100u32 | (16 + k as u32);
    (ntp4_frame(PEER_MAC, ip, 50000, &ntp_payload(0x23, 48, x)), x)
}

/// Plays `batches` batches of `per` fuzz frames (kinds in `mask`), each
/// followed by a probe, and checks the oracle. Returns (frames, cycles).
fn campaign(seed: u64, batches: usize, per: usize, mask: u32) -> (usize, usize) {
    let mut r = Rng(seed);
    let tpl = templates();
    let mut b = PinBench::new();
    let mut inputs: Vec<Vec<u8>> = vec![];
    let mut probes = vec![];
    let mut wire_in = 0usize;
    for k in 0..batches {
        for _ in 0..per {
            let (f, bad, gap, er) = fuzz_frame(&mut r, &tpl, mask);
            b.frame(&f, gap, bad);
            if er {
                // rx_er on a random nibble of the frame just appended.
                let n = b.wire.len();
                let len = 2 * (f.len() + 4);
                let i = n - 1 - r.below(len);
                b.wire[i].2 = true;
            }
            wire_in += f.len().max(60) + 4;
            if !bad && !er {
                // What the parsers saw (mii_rx keeps at most 1518 bytes).
                inputs.push(f[..f.len().min(1514)].to_vec());
            }
        }
        // Let the queues drain, then the probe.
        let (p, x) = probe(seed, k);
        b.frame(&p, 4000, false);
        wire_in += p.len().max(60) + 4;
        inputs.push(p);
        probes.push(x);
        b.idle(1000);
    }
    let cycles = b.len() + 6000;
    let sent = b.run::<FixedSync>(6000);
    let mut wire_out = 0;
    let mut answered = vec![false; probes.len()];
    let mut kinds = std::collections::BTreeMap::<String, usize>::new();
    for (raw, t) in &sent {
        let s = parse_sent(raw, *t);
        wire_out += raw.len();
        assert!(
            inputs.iter().any(|i| explains(i, &s)),
            "seed {seed:#x}: unexplained transmitted frame {}",
            raw.iter().map(|x| format!("{x:02x}")).collect::<String>()
        );
        if let Some(u) = &s.udp {
            let org = u64::from_be_bytes(u.5[24..32].try_into().unwrap());
            if let Some(i) = probes.iter().position(|&x| x == org) {
                assert!(!answered[i], "probe {i} answered twice");
                answered[i] = true;
            }
        }
        let kname = match (s.ethertype, &s.udp) {
            (0x0806, _) => "arp".to_string(),
            (_, Some(_)) => "ntp".to_string(),
            (0x0800, None) => "icmp echo".to_string(),
            _ => format!("icmpv6 {}", s.bytes[54]),
        };
        *kinds.entry(kname).or_default() += 1;
    }
    let missing: Vec<usize> = (0..probes.len()).filter(|&i| !answered[i]).collect();
    assert!(missing.is_empty(), "seed {seed:#x}: probes not answered (pipeline stuck?): {missing:?}");
    assert!(
        wire_out <= wire_in,
        "seed {seed:#x}: {wire_out} bytes out for {wire_in} in"
    );
    println!(
        "seed {seed:#x}: {} frames in ({} bytes), {} out ({} bytes, {:?}), {} cycles",
        batches * (per + 1),
        wire_in,
        sent.len(),
        wire_out,
        kinds,
        cycles
    );
    (batches * (per + 1), cycles)
}

/// Random bytes and golden frames with random corruptions, truncations,
/// extensions, bad FCS and rx_er, mostly at line rate.
#[test]
fn fuzz_random_and_corrupted_frames() {
    campaign(0x5EC0_0001, 8, 50, 0b0001111);
}

/// Structurally malformed IPv4/IPv6/UDP/ICMP/ARP/NDP frames (options,
/// fragments, length mismatches, extension header chains, 802.1Q, runts,
/// jumbo frames, odd EtherTypes) mixed with valid requests of random size.
#[test]
fn fuzz_malformed_headers() {
    campaign(0x5EC0_0002, 8, 30, 0b0110000);
}

/// Everything mixed, another seed.
#[test]
fn fuzz_everything_mixed() {
    campaign(0x5EC0_0003, 8, 50, 0b1111111);
}

/// Short frames (at most 64 bytes: runts, cut headers) back to back at
/// line rate, where the parsers' truncation paths are exercised the most.
#[test]
fn fuzz_short_frames_line_rate() {
    campaign(0x5EC0_0004, 8, 120, 0b1000000);
}
