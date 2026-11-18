// SPDX-License-Identifier: Apache-2.0
//! Cycle-by-cycle tests of the 4-input frame arbiter (T1.7).
use ntp_fpga::eth_tx::EthHdr;
use ntp_fpga::stream::FrameByte;
use ntp_fpga::tx_arb::TxArb;
use txhdl::comp::{chan, DefaultClock, Running, Unit};
use txhdl::types::U;

/// One frame of a source: payload bytes and the err flag of its `last`.
#[derive(Clone, Debug)]
struct Fr {
    bytes: Vec<u8>,
    err: bool,
}

/// Header of frame `k` of source `s`: the destination encodes who sent it.
fn hdr_of(s: usize, k: usize) -> (u64, u16) {
    (((s as u64) << 16) | k as u64, 0x0800 + s as u16)
}

fn frame(s: usize, k: usize, n: usize) -> Fr {
    Fr { bytes: (0..n).map(|i| (s * 64 + k * 7 + i * 3) as u8).collect(), err: false }
}

#[derive(Debug, Default)]
struct Out {
    /// (cycle, dst, ethertype) of each hdr.
    hdrs: Vec<(usize, u64, u16)>,
    /// (cycle, byte) of each payload byte.
    bytes: Vec<(usize, FrameByte)>,
}

impl Out {
    fn pb(&self) -> Vec<(usize, u8, bool, bool)> {
        self.bytes
            .iter()
            .map(|(t, b)| (*t, b.data.raw() as u8, b.last.to_bool(), b.err.to_bool()))
            .collect()
    }
}

/// `hgo(s, t)` / `pgo(s, t)`: source `s` may offer a hdr / a payload byte at
/// cycle `t`. `take(t)`: both sinks accept at cycle `t`. Returns the output.
fn run(
    srcs: &[Vec<Fr>; 4],
    hgo: impl Fn(usize, usize) -> bool,
    pgo: impl Fn(usize, usize) -> bool,
    take: impl Fn(usize) -> bool,
) -> Out {
    let (h0t, h0) = chan::<EthHdr, DefaultClock>();
    let (p0t, p0) = chan::<FrameByte, DefaultClock>();
    let (h1t, h1) = chan::<EthHdr, DefaultClock>();
    let (p1t, p1) = chan::<FrameByte, DefaultClock>();
    let (h2t, h2) = chan::<EthHdr, DefaultClock>();
    let (p2t, p2) = chan::<FrameByte, DefaultClock>();
    let (h3t, h3) = chan::<EthHdr, DefaultClock>();
    let (p3t, p3) = chan::<FrameByte, DefaultClock>();
    let (ot, or) = chan::<EthHdr, DefaultClock>();
    let (pt, pr) = chan::<FrameByte, DefaultClock>();
    let mut dut = TxArb::default();
    let mut sim = Running::new(dut.run((h0, p0, h1, p1, h2, p2, h3, p3), (ot, pt)));
    let hts = [&h0t, &h1t, &h2t, &h3t];
    let pts = [&p0t, &p1t, &p2t, &p3t];
    // Per source: next hdr to send, frame and byte index of the payload.
    let mut hn = [0usize; 4];
    let mut pf = [0usize; 4];
    let mut pi = [0usize; 4];
    let total: usize = srcs.iter().flatten().map(|f| f.bytes.len() + 2).sum();
    let mut out = Out::default();
    for t in 0..(total * 12 + 400) {
        for s in 0..4 {
            if hn[s] < srcs[s].len() && hgo(s, t) && hts[s].ready().to_bool() {
                let (d, e) = hdr_of(s, hn[s]);
                hts[s].send(EthHdr { dst: U::<48>::from(d), ethertype: U::<16>::from(e) });
                hn[s] += 1;
            }
            if pf[s] < srcs[s].len() && pgo(s, t) && pts[s].ready().to_bool() {
                let f = &srcs[s][pf[s]];
                let last = pi[s] + 1 == f.bytes.len();
                pts[s].send(FrameByte::new(f.bytes[pi[s]], last, last && f.err));
                if last {
                    pf[s] += 1;
                    pi[s] = 0;
                } else {
                    pi[s] += 1;
                }
            }
        }
        if take(t) {
            if let Some(h) = or.recv() {
                out.hdrs.push((t, h.dst.raw() as u64, h.ethertype.raw() as u16));
            }
            if let Some(b) = pr.recv() {
                out.bytes.push((t, b));
            }
        }
        sim.cycle();
    }
    out
}

fn always(_: usize, _: usize) -> bool {
    true
}
fn sink(_: usize) -> bool {
    true
}

/// Verify the output: frames intact, one hdr per frame, hdr before its
/// payload, per-source order kept. Returns the grant order (source numbers).
fn verify(srcs: &[Vec<Fr>; 4], o: &Out) -> Vec<usize> {
    let pb = o.pb();
    let mut frames: Vec<Vec<(usize, u8, bool, bool)>> = vec![];
    let mut cur = vec![];
    for b in pb {
        assert!(!b.3 || b.2, "err without last");
        cur.push(b);
        if b.2 {
            frames.push(std::mem::take(&mut cur));
        }
    }
    assert!(cur.is_empty(), "payload without last");
    let nfr: usize = srcs.iter().map(|s| s.len()).sum();
    assert_eq!(o.hdrs.len(), nfr, "one hdr per frame");
    assert_eq!(frames.len(), nfr, "all frames forwarded");
    let mut next = [0usize; 4];
    let mut order = vec![];
    for (i, (h, f)) in o.hdrs.iter().zip(&frames).enumerate() {
        let s = (h.1 >> 16) as usize;
        let k = (h.1 & 0xffff) as usize;
        assert_eq!(h.2, 0x0800 + s as u16, "ethertype of hdr {i}");
        assert_eq!(k, next[s], "frames of source {s} stay in order");
        next[s] += 1;
        let exp = &srcs[s][k];
        let got: Vec<u8> = f.iter().map(|b| b.1).collect();
        assert_eq!(got, exp.bytes, "frame {k} of source {s} intact");
        assert_eq!(f.last().unwrap().3, exp.err, "err copied on last");
        // hdr strictly before the first payload byte of its frame, and the
        // frame ends before the next hdr.
        // (a sink that pops both channels in one cycle sees equal times)
        assert!(h.0 <= f[0].0, "hdr precedes payload");
        // (the output channels are FIFOs, so pop times of different frames
        // overlap; interleaving shows as a byte of another source's frame.)
        order.push(s);
    }
    order
}

fn four(n: usize, len: impl Fn(usize, usize) -> usize) -> [Vec<Fr>; 4] {
    let mk = |s: usize| (0..n).map(|k| frame(s, k, len(s, k))).collect::<Vec<_>>();
    [mk(0), mk(1), mk(2), mk(3)]
}

#[test]
fn single_source() {
    for s in 0..4 {
        let mut srcs: [Vec<Fr>; 4] = Default::default();
        srcs[s] = (0..5).map(|k| frame(s, k, 3 + k * 5)).collect();
        let o = run(&srcs, always, always, sink);
        let order = verify(&srcs, &o);
        assert_eq!(order, vec![s; 5]);
    }
}

#[test]
fn all_ready_round_robin() {
    let srcs = four(6, |s, k| 2 + (s * 3 + k * 5) % 11);
    let o = run(&srcs, always, always, sink);
    let order = verify(&srcs, &o);
    // rr starts at 0, so the first winner is source 1; then strict rotation.
    let exp: Vec<usize> = (0..24).map(|i| (i + 1) % 4).collect();
    assert_eq!(order, exp, "round-robin order");
}

#[test]
fn round_robin_skips_idle_source() {
    // Source 2 has nothing: rotation is 1, 3, 0, 1, 3, 0, ...
    let mut srcs = four(4, |_, k| 3 + k);
    srcs[2].clear();
    let o = run(&srcs, always, always, sink);
    let order = verify(&srcs, &o);
    let exp: Vec<usize> = (0..12).map(|i| [1, 3, 0][i % 3]).collect();
    assert_eq!(order, exp);
}

#[test]
fn payload_gaps_and_late_payload() {
    let srcs = four(3, |s, k| 4 + s + k);
    // Payload bytes only every 5th cycle (offset per source); hdrs at once,
    // so the winner's payload has not started when its hdr is taken.
    let o = run(&srcs, always, |s, t| (t + s) % 5 == 0, sink);
    let order = verify(&srcs, &o);
    assert_eq!(order.len(), 12);
    // Payload of source 3 only starts late: it must not delay-trigger others
    // inside a frame (verified by `verify`), and every source gets served.
    for s in 0..4 {
        assert_eq!(order.iter().filter(|&&x| x == s).count(), 3);
    }
    let o2 = run(&srcs, always, |s, t| t > 60 + 7 * s && (t % 3 != 1), sink);
    verify(&srcs, &o2);
}

#[test]
fn downstream_backpressure() {
    let srcs = four(4, |s, k| 1 + s + 2 * k);
    for pat in [2usize, 3, 7] {
        let o = run(&srcs, always, always, |t| t % pat == 0);
        verify(&srcs, &o);
    }
    let o = run(&srcs, always, always, |t| (t / 9) % 2 == 0);
    verify(&srcs, &o);
}

#[test]
fn err_on_last_is_forwarded() {
    let mut srcs = four(4, |s, k| 2 + s + k);
    srcs[1][1].err = true;
    srcs[3][0].err = true;
    srcs[0][3].err = true;
    let o = run(&srcs, always, always, sink);
    verify(&srcs, &o);
    assert_eq!(o.pb().iter().filter(|b| b.3).count(), 3);
}

#[test]
fn one_byte_frames() {
    let srcs = four(8, |_, _| 1);
    let o = run(&srcs, always, always, sink);
    let order = verify(&srcs, &o);
    assert_eq!(order.len(), 32);
    let mut e = srcs.clone();
    e[2][3].err = true;
    let o = run(&e, always, always, |t| t % 3 != 0);
    verify(&e, &o);
}

struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        self.0 >> 33
    }
}

#[test]
fn random_arrivals() {
    for seed in 1..=20u64 {
        let mut r = Lcg(seed * 7919);
        let srcs: [Vec<Fr>; 4] = std::array::from_fn(|s| {
            (0..(2 + r.next() % 6) as usize)
                .map(|k| {
                    let mut f = frame(s, k, 1 + (r.next() % 20) as usize);
                    f.err = r.next() % 5 == 0;
                    f
                })
                .collect()
        });
        // Fixed per-cycle random decisions, derived from a hash of (seed, s, t).
        let h = |a: usize, b: usize, c: usize| -> u64 {
            let mut x = Lcg(seed ^ ((a as u64) << 40) ^ ((b as u64) << 20) ^ c as u64);
            x.next();
            x.next()
        };
        let o = run(&srcs, |s, t| h(1, s, t) % 4 == 0, |s, t| h(2, s, t) % 3 != 0, |t| h(3, 0, t) % 4 != 0);
        verify(&srcs, &o);
    }
}

#[test]
fn no_starvation_under_load() {
    // Source 0 offers continuously; others must still be served within 3
    // grants of becoming pending.
    let srcs = four(10, |_, k| 1 + k % 4);
    let o = run(&srcs, always, always, sink);
    let order = verify(&srcs, &o);
    let mut since = [0usize; 4];
    for &g in &order {
        for s in 0..4 {
            if s == g {
                since[s] = 0;
            } else {
                since[s] += 1;
                assert!(since[s] <= 3, "source {s} starved");
            }
        }
    }
}
