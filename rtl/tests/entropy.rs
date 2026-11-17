// SPDX-License-Identifier: Apache-2.0
//! Tests of the entropy unit (`rtl/src/entropy.rs`, security finding F3):
//! bit-exact against a software model (`Model`, the header's equations),
//! exactly xoshiro256++ without stirring, statistical sanity of the output
//! (distinct values, no stuck bit, bit balance, independent consecutive
//! draws), the effect of each stirring input and of the seed, and the
//! Verilog lowering.
use ntp_fpga::entropy::{Entropy, K0, K1, K2, K3};
use std::future::Future;
use txhdl::comp::{signal, DefaultClock, In, Out, Running, Unit};
use txhdl::types::U;

const INCR: u128 = ntp_fpga::clock::INCR;

/// The header's equations, one call per cycle; returns `rnd` of the cycle.
#[derive(Clone, Default)]
struct Model {
    s: [u64; 4],
    init: bool,
    sum: u64,
    s0d: u64,
    out: u64,
}

impl Model {
    fn step(&mut self, seed: u64, time: u64, ev: u64) -> u64 {
        let rnd = self.out;
        let [s0, s1, s2, s3] = self.s;
        let (sum, s0d) = (s0.wrapping_add(s3), s0);
        self.out = self.sum.rotate_left(23).wrapping_add(self.s0d);
        self.sum = sum;
        self.s0d = s0d;
        if self.init {
            let t = s1 << 17;
            let a2 = s2 ^ s0;
            let a3 = s3 ^ s1;
            self.s = [s0 ^ a3, s1 ^ a2 ^ ev, a2 ^ t ^ time, a3.rotate_left(45)];
        } else {
            self.s = [seed ^ K0, seed ^ K1, K2, K3];
            self.init = true;
        }
        rnd
    }
}

/// Reference xoshiro256++ (Blackman and Vigna), from state `s`.
fn xoshiro(s: &mut [u64; 4]) -> u64 {
    let r = s[0].wrapping_add(s[3]).rotate_left(23).wrapping_add(s[0]);
    let t = s[1] << 17;
    s[2] ^= s[0];
    s[3] ^= s[1];
    s[1] ^= s[2];
    s[0] ^= s[3];
    s[2] ^= t;
    s[3] = s[3].rotate_left(45);
    r
}

struct Bench<F: Future<Output = ()>> {
    sim: Running<F>,
    seed: Out<U<64>>,
    time: Out<U<96>>,
    ev: Out<U<64>>,
    rnd: In<U<64>>,
}

fn bench(seed: u64) -> Bench<impl Future<Output = ()>> {
    let (seed_o, seed_i) = signal::<U<64>, DefaultClock>();
    let (time_o, time_i) = signal::<U<96>, DefaultClock>();
    let (ev_o, ev_i) = signal::<U<64>, DefaultClock>();
    let (rnd_o, rnd) = signal::<U<64>, DefaultClock>();
    let u: &'static mut Entropy = Box::leak(Box::default());
    let sim = Running::new(u.run((seed_i, time_i, ev_i), rnd_o));
    seed_o.set(U::<64>::from(seed));
    time_o.set(U::<96>::from(0u64));
    ev_o.set(U::<64>::from(0u64));
    Bench {
        sim,
        seed: seed_o,
        time: time_o,
        ev: ev_o,
        rnd,
    }
}

impl<F: Future<Output = ()>> Bench<F> {
    /// One cycle with these inputs; returns `rnd` as seen after the cycle.
    fn cycle(&mut self, time: u128, ev: u64) -> u64 {
        self.time.set(U::<96>::new(time & ((1u128 << 96) - 1)));
        self.ev.set(U::<64>::from(ev));
        self.sim.cycle();
        self.rnd.get().raw() as u64
    }
}

/// Inputs as on the board: `time` advancing by about INCR per cycle (with
/// a varying trim), `ev` changing at irregular cycles.
fn inputs(n: usize) -> Vec<(u128, u64)> {
    let mut x = 0x2545_F491_4F6C_DD1Du64;
    let mut next = move || {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        x
    };
    let mut time = 0xECC5_8CC0_3FFF_0000u128 << 32;
    let mut ev = 0u64;
    (0..n)
        .map(|_| {
            time += INCR + (next() % 1000) as u128;
            if next() % 97 == 0 {
                ev = (time >> 32) as u64 + next() % 4096;
            }
            (time, ev)
        })
        .collect()
}

/// Runs a unit and the model with the same inputs; returns the unit's
/// `rnd` of every cycle after checking each against the model.
fn run_checked(seed: u64, ins: &[(u128, u64)]) -> Vec<u64> {
    let mut b = bench(seed);
    let mut m = Model::default();
    let mut got = vec![];
    for (k, &(t, e)) in ins.iter().enumerate() {
        let r = b.cycle(t, e);
        assert_eq!(r, m.step(seed, t as u64, e), "cycle {k}");
        got.push(r);
    }
    got
}

#[test]
fn bit_exact_with_the_model() {
    let ins = inputs(5000);
    for seed in [0u64, 0x0123_4567_89AB_CDEF, u64::MAX] {
        let got = run_checked(seed, &ins);
        assert!(got.iter().skip(4).all(|&r| r != 0));
    }
}

/// Without stirring (`time` = `ev` = 0) the output is exactly xoshiro256++
/// from the seeded state, two pipeline stages (plus the seeding cycle) late.
#[test]
fn plain_xoshiro256pp_without_stirring() {
    let seed = 0xDEAD_BEEF_0BAD_F00Du64;
    let mut b = bench(seed);
    let got: Vec<u64> = (0..1000).map(|_| b.cycle(0, 0)).collect();
    let mut s = [seed ^ K0, seed ^ K1, K2, K3];
    let want: Vec<u64> = (0..990).map(|_| xoshiro(&mut s)).collect();
    // cycle 0 seeds, cycle 1 computes the first sum, cycle 2 the first out,
    // shown from cycle 3 on.
    assert_eq!(&got[3..993], &want[..]);
}

/// The seed is sampled once, in the first cycle after reset.
#[test]
fn seed_sampled_once() {
    let ins = inputs(300);
    let a = run_checked(7, &ins);
    let mut b = bench(7);
    let mut got = vec![];
    for (k, &(t, e)) in ins.iter().enumerate() {
        if k == 1 {
            b.seed.set(U::<64>::from(8u64));
        }
        got.push(b.cycle(t, e));
    }
    assert_eq!(a, got);
}

fn ones(x: u64) -> u32 {
    x.count_ones()
}

/// Statistical sanity of 20 000 consecutive outputs with board-like inputs
/// (a weak check, not a randomness test suite): all distinct (and their
/// 32-bit low halves, the nonce fraction, nearly so: a few birthday
/// collisions at most), no stuck bit, every bit set in 50 % +- 2 % of the
/// draws, consecutive draws differing in about 32 bits, and the low 32 bits
/// spread over all 16 buckets of their top nibble.
#[test]
fn statistical_sanity() {
    let n = 20_000;
    let got = run_checked(0x6E74_705F_7365_6564, &inputs(n + 3));
    let v = &got[3..];
    let mut d = v.to_vec();
    d.sort();
    d.dedup();
    assert_eq!(d.len(), n, "64-bit values repeat");
    let mut lo: Vec<u32> = v.iter().map(|&x| x as u32).collect();
    lo.sort();
    lo.dedup();
    // expected collisions n^2 / 2^33 ~ 0.05
    assert!(n - lo.len() <= 3, "{} 32-bit collisions", n - lo.len());
    for bit in 0..64 {
        let c = v.iter().filter(|&&x| (x >> bit) & 1 == 1).count();
        let f = c as f64 / n as f64;
        assert!((0.48..=0.52).contains(&f), "bit {bit}: {f}");
    }
    let mean = v.windows(2).map(|w| ones(w[0] ^ w[1]) as f64).sum::<f64>() / (n - 1) as f64;
    assert!((31.5..=32.5).contains(&mean), "consecutive distance {mean}");
    let mut buckets = [0usize; 16];
    for &x in v {
        buckets[((x >> 28) & 15) as usize] += 1;
    }
    for c in buckets {
        assert!((1100..=1400).contains(&c), "{buckets:?}");
    }
}

/// Same seed and same inputs: the same stream (the generator is
/// deterministic: only the seed and the inputs make it unpredictable).
/// One different `ev` value in one cycle, or one different low bit of
/// `time`, or another seed: the streams differ from then on (every output
/// from a few cycles later differs).
#[test]
fn stirring_changes_the_stream() {
    let ins = inputs(3000);
    let a = run_checked(42, &ins);
    assert_eq!(a, run_checked(42, &ins));
    let k = 1000;
    let mut ins_e = ins.clone();
    ins_e[k].1 ^= 1;
    let e = run_checked(42, &ins_e);
    let mut ins_t = ins.clone();
    ins_t[k].0 ^= 1;
    let t = run_checked(42, &ins_t);
    for other in [&e, &t] {
        assert_eq!(&a[..k + 3], &other[..k + 3], "nothing changes before");
        // The stirred word reaches the output through the state (one or two
        // steps) and the two output stages.
        let diff = (k + 6..3000).filter(|&i| a[i] != other[i]).count();
        assert_eq!(diff, 3000 - k - 6, "every later output differs");
        let bits: u32 = (k + 10..3000).map(|i| ones(a[i] ^ other[i])).sum();
        let mean = bits as f64 / (3000 - k - 10) as f64;
        assert!((30.0..=34.0).contains(&mean), "{mean}");
    }
    let s = run_checked(43, &ins);
    assert!((3..3000).all(|i| a[i] != s[i]));
}

#[test]
fn lowers_to_verilog() {
    let v = Entropy::verilog("entropy");
    assert!(v.contains("module entropy"));
}
