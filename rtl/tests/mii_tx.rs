// SPDX-License-Identifier: Apache-2.0
//! Testbench of `mii_tx` (T1.2): reads the MII lines cycle by cycle and
//! reassembles frames.
use ntp_fpga::clocks::ClkTx;
use ntp_fpga::mii_tx::MiiTx;
use ntp_fpga::stream::FrameByte;
use txhdl::comp::{chan, signal, Running, Unit};
use txhdl::types::{Bit, U};

#[derive(Clone, Copy, Debug, PartialEq)]
struct Cy {
    txd: u8,
    en: bool,
    sfd: bool,
    und: bool,
}

/// One input byte and the number of idle cycles before it is offered.
type Item = (FrameByte, usize);

fn frame(data: &[u8], err: bool) -> Vec<Item> {
    let n = data.len();
    data.iter()
        .enumerate()
        .map(|(i, &b)| (FrameByte::new(b, i + 1 == n, err && i + 1 == n), 0))
        .collect()
}

fn pattern(n: usize) -> Vec<u8> {
    (0..n).map(|i| (i * 7 + 3) as u8).collect()
}

/// Drive `items` into the unit for `cycles` cycles, returning the lines
/// after each clock edge and the cycle at which each item was accepted.
fn run(items: &[Item], cycles: usize) -> (Vec<Cy>, Vec<usize>) {
    let (tx, rx) = chan::<FrameByte, ClkTx>();
    let (txd_o, txd) = signal::<U<4>, ClkTx>();
    let (en_o, en) = signal::<Bit, ClkTx>();
    let (sfd_o, sfd) = signal::<Bit, ClkTx>();
    let (und_o, und) = signal::<Bit, ClkTx>();
    let mut u = MiiTx::default();
    let mut sim = Running::new(u.run(rx, (txd_o, en_o, sfd_o, und_o)));
    let mut next = 0;
    let mut wait = items.first().map_or(0, |i| i.1);
    let mut trace = Vec::new();
    let mut accepted = Vec::new();
    for t in 0..cycles {
        if next < items.len() {
            if wait > 0 {
                wait -= 1;
            } else if tx.ready().to_bool() {
                tx.send(items[next].0);
                accepted.push(t);
                next += 1;
                wait = items.get(next).map_or(0, |i| i.1);
            }
        }
        sim.cycle();
        trace.push(Cy {
            txd: txd.get().raw() as u8,
            en: en.get().to_bool(),
            sfd: sfd.get().to_bool(),
            und: und.get().to_bool(),
        });
    }
    (trace, accepted)
}

/// Runs of consecutive `en` cycles: (start, end exclusive).
fn bursts(t: &[Cy]) -> Vec<(usize, usize)> {
    let mut v = Vec::new();
    let mut i = 0;
    while i < t.len() {
        if t[i].en {
            let s = i;
            while i < t.len() && t[i].en {
                i += 1;
            }
            v.push((s, i));
        } else {
            i += 1;
        }
    }
    v
}

fn nibbles(t: &[Cy], (s, e): (usize, usize)) -> Vec<u8> {
    t[s..e].iter().map(|c| c.txd).collect()
}

/// preamble (14x5) + SFD (5, D) as nibbles.
fn head() -> Vec<u8> {
    let mut v = vec![5u8; 15];
    v.push(0xD);
    v
}

fn nib_of(bytes: &[u8]) -> Vec<u8> {
    bytes.iter().flat_map(|b| [b & 15, b >> 4]).collect()
}

fn check_clean(t: &[Cy], data: &[u8], b: (usize, usize)) {
    let mut want = head();
    want.extend(nib_of(data));
    assert_eq!(nibbles(t, b), want, "preamble + SFD + frame nibbles");
    assert_eq!(b.1 - b.0, 16 + 2 * data.len(), "tx_en cycle count");
    let sfd: Vec<usize> = (b.0..b.1).filter(|&i| t[i].sfd).collect();
    assert_eq!(sfd, vec![b.0 + 15], "sfd once, with the 0xD nibble");
    assert_eq!(t[b.0 + 15].txd, 0xD);
}

#[test]
fn frames_of_64_and_1518_bytes() {
    for n in [64usize, 1518] {
        let d = pattern(n);
        let (t, _) = run(&frame(&d, false), 2 * n + 120);
        let b = bursts(&t);
        assert_eq!(b.len(), 1);
        check_clean(&t, &d, b[0]);
        assert!(t.iter().all(|c| !c.und));
        assert_eq!(t.iter().filter(|c| c.sfd).count(), 1);
    }
}

#[test]
fn every_byte_value_passes() {
    let d: Vec<u8> = (0..=255u8).collect();
    let (t, _) = run(&frame(&d, false), 700);
    let b = bursts(&t);
    check_clean(&t, &d, b[0]);
}

#[test]
fn back_to_back_frames_keep_the_gap_and_none_is_lost() {
    let frames = [pattern(64), pattern(60), pattern(100), pattern(64)];
    let mut items = Vec::new();
    for f in &frames {
        items.extend(frame(f, false));
    }
    let (t, _) = run(&items, 1200);
    let b = bursts(&t);
    assert_eq!(b.len(), 4, "all queued frames are sent");
    for (f, bb) in frames.iter().zip(&b) {
        check_clean(&t, f, *bb);
    }
    for w in b.windows(2) {
        assert!(w[1].0 - w[0].1 >= 24, "IFG >= 24 cycles");
        assert_eq!(w[1].0 - w[0].1, 24, "and no longer than needed when queued");
    }
}

#[test]
fn err_on_last_ends_with_an_odd_nibble() {
    let d = pattern(64);
    let mut items = frame(&d, true);
    items.extend(frame(&pattern(60), false));
    let (t, _) = run(&items, 500);
    let b = bursts(&t);
    assert_eq!(b.len(), 2);
    let mut want = head();
    want.extend(nib_of(&d));
    want.pop(); // high nibble of the aborted last byte is not sent
    assert_eq!(nibbles(&t, b[0]), want);
    assert_eq!(b[0].1 - b[0].0, 16 + 2 * 64 - 1);
    assert!(t.iter().all(|c| !c.und), "an abort is not an underrun");
    assert!(b[1].0 - b[0].1 >= 24);
    check_clean(&t, &pattern(60), b[1]);
}

#[test]
fn underrun_ends_the_frame_and_the_tail_is_discarded() {
    // 40-byte frame, 30 idle cycles before its 11th byte.
    let d = pattern(40);
    let mut items = frame(&d, false);
    items[10].1 = 30;
    let follow = pattern(64);
    items.extend(frame(&follow, false));
    let (t, _) = run(&items, 600);
    let b = bursts(&t);
    assert_eq!(b.len(), 2);
    // First burst: preamble, SFD, 10 bytes (the 11th never came).
    let mut want = head();
    want.extend(nib_of(&d[..10]));
    assert_eq!(nibbles(&t, b[0]), want);
    // One underrun pulse, in the first cycle with tx_en low.
    let u: Vec<usize> = (0..t.len()).filter(|&i| t[i].und).collect();
    assert_eq!(u, vec![b[0].1]);
    assert!(!t[b[0].1].en);
    // The late tail was dropped; the next frame is intact and after a gap.
    assert!(b[1].0 - b[0].1 >= 24);
    check_clean(&t, &follow, b[1]);
}

#[test]
fn over_long_frame_is_truncated_like_an_underrun() {
    let d = pattern(1519);
    let mut items = frame(&d, false);
    let follow = pattern(64);
    items.extend(frame(&follow, false));
    let (t, _) = run(&items, 2 * 1519 + 300);
    let b = bursts(&t);
    assert_eq!(b.len(), 2);
    let mut want = head();
    want.extend(nib_of(&d[..1518]));
    assert_eq!(nibbles(&t, b[0]), want, "truncated to 1518 bytes");
    assert_eq!(t.iter().filter(|c| c.und).count(), 1);
    assert!(t[b[0].1].und);
    assert!(b[1].0 - b[0].1 >= 24);
    check_clean(&t, &follow, b[1]);
}

#[test]
fn frame_queued_late_is_accepted_after_gap_and_idle_is_quiet() {
    let mut items = frame(&pattern(64), false);
    items[0].1 = 5;
    let (t, _) = run(&items, 300);
    let b = bursts(&t);
    assert_eq!(b.len(), 1);
    // Reset acts as a gap too.
    assert!(b[0].0 >= 24);
    assert!(t[..b[0].0]
        .iter()
        .all(|c| !c.en && !c.sfd && !c.und && c.txd == 0));
}
