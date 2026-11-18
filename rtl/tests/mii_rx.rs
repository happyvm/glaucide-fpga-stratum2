// SPDX-License-Identifier: Apache-2.0
//! Cycle-by-cycle tests of the MII receiver (T1.1).
use ntp_fpga::clocks::ClkRx;
use ntp_fpga::mii_rx::MiiRx;
use ntp_fpga::stream::FrameByte;
use txhdl::comp::{chan, signal, Running, Unit};
use txhdl::types::{Bit, U};

/// One cycle on the wire.
#[derive(Clone, Copy)]
struct Cy {
    d: u8,
    dv: bool,
    er: bool,
}

#[derive(Debug, Clone, PartialEq)]
struct Got {
    t: usize,
    data: u8,
    last: bool,
    err: bool,
}

struct Out {
    bytes: Vec<Got>,
    sfd: Vec<usize>,
}

/// Preamble of `pre` nibbles 0x5, SFD nibble 0xD, then the payload nibbles.
fn wire(pre: usize, payload: &[u8], odd: bool, er_at: Option<usize>) -> Vec<Cy> {
    let mut v = Vec::new();
    for _ in 0..pre {
        v.push(Cy { d: 5, dv: true, er: false });
    }
    v.push(Cy { d: 0xd, dv: true, er: false });
    let mut nibs: Vec<u8> = payload.iter().flat_map(|b| [b & 15, b >> 4]).collect();
    if odd {
        nibs.push(0x7);
    }
    for (i, n) in nibs.iter().enumerate() {
        v.push(Cy { d: *n, dv: true, er: er_at == Some(i) });
    }
    v
}

fn gap(n: usize) -> Vec<Cy> {
    vec![Cy { d: 0, dv: false, er: false }; n]
}

/// Drive `cycles`, one per clock; `take(t)` tells whether the consumer
/// accepts at cycle `t`.
fn run(cycles: &[Cy], take: impl Fn(usize) -> bool) -> Out {
    let (d_o, d) = signal::<U<4>, ClkRx>();
    let (dv_o, dv) = signal::<Bit, ClkRx>();
    let (er_o, er) = signal::<Bit, ClkRx>();
    let (tx, rx) = chan::<FrameByte, ClkRx>();
    let (sfd_o, sfd) = signal::<Bit, ClkRx>();
    let mut m = MiiRx::default();
    let mut sim = Running::new(m.run((d, dv, er), (tx, sfd_o)));
    let mut out = Out { bytes: vec![], sfd: vec![] };
    for t in 0..cycles.len() + 12 {
        let c = cycles.get(t).copied().unwrap_or(Cy { d: 0, dv: false, er: false });
        d_o.set(U::<4>::from(c.d));
        dv_o.set(Bit::from_bool(c.dv));
        er_o.set(Bit::from_bool(c.er));
        if sfd.get().to_bool() {
            out.sfd.push(t);
        }
        if take(t) {
            if let Some(b) = rx.recv() {
                out.bytes.push(Got {
                    t,
                    data: b.data.raw() as u8,
                    last: b.last.to_bool(),
                    err: b.err.to_bool(),
                });
            }
        }
        sim.cycle();
    }
    out
}

fn payload(n: usize) -> Vec<u8> {
    (0..n).map(|i| (i * 7 + 3) as u8).collect()
}

/// The frames of `o`: each a list of bytes ending with `last`.
fn frames(o: &Out) -> Vec<Vec<Got>> {
    let mut all = vec![];
    let mut cur: Vec<Got> = vec![];
    for b in &o.bytes {
        cur.push(b.clone());
        if b.last {
            all.push(std::mem::take(&mut cur));
        }
    }
    assert!(cur.is_empty(), "bytes after the last frame's last: {cur:?}");
    all
}

fn data(f: &[Got]) -> Vec<u8> {
    f.iter().map(|g| g.data).collect()
}

/// The stream contract on every frame: `last` only at the end, `err` only
/// with `last`.
fn contract(o: &Out) {
    let mut open = false;
    for b in &o.bytes {
        assert!(!b.err || b.last, "err without last: {b:?}");
        open = !b.last;
    }
    assert!(!open, "frame without last");
}

/// In the Rust simulation `sfd` reads two cycles after the cycle that
/// drove the SFD nibble (the netlist raises it one edge after the nibble
/// is sampled, with the first data nibble; the run shows an output one
/// read later, as for any `Out`).
const SFD_LAG: usize = 2;

fn plain_frame(n: usize, pre: usize) {
    let p = payload(n);
    let mut w = gap(3);
    let sfd_at = w.len() + pre;
    w.extend(wire(pre, &p, false, None));
    w.extend(gap(4));
    let o = run(&w, |_| true);
    contract(&o);
    let f = frames(&o);
    assert_eq!(f.len(), 1);
    assert_eq!(data(&f[0]), p);
    assert!(!f[0].last().unwrap().err);
    assert_eq!(o.sfd, vec![sfd_at + SFD_LAG], "sfd pulses once, at the SFD");
    // Latency: the final byte is sent one edge after the last nibble.
    let end = 3 + pre + 1 + 2 * n; // first cycle with rx_dv low
    assert_eq!(f[0].last().unwrap().t, end + 1);
}

#[test]
fn frame_64() {
    plain_frame(64, 15);
}

#[test]
fn frame_1518() {
    plain_frame(1518, 15);
}

#[test]
fn short_preamble_accepted() {
    plain_frame(20, 3);
    plain_frame(20, 1);
    plain_frame(20, 0);
}

#[test]
fn rx_er_marks_last() {
    let p = payload(64);
    let mut w = gap(2);
    w.extend(wire(15, &p, false, Some(50)));
    w.extend(gap(4));
    let o = run(&w, |_| true);
    contract(&o);
    let f = frames(&o);
    assert_eq!(f.len(), 1);
    assert_eq!(data(&f[0]), p, "the bytes still come out");
    assert!(f[0].last().unwrap().err);
    assert!(f[0][..63].iter().all(|g| !g.err));
}

#[test]
fn odd_nibble_marks_last() {
    let p = payload(30);
    let mut w = gap(2);
    w.extend(wire(15, &p, true, None));
    w.extend(gap(4));
    let o = run(&w, |_| true);
    contract(&o);
    let f = frames(&o);
    assert_eq!(f.len(), 1);
    assert_eq!(data(&f[0]), p);
    assert!(f[0].last().unwrap().err);
}

#[test]
fn bad_sfd_drops_frame() {
    // 0x5 then 0x3: not an SFD. The nibbles that follow, even a
    // look-alike SFD, are ignored until rx_dv falls.
    let mut w = gap(2);
    let mut bad = wire(6, &payload(20), false, None);
    bad[6] = Cy { d: 3, dv: true, er: false };
    bad[10] = Cy { d: 0xd, dv: true, er: false };
    w.extend(bad);
    w.extend(gap(3));
    w.extend(wire(15, &payload(10), false, None));
    w.extend(gap(4));
    let o = run(&w, |_| true);
    contract(&o);
    let f = frames(&o);
    assert_eq!(f.len(), 1, "only the good frame");
    assert_eq!(data(&f[0]), payload(10));
    assert_eq!(o.sfd.len(), 1);
}

#[test]
fn dv_falls_before_sfd() {
    let mut w = gap(2);
    w.extend(vec![Cy { d: 5, dv: true, er: false }; 8]);
    w.extend(gap(2));
    w.extend(wire(15, &payload(5), false, None));
    w.extend(gap(4));
    let o = run(&w, |_| true);
    contract(&o);
    assert_eq!(frames(&o).len(), 1);
    assert_eq!(o.sfd.len(), 1);
}

#[test]
fn frame_without_a_byte_is_dropped() {
    let mut w = gap(2);
    w.extend(wire(7, &[], true, None)); // SFD and one nibble
    w.extend(gap(3));
    w.extend(wire(7, &[], false, None)); // SFD only
    w.extend(gap(3));
    let o = run(&w, |_| true);
    assert!(o.bytes.is_empty());
}

#[test]
fn two_frames_are_independent() {
    let (a, b) = (payload(64), payload(61));
    let mut w = gap(2);
    w.extend(wire(15, &a, false, Some(10)));
    w.extend(gap(24));
    w.extend(wire(15, &b, false, None));
    w.extend(gap(4));
    let o = run(&w, |_| true);
    contract(&o);
    let f = frames(&o);
    assert_eq!(f.len(), 2);
    assert_eq!(data(&f[0]), a);
    assert!(f[0].last().unwrap().err);
    assert_eq!(data(&f[1]), b);
    assert!(!f[1].last().unwrap().err, "the error of frame 1 does not leak");
    assert_eq!(o.sfd.len(), 2);
}

#[test]
fn too_long_frame_is_cut_at_1518() {
    let p = payload(1520);
    let mut w = gap(2);
    w.extend(wire(15, &p, false, None));
    w.extend(gap(4));
    w.extend(wire(15, &payload(8), false, None));
    w.extend(gap(4));
    let o = run(&w, |_| true);
    contract(&o);
    let f = frames(&o);
    assert_eq!(f.len(), 2);
    assert_eq!(data(&f[0]), p[..1518].to_vec());
    assert!(f[0].last().unwrap().err);
    assert!(f[0][..1517].iter().all(|g| !g.err));
    assert_eq!(data(&f[1]), payload(8));
    assert!(!f[1].last().unwrap().err);
}

#[test]
fn exactly_1518_is_not_an_error() {
    plain_frame(1518, 15); // asserts err == false
}

#[test]
fn overrun_mid_frame() {
    // The consumer takes nothing until cycle 200: the simulated channel
    // holds two bytes, the third cannot be sent, so the frame is cut there:
    // bytes 0 and 1 as ordinary bytes, byte 2 kept and sent as last + err
    // once there is room; the 17 others are dropped.
    let p = payload(20);
    let mut w = gap(2);
    w.extend(wire(15, &p, false, None));
    w.extend(gap(250));
    let o = run(&w, |t| t >= 200);
    contract(&o);
    let f = frames(&o);
    assert_eq!(f.len(), 1);
    assert_eq!(data(&f[0]), p[..3].to_vec());
    assert!(f[0][..2].iter().all(|g| !g.err && !g.last));
    assert!(f[0][2].last && f[0][2].err);
}

#[test]
fn overrun_then_next_frame() {
    // Once the consumer is back, a later frame goes through intact.
    let p = payload(20);
    let mut w = gap(2);
    w.extend(wire(15, &p, false, None));
    w.extend(gap(300));
    w.extend(wire(15, &payload(12), false, None));
    w.extend(gap(4));
    let o = run(&w, |t| !(2..100).contains(&t));
    contract(&o);
    let f = frames(&o);
    assert_eq!(f.len(), 2);
    assert!(f[0].last().unwrap().err);
    assert_eq!(data(&f[1]), payload(12));
    assert!(!f[1].last().unwrap().err);
}

#[test]
fn final_byte_waits_for_room() {
    // A 3-byte frame: bytes 0 and 1 fill the channel, the final byte waits
    // for the consumer and is sent intact, without err.
    let p = payload(3);
    let mut w = gap(2);
    w.extend(wire(15, &p, false, None));
    w.extend(gap(150));
    let o = run(&w, |t| t >= 100);
    contract(&o);
    let f = frames(&o);
    assert_eq!(f.len(), 1);
    assert_eq!(data(&f[0]), p);
    assert!(!f[0].last().unwrap().err);
}

#[test]
fn frame_arriving_while_one_is_pending_is_dropped() {
    let p = payload(3);
    let mut w = gap(2);
    w.extend(wire(15, &p, false, None));
    w.extend(gap(30));
    w.extend(wire(15, &payload(9), false, None));
    w.extend(gap(150));
    let o = run(&w, |t| t >= 100);
    contract(&o);
    let f = frames(&o);
    assert_eq!(f.len(), 1, "the second frame found a pending byte");
    assert_eq!(data(&f[0]), p);
    assert_eq!(o.sfd.len(), 1);
}
