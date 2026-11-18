// SPDX-License-Identifier: Apache-2.0
//! Cycle-by-cycle tests of the NTP mode demultiplexer (T4.2a).
use ntp_fpga::ntp_demux::NtpDemux;
use ntp_fpga::stream::{FrameByte, Meta};
use txhdl::comp::{chan, DefaultClock, Running, Unit};
use txhdl::types::{Bit, U};

/// One datagram of the input: its `Meta`, its UDP payload, and whether its
/// last byte carries a late `err` (bad UDP checksum).
#[derive(Clone, Debug)]
struct Dg {
    meta: Meta,
    bytes: Vec<u8>,
    late_err: bool,
}

/// Datagram `k`: byte 0 carries `mode` in its low 3 bits, the other bytes
/// depend on `k` and their index. Every `Meta` field differs per datagram.
fn dg(k: usize, mode: u8, n: usize, late_err: bool) -> Dg {
    let mut bytes: Vec<u8> = (0..n).map(|i| (k * 11 + i * 5 + 3) as u8).collect();
    bytes[0] = (bytes[0] & !7) | (mode & 7);
    let meta = Meta {
        family: Bit::from_bool(k % 2 == 1),
        src: U::<128>::from(0x2001_0db8_0000_0000_0000_0000_0000_0000u128 | k as u128),
        dst: U::<128>::from(0x2001_0db8_ffff_0000_0000_0000_0000_0000u128 | (k as u128) << 4),
        sport: U::<16>::from(40_000u16 + k as u16),
        dport: U::<16>::from(123u16),
        len: U::<16>::from(n as u16),
        eth_src: U::<48>::from(0x0200_0000_0000u64 | k as u64),
        rx_ts: U::<64>::from(0xE000_0000_0000_0000u64 | ((k as u64) << 32) | 0x1234_5678),
    };
    Dg { meta, bytes, late_err }
}

/// What the two output pairs delivered, with the cycle of each transaction.
#[derive(Debug, Default)]
struct Out {
    srv_m: Vec<(usize, Meta)>,
    srv_b: Vec<(usize, FrameByte)>,
    cli_m: Vec<(usize, Meta)>,
    cli_b: Vec<(usize, FrameByte)>,
}

impl Out {
    /// Cycle of the last byte delivered on each side (0 if none).
    fn last_cycle(v: &[(usize, FrameByte)]) -> usize {
        v.iter().rev().find(|(_, b)| b.last.to_bool()).map_or(0, |(t, _)| *t)
    }
}

/// Drive `dgs` into the unit. `gap(t)`: no payload byte is offered at cycle
/// `t`. `srv_ok(t)` / `cli_ok(t)`: the consumer of that pair accepts at `t`.
fn run(
    dgs: &[Dg],
    cycles: usize,
    gap: impl Fn(usize) -> bool,
    srv_ok: impl Fn(usize) -> bool,
    cli_ok: impl Fn(usize) -> bool,
) -> Out {
    let (mt_t, mt) = chan::<Meta, DefaultClock>();
    let (pt_t, pt) = chan::<FrameByte, DefaultClock>();
    let (sm_t, sm) = chan::<Meta, DefaultClock>();
    let (sp_t, sp) = chan::<FrameByte, DefaultClock>();
    let (cm_t, cm) = chan::<Meta, DefaultClock>();
    let (cp_t, cp) = chan::<FrameByte, DefaultClock>();
    let mut dut = NtpDemux::default();
    let mut sim = Running::new(dut.run((mt, pt), (sm_t, sp_t, cm_t, cp_t)));
    // Next datagram whose meta is to be sent; next datagram and byte index
    // of the payload.
    let mut mk = 0usize;
    let mut pk = 0usize;
    let mut pi = 0usize;
    let mut out = Out::default();
    for t in 0..cycles {
        if mk < dgs.len() && mt_t.ready().to_bool() {
            mt_t.send(dgs[mk].meta);
            mk += 1;
        }
        if pk < dgs.len() && pk < mk && !gap(t) && pt_t.ready().to_bool() {
            let d = &dgs[pk];
            let last = pi + 1 == d.bytes.len();
            pt_t.send(FrameByte::new(d.bytes[pi], last, last && d.late_err));
            if last {
                pk += 1;
                pi = 0;
            } else {
                pi += 1;
            }
        }
        if srv_ok(t) {
            if let Some(m) = sm.recv() {
                out.srv_m.push((t, m));
            }
            if let Some(b) = sp.recv() {
                out.srv_b.push((t, b));
            }
        }
        if cli_ok(t) {
            if let Some(m) = cm.recv() {
                out.cli_m.push((t, m));
            }
            if let Some(b) = cp.recv() {
                out.cli_b.push((t, b));
            }
        }
        sim.cycle();
    }
    out
}

/// Cycles needed by `dgs` with no stalls, plus slack.
fn budget(dgs: &[Dg]) -> usize {
    dgs.iter().map(|d| d.bytes.len() + 4).sum::<usize>() * 3 + 100
}

fn run_free(dgs: &[Dg]) -> Out {
    run(dgs, budget(dgs), |_| false, |_| true, |_| true)
}

fn meta_eq(a: &Meta, b: &Meta) -> bool {
    a.family.to_bool() == b.family.to_bool()
        && a.src.raw() == b.src.raw()
        && a.dst.raw() == b.dst.raw()
        && a.sport.raw() == b.sport.raw()
        && a.dport.raw() == b.dport.raw()
        && a.len.raw() == b.len.raw()
        && a.eth_src.raw() == b.eth_src.raw()
        && a.rx_ts.raw() == b.rx_ts.raw()
}

/// Checks one output pair against the datagrams it must carry, in order:
/// metas unchanged, one meta per datagram before its first byte, bytes
/// byte-exact with `last` on the final byte of each datagram and `err`
/// exactly where the input had it.
fn check_side(name: &str, want: &[&Dg], m: &[(usize, Meta)], b: &[(usize, FrameByte)]) {
    assert_eq!(m.len(), want.len(), "{name}: number of metas");
    let total: usize = want.iter().map(|d| d.bytes.len()).sum();
    assert_eq!(b.len(), total, "{name}: number of bytes");
    let mut j = 0usize;
    for (i, d) in want.iter().enumerate() {
        assert!(meta_eq(&m[i].1, &d.meta), "{name}: meta {i} changed");
        let first = b[j].0;
        // A consumer that was stalled may pop the meta and the first byte in
        // the same cycle; the meta was on its channel no later than that.
        assert!(m[i].0 <= first, "{name}: meta {i} must precede its first byte");
        for (k, &byte) in d.bytes.iter().enumerate() {
            let (_, fb) = b[j];
            let last = k + 1 == d.bytes.len();
            assert_eq!(fb.data.raw() as u8, byte, "{name}: datagram {i} byte {k}");
            assert_eq!(fb.last.to_bool(), last, "{name}: datagram {i} byte {k} last");
            assert_eq!(fb.err.to_bool(), last && d.late_err, "{name}: datagram {i} byte {k} err");
            j += 1;
        }
    }
}

/// Full check: every datagram on the pair its mode selects, and nothing on
/// the other one.
fn check_all(dgs: &[Dg], o: &Out) {
    let to_cli: Vec<&Dg> = dgs.iter().filter(|d| d.bytes[0] & 7 == 4).collect();
    let to_srv: Vec<&Dg> = dgs.iter().filter(|d| d.bytes[0] & 7 != 4).collect();
    check_side("srv", &to_srv, &o.srv_m, &o.srv_b);
    check_side("cli", &to_cli, &o.cli_m, &o.cli_b);
}

fn no_gap(_: usize) -> bool {
    false
}
fn always(_: usize) -> bool {
    true
}

// ------------------------------------------------------------ routing

#[test]
fn modes_0_to_7_routed_by_mode() {
    let dgs: Vec<Dg> = (0..8).map(|m| dg(m, m as u8, 5, false)).collect();
    let o = run_free(&dgs);
    check_all(&dgs, &o);
    // Only mode 4 reaches the client pair.
    assert_eq!(o.cli_m.len(), 1);
    assert_eq!(o.srv_m.len(), 7);
}

#[test]
fn mode_bits_above_low_three_are_ignored() {
    // Byte 0 = 0xFC (mode 4 with high bits set) goes to cli, 0xFB (mode 3)
    // to srv: only `byte0 & 7` decides.
    let mut a = dg(1, 4, 6, false);
    a.bytes[0] = 0xFC;
    let mut b = dg(2, 3, 6, false);
    b.bytes[0] = 0xFB;
    let dgs = vec![a, b];
    let o = run_free(&dgs);
    check_all(&dgs, &o);
    assert_eq!(o.cli_m.len(), 1);
    assert_eq!(o.srv_m.len(), 1);
}

// ------------------------------------------------------------ payloads

#[test]
fn payload_lengths_1_48_128_byte_exact() {
    let mut dgs = Vec::new();
    for (k, n) in [1usize, 48, 128, 1, 128, 48].iter().enumerate() {
        let mode = if k % 2 == 0 { 3 } else { 4 };
        dgs.push(dg(k, mode, *n, false));
    }
    let o = run_free(&dgs);
    check_all(&dgs, &o);
}

#[test]
fn single_byte_datagram_is_valid() {
    let dgs = vec![dg(0, 4, 1, false), dg(1, 3, 1, false), dg(2, 4, 1, false)];
    let o = run_free(&dgs);
    check_all(&dgs, &o);
    assert_eq!(o.cli_b.len(), 2);
    assert_eq!(o.srv_b.len(), 1);
}

#[test]
fn late_err_on_last_byte_is_propagated() {
    let dgs = vec![
        dg(0, 4, 1, true),
        dg(1, 3, 48, true),
        dg(2, 4, 128, true),
        dg(3, 3, 48, false),
        dg(4, 4, 128, false),
    ];
    let o = run_free(&dgs);
    check_all(&dgs, &o);
}

// ------------------------------------------------------------ sequencing

#[test]
fn back_to_back_alternating_destinations() {
    let dgs: Vec<Dg> = (0..24)
        .map(|k| dg(k, if k % 2 == 0 { 3 } else { 4 }, 1 + (k * 7) % 20, k % 5 == 0))
        .collect();
    let o = run_free(&dgs);
    check_all(&dgs, &o);
}

#[test]
fn input_gaps_between_payload_bytes() {
    let dgs: Vec<Dg> = (0..10)
        .map(|k| dg(k, [3u8, 4, 0, 4, 6][k % 5], 1 + (k * 13) % 50, k == 4))
        .collect();
    let gap = |t: usize| t % 3 == 1 || t % 11 == 4;
    let o = run(&dgs, budget(&dgs) * 2, gap, always, always);
    check_all(&dgs, &o);
}

// ------------------------------------------------------------ backpressure

#[test]
fn stalled_srv_does_not_delay_cli() {
    // Client replies first, then requests: srv stalls until cycle 400.
    let mut dgs: Vec<Dg> = (0..4).map(|k| dg(k, 4, 40, false)).collect();
    dgs.extend((4..6).map(|k| dg(k, 3, 40, false)));
    let o = run(&dgs, budget(&dgs) + 400, no_gap, |t| t >= 400, always);
    check_all(&dgs, &o);
    assert!(Out::last_cycle(&o.cli_b) < 400, "cli replies must not wait for srv");
}

#[test]
fn stalled_cli_does_not_delay_srv() {
    // Requests first, then replies: cli stalls until cycle 400.
    let mut dgs: Vec<Dg> = (0..4).map(|k| dg(k, 3, 40, false)).collect();
    dgs.extend((4..6).map(|k| dg(k, 4, 40, false)));
    let o = run(&dgs, budget(&dgs) + 400, no_gap, always, |t| t >= 400);
    check_all(&dgs, &o);
    assert!(Out::last_cycle(&o.srv_b) < 400, "srv requests must not wait for cli");
}

#[test]
fn stalled_cli_blocks_later_srv_head_of_line() {
    // srv1 is before the reply: delivered. The reply stalls the unit, so srv2
    // behind it waits; nothing is lost or reordered.
    let dgs = vec![dg(0, 0, 30, false), dg(1, 4, 30, false), dg(2, 3, 30, false)];
    let o = run(&dgs, budget(&dgs) + 300, no_gap, always, |t| t >= 300);
    check_all(&dgs, &o);
    let srv1_done = o.srv_b[29].0;
    assert!(srv1_done < 300, "srv1 is delivered before the stalled reply");
    let cli_first = o.cli_b[0].0;
    assert!(cli_first >= 300, "the reply waits for its consumer");
    assert!(o.srv_b[30].0 > Out::last_cycle(&o.cli_b), "srv2 waits behind the reply");
}

// ------------------------------------------------------------ timing

#[test]
fn first_byte_follows_meta_by_one_cycle() {
    let dgs = vec![dg(0, 3, 1, false)];
    let o = run_free(&dgs);
    check_all(&dgs, &o);
    let (mt, _) = o.srv_m[0];
    let (bt, _) = o.srv_b[0];
    assert_eq!(bt, mt + 1, "meta in cycle c, first byte in c+1");
}

// ------------------------------------------------------------ Verilog

#[test]
fn lowers_to_verilog() {
    let v = NtpDemux::verilog("ntp_demux");
    assert!(v.contains("module ntp_demux"), "ntp_demux");
}
