// SPDX-License-Identifier: Apache-2.0
//! Cycle-by-cycle tests of the Ethernet frame builder (T1.6).
use ntp_fpga::eth_tx::{EthHdr, EthTx};
use ntp_fpga::stream::FrameByte;
use txhdl::comp::{chan, signal, DefaultClock, Running, Unit};
use txhdl::types::U;

const MAC: u64 = 0x0211_2233_4455;
const DST: u64 = 0xffff_ffff_ffff;

/// One (header, payload) pair to feed; `abort` puts `err` on the last byte.
#[derive(Clone)]
struct Job {
    dst: u64,
    et: u16,
    payload: Vec<u8>,
    abort: bool,
}

fn job(n: usize) -> Job {
    Job { dst: DST, et: 0x0800, payload: payload(n), abort: false }
}

fn payload(n: usize) -> Vec<u8> {
    (0..n).map(|i| (i * 7 + 3) as u8).collect()
}

#[derive(Debug, Clone, PartialEq)]
struct Got {
    data: u8,
    last: bool,
    err: bool,
}

/// Independent bitwise CRC-32 (IEEE 802.3, reflected), complemented.
fn crc32_ref(bytes: &[u8]) -> u32 {
    let mut c = 0xffff_ffffu32;
    for &b in bytes {
        for i in 0..8 {
            let bit = ((c ^ (b as u32 >> i)) & 1) != 0;
            c >>= 1;
            if bit {
                c ^= 0xedb8_8320;
            }
        }
    }
    !c
}

/// The frame the unit must produce for a job.
fn expect(j: &Job) -> Vec<u8> {
    let mut f = vec![];
    f.extend_from_slice(&j.dst.to_be_bytes()[2..]);
    f.extend_from_slice(&MAC.to_be_bytes()[2..]);
    f.extend_from_slice(&j.et.to_be_bytes());
    f.extend_from_slice(&j.payload);
    while f.len() < 60 {
        f.push(0);
    }
    let c = crc32_ref(&f);
    f.extend_from_slice(&c.to_le_bytes());
    f
}

/// Feed `jobs` (`pgap(t)`: the payload source offers a byte at cycle t;
/// `take(t)`: the consumer accepts at cycle t), run `cycles`, return the
/// output bytes and, for each job, the cycle its header was offered.
fn run(
    jobs: &[Job],
    cycles: usize,
    pgap: impl Fn(usize) -> bool,
    take: impl Fn(usize) -> bool,
) -> Vec<Got> {
    let (h_tx, h_rx) = chan::<EthHdr, DefaultClock>();
    let (p_tx, p_rx) = chan::<FrameByte, DefaultClock>();
    let (mac_o, mac) = signal::<U<48>, DefaultClock>();
    let (f_tx, f_rx) = chan::<FrameByte, DefaultClock>();
    let mut m = EthTx::default();
    let mut sim = Running::new(m.run((h_rx, p_rx, mac), f_tx));
    // Flatten the jobs into a header queue and a payload byte queue.
    let mut pbytes: Vec<FrameByte> = vec![];
    for j in jobs {
        let n = j.payload.len();
        for (i, &b) in j.payload.iter().enumerate() {
            let l = i + 1 == n;
            pbytes.push(FrameByte::new(b, l, l && j.abort));
        }
    }
    let (mut nh, mut np) = (0, 0);
    let mut out = vec![];
    for t in 0..cycles {
        mac_o.set(U::<48>::from(MAC));
        if nh < jobs.len() && h_tx.ready().to_bool() {
            h_tx.send(EthHdr {
                dst: U::<48>::from(jobs[nh].dst),
                ethertype: U::<16>::from(jobs[nh].et),
            });
            nh += 1;
        }
        if np < pbytes.len() && pgap(t) && p_tx.ready().to_bool() {
            p_tx.send(pbytes[np]);
            np += 1;
        }
        if take(t) {
            if let Some(b) = f_rx.recv() {
                out.push(Got { data: b.data.raw() as u8, last: b.last.to_bool(), err: b.err.to_bool() });
            }
        }
        sim.cycle();
    }
    out
}

/// Split into frames; stream contract: `last` once at the end, `err` only
/// with `last`.
fn frames(o: &[Got]) -> Vec<Vec<Got>> {
    let mut all = vec![];
    let mut cur = vec![];
    for b in o {
        assert!(!b.err || b.last, "err without last");
        cur.push(b.clone());
        if b.last {
            all.push(std::mem::take(&mut cur));
        }
    }
    assert!(cur.is_empty(), "bytes after the last frame's last");
    all
}

fn data(f: &[Got]) -> Vec<u8> {
    f.iter().map(|g| g.data).collect()
}

fn check_good(jobs: &[Job], o: &[Got]) {
    let f = frames(o);
    assert_eq!(f.len(), jobs.len());
    for (fr, j) in f.iter().zip(jobs) {
        let e = expect(j);
        assert_eq!(data(fr), e);
        assert!(!fr.last().unwrap().err);
        // The FCS is independently valid: CRC of the frame with FCS = residue.
        assert_eq!(crc32_ref(&e), 0x2144_df1c);
        let (body, fcs) = e.split_at(e.len() - 4);
        assert_eq!(crc32_ref(body).to_le_bytes(), fcs);
    }
}

fn always(_: usize) -> bool {
    true
}

#[test]
fn payload_46_gives_64_bytes() {
    let j = job(46);
    let o = run(&[j.clone()], 200, always, always);
    check_good(&[j], &o);
    assert_eq!(frames(&o)[0].len(), 64);
}

#[test]
fn payload_100_gives_118_bytes() {
    let j = job(100);
    let o = run(&[j.clone()], 300, always, always);
    check_good(&[j], &o);
    assert_eq!(frames(&o)[0].len(), 118);
}

#[test]
fn payload_1_is_padded() {
    let j = Job { payload: vec![0x45], ..job(1) };
    let o = run(&[j.clone()], 200, always, always);
    check_good(&[j], &o);
    let f = frames(&o);
    assert_eq!(f[0].len(), 64);
    assert!(data(&f[0])[15..60].iter().all(|&b| b == 0));
}

#[test]
fn every_short_length_is_padded() {
    for n in 1..=50 {
        let j = job(n);
        let o = run(&[j.clone()], 300, always, always);
        check_good(&[j], &o);
        assert_eq!(frames(&o)[0].len(), (14 + n).max(60) + 4, "n={n}");
    }
}

#[test]
fn max_payload_1500() {
    let j = job(1500);
    let o = run(&[j.clone()], 1700, always, always);
    check_good(&[j], &o);
    assert_eq!(frames(&o)[0].len(), 1518);
}

#[test]
fn zlib_vectors() {
    // zlib.crc32 of the padded frames (python3), FCS little endian.
    // Frame 1: dst ff*6, src 02:11:22:33:44:55, type 0800, payload 45, zero pad.
    let j = Job { payload: vec![0x45], ..job(1) };
    let o = run(&[j], 200, always, always);
    let f = data(&frames(&o)[0]);
    assert_eq!(&f[60..], &0x4b46_be66u32.to_le_bytes());
    // Frame 2: 100 payload bytes (i*7+3), 114 bytes before the FCS.
    let j = job(100);
    let o = run(&[j], 300, always, always);
    let f = data(&frames(&o)[0]);
    assert_eq!(f.len(), 118);
    assert_eq!(&f[114..], &0xda83_64dau32.to_le_bytes());
}

#[test]
fn header_byte_order() {
    let j = Job { dst: 0x0123_4567_89ab, et: 0x86dd, payload: payload(60), abort: false };
    let o = run(&[j.clone()], 300, always, always);
    let f = data(&frames(&o)[0]);
    assert_eq!(&f[..6], &[0x01, 0x23, 0x45, 0x67, 0x89, 0xab]);
    assert_eq!(&f[6..12], &[0x02, 0x11, 0x22, 0x33, 0x44, 0x55]);
    assert_eq!(&f[12..14], &[0x86, 0xdd]);
    check_good(&[j], &o);
}

#[test]
fn payload_err_aborts() {
    let mut j = job(30);
    j.abort = true;
    let o = run(&[j], 200, always, always);
    let f = frames(&o);
    assert_eq!(f.len(), 1);
    let l = f[0].last().unwrap();
    assert!(l.last && l.err);
    assert!(f[0][..f[0].len() - 1].iter().all(|g| !g.err));
    assert!(f[0].len() < 64, "no padding and no FCS after an abort");
}

#[test]
fn abort_then_good_frame() {
    let mut a = job(30);
    a.abort = true;
    let g = job(50);
    let o = run(&[a, g.clone()], 400, always, always);
    let f = frames(&o);
    assert_eq!(f.len(), 2);
    assert!(f[0].last().unwrap().err);
    assert_eq!(data(&f[1]), expect(&g));
    assert!(!f[1].last().unwrap().err);
}

#[test]
fn too_long_payload_aborts_and_is_swallowed() {
    let long = job(1501);
    let g = job(40);
    let o = run(&[long, g.clone()], 3600, always, always);
    let f = frames(&o);
    assert_eq!(f.len(), 2);
    assert!(f[0].last().unwrap().err);
    assert_eq!(f[0].len(), 1514 + 1, "1514 bytes, then the abort byte");
    assert_eq!(data(&f[1]), expect(&g));
    // Exactly 1500 is fine and 1501 is not, also when the long one is the
    // very last byte of the payload.
    let o = run(&[job(1500)], 1700, always, always);
    assert!(!frames(&o)[0].last().unwrap().err);
}

#[test]
fn back_to_back_frames() {
    let jobs: Vec<Job> = [46, 1, 100, 60, 3, 46].iter().map(|&n| job(n)).collect();
    let o = run(&jobs, 1500, always, always);
    check_good(&jobs, &o);
}

#[test]
fn payload_with_gaps() {
    let jobs = [job(20), job(100), job(5)];
    let o = run(&jobs, 3000, |t| t % 3 == 0 || t % 7 == 1, always);
    check_good(&jobs, &o);
    let o = run(&jobs, 3000, |t| t % 11 == 0, always);
    check_good(&jobs, &o);
}

#[test]
fn output_backpressure() {
    let jobs = [job(20), job(100), job(46)];
    let o = run(&jobs, 3000, always, |t| t % 3 == 0);
    check_good(&jobs, &o);
    let o = run(&jobs, 3000, |t| t % 5 != 2, |t| (t / 7) % 2 == 0);
    check_good(&jobs, &o);
}

/// Cover points too deep for a bounded formal search (64+ bytes, a
/// 1514-byte abort) are reached here, in simulation, on one thread.
#[test]
fn all_cover_points_reached_in_simulation() {
    let mut abort = job(30);
    abort.abort = true;
    let _ = run(&[job(100), job(1), job(46), abort, job(1501)], 4200, always, always);
    for msg in [
        "a long frame is sent complete",
        "a short frame is padded",
        "a frame ends with its FCS at the minimum size",
        "a frame is aborted by the payload",
        "a frame is aborted for length",
    ] {
        assert!(txhdl::formal::covered(msg) > 0, "cover point never reached: {msg}");
    }
}
