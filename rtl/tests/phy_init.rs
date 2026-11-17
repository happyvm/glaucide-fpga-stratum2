// SPDX-License-Identifier: Apache-2.0
//! Testbench of `phy_init` (T1.8): the unit drives a behavioural DP83848J
//! model on the MDIO lines. The model decodes each clause 22 frame bit by
//! bit as MDC rises, answers reads at its own address only, keeps the
//! BMCR write and the latched BMSR link bit, and records every bit of
//! every frame so that the tests compare the exact sequences.
use ntp_fpga::phy_init::{PhyInit, PhyTiming, Sim};
use txhdl::comp::{signal, DefaultClock, Running, Unit};
use txhdl::types::{Bit, U};

/// One sampled cycle of the outputs.
#[derive(Clone, Copy, Debug)]
struct Pins {
    rstn: bool,
    mdc: bool,
    o: bool,
    oe: bool,
    link: bool,
    idok: bool,
    ready: bool,
}

/// One frame: for each of its 64 bits, (the master drives the line, the
/// master's level, the level the PHY put on the line).
#[derive(Clone, Debug)]
struct Frame {
    read: bool,
    phy: u8,
    reg: u8,
    bits: Vec<(bool, bool, bool)>,
}

/// The PHY model. `chip` is the address its strap gives it (1 on the
/// Arty); a frame to another address gets no answer.
struct Phy {
    chip: u8,
    id1: u16,
    id2: u16,
    bmcr: u16,
    link: bool,
    /// The link fell since the last BMSR read: the latched-low bit.
    down: bool,
    prev_mdc: bool,
    prev_oe: bool,
    active: bool,
    edge: usize,
    cur: Vec<(bool, bool, bool)>,
    drive: bool,
    drive_val: bool,
    ours: bool,
    read: bool,
    reg: u8,
    resp: u16,
    frames: Vec<Frame>,
}

/// The bits of a field, first bit on top.
fn master(b: &[(bool, bool, bool)]) -> u16 {
    b.iter().fold(0u16, |a, x| (a << 1) | x.1 as u16)
}

impl Phy {
    fn new(chip: u8) -> Self {
        Phy {
            chip,
            id1: 0x2000,
            id2: 0x5c91,
            bmcr: 0,
            link: false,
            down: false,
            prev_mdc: false,
            prev_oe: false,
            active: false,
            edge: 0,
            cur: Vec::new(),
            drive: false,
            drive_val: false,
            ours: false,
            read: false,
            reg: 0,
            resp: 0,
            frames: Vec::new(),
        }
    }

    /// The link as the test sets it. A fall is latched until BMSR is read.
    fn set_link(&mut self, up: bool) {
        if !up {
            self.down = true;
        }
        self.link = up;
    }

    /// What the PHY puts on MDIO: its bit, or the pull-up's one.
    fn level(&self) -> bool {
        if self.drive {
            self.drive_val
        } else {
            true
        }
    }

    /// A register's value. Reading BMSR clears the latch when the link is up.
    fn reg_value(&mut self, reg: u8) -> u16 {
        match reg {
            0 => self.bmcr,
            1 => {
                let visible = self.link && !self.down;
                self.down = !self.link;
                0x7809 | if visible { 0x0004 } else { 0 }
            }
            2 => self.id1,
            3 => self.id2,
            _ => 0,
        }
    }

    /// One cycle's outputs of the master. A rising MDC edge takes the
    /// master's bit and decides the PHY's bit for the next one.
    fn observe(&mut self, mdc: bool, oe: bool, o: bool) {
        let rise = mdc && !self.prev_mdc;
        if oe && !self.prev_oe {
            self.active = true;
            self.edge = 0;
            self.cur.clear();
            self.drive = false;
        }
        self.prev_mdc = mdc;
        self.prev_oe = oe;
        if !(rise && self.active) {
            return;
        }
        let b = self.edge;
        self.cur.push((oe, o, self.level()));
        if b == 45 {
            // The header is complete: ST, OP, PHY address and register.
            let op = ((self.cur[34].1 as u8) << 1) | self.cur[35].1 as u8;
            let phy = master(&self.cur[36..41]) as u8;
            let reg = master(&self.cur[41..46]) as u8;
            self.read = op == 2;
            self.ours = phy == self.chip;
            self.reg = reg;
            self.resp = if self.ours && self.read {
                self.reg_value(reg)
            } else {
                0
            };
        }
        // The PHY's bit for the next position: the turnaround's zero (bit
        // 47), then the data, most significant bit first (bits 48 to 63).
        let nb = b + 1;
        self.drive = self.ours && self.read && (47..=63).contains(&nb);
        self.drive_val = (48..=63).contains(&nb) && (self.resp >> (63 - nb)) & 1 == 1;
        self.edge += 1;
        if self.edge == 64 {
            let data = master(&self.cur[48..64]);
            if !self.read && self.ours && self.reg == 0 {
                self.bmcr = data;
            }
            self.frames.push(Frame {
                read: self.read,
                phy: master(&self.cur[36..41]) as u8,
                reg: self.reg,
                bits: self.cur.clone(),
            });
            self.active = false;
            self.drive = false;
        }
    }
}

/// Runs the unit for `cycles` cycles. The master's address is `unit_addr`,
/// the PHY answers at `chip`. `act` runs before each cycle and may change
/// the model.
fn bench(
    unit_addr: u8,
    chip: u8,
    cycles: usize,
    mut act: impl FnMut(usize, &mut Phy),
) -> (Vec<Pins>, Phy) {
    let (addr_d, addr) = signal::<U<5>, DefaultClock>();
    let (mi_d, mi) = signal::<Bit, DefaultClock>();
    let (rstn_d, rstn) = signal::<Bit, DefaultClock>();
    let (mdc_d, mdc) = signal::<Bit, DefaultClock>();
    let (mdo_d, mdo) = signal::<Bit, DefaultClock>();
    let (oe_d, oe) = signal::<Bit, DefaultClock>();
    let (link_d, link) = signal::<Bit, DefaultClock>();
    let (idok_d, idok) = signal::<Bit, DefaultClock>();
    let (ready_d, ready) = signal::<Bit, DefaultClock>();
    let mut u = PhyInit::<Sim>::default();
    let mut sim = Running::new(u.run(
        (addr, mi),
        (rstn_d, mdc_d, mdo_d, oe_d, link_d, idok_d, ready_d),
    ));
    let mut phy = Phy::new(chip);
    let mut trace = Vec::with_capacity(cycles);
    for t in 0..cycles {
        act(t, &mut phy);
        addr_d.set(U::<5>::from(unit_addr));
        mi_d.set(Bit::from(phy.level()));
        sim.cycle();
        let p = Pins {
            rstn: rstn.get().to_bool(),
            mdc: mdc.get().to_bool(),
            o: mdo.get().to_bool(),
            oe: oe.get().to_bool(),
            link: link.get().to_bool(),
            idok: idok.get().to_bool(),
            ready: ready.get().to_bool(),
        };
        phy.observe(p.mdc, p.oe, p.o);
        trace.push(p);
    }
    (trace, phy)
}

/// The 46 bits a master drives before the turnaround: preamble, ST, OP,
/// PHY address and register.
fn header(read: bool, phy: u8, reg: u8) -> Vec<bool> {
    let mut v = vec![true; 32];
    v.extend([false, true]);
    v.extend(if read { [true, false] } else { [false, true] });
    v.extend((0..5).rev().map(|i| (phy >> i) & 1 == 1));
    v.extend((0..5).rev().map(|i| (reg >> i) & 1 == 1));
    assert_eq!(v.len(), 46);
    v
}

fn check_read(f: &Frame, phy: u8, reg: u8, data: u16) {
    assert!(f.read, "a read frame");
    assert_eq!((f.phy, f.reg), (phy, reg), "address and register");
    assert_eq!(f.bits.len(), 64, "a frame is 64 bits");
    let hdr = header(true, phy, reg);
    for i in 0..46 {
        assert!(f.bits[i].0, "bit {i}: the master drives the header");
        assert_eq!(f.bits[i].1, hdr[i], "bit {i} of the header");
    }
    for i in 46..64 {
        assert!(!f.bits[i].0, "bit {i}: released for the turnaround and data");
    }
    assert!(!f.bits[47].2, "the PHY's turnaround bit is zero");
    for i in 0..16 {
        let want = (data >> (15 - i)) & 1 == 1;
        assert_eq!(f.bits[48 + i].2, want, "PHY data bit {i}");
    }
}

fn check_write(f: &Frame, phy: u8, reg: u8, data: u16) {
    assert!(!f.read, "a write frame");
    assert_eq!((f.phy, f.reg), (phy, reg), "address and register");
    assert_eq!(f.bits.len(), 64, "a frame is 64 bits");
    let hdr = header(false, phy, reg);
    for i in 0..46 {
        assert!(f.bits[i].0, "bit {i}: the master drives the header");
        assert_eq!(f.bits[i].1, hdr[i], "bit {i} of the header");
    }
    // Turnaround `10`, then the word, all driven by the master.
    assert!(f.bits[46].0 && f.bits[46].1, "turnaround bit 1");
    assert!(f.bits[47].0 && !f.bits[47].1, "turnaround bit 0");
    for i in 0..16 {
        assert!(f.bits[48 + i].0, "data bit {i} is driven");
        assert_eq!(f.bits[48 + i].1, (data >> (15 - i)) & 1 == 1, "data bit {i}");
    }
}

/// Cycles where a frame starts: MDIO enable rises.
fn frame_starts(t: &[Pins]) -> Vec<usize> {
    (0..t.len())
        .filter(|&i| t[i].oe && (i == 0 || !t[i - 1].oe))
        .collect()
}

#[test]
fn id_reads_and_bmcr_write_are_bit_exact() {
    let (t, phy) = bench(1, 1, 40_000, |_, _| {});
    assert!(phy.frames.len() >= 4, "two ID reads, a write and a poll");
    check_read(&phy.frames[0], 1, 2, 0x2000);
    check_read(&phy.frames[1], 1, 3, 0x5c91);
    check_write(&phy.frames[2], 1, 0, 0x1200);
    assert_eq!(phy.bmcr, 0x1200, "BMCR holds auto-negotiation enable and restart");
    assert!(phy.frames[3..].iter().all(|f| f.read && f.reg == 1), "then BMSR polls");
    assert!(t.iter().any(|p| p.ready), "ready is reached");
    let first_ready = t.iter().position(|p| p.ready).unwrap();
    let write_end = frame_starts(&t)[2] + 6400;
    assert!(first_ready >= write_end, "ready only after the BMCR write");
}

#[test]
fn mdc_is_one_megahertz_and_only_toggles_on_the_divider() {
    let (t, _) = bench(1, 1, 10_000, |_, _| {});
    let changes: Vec<usize> = (1..t.len()).filter(|&i| t[i].mdc != t[i - 1].mdc).collect();
    assert!(changes.len() >= 199, "MDC toggles every 50 cycles");
    // Every change falls on the half-period grid, 50 cycles apart.
    for w in changes.windows(2) {
        assert_eq!(w[1] - w[0], 50, "MDC half-period is 50 cycles");
    }
    let rises = (1..t.len()).filter(|&i| t[i].mdc && !t[i - 1].mdc).count();
    assert!((99..=101).contains(&rises), "1 MHz: {rises} rises in 10 000 cycles");
}

#[test]
fn reset_is_held_low_then_mdio_waits_for_the_post_reset_delay() {
    let (t, _) = bench(1, 1, 40_000, |_, _| {});
    let low = t.iter().take_while(|p| !p.rstn).count();
    assert_eq!(low, Sim::RST as usize, "eth_rstn low for the reset period");
    assert!(t[low..].iter().all(|p| p.rstn), "eth_rstn stays released");
    assert!(t[..low].iter().all(|p| !p.oe), "no MDIO during reset");
    let first = frame_starts(&t)[0];
    assert!(first >= low + Sim::POST as usize, "MDIO waits for the post-reset delay");
}

#[test]
fn id_mismatch_is_retried_after_the_delay() {
    let (t, phy) = bench(1, 1, 60_000, |c, phy| {
        // The first PHYIDR1 read is wrong; afterwards the PHY is right.
        if c == 0 {
            phy.id1 = 0x1234;
        }
        if !phy.frames.is_empty() {
            phy.id1 = 0x2000;
        }
    });
    // Both IDs are read before the verdict: a PHYIDR1 mismatch sends the
    // sequence back to PHYIDR1 after the retry delay.
    check_read(&phy.frames[0], 1, 2, 0x1234);
    check_read(&phy.frames[1], 1, 3, 0x5c91);
    check_read(&phy.frames[2], 1, 2, 0x2000);
    check_read(&phy.frames[3], 1, 3, 0x5c91);
    check_write(&phy.frames[4], 1, 0, 0x1200);
    let s = frame_starts(&t);
    let gap = s[2] - s[1] - 6400;
    assert!(gap >= Sim::RETRY as usize, "the retry waits RETRY cycles (gap {gap})");
    assert!(
        t.iter().position(|p| p.idok).unwrap() > s[3],
        "id_ok only once both IDs match"
    );
    assert!(!t[..s[3]].iter().any(|p| p.idok), "id_ok stays low through the mismatch");
}

#[test]
fn wrong_phy_address_gets_no_answer_and_id_stays_low() {
    // The unit addresses PHY 2; the chip answers at 1.
    let (t, phy) = bench(2, 1, 60_000, |_, _| {});
    assert!(phy.frames.len() >= 4, "the two ID reads are retried");
    for (i, f) in phy.frames.iter().take(4).enumerate() {
        let reg = if i % 2 == 0 { 2 } else { 3 };
        assert!(f.read && f.reg == reg && f.phy == 2, "ID read at address 2");
        assert!(f.bits[48..].iter().all(|b| b.2), "no PHY drives: the pull-up reads ones");
    }
    assert!(t.iter().all(|p| !p.idok && !p.ready), "id_ok and ready stay low");
    assert_eq!(phy.bmcr, 0, "nothing is written to a PHY that does not answer");
    assert!(t.iter().all(|p| !p.link), "no link without an answer");
}

#[test]
fn link_goes_up_and_down_through_the_latched_status() {
    let (t, phy) = bench(1, 1, 200_000, |c, phy| {
        if c == 40_000 {
            phy.set_link(true);
        }
        if c == 120_000 {
            phy.set_link(false);
        }
    });
    assert!(t[..40_000].iter().all(|p| !p.link), "link_up low before the link");
    let up = t[40_000..].iter().position(|p| p.link).expect("link_up rises");
    assert!(up < 20_000, "seen within two polls ({up} cycles)");
    let down = t[120_000..].iter().position(|p| !p.link).expect("link_up falls");
    assert!(down < 20_000, "seen within two polls ({down} cycles)");
    assert!(t.last().unwrap().ready && t.last().unwrap().idok);
    let bmsr = phy.frames.iter().filter(|f| f.read && f.reg == 1).count();
    assert!(bmsr >= 6, "BMSR is polled ({bmsr} reads)");
}

#[test]
fn ready_never_without_id_ok() {
    let (t, _) = bench(1, 1, 40_000, |_, _| {});
    assert!(t.iter().all(|p| !p.ready || p.idok));
}
