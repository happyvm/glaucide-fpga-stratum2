// SPDX-License-Identifier: Apache-2.0
//! Tests of the static configuration unit (T4.1).
//!
//! Expected values are literals, cross-checked with Python 3 (`ipaddress`
//! and the RFC 4291 A / RFC 2464 rules written out by hand):
//!
//! ```text
//! python3 -I -c "import ipaddress; m=[0x02,0,0,0,0,1]; m[0]^=2; \
//!   print(ipaddress.IPv6Address(bytes([0xfe,0x80]+[0]*6+m[:3]+[0xff,0xfe]+m[3:])))"
//! # -> fe80::ff:fe00:1
//! python3 -I -c "import hashlib,ipaddress; \
//!   print(hashlib.md5(ipaddress.IPv6Address('2001:db8::99').packed).digest()[:4].hex())"
//! # -> b70a6091 (the golden Reference ID of tools/golden/vectors.py)
//! ```
use ntp_fpga::config::{
    eui64_global, eui64_link_local, mac_is_unicast, solicited_node_low24, Board, Config, ConfigSet,
    Test,
};
use txhdl::comp::{signal, DefaultClock, In, Running, Unit};
use txhdl::types::{Bit, U};

// ---------------------------------------------------------------- EUI-64

#[test]
fn link_local_of_the_golden_server_mac() {
    // 02:00:00:00:00:01: the u/l bit is set, so the flip clears it.
    assert_eq!(
        eui64_link_local(0x02_00_00_00_00_01),
        0xFE80_0000_0000_0000_0000_00FF_FE00_0001
    );
    // The golden model's SRV_LL, fe80::ff:fe00:1 (the same value).
}

#[test]
fn link_local_flips_a_clear_bit() {
    // 00:11:22:33:44:55: the u/l bit is clear, the flip sets it: 02:11:...
    // -> fe80::211:22ff:fe33:4455 (the classic example of RFC 4291 A).
    assert_eq!(
        eui64_link_local(0x00_11_22_33_44_55),
        0xFE80_0000_0000_0000_0211_22FF_FE33_4455
    );
}

#[test]
fn link_local_of_the_board_mac() {
    // 02:4e:54:50:00:01 -> fe80::4e:54ff:fe50:1
    assert_eq!(
        eui64_link_local(0x02_4E_54_50_00_01),
        0xFE80_0000_0000_0000_004E_54FF_FE50_0001
    );
}

#[test]
fn global_uses_the_prefix_and_the_same_interface_id() {
    // Test set: 2001:db8::/64 + ff:fe00:1 = 2001:db8::ff:fe00:1
    assert_eq!(
        eui64_global(0x2001_0DB8_0000_0000, 0x02_00_00_00_00_01),
        0x2001_0DB8_0000_0000_0000_00FF_FE00_0001
    );
    // Board set: 2001:db8:0:1::/64 + 4e:54ff:fe50:1
    assert_eq!(
        eui64_global(0x2001_0DB8_0000_0001, 0x02_4E_54_50_00_01),
        0x2001_0DB8_0000_0001_004E_54FF_FE50_0001
    );
    // The interface id of the global address is that of the link-local one.
    let ll = eui64_link_local(0x02_4E_54_50_00_01);
    let gua = eui64_global(0x2001_0DB8_0000_0001, 0x02_4E_54_50_00_01);
    assert_eq!(ll as u64, gua as u64);
}

// ---------------------------------------------------------------- MAC and solicited-node

#[test]
fn unicast_predicate() {
    assert!(mac_is_unicast(0x02_00_00_00_00_01));
    assert!(mac_is_unicast(0x00_11_22_33_44_55));
    assert!(mac_is_unicast(0xFE_FF_FF_FF_FF_FE));
    // Group bit set: multicast and broadcast.
    assert!(!mac_is_unicast(0x01_00_5E_00_00_01));
    assert!(!mac_is_unicast(0x03_00_00_00_00_00));
    assert!(!mac_is_unicast(0xFF_FF_FF_FF_FF_FF));
    // More than 48 bits: not a MAC.
    assert!(!mac_is_unicast(0x1_0200_0000_0001));
}

#[test]
fn solicited_node_takes_the_low_24_bits() {
    // 2001:db8::ff:fe00:1 and fe80::ff:fe00:1 both end in 00:00:01.
    assert_eq!(
        solicited_node_low24(0x2001_0DB8_0000_0000_0000_00FF_FE00_0001),
        0x00_0001
    );
    assert_eq!(
        solicited_node_low24(0xFE80_0000_0000_0000_0000_00FF_FE00_0001),
        0x00_0001
    );
    // Board: 4e:54ff:fe50:1 ends in 50:00:01.
    assert_eq!(
        solicited_node_low24(0xFE80_0000_0000_0000_004E_54FF_FE50_0001),
        0x50_0001
    );
    assert_eq!(
        solicited_node_low24(0x2001_0DB8_0000_0000_0000_0000_DEAD_BEEF),
        0xAD_BEEF
    );
}

// ---------------------------------------------------------------- the sets

#[test]
fn every_set_has_a_unicast_mac_and_a_valid_poll() {
    fn check<C: ConfigSet>() {
        assert!(mac_is_unicast(C::LOCAL_MAC), "LOCAL_MAC is unicast");
        assert!((4..=10).contains(&C::POLL_LOG2), "POLL_LOG2 is in 4..=10");
        assert!(C::LOCAL_MAC < 1 << 48, "LOCAL_MAC fits in 48 bits");
    }
    check::<Board>();
    check::<Test>();
}

#[test]
fn test_set_is_the_golden_model() {
    // tools/golden/vectors.py: SRV_MAC, SRV_IP4, SRV_LL.
    assert_eq!(Test::LOCAL_MAC, 0x02_00_00_00_00_01);
    assert_eq!(Test::LOCAL_IP4, 0xC000_0201); // 192.0.2.1
    assert_eq!(
        eui64_link_local(Test::LOCAL_MAC),
        0xFE80_0000_0000_0000_0000_00FF_FE00_0001 // fe80::ff:fe00:1
    );
    // Upstream: 192.0.2.99 and 2001:db8::99, the golden Reference IDs
    // (c0000263 and b70a6091, see the header of this file).
    assert_eq!(Test::UPSTREAM[0], (false, 0xC000_0263, true));
    assert_eq!(
        Test::UPSTREAM[1],
        (true, 0x2001_0DB8_0000_0000_0000_0000_0000_0099, true)
    );
    assert_eq!(Test::POLL_LOG2, 6);
    assert_eq!(Test::PRECISION_LOG2, -24);
    assert_eq!(Test::STRATUM_OFFSET, 1);
}

/// NOTE (deviation, to be decided by the owner): the golden model names
/// the server's global address `2001:db8::1` (`SRV_GUA`), whereas the spec
/// of T4.1 builds it as prefix + EUI-64 of the MAC, which gives
/// `2001:db8::ff:fe00:1`. The unit follows the spec. The golden vectors
/// `ping6`, `ntp6_basic` and `ndp_ns_na` use `2001:db8::1` and must be
/// regenerated (or the rule changed) before the end-to-end tests agree.
#[test]
fn test_set_global_address_follows_eui64_not_golden_gua() {
    assert_eq!(
        eui64_global(Test::GUA_PREFIX, Test::LOCAL_MAC),
        0x2001_0DB8_0000_0000_0000_00FF_FE00_0001 // 2001:db8::ff:fe00:1
    );
    assert_ne!(
        eui64_global(Test::GUA_PREFIX, Test::LOCAL_MAC),
        0x2001_0DB8_0000_0000_0000_0000_0000_0001 // golden 2001:db8::1
    );
}

// ---------------------------------------------------------------- the unit

/// The readers of the unit's wires, in the port order of `Config`.
struct Wires {
    local_mac: In<U<48>>,
    local_ip4: In<U<32>>,
    local_ll: In<U<128>>,
    local_gua: In<U<128>>,
    gua_valid: In<Bit>,
    sn_low: In<U<24>>,
    gateway: In<U<32>>,
    netmask: In<U<32>>,
    up: [(In<U<128>>, In<Bit>, In<Bit>); 4],
    poll_log2: In<U<8>>,
    precision: In<U<8>>,
    stratum_offset: In<U<8>>,
    rate_limit: In<U<16>>,
}

fn bench<C: ConfigSet + 'static>() -> (Wires, Running<impl std::future::Future<Output = ()>>) {
    let (local_mac_d, local_mac) = signal::<U<48>, DefaultClock>();
    let (local_ip4_d, local_ip4) = signal::<U<32>, DefaultClock>();
    let (local_ll_d, local_ll) = signal::<U<128>, DefaultClock>();
    let (local_gua_d, local_gua) = signal::<U<128>, DefaultClock>();
    let (gua_valid_d, gua_valid) = signal::<Bit, DefaultClock>();
    let (sn_low_d, sn_low) = signal::<U<24>, DefaultClock>();
    let (gateway_d, gateway) = signal::<U<32>, DefaultClock>();
    let (netmask_d, netmask) = signal::<U<32>, DefaultClock>();
    let (u0a_d, u0a) = signal::<U<128>, DefaultClock>();
    let (u0v_d, u0v) = signal::<Bit, DefaultClock>();
    let (u0e_d, u0e) = signal::<Bit, DefaultClock>();
    let (u1a_d, u1a) = signal::<U<128>, DefaultClock>();
    let (u1v_d, u1v) = signal::<Bit, DefaultClock>();
    let (u1e_d, u1e) = signal::<Bit, DefaultClock>();
    let (u2a_d, u2a) = signal::<U<128>, DefaultClock>();
    let (u2v_d, u2v) = signal::<Bit, DefaultClock>();
    let (u2e_d, u2e) = signal::<Bit, DefaultClock>();
    let (u3a_d, u3a) = signal::<U<128>, DefaultClock>();
    let (u3v_d, u3v) = signal::<Bit, DefaultClock>();
    let (u3e_d, u3e) = signal::<Bit, DefaultClock>();
    let (poll_d, poll_log2) = signal::<U<8>, DefaultClock>();
    let (prec_d, precision) = signal::<U<8>, DefaultClock>();
    let (strat_d, stratum_offset) = signal::<U<8>, DefaultClock>();
    let (rate_d, rate_limit) = signal::<U<16>, DefaultClock>();

    let c: &'static mut Config<C> = Box::leak(Box::default());
    let sim = Running::new(c.run(
        (),
        (
            local_mac_d,
            local_ip4_d,
            local_ll_d,
            local_gua_d,
            gua_valid_d,
            sn_low_d,
            gateway_d,
            netmask_d,
            u0a_d,
            u0v_d,
            u0e_d,
            u1a_d,
            u1v_d,
            u1e_d,
            u2a_d,
            u2v_d,
            u2e_d,
            u3a_d,
            u3v_d,
            u3e_d,
            poll_d,
            prec_d,
            strat_d,
            rate_d,
        ),
    ));
    let wires = Wires {
        local_mac,
        local_ip4,
        local_ll,
        local_gua,
        gua_valid,
        sn_low,
        gateway,
        netmask,
        up: [
            (u0a, u0v, u0e),
            (u1a, u1v, u1e),
            (u2a, u2v, u2e),
            (u3a, u3v, u3e),
        ],
        poll_log2,
        precision,
        stratum_offset,
        rate_limit,
    };
    (wires, sim)
}

/// A bit as 0 or 1.
fn bit(b: Bit) -> u128 {
    b.to_bool() as u128
}

/// The value of every wire after one cycle, flat, in port order (booleans
/// as 0/1, each upstream slot as address, v6, en).
fn sample(w: &Wires) -> Vec<u128> {
    let mut v = vec![
        w.local_mac.get().raw(),
        w.local_ip4.get().raw(),
        w.local_ll.get().raw(),
        w.local_gua.get().raw(),
        bit(w.gua_valid.get()),
        w.sn_low.get().raw(),
        w.gateway.get().raw(),
        w.netmask.get().raw(),
    ];
    for (a, vb, e) in &w.up {
        v.extend([a.get().raw(), bit(vb.get()), bit(e.get())]);
    }
    v.extend([
        w.poll_log2.get().raw(),
        w.precision.get().raw(),
        w.stratum_offset.get().raw(),
        w.rate_limit.get().raw(),
    ]);
    v
}

#[test]
fn board_wires_carry_the_board_constants() {
    let (w, mut sim) = bench::<Board>();
    // Three cycles: the wires show the same constants every cycle.
    for _ in 0..3 {
        sim.cycle();
        assert_eq!(
            sample(&w),
            vec![
                0x02_4E_54_50_00_01,
                0xC000_020A,
                0xFE80_0000_0000_0000_004E_54FF_FE50_0001,
                0x2001_0DB8_0000_0001_004E_54FF_FE50_0001,
                1,
                0x50_0001,
                0xC000_02FE,
                0xFFFF_FF00,
                0xC000_0201,
                0,
                1,
                0x2001_0DB8_0000_0000_0000_0000_0000_0001,
                1,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
                6,
                0xE8, // -24 in two's complement
                1,
                8,
            ]
        );
    }
}

#[test]
fn test_wires_carry_the_golden_constants() {
    let (w, mut sim) = bench::<Test>();
    for _ in 0..3 {
        sim.cycle();
        assert_eq!(
            sample(&w),
            vec![
                0x02_00_00_00_00_01,
                0xC000_0201,
                0xFE80_0000_0000_0000_0000_00FF_FE00_0001,
                0x2001_0DB8_0000_0000_0000_00FF_FE00_0001,
                1,
                0x00_0001,
                0xC000_02FE,
                0xFFFF_FF00,
                0xC000_0263,
                0,
                1,
                0x2001_0DB8_0000_0000_0000_0000_0000_0099,
                1,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
                6,
                0xE8,
                1,
                8,
            ]
        );
    }
}

#[test]
fn local_mac_wire_is_unicast_in_simulation() {
    let (w, mut sim) = bench::<Board>();
    sim.cycle();
    assert!(mac_is_unicast(w.local_mac.get().raw() as u64));
    let (w, mut sim) = bench::<Test>();
    sim.cycle();
    assert!(mac_is_unicast(w.local_mac.get().raw() as u64));
}

#[test]
fn verilog_lowers_both_sets() {
    let b = Config::<Board>::verilog("config_board");
    assert!(b.contains("module config_board"), "Board lowers to Verilog");
    let t = Config::<Test>::verilog("config_test");
    assert!(t.contains("module config_test"), "Test lowers to Verilog");
}
