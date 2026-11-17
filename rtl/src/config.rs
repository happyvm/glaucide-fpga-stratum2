// SPDX-License-Identifier: Apache-2.0
//! Static configuration (task T4.1), domain `DefaultClock` (`clk_sys`).
//!
//! There is no CPU and no bus in v1: the configuration is chosen at
//! synthesis time by a marker type implementing [`ConfigSet`], in the same
//! way as the timing sets of `phy_init` ([`Board`] is the real design,
//! [`Test`] the addresses of the Python model `tools/golden`, so that the
//! testbenches and the golden vectors agree). [`Config`] turns the chosen
//! set into output wires driven every cycle. It holds no state.
//!
//! Gateway resolution (ARP for the IPv4 gateway, NDP for an IPv6 upstream)
//! is NOT here: `gateway` is only an address, resolved by `arp.rs`.
//!
//! # Configuration sets
//!
//! | Constant | [`Board`] (default) | [`Test`] (golden model) |
//! |---|---|---|
//! | `LOCAL_MAC` | `02:4e:54:50:00:01` | `02:00:00:00:00:01` |
//! | `LOCAL_IP4` | `192.0.2.10` | `192.0.2.1` |
//! | `GATEWAY_IP4`, `NETMASK4` | `192.0.2.254`, `255.255.255.0` | same |
//! | `GATEWAY_IP6` | `fe80::1` | `fe80::1` (on a real network, the router's link-local address) |
//! | `LOCAL_GUA`, `GUA_ENABLED` | `2001:db8:0:1::50:1`, on | `2001:db8::1`, on |
//! | `UPSTREAM` | `192.0.2.1`, `2001:db8::1`, on; two slots off | `192.0.2.99`, `2001:db8::99`, on; two slots off |
//! | `POLL_LOG2` | 6 (64 s) | 6 |
//! | `PRECISION_LOG2` | -24 | -24 |
//! | `RATE_LIMIT_PER_S` | 8 | 8 |
//!
//! The upstream servers of `Board` are the documentation addresses given
//! in the roadmap; they are placeholders to replace before use on a
//! network. `Test` uses 192.0.2.99 and 2001:db8::99 because the golden
//! model's Reference ID is that of 192.0.2.99 (`ntp.refid_ipv4`), and our
//! own address cannot be our upstream.
//!
//! # Address rules (all fixed by the pure functions below)
//!
//! * MACs are `u64` (or `U<48>`) with byte 0, the first on the wire, in the
//!   most significant byte, as in `eth_rx`. `LOCAL_MAC` is unicast by
//!   construction: [`mac_is_unicast`] (group bit clear) is asserted at
//!   compile time for every set; this is the guarantee `local_mac` unicast
//!   that `eth_rx` (T1.5) assumes (`docs/formal.md`, registre des
//!   hypothèses). The orchestrator adds the `check!`; this unit only makes
//!   the value unicast and tests it.
//! * IPv4 addresses are `u32`/`U<32>`, the first byte most significant.
//! * IPv6 addresses are `u128`. Interface identifiers are the modified
//!   EUI-64 of `LOCAL_MAC` ([`eui64_link_local`], [`eui64_global`]):
//!   `ff:fe` inserted in the middle, universal/local bit flipped. The
//!   global address uses this interface identifier, not the `::1` of the
//!   golden model (see the note in `rtl/tests/config.rs`).
//! * `sn_low` is the low 24 bits of the link-local address
//!   ([`solicited_node_low24`]). Both addresses share the same interface
//!   identifier, so their low 24 bits are equal, and the solicited-node
//!   group `33:33:ff:<sn_low>` covers both (RFC 4291 2.7.1, RFC 2464 7).
//!
//! # Ports (no inputs; every output is a wire driven each cycle)
//!
//! | Port | Width | Value |
//! |---|---|---|
//! | `local_mac` | `U<48>` | `LOCAL_MAC` |
//! | `local_ip4` | `U<32>` | `LOCAL_IP4` |
//! | `local_ll` | `U<128>` | `eui64_link_local(LOCAL_MAC)` |
//! | `local_gua` | `U<128>` | `LOCAL_GUA`, valid or not |
//! | `gua_valid` | `Bit` | `GUA_ENABLED`; `local_gua` is only in use when high |
//! | `sn_low` | `U<24>` | `solicited_node_low24(local_ll)` |
//! | `gateway` | `U<32>` | `GATEWAY_IP4` |
//! | `gateway6` | `U<128>` | `GATEWAY_IP6` |
//! | `netmask` | `U<32>` | `NETMASK4` |
//! | `up<i>_addr` | `U<128>` | `UPSTREAM[i].1`, `i` in 0..4; IPv4 in the low 32 bits |
//! | `up<i>_v6` | `Bit` | `UPSTREAM[i].0` (high: IPv6) |
//! | `up<i>_en` | `Bit` | `UPSTREAM[i].2` (high: slot in use) |
//! | `poll_log2` | `U<8>` | `POLL_LOG2` |
//! | `precision` | `U<8>` | `PRECISION_LOG2`, two's complement |
//! | `rate_limit` | `U<16>` | `RATE_LIMIT_PER_S` |
//!
//! Timing: each wire is driven at every rising edge of `clk_sys` with a
//! constant, so from the first edge after reset on it shows that constant
//! for the whole run. A consumer needs no handshake and no synchroniser.
//! The hypothesis that `local_mac` is unicast is guaranteed by the
//! compile-time assertion below (for every set) and tested in
//! `rtl/tests/config.rs`; the `check!` of the proof is written by the
//! orchestrator from this header, not here.
//!
//! # Compile-time checks
//!
//! Every [`ConfigSet`] is checked, when a [`Config`] is built, for: a
//! unicast `LOCAL_MAC` that fits in 48 bits, `GATEWAY_IP6` neither multicast
//! nor unspecified ([`ip6_is_valid_gateway`]), and `POLL_LOG2` in 4..=10.
//! The check is `Config::<C>::CHECK`, forced by `Default`, and the sets
//! [`Board`] and [`Test`] also carry a module-level `const` assertion.
use std::marker::PhantomData;
use txhdl::comp::{Clock, DefaultClock, Out, Unit};
use txhdl::types::{Bit, U};
use txhdl::{check, lower, Trace};

/// The configuration of one build: every constant the design reads.
pub trait ConfigSet {
    /// Station MAC address, 48 bits, byte 0 in the top byte. Unicast.
    const LOCAL_MAC: u64;
    /// Station IPv4 address.
    const LOCAL_IP4: u32;
    /// IPv4 default gateway (resolved by `arp.rs`, not here).
    const GATEWAY_IP4: u32;
    /// IPv4 netmask (1 bits up to the network part).
    const NETMASK4: u32;
    /// IPv6 default gateway (the router): link-local `fe80::/10` or global.
    /// Must be neither multicast (`ff00::/8`) nor unspecified (`::`).
    const GATEWAY_IP6: u128;
    /// Global IPv6 address, static (a server keeps a fixed, memorable address;
    /// no SLAAC in v1). Its low 24 bits MUST equal those of the link-local
    /// address (`eui64_link_local(LOCAL_MAC)`): `ipv6_rx` has a single
    /// `sn_low` for both solicited-node groups (checked at compile time).
    const LOCAL_GUA: u128;
    /// Whether the global address is in use.
    const GUA_ENABLED: bool;
    /// Up to four upstream servers: (is IPv6, address, in use). An IPv4
    /// address is in the low 32 bits of the `u128`.
    const UPSTREAM: [(bool, u128, bool); 4];
    /// Poll interval as a power of two of seconds, 4..=10 (6 is 64 s).
    const POLL_LOG2: u8;
    /// Precision of the server, as a power of two of seconds (-24: 40 ns).
    const PRECISION_LOG2: i8;
    /// Responses allowed per second and per source before rate limiting.
    const RATE_LIMIT_PER_S: u16;
}

/// The real design (roadmap addresses, see the module header).
#[derive(Default)]
pub struct Board;
impl ConfigSet for Board {
    const LOCAL_MAC: u64 = 0x02_4E_54_50_00_01;
    const LOCAL_IP4: u32 = 0xC000_020A; // 192.0.2.10
    const GATEWAY_IP4: u32 = 0xC000_02FE; // 192.0.2.254
    const NETMASK4: u32 = 0xFFFF_FF00; // /24
    const GATEWAY_IP6: u128 = 0xFE80_0000_0000_0000_0000_0000_0000_0001; // fe80::1
    // 2001:db8:0:1::50:1 (low 24 bits 0x500001 = those of the link-local address)
    const LOCAL_GUA: u128 = 0x2001_0DB8_0000_0001_0000_0000_0050_0001;
    const GUA_ENABLED: bool = true;
    const UPSTREAM: [(bool, u128, bool); 4] = [
        (false, 0xC000_0201, true),                              // 192.0.2.1
        (true, 0x2001_0DB8_0000_0000_0000_0000_0000_0001, true), // 2001:db8::1
        (false, 0, false),
        (false, 0, false),
    ];
    const POLL_LOG2: u8 = 6;
    const PRECISION_LOG2: i8 = -24;
    const RATE_LIMIT_PER_S: u16 = 8;
}

/// The Python model (`tools/golden/vectors.py`): the addresses of the
/// golden vectors. The server is 02:00:00:00:00:01, 192.0.2.1, the upstream
/// is 192.0.2.99 / 2001:db8::99 (the golden Reference ID).
#[derive(Default)]
pub struct Test;
impl ConfigSet for Test {
    const LOCAL_MAC: u64 = 0x02_00_00_00_00_01;
    const LOCAL_IP4: u32 = 0xC000_0201; // 192.0.2.1
    const GATEWAY_IP4: u32 = 0xC000_02FE; // 192.0.2.254
    const NETMASK4: u32 = 0xFFFF_FF00; // /24
    // fe80::1: on a real network this is the router's link-local address.
    const GATEWAY_IP6: u128 = 0xFE80_0000_0000_0000_0000_0000_0000_0001; // fe80::1
    // 2001:db8::1, the golden model's SRV_GUA (low 24 bits 0x000001)
    const LOCAL_GUA: u128 = 0x2001_0DB8_0000_0000_0000_0000_0000_0001;
    const GUA_ENABLED: bool = true;
    const UPSTREAM: [(bool, u128, bool); 4] = [
        (false, 0xC000_0263, true),                              // 192.0.2.99
        (true, 0x2001_0DB8_0000_0000_0000_0000_0000_0099, true), // 2001:db8::99
        (false, 0, false),
        (false, 0, false),
    ];
    const POLL_LOG2: u8 = 6;
    const PRECISION_LOG2: i8 = -24;
    const RATE_LIMIT_PER_S: u16 = 8;
}

/// A MAC address (48 bits, byte 0 in the top byte) is unicast when the
/// group bit (bit 0 of byte 0, bit 40 of the value) is clear.
pub const fn mac_is_unicast(mac: u64) -> bool {
    mac >> 48 == 0 && (mac >> 40) & 1 == 0
}

/// The modified EUI-64 interface identifier of a MAC (RFC 4291 A): the
/// universal/local bit (bit 1 of byte 0, bit 41 of the value) is flipped,
/// and `ff:fe` is inserted between the third and fourth bytes.
pub const fn eui64_iid(mac: u64) -> u64 {
    let m = mac ^ (1 << 41);
    ((m >> 24) << 40) | (0xFFFE << 24) | (m & 0xFF_FFFF)
}

/// The link-local address `fe80::/64` plus the modified EUI-64 of `mac`.
pub const fn eui64_link_local(mac: u64) -> u128 {
    (0xFE80u128 << 112) | eui64_iid(mac) as u128
}

/// The global address `prefix::/64` plus the modified EUI-64 of `mac`.
pub const fn eui64_global(prefix: u64, mac: u64) -> u128 {
    ((prefix as u128) << 64) | eui64_iid(mac) as u128
}

/// The low 24 bits of an IPv6 address: the solicited-node group is
/// `ff02::1:ff00:0/104` plus these bits.
pub const fn solicited_node_low24(addr: u128) -> u32 {
    (addr & 0xFF_FFFF) as u32
}

/// An IPv6 default gateway is a usable unicast address: not multicast
/// (`ff00::/8`) and not the unspecified address `::`.
pub const fn ip6_is_valid_gateway(addr: u128) -> bool {
    addr != 0 && addr >> 120 != 0xFF
}

/// The compile-time checks of a configuration set (see the module header).
pub struct Checks<C: ConfigSet>(PhantomData<C>);
impl<C: ConfigSet> Checks<C> {
    pub const OK: () = {
        assert!(
            ip6_is_valid_gateway(C::GATEWAY_IP6),
            "GATEWAY_IP6 must not be multicast (ff00::/8) nor unspecified (::)"
        );
        assert!(
            mac_is_unicast(C::LOCAL_MAC),
            "LOCAL_MAC must be a 48-bit unicast address"
        );
        assert!(
            solicited_node_low24(C::LOCAL_GUA) == solicited_node_low24(eui64_link_local(C::LOCAL_MAC)),
            "LOCAL_GUA and the link-local address must share their low 24 bits (one sn_low)"
        );
        assert!(
            C::POLL_LOG2 >= 4 && C::POLL_LOG2 <= 10,
            "POLL_LOG2 must be in 4..=10"
        );
    };
}

// The same checks, on the two sets of this file, at compile time.
const _: () = Checks::<Board>::OK;
const _: () = Checks::<Test>::OK;

/// The configuration unit. Drives the wires of the set `C`.
#[derive(Trace)]
pub struct Config<C: ConfigSet> {
    _c: PhantomData<C>,
}

impl<C: ConfigSet> Config<C> {
    /// The derived constants, computed once here so that the lowered body
    /// only names constants.
    const LL: u128 = eui64_link_local(C::LOCAL_MAC);
    const GUA: u128 = C::LOCAL_GUA;
    const SN: u32 = solicited_node_low24(Self::LL);
}

impl<C: ConfigSet> Default for Config<C> {
    fn default() -> Self {
        // Forces the compile-time checks of the set when a unit is built.
        let () = Checks::<C>::OK;
        Config { _c: PhantomData }
    }
}

#[lower]
impl<C: ConfigSet>
    Unit<
        (),
        (
            Out<U<48>>,
            Out<U<32>>,
            Out<U<128>>,
            Out<U<128>>,
            Out<Bit>,
            Out<U<24>>,
            Out<U<32>>,
            Out<U<128>>,
            Out<U<32>>,
            Out<U<128>>,
            Out<Bit>,
            Out<Bit>,
            Out<U<128>>,
            Out<Bit>,
            Out<Bit>,
            Out<U<128>>,
            Out<Bit>,
            Out<Bit>,
            Out<U<128>>,
            Out<Bit>,
            Out<Bit>,
            Out<U<8>>,
            Out<U<8>>,
            Out<U<16>>,
        ),
    > for Config<C>
{
    async fn run(
        &mut self,
        _inputs: (),
        (
            local_mac,
            local_ip4,
            local_ll,
            local_gua,
            gua_valid,
            sn_low,
            gateway,
            gateway6,
            netmask,
            up0_addr,
            up0_v6,
            up0_en,
            up1_addr,
            up1_v6,
            up1_en,
            up2_addr,
            up2_v6,
            up2_en,
            up3_addr,
            up3_v6,
            up3_en,
            poll_log2,
            precision,
            rate_limit,
        ): (
            Out<U<48>>,
            Out<U<32>>,
            Out<U<128>>,
            Out<U<128>>,
            Out<Bit>,
            Out<U<24>>,
            Out<U<32>>,
            Out<U<128>>,
            Out<U<32>>,
            Out<U<128>>,
            Out<Bit>,
            Out<Bit>,
            Out<U<128>>,
            Out<Bit>,
            Out<Bit>,
            Out<U<128>>,
            Out<Bit>,
            Out<Bit>,
            Out<U<128>>,
            Out<Bit>,
            Out<Bit>,
            Out<U<8>>,
            Out<U<8>>,
            Out<U<16>>,
        ),
    ) {
        loop {
            DefaultClock::rising().await;
            // Guarantees (formal, written by the orchestrator): the texts are
            // exactly those of the `assume!`s of the consumers, so that
            // tools/formal/contracts.py pairs them.
            check!(
                !U::<48>::new(C::LOCAL_MAC as u128).bit(40),
                "local_mac is a unicast address (group bit clear)"
            );
            check!(
                U::<24>::new(Self::SN as u128) == U::<24>::new((Self::GUA & 0xFF_FFFF) as u128),
                "sn_low is the low 24 bits of both of our IPv6 addresses"
            );
            check!(
                U::<8>::new((C::GATEWAY_IP6 >> 120) as u128) != U::<8>::new(0xFF),
                "the IPv6 gateway is not a multicast address (ff00::/8)"
            );
            check!(
                U::<128>::new(C::GATEWAY_IP6) != U::<128>::new(0),
                "the IPv6 gateway is not the unspecified address ::"
            );
            local_mac.set(U::<48>::new(C::LOCAL_MAC as u128));
            local_ip4.set(U::<32>::new(C::LOCAL_IP4 as u128));
            local_ll.set(U::<128>::new(Self::LL));
            local_gua.set(U::<128>::new(Self::GUA));
            gua_valid.set(Bit::from_bool(C::GUA_ENABLED));
            sn_low.set(U::<24>::new(Self::SN as u128));
            gateway.set(U::<32>::new(C::GATEWAY_IP4 as u128));
            gateway6.set(U::<128>::new(C::GATEWAY_IP6));
            netmask.set(U::<32>::new(C::NETMASK4 as u128));
            up0_addr.set(U::<128>::new(C::UPSTREAM[0].1));
            up0_v6.set(Bit::from_bool(C::UPSTREAM[0].0));
            up0_en.set(Bit::from_bool(C::UPSTREAM[0].2));
            up1_addr.set(U::<128>::new(C::UPSTREAM[1].1));
            up1_v6.set(Bit::from_bool(C::UPSTREAM[1].0));
            up1_en.set(Bit::from_bool(C::UPSTREAM[1].2));
            up2_addr.set(U::<128>::new(C::UPSTREAM[2].1));
            up2_v6.set(Bit::from_bool(C::UPSTREAM[2].0));
            up2_en.set(Bit::from_bool(C::UPSTREAM[2].2));
            up3_addr.set(U::<128>::new(C::UPSTREAM[3].1));
            up3_v6.set(Bit::from_bool(C::UPSTREAM[3].0));
            up3_en.set(Bit::from_bool(C::UPSTREAM[3].2));
            poll_log2.set(U::<8>::new(C::POLL_LOG2 as u128));
            precision.set(U::<8>::new(C::PRECISION_LOG2 as u8 as u128));
            rate_limit.set(U::<16>::new(C::RATE_LIMIT_PER_S as u128));
        }
    }
}
