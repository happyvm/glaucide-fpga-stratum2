// SPDX-License-Identifier: Apache-2.0
//! Formal check of `ipv4_tx::ipv4_hdr_cksum` (orchestrator, F-IPv4).
//!
//! RFC 1071 §1: a header is valid iff the one's complement sum of ALL its 16-bit
//! words, checksum field included, is 0xFFFF (after folding the carries). This unit
//! asserts exactly that for the checksum our builder computes, for every
//! (source, destination, protocol, total length): it does not reuse the code under
//! test (`csum_add`/`csum_fold`): plain 32-bit sums and two explicit folds.
//! The ten words are those of the table in the header of `ipv4_tx`.
use crate::ipv4_tx::{ipv4_hdr_cksum, TTL};
use txhdl::comp::{Clock, DefaultClock, In, Out, Reg, Unit};
use txhdl::types::{Bit, U};
use txhdl::{check, lower, Trace};

#[derive(Trace, Default)]
pub struct Ipv4HdrEquiv {
    pub dummy: Reg<U<1>>,
}

#[lower]
impl Unit for Ipv4HdrEquiv {
    async fn run(
        &mut self,
        (ip, dst, proto, total): (In<U<32>>, In<U<32>>, In<U<8>>, In<U<16>>),
        ok: Out<Bit>,
    ) {
        loop {
            DefaultClock::rising().await;
            let ck = ipv4_hdr_cksum(ip.get(), dst.get(), proto.get(), total.get());
            // The nine non-checksum words in table order (version/IHL/DSCP, total
            // length, flags/offset, TTL/protocol, source hi/lo, destination hi/lo),
            // summed in the same order as the builder so the solver only has to
            // reason about the RFC 1071 identity (fold, complement), not about the
            // associativity of the additions; then the checksum word itself.
            let words = U::<32>::from(0x4500u32)
                + total.get().zext::<32>()
                + U::<32>::from(0x4000u32)
                + (U::<32>::from((TTL as u32) << 8) | proto.get().zext::<32>())
                + ip.get().slice::<16, 16>().zext::<32>()
                + ip.get().slice::<0, 16>().zext::<32>()
                + dst.get().slice::<16, 16>().zext::<32>()
                + dst.get().slice::<0, 16>().zext::<32>();
            let sum = words + ck.zext::<32>();
            // At most ten words of 16 bits: the sum is below 2^20; two folds suffice.
            let f1 = (sum & U::<32>::from(0xFFFFu32)) + (sum >> 16u32);
            let f2 = (f1 & U::<32>::from(0xFFFFu32)) + (f1 >> 16u32);
            check!(
                f2.slice::<0, 16>() == U::<16>::from(0xFFFFu16),
                "the IPv4 header checksum verifies (RFC 1071: the folded sum of all ten words is 0xFFFF)"
            );
            ok.set(Bit::One);
        }
    }
}
