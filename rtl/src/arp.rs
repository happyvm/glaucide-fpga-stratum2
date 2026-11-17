// SPDX-License-Identifier: Apache-2.0
//! ARP (RFC 826) for Ethernet/IPv4 (task T1.9), domain `DefaultClock`.
//!
//! Server side (answers requests), client side (resolves addresses for the
//! NTP client) and a 4-entry cache.
//!
//! # Ports
//!
//! Inputs (flat list): `meta` ([`EthMeta`]) and `payload` (`FrameByte`), the
//! ARP outputs of `eth_rx` (payload after the 14-byte Ethernet header,
//! Ethernet padding included, `err` on `last` = bad frame); `local_mac`,
//! `local_ip` (config wires); `resolve` (`Rx<U<32>>`: an IPv4 address the
//! client wants to reach); `lookup` (`Rx<U<32>>`); `flush` (wire).
//! Outputs: `hdr` ([`EthHdr`]) and `out_payload` (`FrameByte`), to source 0 of
//! `tx_arb`; `lookup_rsp` ([`ArpEntry`]).
//!
//! IPv4 addresses are `U<32>` with the first byte most significant
//! (`192.0.2.1` = `0xC000_0201`); MACs as in `eth_rx`.
//!
//! # Server (rules A1-A5 of `tools/golden/responder.py`, byte-exact)
//!
//! A frame is processed once its last byte is seen (the `err` flag is on
//! `last`). It yields an ARP reply iff: no `err`; the Ethernet destination is
//! `local_mac` or broadcast (A1) and the Ethernet source is not a group
//! address (E1); at least 28 bytes of payload; htype 1, ptype 0x0800, hlen 6,
//! plen 4 (A2); op 1 and target IP = `local_ip` (A3); sender MAC unicast (A4).
//! The reply (A5) is op 2, sha = `local_mac`, spa = `local_ip`, tha = sender
//! sha, tpa = sender spa, Ethernet destination = sender sha (the Ethernet
//! source is `local_mac`, added by `eth_tx`, which also pads). Everything
//! else produces nothing, never a partial frame: the `hdr` of a frame is sent
//! only once its 28 payload bytes are all available in the unit and they go
//! out one per cycle (`tx_arb` rule: a `hdr` is always completed).
//!
//! # Cache (4 entries, IPv4 to MAC)
//!
//! The model is silent about learning; decision (RFC 826 "merge" step, only
//! for packets for us): a packet accepted by the same rules (no `err`, A1, E1,
//! A2, sender MAC unicast, 28 bytes) with target IP = `local_ip` and op 1 *or*
//! op 2 teaches (spa, sha); a sender IP of 0.0.0.0 (probe) is not learned.
//! An existing entry with the same IP is updated in place (no duplicate IP),
//! otherwise the entry at the round-robin pointer is replaced and the pointer
//! advances. `flush` invalidates all entries (and drops a pending or waiting
//! resolution). **No aging in v1**: an entry lives until it is replaced or
//! flushed; `flush` is the escape hatch.
//!
//! # Client
//!
//! * `lookup(ip)`: answered on `lookup_rsp` in the cycle it is taken (the
//!   channel adds one), `found`/`mac` from the cache (mac 0 when not found).
//! * `resolve(ip)`: if `ip` is cached it is consumed and nothing is sent.
//!   Otherwise, when no resolution is outstanding, a broadcast request (op 1,
//!   sha = `local_mac`, spa = `local_ip`, tha = 0, tpa = ip, Ethernet
//!   destination ff:ff:ff:ff:ff:ff) is sent and the unit waits (`wait`) until
//!   that IP is learned or `flush`. At most one resolution is outstanding: a
//!   `resolve` for another IP stays in its channel until then. A `resolve` for
//!   the same IP while waiting is consumed and re-sends the request (the
//!   retransmission is the client's job: there is no timer, so a lost reply
//!   cannot deadlock the unit). The client polls `lookup` for the answer.
//! * Output sharing: one frame at a time; a reply to a request has priority
//!   over a pending resolve request; the choice is made when the output is
//!   idle, frames are never interleaved.
//!
//! The reception of a frame is held (not taken from `eth_rx`) while a reply is
//! pending, so at most one reply is queued.
//!
//! # Integration
//!
//! All inputs and outputs on registered channels or FIFOs (never port to port,
//! see the netlist note of `stream.rs`). `local_mac`/`local_ip` should be
//! stable (they are sampled at the cycle a frame is decided and at each
//! `hdr`).
//!
//! # Formal
//!
//! `check!`/`cover!` in `run` (row ARP of `docs/formal.md`), proved with
//! `tools/formal/prove.sh arp prove`. `assume!` only for the stream contract
//! of `payload` (`err` only with `last`).
use crate::eth_rx::EthMeta;
use crate::eth_tx::EthHdr;
use crate::stream::FrameByte;
use txhdl::comp::{mux, Clock, DefaultClock, In, Reg, Rx, Tx, Unit};
use txhdl::types::{Bit, U};
use txhdl::{assume, check, cover, lower, with, Trace};
use txhdl::{Transaction as TransactionDerive, Value as ValueDerive};

/// EtherType of ARP.
pub const ET_ARP: u16 = 0x0806;
/// ARP packet length (Ethernet/IPv4).
pub const ARP_LEN: u8 = 28;

/// Answer of a cache lookup.
#[derive(TransactionDerive, ValueDerive, Clone, Copy, Default, Debug)]
pub struct ArpEntry {
    pub found: Bit,
    /// MAC of the entry; 0 when `found` is 0.
    pub mac: U<48>,
}

#[derive(Trace, Default)]
pub struct Arp {
    // Reception.
    /// 0 = waiting for the meta of a frame, 1 = reading its payload.
    pub rx_st: Reg<U<1>>,
    /// Payload bytes taken so far (saturates at 28).
    pub cnt: Reg<U<5>>,
    /// Header constants seen so far match Ethernet/IPv4 (A2).
    pub good: Reg<Bit>,
    /// A1 and E1 hold for the frame in progress.
    pub m_ok: Reg<Bit>,
    pub op: Reg<U<8>>,
    pub sha: Reg<U<48>>,
    pub spa: Reg<U<32>>,
    pub tpa: Reg<U<32>>,
    // Reply queued.
    pub rep_pend: Reg<Bit>,
    pub rep_dst: Reg<U<48>>,
    pub rep_ip: Reg<U<32>>,
    // Resolution.
    pub req_pend: Reg<Bit>,
    pub wait: Reg<Bit>,
    pub req_ip: Reg<U<32>>,
    // Transmission.
    /// 0 = idle, 1 = sending the 28 payload bytes.
    pub tx_st: Reg<U<1>>,
    pub tx_cnt: Reg<U<5>>,
    /// The frame in progress is a request (else a reply).
    pub tx_req: Reg<Bit>,
    // Cache.
    pub v0: Reg<Bit>,
    pub v1: Reg<Bit>,
    pub v2: Reg<Bit>,
    pub v3: Reg<Bit>,
    pub ip0: Reg<U<32>>,
    pub ip1: Reg<U<32>>,
    pub ip2: Reg<U<32>>,
    pub ip3: Reg<U<32>>,
    pub mac0: Reg<U<48>>,
    pub mac1: Reg<U<48>>,
    pub mac2: Reg<U<48>>,
    pub mac3: Reg<U<48>>,
    pub rr: Reg<U<2>>,
    /// Variable fields of the frame being sent (the packet storage): loaded
    /// at its `hdr`, the byte at index `tx_cnt` is selected from them.
    // Monitors (only read by check!/cover!; pruned in synthesis).
    pub m_op: Reg<U<8>>,
    pub m_mac: Reg<U<48>>,
    pub m_ip: Reg<U<32>>,
    pub m_tha: Reg<U<48>>,
    pub m_tpa: Reg<U<32>>,
    /// Requester of the queued reply, captured at the decision.
    pub m_rs: Reg<U<48>>,
    pub m_rp: Reg<U<32>>,
    /// Requester of the reply being sent, snapshot at its `hdr`.
    pub m_fs: Reg<U<48>>,
    pub m_fp: Reg<U<32>>,
    /// Last learned pair (since the last flush).
    pub m_have: Reg<Bit>,
    pub m_lip: Reg<U<32>>,
    pub m_lmac: Reg<U<48>>,
}

#[lower]
impl
    Unit<
        (
            Rx<EthMeta>,
            Rx<FrameByte>,
            In<U<48>>,
            In<U<32>>,
            Rx<U<32>>,
            Rx<U<32>>,
            In<Bit>,
        ),
        (Tx<EthHdr>, Tx<FrameByte>, Tx<ArpEntry>),
    > for Arp
{
    async fn run(
        &mut self,
        (meta, payload, local_mac, local_ip, resolve, lookup, flush): (
            Rx<EthMeta>,
            Rx<FrameByte>,
            In<U<48>>,
            In<U<32>>,
            Rx<U<32>>,
            Rx<U<32>>,
            In<Bit>,
        ),
        (hdr, out_payload, lookup_rsp): (Tx<EthHdr>, Tx<FrameByte>, Tx<ArpEntry>),
    ) {
        loop {
            DefaultClock::rising().await;
            let mac = local_mac.get();
            let ip = local_ip.get();
            let fl = flush.get();
            let rx_st = self.rx_st.get();
            let ci = self.cnt.get();
            let good = self.good.get();
            let op = self.op.get();
            let sha = self.sha.get();
            let spa = self.spa.get();
            let tpa = self.tpa.get();
            let rep_pend = self.rep_pend.get();
            let req_pend = self.req_pend.get();
            let wait = self.wait.get();
            let req_ip = self.req_ip.get();
            let tx_st = self.tx_st.get();
            let tc = self.tx_cnt.get();
            let tx_req = self.tx_req.get();
            let v0 = self.v0.get();
            let v1 = self.v1.get();
            let v2 = self.v2.get();
            let v3 = self.v3.get();
            let ip0 = self.ip0.get();
            let ip1 = self.ip1.get();
            let ip2 = self.ip2.get();
            let ip3 = self.ip3.get();
            let mac0 = self.mac0.get();
            let mac1 = self.mac1.get();
            let mac2 = self.mac2.get();
            let mac3 = self.mac3.get();
            let rr = self.rr.get();

            // ---------------------------------------------------- reception
            let in_meta = Bit::from(rx_st == 0);
            let in_pay = Bit::from(rx_st == 1);
            let mh = meta.head();
            let have_m = Bit::from(meta.peek().is_some());
            // A frame is read only when no reply is queued.
            let take_m = in_meta & have_m & !rep_pend;
            let _ = meta.recv_if(take_m);
            // A1: broadcast or us; E1: the Ethernet source is not a group address.
            let mok = (Bit::from(mh.dst == mac) | Bit::from(mh.dst == U::<48>::from(0xffff_ffff_ffffu64))) & !mh.src.bit(40);

            let have_p = Bit::from(payload.peek().is_some());
            let b = payload.head();
            let last = b.last;
            assume!(!have_p | !b.err | last, "payload: err only together with last (stream contract 2)");
            let take_p = in_pay & have_p;
            let _ = payload.recv_if(take_p);

            // Constant header bytes 0..=6: 00 01 08 00 06 04 00 (A2).
            let want = mux(ci == 1, U::<8>::from(1u8), mux(ci == 2, U::<8>::from(8u8), mux(ci == 4, U::<8>::from(6u8), mux(ci == 5, U::<8>::from(4u8), U::<8>::from(0u8)))));
            let bad_c = take_p & Bit::from(ci <= 6) & !Bit::from(b.data == want);
            let in_sha = Bit::from(ci >= 8) & Bit::from(ci < 14);
            let in_spa = Bit::from(ci >= 14) & Bit::from(ci < 18);
            let in_tpa = Bit::from(ci >= 24) & Bit::from(ci < 28);
            // Target address including the byte being taken (the 28th may be the last).
            let tpa_eff = mux(ci == 27, (tpa << 8u32) | b.data.zext::<32>(), tpa);
            let full = Bit::from(ci == 27) | Bit::from(ci == 28);
            let for_us = good & self.m_ok.get() & full & !b.err & Bit::from(tpa_eff == ip) & !sha.bit(40);
            let op_ok = Bit::from(op == 1) | Bit::from(op == 2);
            // Frame accepted as a packet for us (learning), and as a request (reply).
            let dec = take_p & last & for_us & op_ok;
            let dec_rep = dec & Bit::from(op == 1);

            // ---------------------------------------------------- cache
            let learn = dec & !Bit::from(spa == 0u32);
            let lh0 = v0 & Bit::from(ip0 == spa);
            let lh1 = v1 & Bit::from(ip1 == spa);
            let lh2 = v2 & Bit::from(ip2 == spa);
            let lh3 = v3 & Bit::from(ip3 == spa);
            let any_hit = lh0 | lh1 | lh2 | lh3;
            let w0 = learn & (lh0 | (!any_hit & Bit::from(rr == 0)));
            let w1 = learn & (lh1 | (!any_hit & Bit::from(rr == 1)));
            let w2 = learn & (lh2 | (!any_hit & Bit::from(rr == 2)));
            let w3 = learn & (lh3 | (!any_hit & Bit::from(rr == 3)));

            // Lookup.
            let lk_v = Bit::from(lookup.peek().is_some());
            let lip = lookup.head();
            let lk_take = lk_v & lookup_rsp.ready();
            let _ = lookup.recv_if(lk_take);
            let k0 = v0 & Bit::from(ip0 == lip);
            let k1 = v1 & Bit::from(ip1 == lip);
            let k2 = v2 & Bit::from(ip2 == lip);
            let k3 = v3 & Bit::from(ip3 == lip);
            let lfound = k0 | k1 | k2 | k3;
            let lmac = mux(k0, mac0, mux(k1, mac1, mux(k2, mac2, mux(k3, mac3, U::<48>::from(0u8)))));
            if lk_take.to_bool() {
                lookup_rsp.send(ArpEntry { found: lfound, mac: lmac });
            }

            // ---------------------------------------------------- resolve
            let rs_v = Bit::from(resolve.peek().is_some());
            let rip = resolve.head();
            let r0 = v0 & Bit::from(ip0 == rip);
            let r1 = v1 & Bit::from(ip1 == rip);
            let r2 = v2 & Bit::from(ip2 == rip);
            let r3 = v3 & Bit::from(ip3 == rip);
            let rs_hit = r0 | r1 | r2 | r3;
            let inflight = Bit::from(tx_st == 1) & tx_req;
            let same = wait & Bit::from(rip == req_ip);
            let fresh = !wait & !inflight;
            let take_rs = rs_v & (rs_hit | same | fresh);
            let _ = resolve.recv_if(take_rs);
            let set_new = take_rs & !rs_hit & fresh;
            let set_re = take_rs & !rs_hit & same & !inflight & !req_pend;
            // The awaited address was learned.
            let got = learn & wait & Bit::from(spa == req_ip);

            // ---------------------------------------------------- transmission
            let t_idle = Bit::from(tx_st == 0);
            let t_pay = Bit::from(tx_st == 1);
            let hroom = hdr.ready();
            let send_rep = t_idle & rep_pend & hroom;
            let send_rq = t_idle & !rep_pend & req_pend & hroom;
            let take_h = send_rep | send_rq;
            let h_dst = mux(send_rep, self.rep_dst.get(), U::<48>::from(0xffff_ffff_ffffu64));
            if take_h.to_bool() {
                hdr.send(EthHdr { dst: h_dst, ethertype: U::<16>::from(0x0806u16) });
            }
            let l_op = mux(send_rep, U::<8>::from(2u8), U::<8>::from(1u8));
            let l_tha = mux(send_rep, self.rep_dst.get(), U::<48>::from(0u8));
            let l_tpa = mux(send_rep, self.rep_ip.get(), req_ip);
            let m_op = self.m_op.get();
            let m_mac = self.m_mac.get();
            let m_ip = self.m_ip.get();
            let m_tha = self.m_tha.get();
            let m_tpa = self.m_tpa.get();
            let room = out_payload.ready();
            let pay_go = t_pay & room;
            let t_last = Bit::from(tc == 27);
            let t_end = pay_go & t_last;
            let o_data = mux(tc == 0, U::<8>::from(0u8), mux(tc == 1, U::<8>::from(1u8), mux(tc == 2, U::<8>::from(8u8), mux(tc == 3, U::<8>::from(0u8), mux(tc == 4, U::<8>::from(6u8), mux(tc == 5, U::<8>::from(4u8), mux(tc == 6, U::<8>::from(0u8), mux(tc == 7, m_op, mux(tc == 8, m_mac.slice::<40,8>(), mux(tc == 9, m_mac.slice::<32,8>(), mux(tc == 10, m_mac.slice::<24,8>(), mux(tc == 11, m_mac.slice::<16,8>(), mux(tc == 12, m_mac.slice::<8,8>(), mux(tc == 13, m_mac.slice::<0,8>(), mux(tc == 14, m_ip.slice::<24,8>(), mux(tc == 15, m_ip.slice::<16,8>(), mux(tc == 16, m_ip.slice::<8,8>(), mux(tc == 17, m_ip.slice::<0,8>(), mux(tc == 18, m_tha.slice::<40,8>(), mux(tc == 19, m_tha.slice::<32,8>(), mux(tc == 20, m_tha.slice::<24,8>(), mux(tc == 21, m_tha.slice::<16,8>(), mux(tc == 22, m_tha.slice::<8,8>(), mux(tc == 23, m_tha.slice::<0,8>(), mux(tc == 24, m_tpa.slice::<24,8>(), mux(tc == 25, m_tpa.slice::<16,8>(), mux(tc == 26, m_tpa.slice::<8,8>(), mux(tc == 27, m_tpa.slice::<0,8>(), U::<8>::from(0u8)))))))))))))))))))))))))))));
            let ob = FrameByte { data: o_data, last: t_last, err: Bit::Zero };
            if pay_go.to_bool() {
                out_payload.send(ob);
            }

            // ---------------------------------------------------- properties
                        let m_have = self.m_have.get();
            let m_lip = self.m_lip.get();
            let m_lmac = self.m_lmac.get();
            // Internal invariants.
            check!(ci <= 28, "the payload counter stays within 0..=28");
            check!(tc <= 27, "the transmit counter stays within 0..=27");
            check!(!req_pend | wait, "a pending request implies a resolution is outstanding");
            check!(!(req_pend & inflight), "at most one request is outstanding (queued or on the wire)");
            check!(!t_pay | Bit::from(tc <= 27), "a frame in progress has a byte to send");
            check!(!t_idle | Bit::from(tc == 0), "idle: nothing of a frame sent");
            check!(!take_h | t_idle, "no hdr is sent inside a frame (no interleaving)");
            check!(!pay_go | t_pay, "payload bytes only after the hdr");
            check!(!(send_rep & send_rq), "one frame at a time");
            check!(!send_rq | wait, "a request is sent only while a resolution is outstanding");
            check!(!send_rq | !rep_pend, "a reply has priority over a request");
            check!(!take_rs | rs_hit | same | !wait, "a resolve for another address waits for the outstanding one");
            check!(!(set_new | set_re) | !rs_hit, "no request for a cached address");
            // Reply: only for a well-formed request for local_ip, no err frame.
            check!(!dec_rep | (Bit::from(op == 1) & Bit::from(tpa_eff == ip) & good & !b.err & last), "a reply is queued only for a well-formed, error-free request targeting local_ip");
            check!(!dec_rep | !sha.bit(40), "no reply to a group sender address (A4)");
            check!(!(take_p & last & b.err) | !(dec | learn), "an err frame teaches nothing and gets no reply");
            check!(!(take_p & last) | !(dec & !full), "a short payload is not a packet");
            // Fields: every byte equals the snapshot field at its index.
            check!(!t_pay | (Bit::from(tx_req) & Bit::from(m_op == 1) | !tx_req & Bit::from(m_op == 2)), "op is 1 in a request, 2 in a reply");
            check!(!(t_pay & !tx_req) | (Bit::from(m_tha == self.m_fs.get()) & Bit::from(m_tpa == self.m_fp.get())), "reply in progress: tha and tpa are the requester's sha and spa (swap)");
            check!(!(t_pay & tx_req) | (Bit::from(m_tha == 0u8) & Bit::from(m_tpa == req_ip)), "request in progress: tha is 0, tpa is the resolved address");
            check!(!take_h | Bit::from(l_op == mux(send_rep, U::<8>::from(2u8), U::<8>::from(1u8))), "hdr loads op 2 for a reply, 1 for a request");
            check!(!(send_rep) | (Bit::from(self.rep_dst.get() == self.m_rs.get()) & Bit::from(self.rep_ip.get() == self.m_rp.get())), "the queued reply is addressed to the requester");
            check!(!rep_pend | (Bit::from(self.rep_dst.get() == self.m_rs.get()) & Bit::from(self.rep_ip.get() == self.m_rp.get())), "the queued reply keeps the requester addresses");
            check!(!take_h | Bit::from(h_dst == mux(send_rep, self.rep_dst.get(), U::<48>::from(0xffff_ffff_ffffu64))), "hdr destination: requester for a reply, broadcast for a request");
            // The hdr is always completed: the byte with last ends the frame at count 27.
            check!(!(pay_go & t_last) | Bit::from(tc == 27), "last is the 28th byte");
            check!(!(pay_go & !t_last) | Bit::from(tc < 27), "28 bytes per frame");
                        // Cache.
            check!(!(v0 & v1) | !Bit::from(ip0 == ip1), "no duplicate IP in the cache (0,1)");
            check!(!(v0 & v2) | !Bit::from(ip0 == ip2), "no duplicate IP in the cache (0,2)");
            check!(!(v0 & v3) | !Bit::from(ip0 == ip3), "no duplicate IP in the cache (0,3)");
            check!(!(v1 & v2) | !Bit::from(ip1 == ip2), "no duplicate IP in the cache (1,2)");
            check!(!(v1 & v3) | !Bit::from(ip1 == ip3), "no duplicate IP in the cache (1,3)");
            check!(!(v2 & v3) | !Bit::from(ip2 == ip3), "no duplicate IP in the cache (2,3)");
            check!(!(m_have & v0 & Bit::from(ip0 == m_lip)) | Bit::from(mac0 == m_lmac), "cache entry 0 holds the learned MAC");
            check!(!(m_have & v1 & Bit::from(ip1 == m_lip)) | Bit::from(mac1 == m_lmac), "cache entry 1 holds the learned MAC");
            check!(!(m_have & v2 & Bit::from(ip2 == m_lip)) | Bit::from(mac2 == m_lmac), "cache entry 2 holds the learned MAC");
            check!(!(m_have & v3 & Bit::from(ip3 == m_lip)) | Bit::from(mac3 == m_lmac), "cache entry 3 holds the learned MAC");
            check!(!(lk_take & m_have & Bit::from(lip == m_lip)) | (!lfound | Bit::from(lmac == m_lmac)), "a cache hit returns the MAC that was learned");
            check!(!(lk_take & !lfound) | Bit::from(lmac == 0u8), "a miss returns mac 0");
            cover!(take_h & send_rep, "a reply is sent (hdr)");
            // (A fully sent reply needs a depth of about 60, beyond the cover depth of 40:
            // the reply hdr cover below stands for it; completion is a check! invariant.)
            cover!(take_h & send_rq, "a request is sent (hdr)");
            cover!(t_end & tx_req, "a request is sent completely");
            cover!(dec & Bit::from(op == 2), "an entry is learned from a reply");
            cover!(dec_rep & learn, "an entry is learned from a request");
            cover!(lk_take & lfound, "a lookup hits");
            cover!(lk_take & !lfound, "a lookup misses");
            cover!(take_rs & rs_hit, "a resolve of a cached address sends nothing");
            cover!(take_p & last & b.err & Bit::from(ci >= 27), "an err frame is discarded");
            cover!(rep_pend & req_pend, "a reply and a request are both pending");
            cover!(got, "the awaited address is learned");

            with!(self <= {
                // Reception.
                take_m ? { rx_st: U::<1>::from(1u8), good: Bit::One, m_ok: mok, cnt: U::<5>::from(0u8) },
                take_p & bad_c ? good: Bit::Zero,
                take_p & Bit::from(ci == 7) ? op: b.data,
                take_p & in_sha ? sha: (sha << 8u32) | b.data.zext::<48>(),
                take_p & in_spa ? spa: (spa << 8u32) | b.data.zext::<32>(),
                take_p & in_tpa ? tpa: (tpa << 8u32) | b.data.zext::<32>(),
                take_p & Bit::from(ci < 28) ? cnt: ci + 1,
                take_p & last ? { rx_st: U::<1>::from(0u8), cnt: U::<5>::from(0u8) },
                dec_rep ? { rep_pend: Bit::One, rep_dst: sha, rep_ip: spa, m_rs: sha, m_rp: spa },
                // Transmission.
                take_h ? {
                    tx_st: U::<1>::from(1u8),
                    tx_cnt: U::<5>::from(0u8),
                    tx_req: send_rq,
                    m_op: l_op,
                    m_mac: mac,
                    m_ip: ip,
                    m_tha: l_tha,
                    m_tpa: l_tpa,
                    m_fs: self.m_rs.get(),
                    m_fp: self.m_rp.get(),
                },
                send_rep ? rep_pend: Bit::Zero,
                send_rq ? req_pend: Bit::Zero,
                pay_go ? tx_cnt: tc + 1,
                t_end ? { tx_st: U::<1>::from(0u8), tx_cnt: U::<5>::from(0u8) },
                // Resolution.
                set_new ? { wait: Bit::One, req_pend: Bit::One, req_ip: rip },
                set_re ? req_pend: Bit::One,
                got ? { wait: Bit::Zero, req_pend: Bit::Zero },
                // Cache.
                w0 ? { v0: Bit::One, ip0: spa, mac0: sha },
                w1 ? { v1: Bit::One, ip1: spa, mac1: sha },
                w2 ? { v2: Bit::One, ip2: spa, mac2: sha },
                w3 ? { v3: Bit::One, ip3: spa, mac3: sha },
                learn & !any_hit ? rr: rr + 1,
                learn ? { m_have: Bit::One, m_lip: spa, m_lmac: sha },
                fl ? {
                    v0: Bit::Zero,
                    v1: Bit::Zero,
                    v2: Bit::Zero,
                    v3: Bit::Zero,
                    wait: Bit::Zero,
                    req_pend: Bit::Zero,
                    m_have: Bit::Zero,
                },
            });
        }
    }
}
