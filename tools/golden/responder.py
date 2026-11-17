"""Behavioural reference of the server: respond(frame, config) -> list of frames to send.

Frames are Ethernet without FCS. Input frames may carry Ethernet padding; output
frames are NOT padded (they are the Ethernet builder output, `eth_tx` pads/adds FCS).
Anything not explicitly accepted by a rule below yields [] (silence).
"""
import ipaddress
from dataclasses import dataclass
from typing import List, Optional

from . import arp, eth, icmp, icmpv6, ipv4, ipv6, ntp, udp
from .errors import ParseError

NTP_PORT = 123
REPLY_TTL = 64        # RFC 1122 section 3.2.1.7 / RFC 8200: any sensible default
REPLY_HOP_LIMIT = 64  # echo and UDP replies; NDP uses 255 (RFC 4861 section 7.1)
ALL_NODES = ipv6.addr("ff02::1")


@dataclass
class Config:
    mac: bytes
    ipv4: bytes
    ipv6_ll: bytes
    ipv6_global: Optional[bytes] = None
    stratum: int = 2
    ref_id: bytes = b"\xc0\x00\x02\x63"      # IPv4 reference ID (family rule, see ntp.py)
    ref_id_v6: Optional[bytes] = None          # reference ID in IPv6 replies; None = ref_id
    precision: int = -24
    root_delay: int = 0                         # raw 16.16
    root_dispersion: int = 0                    # raw 16.16
    ref_ts: int = 0                             # reference timestamp (last update)
    recv_ts: int = 0                            # T2: timestamp of the request's arrival
    xmit_ts: int = 0                            # T3: timestamp of the reply's departure
    synchronised: bool = True

    def v6_addresses(self):
        return [a for a in (self.ipv6_ll, self.ipv6_global) if a is not None]


def respond(frame: bytes, cfg: Config) -> List[bytes]:
    try:
        return _respond(frame, cfg)
    except ParseError:
        return []  # every malformed input is dropped silently


def _respond(frame: bytes, cfg: Config) -> List[bytes]:
    f = eth.EthFrame.parse(frame)  # runt (< 14 bytes) -> ParseError -> silence
    # Rule E1 (IEEE 802.3): a source address with the group bit set is illegal -> drop.
    if eth.is_multicast(f.src):
        return []
    if f.ethertype == eth.ETHERTYPE_ARP:
        return _arp(f, cfg)
    if f.ethertype == eth.ETHERTYPE_IPV4:
        return _ipv4(f, cfg)
    if f.ethertype == eth.ETHERTYPE_IPV6:
        return _ipv6(f, cfg)
    return []  # Rule E2: other EtherTypes are ignored


# ---------------------------------------------------------------- ARP

def _arp(f: eth.EthFrame, cfg: Config) -> List[bytes]:
    # Rule A1 (RFC 826): the frame goes to broadcast or to us.
    if f.dst not in (eth.BROADCAST, cfg.mac):
        return []
    p = arp.ArpPacket.parse(f.payload)  # A2: htype/ptype/hlen/plen must be Ethernet/IPv4
    # A3: only requests for OUR address are answered (never unsolicited, never replies).
    if p.oper != arp.OP_REQUEST or p.tpa != cfg.ipv4:
        return []
    # A4: sender hardware address must be unicast (a reply to a group address is meaningless).
    if eth.is_multicast(p.sha):
        return []
    # A5 (RFC 826 "Packet Reception"): swap sender/target, put our MAC/IP as sender, oper=2.
    rep = arp.ArpPacket(arp.OP_REPLY, cfg.mac, cfg.ipv4, p.sha, p.spa)
    return [eth.EthFrame(p.sha, cfg.mac, eth.ETHERTYPE_ARP, rep.build()).build()]


# ---------------------------------------------------------------- IPv4

def _ipv4(f: eth.EthFrame, cfg: Config) -> List[bytes]:
    # Rule I0: unicast to our MAC only (no answer to broadcast/multicast frames).
    if f.dst != cfg.mac:
        return []
    # I1 (RFC 791): version 4, IHL >= 5, total length consistent, header checksum ok.
    p = ipv4.IPv4Packet.parse(f.payload)
    # I2: options (IHL != 5) are not handled -> drop.
    if p.options:
        return []
    # I3 (RFC 791 section 3.2): fragments (MF or offset != 0) are not reassembled -> drop.
    if p.is_fragment:
        return []
    # I4 (RFC 791 / RFC 1812 section 5.3.1): TTL 0 must not be accepted.
    if p.ttl == 0:
        return []
    # I5: destination is exactly our unicast address.
    if p.dst != cfg.ipv4:
        return []
    # I6 (RFC 1122 section 3.2.1.3): never answer a source that is 0.0.0.0, multicast or broadcast.
    a = ipaddress.IPv4Address(p.src)
    if a.is_unspecified or a.is_multicast or p.src == b"\xff\xff\xff\xff":
        return []
    if p.proto == ipv4.PROTO_ICMP:
        return _icmp4(f, p, cfg)
    if p.proto == ipv4.PROTO_UDP:
        return _udp(f, p.src, p.dst, p.payload, cfg, v6=False)
    return []


def _icmp4(f, p, cfg) -> List[bytes]:
    e = icmp.IcmpEcho.parse(p.payload)  # J0: checksum verified
    # J1 (RFC 792): only Echo Request, code 0, is answered.
    if e.type != icmp.TYPE_ECHO_REQUEST or e.code != 0:
        return []
    # J2 (RFC 792): reply = type 0, same identifier, sequence and data; checksum recomputed.
    rep = icmp.IcmpEcho(icmp.TYPE_ECHO_REPLY, e.ident, e.seq, e.data).build()
    ip = ipv4.IPv4Packet(cfg.ipv4, p.src, ipv4.PROTO_ICMP, rep, ttl=REPLY_TTL, ident=0, df=True)
    return [eth.EthFrame(f.src, cfg.mac, eth.ETHERTYPE_IPV4, ip.build()).build()]


# ---------------------------------------------------------------- UDP / NTP (both families)

def _udp(f, src_ip: bytes, dst_ip: bytes, payload: bytes, cfg: Config, v6: bool) -> List[bytes]:
    # U1 (RFC 768, RFC 8200 section 8.1): length consistent; checksum correct, or 0 accepted
    # in IPv4 only; mandatory in IPv6. Violations raise ParseError.
    d = udp.UdpDatagram.parse(src_ip, dst_ip, payload)
    # U2: only destination port 123 is served.
    if d.dport != NTP_PORT:
        return []
    # N1 (RFC 5905 section 7.3): at least the 48-byte header (extension fields tolerated).
    req = ntp.NtpPacket.parse(d.payload)
    # N2 (RFC 5905 section 9.1 / 7.3): only client mode (3) requests are answered; modes 1, 2,
    # 4, 5, 6, 7 are ignored (symmetric, broadcast, control... are out of scope).
    if req.mode != ntp.MODE_CLIENT:
        return []
    # N3: versions 3 and 4 accepted (RFC 5905 section 7.3 compat with RFC 1305); others dropped.
    if req.vn not in (3, 4):
        return []
    sync = cfg.synchronised
    rep = ntp.NtpPacket(
        # N4 (RFC 5905 section 7.3 "li"): LI 3 (alarm) iff not synchronised, else 0.
        li=ntp.LI_NONE if sync else ntp.LI_UNSYNC,
        # N5: version echoed, capped at 4 (our own).
        vn=min(req.vn, 4),
        mode=ntp.MODE_SERVER,
        # N6: stratum from config when synchronised, 16 (unsynchronised) otherwise; never 0.
        stratum=cfg.stratum if sync else 16,
        # N7 (RFC 5905 fast_xmit): poll copied from the request.
        poll=req.poll,
        precision=cfg.precision,
        root_delay=cfg.root_delay,
        root_dispersion=cfg.root_dispersion,
        # N8: Reference ID per address family of the request.
        ref_id=(cfg.ref_id_v6 or cfg.ref_id) if v6 else cfg.ref_id,
        ref_ts=cfg.ref_ts,
        # N9 (RFC 5905 section 8): origin = client's transmit timestamp, copied verbatim.
        orig_ts=req.xmit_ts,
        recv_ts=cfg.recv_ts,   # T2
        xmit_ts=cfg.xmit_ts,   # T3
        # N10: exactly 48 bytes, no extensions echoed (also bounds the amplification factor).
    )
    # U3: ports swapped, checksum recomputed (v6 always, v4 always non-zero).
    out = udp.UdpDatagram(d.dport, d.sport, rep.build()).build(dst_ip, src_ip)
    # Reply goes back to the sender's MAC, from our MAC, addresses swapped.
    if v6:
        pkt = ipv6.IPv6Packet(dst_ip, src_ip, ipv6.NH_UDP, out, hop_limit=REPLY_HOP_LIMIT).build()
        et = eth.ETHERTYPE_IPV6
    else:
        pkt = ipv4.IPv4Packet(dst_ip, src_ip, ipv4.PROTO_UDP, out, ttl=REPLY_TTL, ident=0, df=True).build()
        et = eth.ETHERTYPE_IPV4
    return [eth.EthFrame(f.src, cfg.mac, et, pkt).build()]


# ---------------------------------------------------------------- IPv6

def _ipv6(f: eth.EthFrame, cfg: Config) -> List[bytes]:
    # G1 (RFC 8200 section 3): version 6, payload length consistent with the frame.
    p = ipv6.IPv6Packet.parse(f.payload)
    # G2 (RFC 8200 section 4): extension headers are not handled; only UDP/ICMPv6 directly.
    if p.next_header in ipv6.EXTENSION_HEADERS or p.next_header not in (ipv6.NH_UDP, ipv6.NH_ICMPV6):
        return []
    # G3 (RFC 8200 section 3): hop limit 0 is dropped (NDP needs exactly 255, checked later).
    if p.hop_limit == 0:
        return []
    # G4 (RFC 4291 section 2.7): a multicast source address is illegal.
    if p.src[0] == 0xFF:
        return []
    ours = cfg.v6_addresses()
    is_ns_dest = any(p.dst == ipv6.solicited_node(a) for a in ours)
    if p.dst in ours:
        # G5: unicast to us must arrive on our MAC.
        if f.dst != cfg.mac:
            return []
    elif is_ns_dest:
        # G6 (RFC 2464 section 7): solicited-node multicast must use 33:33:ff:xx:xx:xx.
        if f.dst != ipv6.multicast_mac(p.dst):
            return []
    else:
        return []  # G7: not for us
    if p.next_header == ipv6.NH_UDP:
        if p.dst not in ours:
            return []  # G8: NTP only over unicast
        return _udp(f, p.src, p.dst, p.payload, cfg, v6=True)
    return _icmp6(f, p, cfg, is_ns_dest and p.dst not in ours)


def _icmp6(f, p, cfg, to_solicited_node: bool) -> List[bytes]:
    t = p.payload[0] if p.payload else None
    if t == icmpv6.T_ECHO_REQUEST:
        # K1 (RFC 4443 section 4.1): echo to a unicast address of ours (not multicast).
        if to_solicited_node:
            return []
        e = icmpv6.Icmpv6Echo.parse(p.src, p.dst, p.payload)  # checksum verified
        # K2 (RFC 4443 section 4.2): reply type 129, same id/seq/data, src = request's dst.
        body = icmpv6.Icmpv6Echo(icmpv6.T_ECHO_REPLY, e.ident, e.seq, e.data).build(p.dst, p.src)
        return [_ip6_frame(cfg.mac, f.src, p.dst, p.src, body, REPLY_HOP_LIMIT)]
    if t == icmpv6.T_NS:
        # M1 (RFC 4861 section 7.1.1): hop limit must be 255 (proves on-link origin).
        if p.hop_limit != 255:
            return []
        ns = icmpv6.NeighborSolicitation.parse(p.src, p.dst, p.payload)  # code 0, len>=24, cksum
        # M2: target not multicast, and one of OUR addresses.
        if ns.target[0] == 0xFF or ns.target not in cfg.v6_addresses():
            return []
        # M3: destination is the target itself (NUD) or its solicited-node multicast address.
        if p.dst != ns.target and p.dst != ipv6.solicited_node(ns.target):
            return []
        slla = [v for (ot, v) in ns.options if ot == icmpv6.OPT_SLLA]
        unspecified = p.src == bytes(16)
        if unspecified:
            # M4 (RFC 4861 section 7.1.1): DAD probe: dst must be solicited-node multicast and no
            # source link-layer option is allowed.
            if p.dst == ns.target or slla:
                return []
            # M5 (RFC 4861 section 7.2.4): answer to all-nodes ff02::1, S=0, MAC 33:33:00:00:00:01.
            na = icmpv6.NeighborAdvertisement(ns.target, False, False, True,
                                              [(icmpv6.OPT_TLLA, cfg.mac)])
            body = na.build(ns.target, ALL_NODES)
            return [_ip6_frame(cfg.mac, ipv6.multicast_mac(ALL_NODES), ns.target, ALL_NODES, body, 255)]
        if len(slla) > 1 or (slla and len(slla[0]) != 6):
            return []
        # M6 (RFC 4861 section 7.2.4): solicited NA: R=0 (host), S=1, O=1 (we are the owner),
        # target link-layer address option = our MAC, hop limit 255, src = target,
        # unicast to the solicitation's source at the link address in SLLA (else Ethernet src).
        na = icmpv6.NeighborAdvertisement(ns.target, False, True, True, [(icmpv6.OPT_TLLA, cfg.mac)])
        body = na.build(ns.target, p.src)
        return [_ip6_frame(cfg.mac, slla[0] if slla else f.src, ns.target, p.src, body, 255)]
    # M7: Router Advertisements are consumed by the SLAAC logic (T3.5), nothing is sent;
    # every other ICMPv6 type is ignored.
    return []


def _ip6_frame(src_mac, dst_mac, src, dst, icmp_body, hop_limit) -> bytes:
    pkt = ipv6.IPv6Packet(src, dst, ipv6.NH_ICMPV6, icmp_body, hop_limit=hop_limit).build()
    return eth.EthFrame(dst_mac, src_mac, eth.ETHERTYPE_IPV6, pkt).build()
