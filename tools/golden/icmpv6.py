"""ICMPv6: echo (RFC 4443), NDP NS/NA/RA (RFC 4861). Checksum uses the IPv6 pseudo-header."""
import struct
from dataclasses import dataclass, field
from typing import List, Tuple

from . import csum
from .errors import ParseError
from .ipv6 import NH_ICMPV6

T_ECHO_REQUEST = 128
T_ECHO_REPLY = 129
T_RA = 134
T_NS = 135
T_NA = 136

OPT_SLLA = 1   # source link-layer address
OPT_TLLA = 2   # target link-layer address
OPT_PREFIX = 3  # prefix information

NA_R = 0x80000000
NA_S = 0x40000000
NA_O = 0x20000000


def _finish(src: bytes, dst: bytes, body: bytes) -> bytes:
    """Insert the checksum at bytes 2-3 of `body` (which has them zero)."""
    c = csum.upper_layer_checksum(csum.pseudo_header_v6(src, dst, NH_ICMPV6, len(body)), body)
    return body[:2] + struct.pack(">H", c) + body[4:]


def _check(src: bytes, dst: bytes, data: bytes) -> None:
    if len(data) < 8:
        raise ParseError("ICMPv6 shorter than 8 bytes")
    if not csum.upper_layer_ok(csum.pseudo_header_v6(src, dst, NH_ICMPV6, len(data)), data):
        raise ParseError("bad ICMPv6 checksum")


def build_options(opts: List[Tuple[int, bytes]]) -> bytes:
    """Options are TLV with length in units of 8 bytes, including type and length (RFC 4861 4.6)."""
    out = b""
    for t, v in opts:
        if (len(v) + 2) % 8:
            raise ValueError("option value must make a multiple of 8 bytes with its header")
        out += bytes([t, (len(v) + 2) // 8]) + v
    return out


def parse_options(data: bytes) -> List[Tuple[int, bytes]]:
    out = []
    i = 0
    while i < len(data):
        if len(data) - i < 2:
            raise ParseError("truncated NDP option")
        t, ln = data[i], data[i + 1]
        if ln == 0:
            raise ParseError("NDP option of length 0 (RFC 4861 7.1.1)")
        if i + ln * 8 > len(data):
            raise ParseError("NDP option overruns message")
        out.append((t, data[i + 2: i + ln * 8]))
        i += ln * 8
    return out


@dataclass
class Icmpv6Echo:
    type: int
    ident: int
    seq: int
    data: bytes = b""

    def build(self, src: bytes, dst: bytes) -> bytes:
        return _finish(src, dst, struct.pack(">BBHHH", self.type, 0, 0, self.ident, self.seq) + self.data)

    @staticmethod
    def parse(src: bytes, dst: bytes, data: bytes) -> "Icmpv6Echo":
        _check(src, dst, data)
        t, code, _c, ident, seq = struct.unpack(">BBHHH", data[:8])
        if t not in (T_ECHO_REQUEST, T_ECHO_REPLY) or code != 0:
            raise ParseError("not an ICMPv6 echo")
        return Icmpv6Echo(t, ident, seq, data[8:])


@dataclass
class NeighborSolicitation:
    target: bytes
    options: List[Tuple[int, bytes]] = field(default_factory=list)

    def build(self, src: bytes, dst: bytes) -> bytes:
        body = struct.pack(">BBHI", T_NS, 0, 0, 0) + self.target + build_options(self.options)
        return _finish(src, dst, body)

    @staticmethod
    def parse(src: bytes, dst: bytes, data: bytes) -> "NeighborSolicitation":
        _check(src, dst, data)
        if data[0] != T_NS or data[1] != 0:
            raise ParseError("not a neighbor solicitation")
        if len(data) < 24:
            raise ParseError("NS shorter than 24 bytes")
        return NeighborSolicitation(data[8:24], parse_options(data[24:]))


@dataclass
class NeighborAdvertisement:
    target: bytes
    router: bool = False
    solicited: bool = True
    override: bool = True
    options: List[Tuple[int, bytes]] = field(default_factory=list)

    def build(self, src: bytes, dst: bytes) -> bytes:
        flags = (NA_R if self.router else 0) | (NA_S if self.solicited else 0) | (NA_O if self.override else 0)
        body = struct.pack(">BBHI", T_NA, 0, 0, flags) + self.target + build_options(self.options)
        return _finish(src, dst, body)

    @staticmethod
    def parse(src: bytes, dst: bytes, data: bytes) -> "NeighborAdvertisement":
        _check(src, dst, data)
        if data[0] != T_NA or data[1] != 0:
            raise ParseError("not a neighbor advertisement")
        if len(data) < 24:
            raise ParseError("NA shorter than 24 bytes")
        (flags,) = struct.unpack(">I", data[4:8])
        return NeighborAdvertisement(data[8:24], bool(flags & NA_R), bool(flags & NA_S),
                                     bool(flags & NA_O), parse_options(data[24:]))


@dataclass
class RouterAdvertisement:
    cur_hop_limit: int
    managed: bool
    other: bool
    router_lifetime: int
    reachable_time: int
    retrans_timer: int
    options: List[Tuple[int, bytes]] = field(default_factory=list)

    def build(self, src: bytes, dst: bytes) -> bytes:
        flags = (0x80 if self.managed else 0) | (0x40 if self.other else 0)
        body = struct.pack(">BBHBBHII", T_RA, 0, 0, self.cur_hop_limit, flags, self.router_lifetime,
                           self.reachable_time, self.retrans_timer) + build_options(self.options)
        return _finish(src, dst, body)

    @staticmethod
    def parse(src: bytes, dst: bytes, data: bytes) -> "RouterAdvertisement":
        _check(src, dst, data)
        if data[0] != T_RA or data[1] != 0:
            raise ParseError("not a router advertisement")
        if len(data) < 16:
            raise ParseError("RA shorter than 16 bytes")
        _t, _c, _ck, chl, flags, life, reach, retr = struct.unpack(">BBHBBHII", data[:16])
        return RouterAdvertisement(chl, bool(flags & 0x80), bool(flags & 0x40), life, reach, retr,
                                   parse_options(data[16:]))


def prefix_option(prefix: bytes, plen: int, valid: int, preferred: int, on_link=True, autonomous=True):
    """Prefix Information option (RFC 4861 4.6.2) as an (type, value) tuple."""
    flags = (0x80 if on_link else 0) | (0x40 if autonomous else 0)
    return (OPT_PREFIX, struct.pack(">BBIII", plen, flags, valid, preferred, 0) + prefix)


def parse_prefix_option(value: bytes):
    """-> (prefix_len, on_link, autonomous, valid, preferred, prefix)"""
    if len(value) != 30:
        raise ParseError("bad prefix information option")
    plen, flags, valid, pref, _r = struct.unpack(">BBIII", value[:14])
    return plen, bool(flags & 0x80), bool(flags & 0x40), valid, pref, value[14:30]
