"""UDP (RFC 768); checksum optional (0) in IPv4, mandatory in IPv6 (RFC 8200 section 8.1)."""
import struct
from dataclasses import dataclass

from . import csum
from .errors import ParseError
from .ipv4 import PROTO_UDP
from .ipv6 import NH_UDP


def _pseudo(src: bytes, dst: bytes, length: int) -> bytes:
    if len(src) == 4:
        return csum.pseudo_header_v4(src, dst, PROTO_UDP, length)
    return csum.pseudo_header_v6(src, dst, NH_UDP, length)


@dataclass
class UdpDatagram:
    sport: int
    dport: int
    payload: bytes

    def build(self, src: bytes, dst: bytes, zero_checksum: bool = False) -> bytes:
        """`src`/`dst` are 4 or 16 byte addresses. A computed 0 is sent as 0xFFFF (RFC 768).
        `zero_checksum` (IPv4 only) sends 0 = no checksum."""
        length = 8 + len(self.payload)
        seg = struct.pack(">HHHH", self.sport, self.dport, length, 0) + self.payload
        if zero_checksum:
            if len(src) != 4:
                raise ValueError("zero checksum is not allowed in IPv6")
            return seg
        c = csum.upper_layer_checksum(_pseudo(src, dst, length), seg) or 0xFFFF
        return seg[:6] + struct.pack(">H", c) + seg[8:]

    @staticmethod
    def parse(src: bytes, dst: bytes, data: bytes) -> "UdpDatagram":
        """Bytes after the UDP length field are dropped."""
        if len(data) < 8:
            raise ParseError("UDP header truncated")
        sport, dport, length, c = struct.unpack(">HHHH", data[:8])
        if length < 8 or length > len(data):
            raise ParseError("UDP length inconsistent")
        seg = data[:length]
        if c == 0:
            if len(src) == 16:
                raise ParseError("UDP checksum 0 is illegal over IPv6")
        elif not csum.upper_layer_ok(_pseudo(src, dst, length), seg):
            raise ParseError("bad UDP checksum")
        return UdpDatagram(sport, dport, seg[8:])
