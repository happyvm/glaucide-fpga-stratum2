"""IPv4 (RFC 791)."""
import struct
from dataclasses import dataclass

from . import csum
from .errors import ParseError

PROTO_ICMP = 1
PROTO_UDP = 17


@dataclass
class IPv4Packet:
    src: bytes  # 4 bytes
    dst: bytes  # 4 bytes
    proto: int
    payload: bytes
    ttl: int = 64
    ident: int = 0
    df: bool = True
    mf: bool = False
    frag_offset: int = 0  # in 8-byte units
    tos: int = 0
    options: bytes = b""  # length must be a multiple of 4

    def build(self) -> bytes:
        if len(self.options) % 4:
            raise ValueError("options must be a multiple of 4 bytes")
        ihl = 5 + len(self.options) // 4
        total = ihl * 4 + len(self.payload)
        if total > 0xFFFF:
            raise ValueError("packet too long")
        flags = (0x4000 if self.df else 0) | (0x2000 if self.mf else 0) | (self.frag_offset & 0x1FFF)
        hdr = struct.pack(">BBHHHBBH", 0x40 | ihl, self.tos, total, self.ident, flags,
                          self.ttl, self.proto, 0) + self.src + self.dst + self.options
        c = csum.ipv4_header_checksum(hdr)
        hdr = hdr[:10] + struct.pack(">H", c) + hdr[12:]
        return hdr + self.payload

    @staticmethod
    def parse(data: bytes) -> "IPv4Packet":
        """Strict parse. Bytes after Total Length (Ethernet padding) are dropped."""
        if len(data) < 20:
            raise ParseError("IPv4 header truncated")
        vihl, tos, total, ident, flags, ttl, proto, _c = struct.unpack(">BBHHHBBH", data[:12])
        if vihl >> 4 != 4:
            raise ParseError("IP version is not 4")
        ihl = vihl & 0xF
        if ihl < 5:
            raise ParseError("IHL < 5")
        if len(data) < ihl * 4:
            raise ParseError("IPv4 options truncated")
        if total < ihl * 4 or total > len(data):
            raise ParseError("IPv4 total length inconsistent with frame")
        if not csum.verify(data[: ihl * 4]):
            raise ParseError("bad IPv4 header checksum")
        return IPv4Packet(src=data[12:16], dst=data[16:20], proto=proto,
                          payload=data[ihl * 4: total], ttl=ttl, ident=ident,
                          df=bool(flags & 0x4000), mf=bool(flags & 0x2000),
                          frag_offset=flags & 0x1FFF, tos=tos, options=data[20: ihl * 4])

    @property
    def is_fragment(self) -> bool:
        return self.mf or self.frag_offset != 0
