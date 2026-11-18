"""IPv6 fixed header (RFC 8200)."""
import struct
from dataclasses import dataclass

from .errors import ParseError

NH_UDP = 17
NH_ICMPV6 = 58
# Extension headers (RFC 8200 section 4, RFC 4302/4303): hop-by-hop, routing, fragment, ESP, AH, dest options
EXTENSION_HEADERS = (0, 43, 44, 50, 51, 60)


def addr(text: str) -> bytes:
    import ipaddress
    return ipaddress.IPv6Address(text).packed


def solicited_node(a: bytes) -> bytes:
    """ff02::1:ffXX:XXXX built from the low 24 bits of the unicast address (RFC 4291 section 2.7.1)."""
    return bytes.fromhex("ff0200000000000000000001ff") + a[13:16]


def multicast_mac(a: bytes) -> bytes:
    """33:33 + low 32 bits of an IPv6 multicast address (RFC 2464 section 7)."""
    return b"\x33\x33" + a[12:16]


@dataclass
class IPv6Packet:
    src: bytes  # 16 bytes
    dst: bytes  # 16 bytes
    next_header: int
    payload: bytes
    hop_limit: int = 64
    traffic_class: int = 0
    flow_label: int = 0

    def build(self) -> bytes:
        if len(self.payload) > 0xFFFF:
            raise ValueError("payload too long")
        first = (6 << 28) | (self.traffic_class << 20) | self.flow_label
        return struct.pack(">IHBB", first, len(self.payload), self.next_header, self.hop_limit) \
            + self.src + self.dst + self.payload

    @staticmethod
    def parse(data: bytes) -> "IPv6Packet":
        """Bytes after Payload Length (Ethernet padding) are dropped. Extension headers are
        not walked: next_header is returned as is and the caller decides."""
        if len(data) < 40:
            raise ParseError("IPv6 header truncated")
        first, plen, nh, hl = struct.unpack(">IHBB", data[:8])
        if first >> 28 != 6:
            raise ParseError("IP version is not 6")
        if 40 + plen > len(data):
            raise ParseError("IPv6 payload length exceeds frame")
        return IPv6Packet(data[8:24], data[24:40], nh, data[40: 40 + plen], hl,
                          (first >> 20) & 0xFF, first & 0xFFFFF)
