"""ARP for Ethernet/IPv4 (RFC 826)."""
import struct
from dataclasses import dataclass

from .errors import ParseError

ARP_LEN = 28
OP_REQUEST = 1
OP_REPLY = 2


@dataclass
class ArpPacket:
    oper: int
    sha: bytes  # sender hardware address
    spa: bytes  # sender protocol address (4 bytes)
    tha: bytes  # target hardware address
    tpa: bytes  # target protocol address (4 bytes)

    def build(self) -> bytes:
        # htype=1 (Ethernet), ptype=0x0800, hlen=6, plen=4 (RFC 826)
        return struct.pack(">HHBBH", 1, 0x0800, 6, 4, self.oper) + self.sha + self.spa + self.tha + self.tpa

    @staticmethod
    def parse(data: bytes) -> "ArpPacket":
        """Trailing bytes (Ethernet padding) are ignored."""
        if len(data) < ARP_LEN:
            raise ParseError("ARP shorter than 28 bytes")
        htype, ptype, hlen, plen, oper = struct.unpack(">HHBBH", data[:8])
        if (htype, ptype, hlen, plen) != (1, 0x0800, 6, 4):
            raise ParseError("ARP is not Ethernet/IPv4")
        if oper not in (OP_REQUEST, OP_REPLY):
            raise ParseError("ARP operation %d unsupported" % oper)
        return ArpPacket(oper, data[8:14], data[14:18], data[18:24], data[24:28])
