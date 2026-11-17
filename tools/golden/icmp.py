"""ICMPv4 echo (RFC 792)."""
import struct
from dataclasses import dataclass

from . import csum
from .errors import ParseError

TYPE_ECHO_REPLY = 0
TYPE_ECHO_REQUEST = 8


@dataclass
class IcmpEcho:
    type: int  # 8 request, 0 reply
    ident: int
    seq: int
    data: bytes = b""
    code: int = 0

    def build(self) -> bytes:
        body = struct.pack(">BBHHH", self.type, self.code, 0, self.ident, self.seq) + self.data
        c = csum.internet_checksum(body)
        return body[:2] + struct.pack(">H", c) + body[4:]

    @staticmethod
    def parse(data: bytes) -> "IcmpEcho":
        if len(data) < 8:
            raise ParseError("ICMP shorter than 8 bytes")
        t, code, _c, ident, seq = struct.unpack(">BBHHH", data[:8])
        if t not in (TYPE_ECHO_REPLY, TYPE_ECHO_REQUEST):
            raise ParseError("ICMP type %d is not echo" % t)
        if not csum.verify(data):
            raise ParseError("bad ICMP checksum")
        return IcmpEcho(t, ident, seq, data[8:], code)
