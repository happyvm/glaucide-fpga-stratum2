"""NTPv4 packet (RFC 5905 section 7.3) and time conversions."""
import hashlib
import ipaddress
import struct
from dataclasses import dataclass
from fractions import Fraction

from .errors import ParseError

NTP_LEN = 48
NTP_UNIX_OFFSET = 2208988800  # seconds from 1900-01-01 to 1970-01-01
MODE_CLIENT = 3
MODE_SERVER = 4
LI_NONE = 0
LI_UNSYNC = 3


def seconds_to_ts(sec) -> int:
    """Seconds (int, Fraction or float) -> 64-bit 32.32 timestamp (truncated, mod 2**64)."""
    return int(Fraction(sec) * (1 << 32)) & 0xFFFFFFFFFFFFFFFF


def ts_to_seconds(ts: int) -> float:
    return ts / float(1 << 32)


def ts_to_fraction(ts: int) -> Fraction:
    return Fraction(ts, 1 << 32)


def unix_to_ts(unix_sec) -> int:
    return seconds_to_ts(Fraction(unix_sec) + NTP_UNIX_OFFSET)


def ts_to_unix(ts: int) -> Fraction:
    return ts_to_fraction(ts) - NTP_UNIX_OFFSET


def seconds_to_short(sec) -> int:
    """Seconds -> 32-bit 16.16 (root delay / dispersion)."""
    return int(Fraction(sec) * (1 << 16)) & 0xFFFFFFFF


def short_to_seconds(v: int) -> float:
    return v / float(1 << 16)


def refid_ipv4(address: str) -> bytes:
    """Reference ID of a stratum>=2 server synchronised to an IPv4 source: its address."""
    return ipaddress.IPv4Address(address).packed


def refid_ipv6(address: str) -> bytes:
    """Same for IPv6 (RFC 5905 section 7.3): first 4 bytes of the MD5 of the address."""
    return hashlib.md5(ipaddress.IPv6Address(address).packed).digest()[:4]


@dataclass
class NtpPacket:
    li: int = 0
    vn: int = 4
    mode: int = 3
    stratum: int = 0
    poll: int = 0
    precision: int = 0  # signed log2 seconds
    root_delay: int = 0  # raw 16.16
    root_dispersion: int = 0  # raw 16.16
    ref_id: bytes = b"\x00\x00\x00\x00"
    ref_ts: int = 0
    orig_ts: int = 0
    recv_ts: int = 0
    xmit_ts: int = 0
    extensions: bytes = b""  # opaque bytes after the 48-byte header (kept, never interpreted)

    def build(self) -> bytes:
        for name, v, lim in (("li", self.li, 3), ("vn", self.vn, 7), ("mode", self.mode, 7)):
            if not 0 <= v <= lim:
                raise ValueError(name + " out of range")
        if len(self.ref_id) != 4:
            raise ValueError("ref_id must be 4 bytes")
        b0 = (self.li << 6) | (self.vn << 3) | self.mode
        return struct.pack(">BBBbII4sQQQQ", b0, self.stratum, self.poll & 0xFF, self.precision,
                           self.root_delay, self.root_dispersion, self.ref_id, self.ref_ts,
                           self.orig_ts, self.recv_ts, self.xmit_ts) + self.extensions

    @staticmethod
    def parse(data: bytes) -> "NtpPacket":
        """Accepts >= 48 bytes (the header length is not forced: extension fields are kept)."""
        if len(data) < NTP_LEN:
            raise ParseError("NTP packet shorter than 48 bytes")
        b0, st, poll, prec, rd, rdisp, rid, rts, ots, vts, xts = struct.unpack(">BBBbII4sQQQQ", data[:NTP_LEN])
        return NtpPacket(b0 >> 6, (b0 >> 3) & 7, b0 & 7, st, poll, prec, rd, rdisp, rid,
                         rts, ots, vts, xts, data[NTP_LEN:])
