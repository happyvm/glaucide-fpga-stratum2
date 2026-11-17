"""Checksums: RFC 1071 Internet checksum, IPv4/IPv6 pseudo-headers, Ethernet FCS."""
import struct
import zlib


def ones_complement_sum(data: bytes, initial: int = 0) -> int:
    """16-bit one's complement sum of big-endian words (RFC 1071 section 1).

    An odd trailing byte is padded with a zero byte on the right.
    """
    if len(data) & 1:
        data = data + b"\x00"
    total = initial
    for (word,) in struct.iter_unpack(">H", data):
        total += word
    while total >> 16:
        total = (total & 0xFFFF) + (total >> 16)
    return total


def internet_checksum(data: bytes) -> int:
    """RFC 1071 checksum: one's complement of the one's complement sum."""
    return (~ones_complement_sum(data)) & 0xFFFF


def verify(data: bytes) -> bool:
    """True when `data` (checksum field included) sums to 0xFFFF (RFC 1071 section 1)."""
    return ones_complement_sum(data) == 0xFFFF


def ipv4_header_checksum(header: bytes) -> int:
    """RFC 791 header checksum; bytes 10-11 (the field itself) are treated as zero."""
    return internet_checksum(header[:10] + b"\x00\x00" + header[12:])


def pseudo_header_v4(src: bytes, dst: bytes, proto: int, length: int) -> bytes:
    """RFC 768 pseudo-header: src, dst, zero, protocol, upper-layer length."""
    return src + dst + struct.pack(">BBH", 0, proto, length)


def pseudo_header_v6(src: bytes, dst: bytes, next_header: int, length: int) -> bytes:
    """RFC 8200 section 8.1 pseudo-header: src, dst, 32-bit length, 3 zero bytes, next header."""
    return src + dst + struct.pack(">I", length) + b"\x00\x00\x00" + bytes([next_header])


def upper_layer_checksum(pseudo: bytes, segment: bytes) -> int:
    """Checksum of pseudo-header + segment (segment checksum field must be zero)."""
    return internet_checksum(pseudo + segment)


def upper_layer_ok(pseudo: bytes, segment: bytes) -> bool:
    """Verify a received segment (checksum field included) against its pseudo-header."""
    return verify(pseudo + segment)


def eth_crc32(frame: bytes) -> int:
    """IEEE 802.3 CRC-32 (same polynomial/reflection/xorout as zlib)."""
    return zlib.crc32(frame) & 0xFFFFFFFF


def eth_fcs(frame: bytes) -> bytes:
    """The 4 FCS bytes in transmission order (least significant byte first)."""
    return struct.pack("<I", eth_crc32(frame))


def append_fcs(frame: bytes) -> bytes:
    return frame + eth_fcs(frame)


def check_fcs(frame_with_fcs: bytes) -> bool:
    """True if the last 4 bytes are the correct FCS of the rest."""
    if len(frame_with_fcs) < 5:
        return False
    return eth_fcs(frame_with_fcs[:-4]) == frame_with_fcs[-4:]
