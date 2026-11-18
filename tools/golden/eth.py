"""Ethernet II frames, without FCS (the convention of rtl/src/stream.rs above the MAC)."""
import struct
from dataclasses import dataclass

from .errors import ParseError

ETH_HEADER_LEN = 14
MIN_FRAME_NO_FCS = 60  # 64 on the wire minus the 4 FCS bytes
ETHERTYPE_IPV4 = 0x0800
ETHERTYPE_ARP = 0x0806
ETHERTYPE_IPV6 = 0x86DD
BROADCAST = b"\xff" * 6


def mac(text: str) -> bytes:
    """'02:00:00:00:00:01' -> 6 bytes."""
    parts = text.split(":")
    if len(parts) != 6:
        raise ValueError("bad MAC: " + text)
    return bytes(int(p, 16) for p in parts)


def mac_str(b: bytes) -> str:
    return ":".join("%02x" % x for x in b)


def is_multicast(m: bytes) -> bool:
    """Group bit (IEEE 802.3): least significant bit of the first octet; includes broadcast."""
    return bool(m[0] & 1)


@dataclass
class EthFrame:
    dst: bytes
    src: bytes
    ethertype: int
    payload: bytes

    def build(self, pad: bool = False) -> bytes:
        """Serialise. `pad=True` pads with zeros to 60 bytes (what a real wire frame carries)."""
        if len(self.dst) != 6 or len(self.src) != 6:
            raise ValueError("MAC must be 6 bytes")
        data = self.dst + self.src + struct.pack(">H", self.ethertype) + self.payload
        if pad and len(data) < MIN_FRAME_NO_FCS:
            data += b"\x00" * (MIN_FRAME_NO_FCS - len(data))
        return data

    @staticmethod
    def parse(data: bytes) -> "EthFrame":
        """Parse; the payload keeps any padding (upper layers trim using their own length)."""
        if len(data) < ETH_HEADER_LEN:
            raise ParseError("Ethernet frame shorter than 14 bytes")
        (et,) = struct.unpack(">H", data[12:14])
        return EthFrame(data[0:6], data[6:12], et, data[14:])


def pad_to_min(frame: bytes) -> bytes:
    return frame + b"\x00" * max(0, MIN_FRAME_NO_FCS - len(frame))
