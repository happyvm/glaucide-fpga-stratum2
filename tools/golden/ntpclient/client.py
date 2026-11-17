"""Reference model of rtl/src/ntp_client.rs (task T4.2), standard library only.

The executable form of the module header of ntp_client.rs:

* `build_request(poll, precision, t1)`: the 48-byte mode-3 request the RTL
  sends (LI 0, VN 4, mode 3, stratum 0, poll, precision, everything else 0
  but the Transmit Timestamp = T1);
* `make_sample(slot, req, reply, t4, ...)`: what the RTL does with one
  reply, in exactly its integer arithmetic (a `Sample`, or a `Reject` with
  the reason and whether the poll is counted as a miss);
* `next_hop4`, `next_hop6`, `src6`: the next-hop and source address rules.

Times are integers in 32.32 seconds (NTP timestamps, modulo 2^64). Signed
quantities (offset) are returned as 64-bit two's complement integers, as on
the RTL port.

Arithmetic (RFC 5905 section 8, T1 origin, T2 receive, T3 transmit, T4
arrival). `w(x)` is `x mod 2^64` read as a signed 64-bit integer:

* offset = floor((w(T2 - T1) + w(T3 - T4)) / 2): the 65-bit signed sum is
  shifted right by one (arithmetic shift, rounding toward minus infinity);
  the result always fits in 64 bits;
* delay = w(T4 - T1) - w(T3 - T2), an exact 65-bit signed difference; 0 when
  negative, else it fits in 64 unsigned bits;
* disp = prec_floor(peer precision) + prec_floor(our precision)
  + ceil(PHI_Q32 * iv / 2^32), with iv = w(T4 - T1) when it is >= 0, else 0,
  and prec_floor(p) = 2^p in 32.32 for p in -32..=-1, 2^32 (1 s) for p >= 0,
  1 (2^-32 s) for p < -32 (the convention of sync_state.rs). The sum is below
  2^34 + 2^47, it never wraps.
"""
import random
import struct
from dataclasses import dataclass
from typing import Optional, Union

NTP_PORT = 123
NTP_LEN = 48
PHI_Q32 = 64425                 # 15 ppm in 2^-32 s per second (rtl/src/ntp_types.rs)
M32 = 0xFFFFFFFF
M64 = (1 << 64) - 1
PEERS = 4

# Timing constants of the RTL scheduler (documented in ntp_client.rs).
IBURST_COUNT = 4                # requests 2 s apart after start-up / enable
IBURST_GAP_S = 2
DEADLINE_S = 2                  # miss when now_sec - sent_sec >= 2 (1 to 2 s after T1)
POLL_MIN = 4
POLL_MAX = 17
RES_TRIES = 8                   # MAC lookups per poll before it is abandoned
RETRY_TICKS = 2                 # 2 ticks of 2^-6 s between two lookups (15.6 to 31.25 ms)

# Reject reasons (the RTL does not report them; they name the rule).
R_ERR = "err"                   # the payload ended with err (bad UDP checksum)
R_SHORT = "short"               # fewer than 48 bytes
R_SOURCE = "source"             # family / address / port do not match the slot
R_IDLE = "idle"                 # no request outstanding for the slot (duplicate, late)
R_MODE = "mode"                 # mode != 4
R_VERSION = "version"           # VN not 3 or 4
R_ORIGIN = "origin"             # Origin != the T1 we sent (spoofed or stale)
R_KOD = "kod"                   # stratum 0 (Kiss-o'-Death): miss
R_STRATUM = "stratum"           # stratum >= 16: miss
R_LEAP = "leap"                 # LI = 3 (alarm): miss


@dataclass(frozen=True)
class Sample:
    peer: int
    offset: int                 # signed 32.32, two's complement in 64 bits
    delay: int
    disp: int
    t4: int
    stratum: int
    leap: int
    ref_id: int
    root_delay: int
    root_disp: int


@dataclass(frozen=True)
class Reject:
    reason: str
    # The reply is authentic (it answers the outstanding request) but not
    # usable: the request is closed and the poll counted as a miss at once.
    miss: bool = False


@dataclass(frozen=True)
class Request:
    """The client's state for one slot when a reply arrives."""
    addr: int                   # the slot's address, IPv4 in the low 32 bits
    v6: bool
    t1: int                     # Transmit Timestamp of the outstanding request
    outstanding: bool = True
    precision: int = 0xE8       # our precision field (PRECISION_LOG2), 8-bit two's complement


def s64(x: int) -> int:
    x &= M64
    return x - (1 << 64) if x >> 63 else x


def prec_floor(pr: int) -> int:
    """2^precision in 32.32; `pr` is the signed log2 as 8-bit two's complement."""
    p = pr - 256 if pr & 0x80 else pr
    if p >= 0:
        return 1 << 32
    if p < -32:
        return 1
    return 1 << (32 + p)


def build_request(poll: int, precision: int, t1: int) -> bytes:
    """The 48-byte request: LI 0, VN 4, mode 3, stratum 0, poll, precision,
    root delay / dispersion 0, ref id 0, ref / origin / receive 0, Transmit T1."""
    return struct.pack(">BBBB36xQ", 0x23, 0, poll & 0xFF, precision & 0xFF, t1 & M64)


def arith(t1: int, t2: int, t3: int, t4: int, peer_prec: int, our_prec: int):
    """(offset, delay, disp) in the RTL's integer arithmetic (module docstring)."""
    a = s64(t2 - t1)
    b = s64(t3 - t4)
    offset = ((a + b) >> 1) & M64          # Python >> on ints is arithmetic (floor)
    d1 = s64(t4 - t1)
    d2 = s64(t3 - t2)
    delay = max(d1 - d2, 0)
    iv = d1 if d1 >= 0 else 0
    phi = (PHI_Q32 * iv + M32) >> 32
    disp = prec_floor(peer_prec) + prec_floor(our_prec) + phi
    assert delay <= M64 and disp <= M64
    return offset, delay, disp


def make_sample(slot: int, req: Request, reply: bytes, t4: int, *, src: Optional[int] = None,
                v6: Optional[bool] = None, sport: int = NTP_PORT,
                err: bool = False) -> Union[Sample, Reject]:
    """One reply for `slot`. `src` / `v6` / `sport` are the reply's Meta (default: the
    slot's own address and family, port 123); `err` = the payload ended with err.

    The checks in the RTL's order; only the first three reasons can apply to a
    reply that is not even matched to a slot (the RTL tries every slot)."""
    if src is None:
        src = req.addr
    if v6 is None:
        v6 = req.v6
    if err:
        return Reject(R_ERR)
    if len(reply) < NTP_LEN:
        return Reject(R_SHORT)
    amask = (1 << 128) - 1 if req.v6 else M32
    if v6 != req.v6 or (src & amask) != (req.addr & amask) or sport != NTP_PORT:
        return Reject(R_SOURCE)
    if not req.outstanding:
        return Reject(R_IDLE)
    b0, stratum, _poll, prec, rdel, rdisp, rid, _rts, org, t2, t3 = _unpack(reply)
    li, vn, mode = b0 >> 6, (b0 >> 3) & 7, b0 & 7
    if mode != 4:
        return Reject(R_MODE)
    if vn not in (3, 4):
        return Reject(R_VERSION)
    if org != req.t1 & M64:
        return Reject(R_ORIGIN)
    if stratum == 0:
        return Reject(R_KOD, miss=True)
    if stratum >= 16:
        return Reject(R_STRATUM, miss=True)
    if li == 3:
        return Reject(R_LEAP, miss=True)
    offset, delay, disp = arith(req.t1, t2, t3, t4, prec, req.precision)
    return Sample(slot, offset, delay, disp, t4 & M64, stratum, li, rid, rdel, rdisp)


def _unpack(reply: bytes):
    b0, st, poll, prec, rdel, rdisp, rid, rts, org, t2, t3 = struct.unpack(
        ">BBBBIIIQQQQ", reply[:NTP_LEN])
    return b0, st, poll, prec, rdel, rdisp, rid, rts, org, t2, t3


def build_reply(*, li=0, vn=4, mode=4, stratum=1, poll=6, precision=0xE9, root_delay=0,
                root_disp=0, ref_id=0, ref_ts=0, org=0, t2=0, t3=0, ext=b"") -> bytes:
    """A server reply (test helper)."""
    b0 = ((li & 3) << 6) | ((vn & 7) << 3) | (mode & 7)
    return struct.pack(">BBBBIIIQQQQ", b0, stratum & 0xFF, poll & 0xFF, precision & 0xFF,
                       root_delay & M32, root_disp & M32, ref_id & M32, ref_ts & M64,
                       org & M64, t2 & M64, t3 & M64) + ext


# ------------------------------------------------------------ addressing

def is_ll6(a: int) -> bool:
    """fe80::/10."""
    return (a >> 118) == 0x3FA


def next_hop4(addr: int, local_ip: int, netmask: int, gateway: int) -> int:
    return addr if (addr & netmask) == (local_ip & netmask) else gateway


def next_hop6(addr: int, local_gua: int, gua_valid: bool, gateway6: int) -> int:
    on_link = is_ll6(addr) or (gua_valid and (addr >> 64) == (local_gua >> 64))
    return addr if on_link else gateway6


def src6(addr: int, local_ll: int, local_gua: int, gua_valid: bool) -> int:
    if is_ll6(addr):
        return local_ll
    return local_gua if gua_valid else local_ll


def poll_interval(poll_log2: int) -> int:
    """Regular poll interval in seconds: 2^clamp(poll_log2, 4, 17)."""
    return 1 << min(max(poll_log2, POLL_MIN), POLL_MAX)


def schedule(poll_log2: int, n: int):
    """Seconds (relative to the enable) at which the first `n` polls fire when every
    poll is resolved and answered at once: iburst, then the regular interval."""
    out, t = [], 0
    for k in range(n):
        out.append(t)
        t += IBURST_GAP_S if k + 1 < IBURST_COUNT else poll_interval(poll_log2)
    return out


# ------------------------------------------------------------ vectors

HEADER = """\
# Test vectors of the NTP client (task T4.2), generated by
# tools/golden/ntpclient/client.py (python3 tools/golden/ntpclient/client.py,
# deterministic: fixed seed). One exchange per line: the slot's request state,
# the reply's Meta and payload, then ' -> ' and the expected result.
#   <slot> <v6> <addr> <t1> <ourprec> <src> <family> <sport> <err> <t4> <payload>
# addr / src in hex (0x, 32 digits), t1 / t4 in hex (0x, 16 digits), payload in
# hex (no 0x, 48 bytes or more), the others in decimal (ourprec: 0..255, the
# 8-bit two's complement precision field).
# Expected:
#   S <peer> <offset> <delay> <disp> <t4> <stratum> <leap> <refid> <rootdelay> <rootdisp>
#     (64-bit fields 0x + 16 digits, 32-bit fields 0x + 8 digits)
#   R <reason> <miss 0/1>
"""

V4_ADDRS = [0xC0000263, 0xC0000264, 0xC6336401, 0xCB007105]
V6_ADDRS = [0x20010DB8000000000000000000000099, 0xFE800000000000000000000000000063,
            0x20010DB8000100000000000000000001, 0x20010DB8FFFF00000000000000000123]


def gen_cases(seed=0x5EED, n=400):
    """Deterministic exchanges: list of (slot, Request, src, v6, sport, err, t4, payload)."""
    rng = random.Random(seed)
    cases = []
    sec = 1 << 32
    for i in range(n):
        slot = rng.randrange(PEERS)
        v6 = rng.random() < 0.5
        addr = rng.choice(V6_ADDRS if v6 else V4_ADDRS)
        kind = rng.random()
        if kind < 0.15:
            # era wrap: T1 just before 2^64, T2..T4 after
            t1 = (M64 - rng.randrange(4 * sec)) & M64
        elif kind < 0.25:
            t1 = rng.randrange(1 << 64)
        else:
            t1 = (0xEC000000 << 32) + rng.randrange(1 << 60)
        t1 &= M64
        # offset of the server (theta) and the path delays
        r = rng.random()
        if r < 0.08:
            theta = (rng.choice([-1, 1]) * ((1 << 63) - 1 - rng.randrange(sec)))
        elif r < 0.2:
            theta = rng.randrange(-(1 << 62), 1 << 62)
        else:
            theta = rng.randrange(-sec // 10, sec // 10)
        d_out = rng.randrange(sec // 20)
        d_back = rng.randrange(sec // 20)
        proc = rng.randrange(sec // 1000)
        if rng.random() < 0.05:
            proc = -rng.randrange(sec)          # server clock goes backward: negative delay
        t2 = (t1 + d_out + theta) & M64
        t3 = (t2 + proc) & M64
        t4 = (t1 + d_out + proc + d_back) & M64
        if rng.random() < 0.03:
            t4 = (t1 - rng.randrange(sec)) & M64  # T4 before T1: iv clamped to 0
        prec_peer = rng.choice([0xE8, 0xE9, 0xEC, 0xFA, 0xDF, 0xE0, 0x00, 0x05, 0xFF])
        our = rng.choice([0xE8, 0xE8, 0xE9, 0xF0, 0xD0, 0x01])
        li = rng.choice([0] * 12 + [1, 2, 3])
        vn = rng.choice([4] * 16 + [3, 3, 3, 2, 5])
        mode = rng.choice([4] * 24 + [3, 5])
        stratum = rng.choice([1] * 8 + [2, 3, 15, 0, 16, 200])
        org = t1
        if rng.random() < 0.08:
            org = (t1 ^ (1 << rng.randrange(64))) & M64
        src, fam, sport, err = addr, v6, NTP_PORT, False
        x = rng.random()
        if x < 0.04:
            src = rng.choice(V6_ADDRS if v6 else V4_ADDRS)
        elif x < 0.06:
            fam = not v6
            if not v6:
                src = addr          # same low bits, other family
        elif x < 0.08:
            sport = rng.choice([1234, 124])
        elif x < 0.10:
            err = True
        ext = b""
        if rng.random() < 0.1:
            ext = bytes(rng.randrange(256) for _ in range(rng.choice([4, 20, 68])))
        pay = build_reply(li=li, vn=vn, mode=mode, stratum=stratum, poll=rng.randrange(4, 11),
                          precision=prec_peer, root_delay=rng.randrange(1 << 32),
                          root_disp=rng.randrange(1 << 32), ref_id=rng.randrange(1 << 32),
                          ref_ts=rng.randrange(1 << 64), org=org, t2=t2, t3=t3, ext=ext)
        if rng.random() < 0.03:
            pay = pay[:rng.randrange(1, 48)]
        cases.append((slot, Request(addr, v6, t1, True, our), src, fam, sport, err, t4, pay))
    return cases


def vector_line(case) -> str:
    slot, req, src, fam, sport, err, t4, pay = case
    res = make_sample(slot, req, pay, t4, src=src, v6=fam, sport=sport, err=err)
    head = "%d %d 0x%032x 0x%016x %d 0x%032x %d %d %d 0x%016x %s" % (
        slot, req.v6, req.addr, req.t1, req.precision, src, fam, sport, err, t4, pay.hex())
    if isinstance(res, Sample):
        tail = "S %d 0x%016x 0x%016x 0x%016x 0x%016x %d %d 0x%08x 0x%08x 0x%08x" % (
            res.peer, res.offset, res.delay, res.disp, res.t4, res.stratum, res.leap,
            res.ref_id, res.root_delay, res.root_disp)
    else:
        tail = "R %s %d" % (res.reason, res.miss)
    return head + " -> " + tail


def write_vectors(path: str) -> None:
    with open(path, "w") as fh:
        fh.write(HEADER)
        for c in gen_cases():
            fh.write(vector_line(c) + "\n")


if __name__ == "__main__":
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    write_vectors(os.path.join(here, "vectors", "client.txt"))
