"""Clock filter of RFC 5905 §10 (appendix A.5.2), fixed-point reference of
`rtl/src/clock_filter.rs` (task T4.3), plus a floating-point (exact rational)
reference of the same algorithm.

Standard library only. `ClockFilter` is bit-exact with the RTL: every value is
a Python int in the fixed-point format of `rtl/src/ntp_types.rs`:

* times, delays, dispersions, jitters: unsigned 32.32 (`U<64>`);
* offsets: two's complement 32.32 in 64 bits (a Python int in
  `0..2**64`; `to_signed` / `from_signed` convert);
* root delay / root dispersion: 16.16 in 32 bits, copied unchanged.

Rules (the RTL header `rtl/src/clock_filter.rs` is the normative text, this
module its executable form):

Per peer: 8 stages, stage 0 is the newest. `reach` (8 bits) doubles as the
validity mask of the stages: bit j is 1 iff stage j holds a real sample.

* `sample`: `reach = (reach << 1 | 1) & 0xFF`, the sample is shifted in at
  stage 0 (the oldest falls out), `tnow = sample.t4`, the metadata (stratum,
  leap, ref_id, root_delay, root_disp) is replaced, then the filter runs.
* `miss`: `reach = (reach << 1) & 0xFF`, an empty stage is shifted in (RFC:
  `clock_filter(p, 0, 0, MAXDISP)`), `tnow` and the metadata are unchanged,
  then the filter runs.
* The filter (RFC 5905 A.5.2). Stage j is *usable* iff
  `reach[j]` and `aged_disp_j < MAXDISP`.
  `aged_disp_j = disp_j + floor(PHI_Q32 * d_j / 2^32)` with
  `d_j = (tnow - t4_j) mod 2^64`, or 0 if that is >= 2^63 (t4_j after tnow).
  `MAXDISP` = 16 s. The stored disp is never modified: the ageing is
  recomputed from the stored sample at each run (no compounding).
* The usable stages are sorted by increasing delay, ties broken by the newest
  stage first (smallest j; selection by a strict `<` scan in age order).
  `m` = number of usable stages. Rank r (0 = best) contributes
  `floor(aged_disp / 2^(r+1))`; ranks r >= m contribute `MAXDISP >> (r+1)`
  (RFC: invalid entries have dispersion MAXDISP): `disp = sum over r = 0..7`.
* `jitter`: `m < 2`: 0. Else with `dd_r = min(|offset_r - offset_0|, MAXDISP)`
  (true signed difference, 65-bit safe), `jitter = isqrt(sum_{r=1}^{m-1}
  dd_r^2 // (m - 1))`: the sum over the `m-1` other samples, integer
  division floor, then floor square root (`isqrt_digit`, equal to
  `math.isqrt`). The RFC's precision floor and popcorn-spike suppressor are
  not applied here (T4.4/T4.5).
* Acceptance (RFC: `if (f[0].t - p->t <= 0) return`): the result is accepted
  iff `m >= 1` and (the peer has no accepted result yet or `t4_0`, the t4 of
  the best sample, is strictly newer than `tlast`, the t4 of the best sample
  of the last accepted update: era-modular compare, see `newer`). So a sample
  is used at most once and the output never goes back in time, but a newer
  sample with a worse delay does not replace an older, better one. Accepted:
  (offset_0, delay_0, disp, jitter) replace the stored result of the peer and
  `tlast = t4_0`. Not accepted (`m == 0`, or the best sample is not newer):
  the stored result is kept unchanged (all four fields together), the
  PeerStat reports it with the fresh `reach`.
* `valid = has_result and reach != 0 and 1 <= stratum <= 15 and leap != 3`
  (stratum, leap of the last *sample* inserted). A peer with no accepted
  result reports zeros for offset, delay, disp, jitter, t4.
"""
import math
import random
from fractions import Fraction

NSTAGE = 8
PEERS = 4
M64 = (1 << 64) - 1
SIGN64 = 1 << 63
MAXDISP = 16 << 32
PHI_Q32 = 64_425
# |offset difference| is saturated at MAXDISP before squaring: squares < 2^72,
# the sum of 7 of them < 2^75.
JSAT = MAXDISP


def to_signed(x):
    """Two's complement 64 bits to a Python int."""
    return x - (1 << 64) if x & SIGN64 else x


def from_signed(x):
    return x & M64


def newer(a, b):
    """`a` is strictly after `b` (era-modular 64-bit compare, 32.32 times)."""
    e = (a - b) & M64
    return e != 0 and e < SIGN64


def isqrt_digit(n):
    """Digit-by-digit (bit-by-bit) floor square root, 38 iterations, the exact
    algorithm of the RTL; valid for 0 <= n < 2^76. Equal to `math.isqrt`."""
    num, res, bit = n, 0, 1 << 74
    for _ in range(38):
        t = res + bit
        if num >= t:
            num -= t
            res = (res >> 1) + bit
        else:
            res >>= 1
        bit >>= 2
    return res


def restoring_div(n, d):
    """Restoring division of a 75-bit dividend by 1..7 (the RTL's divider);
    equal to `n // d`."""
    rem = q = 0
    for i in range(74, -1, -1):
        rem = (rem << 1) | ((n >> i) & 1)
        if rem >= d:
            rem -= d
            q = (q << 1) | 1
        else:
            q <<= 1
    return q


class Sample:
    __slots__ = ("peer", "offset", "delay", "disp", "t4", "stratum", "leap",
                 "ref_id", "root_delay", "root_disp")

    def __init__(self, peer, offset, delay, disp, t4, stratum=2, leap=0,
                 ref_id=0, root_delay=0, root_disp=0):
        self.peer = peer
        self.offset = offset & M64      # two's complement
        self.delay = delay
        self.disp = disp
        self.t4 = t4
        self.stratum = stratum
        self.leap = leap
        self.ref_id = ref_id
        self.root_delay = root_delay
        self.root_disp = root_disp


STAT_FIELDS = ("peer", "valid", "offset", "delay", "disp", "jitter", "reach",
               "stratum", "leap", "ref_id", "root_delay", "root_disp", "t4")


class _Peer:
    def __init__(self):
        self.head = 0
        self.reach = 0
        self.tnow = 0
        self.tlast = 0
        self.has = False
        self.meta = (0, 0, 0, 0, 0)   # stratum, leap, ref_id, root_delay, root_disp
        self.res = (0, 0, 0, 0)       # offset, delay, disp, jitter
        self.slots = [(0, 0, 0, 0)] * NSTAGE   # (offset, delay, disp, t4), ring


class ClockFilter:
    """Bit-exact model of the RTL `ClockFilter`. `sample()` and `miss()` return
    the PeerStat (a dict keyed by `STAT_FIELDS`) the unit emits."""

    def __init__(self):
        self.peers = [_Peer() for _ in range(PEERS)]

    # -- events ----------------------------------------------------------
    def sample(self, s):
        p = self.peers[s.peer]
        p.reach = ((p.reach << 1) | 1) & 0xFF
        p.head = (p.head - 1) & 7
        p.slots[p.head] = (s.offset, s.delay, s.disp, s.t4)
        p.tnow = s.t4
        p.meta = (s.stratum, s.leap, s.ref_id, s.root_delay, s.root_disp)
        return self._run(s.peer)

    def miss(self, peer):
        p = self.peers[peer]
        p.reach = (p.reach << 1) & 0xFF
        p.head = (p.head - 1) & 7
        return self._run(peer)

    # -- the filter ------------------------------------------------------
    def stage(self, p, j):
        """Stage of age j (0 = newest): (offset, delay, disp, t4)."""
        return p.slots[(p.head + j) & 7]

    def aged(self, p, j):
        _, _, dsp, t4 = self.stage(p, j)
        d = (p.tnow - t4) & M64
        if d & SIGN64:
            d = 0
        return dsp + ((d * PHI_Q32) >> 32)

    def _run(self, peer):
        p = self.peers[peer]
        eff = [0] * NSTAGE
        usable = [False] * NSTAGE
        for j in range(NSTAGE):
            if (p.reach >> j) & 1:
                eff[j] = self.aged(p, j)
                usable[j] = eff[j] < MAXDISP
        picked = [False] * NSTAGE
        dsum = jsum = m = 0
        off0 = dly0 = t40 = 0
        for r in range(NSTAGE):
            best = None
            for j in range(NSTAGE):
                if usable[j] and not picked[j]:
                    if best is None or self.stage(p, j)[1] < self.stage(p, best)[1]:
                        best = j
            if best is None:
                dsum += MAXDISP >> (r + 1)
                continue
            picked[best] = True
            off, dly, _, t4 = self.stage(p, best)
            dsum += eff[best] >> (r + 1)
            m += 1
            if r == 0:
                off0, dly0, t40 = off, dly, t4
            else:
                # true |difference| of two signed 64-bit values: < 2^64
                if (off ^ SIGN64) >= (off0 ^ SIGN64):
                    dd = (off - off0) & M64
                else:
                    dd = (off0 - off) & M64
                dd = min(dd, JSAT)
                jsum += dd * dd
        if m >= 1 and (not p.has or newer(t40, p.tlast)):
            jit = isqrt_digit(restoring_div(jsum, m - 1)) if m >= 2 else 0
            p.res = (off0, dly0, dsum, jit)
            p.tlast = t40
            p.has = True
        return self.stat(peer)

    def stat(self, peer):
        p = self.peers[peer]
        stratum, leap, ref_id, rd, rdisp = p.meta
        valid = p.has and p.reach != 0 and 1 <= stratum <= 15 and leap != 3
        off, dly, dsp, jit = p.res
        return {
            "peer": peer, "valid": int(valid), "offset": off, "delay": dly,
            "disp": dsp, "jitter": jit, "reach": p.reach, "stratum": stratum,
            "leap": leap, "ref_id": ref_id, "root_delay": rd,
            "root_disp": rdisp, "t4": p.tlast,
        }


class FloatFilter:
    """Reference of the RFC algorithm in exact rational arithmetic (no
    rounding but the final `float` square root), same rules as the fixed-point
    model. Values are real numbers of seconds (Fraction) except where noted.
    `sample`/`miss` return (valid, offset, delay, disp, jitter) in seconds."""

    def __init__(self):
        self.peers = [dict(reach=0, hist=[], tnow=0, tlast=None, res=None, meta=None)
                      for _ in range(PEERS)]

    def sample(self, s):
        p = self.peers[s.peer]
        p["reach"] = ((p["reach"] << 1) | 1) & 0xFF
        p["hist"].insert(0, (Fraction(to_signed(s.offset), 1 << 32),
                             Fraction(s.delay, 1 << 32),
                             Fraction(s.disp, 1 << 32), s.t4))
        del p["hist"][NSTAGE:]
        p["tnow"] = s.t4
        p["meta"] = (s.stratum, s.leap)
        return self._run(p)

    def miss(self, peer):
        p = self.peers[peer]
        p["reach"] = (p["reach"] << 1) & 0xFF
        p["hist"].insert(0, None)
        del p["hist"][NSTAGE:]
        return self._run(p)

    def _run(self, p):
        phi = Fraction(PHI_Q32, 1 << 32)
        maxdisp = Fraction(16)
        cand = []
        for j, h in enumerate(p["hist"]):
            if h is None:
                continue
            off, dly, dsp, t4 = h
            d = (p["tnow"] - t4) & M64
            d = 0 if d & SIGN64 else d
            eff = dsp + phi * Fraction(d, 1 << 32)
            if eff < maxdisp:
                cand.append((dly, j, off, eff, t4))
        cand.sort(key=lambda c: (c[0], c[1]))     # by delay, newest first on ties
        m = len(cand)
        if m and (p["tlast"] is None or newer(cand[0][4], p["tlast"])):
            disp = Fraction(0)
            for r in range(NSTAGE):
                disp += (cand[r][3] if r < m else maxdisp) / (2 ** (r + 1))
            jit = 0.0
            if m > 1:
                s = sum((c[2] - cand[0][2]) ** 2 for c in cand[1:]) / (m - 1)
                jit = math.sqrt(s)
            p["res"] = (cand[0][2], cand[0][0], disp, jit)
            p["tlast"] = cand[0][4]
        stratum, leap = p["meta"] or (0, 0)
        valid = p["res"] is not None and p["reach"] != 0 and 1 <= stratum <= 15 and leap != 3
        r = p["res"] or (Fraction(0), Fraction(0), Fraction(0), 0.0)
        return (valid,) + r


# -- test vectors -----------------------------------------------------------
HEADER = """\
# Test vectors of the clock filter (task T4.3), generated by
# tools/golden/ntpclient/filter.py (python3 -m ntpclient.filter from tools/golden,
# deterministic: fixed seed). One event per line, then ' -> ' and the expected
# PeerStat after it.
#   sample <peer> <offset> <delay> <disp> <t4> <stratum> <leap> <refid> <rootdelay> <rootdisp>
#   miss <peer>
# 64-bit fields (offset: two's complement 32.32, delay, disp, t4) in hex (0x, 16 digits),
# refid / rootdelay / rootdisp in hex (0x, 8 digits), the others in decimal.
# Expected: peer valid offset delay disp jitter reach stratum leap refid rootdelay rootdisp t4
"""


def _h64(x):
    return "0x%016x" % x


def _h32(x):
    return "0x%08x" % x


def format_stat(st):
    return " ".join([
        str(st["peer"]), str(st["valid"]), _h64(st["offset"]), _h64(st["delay"]),
        _h64(st["disp"]), _h64(st["jitter"]), str(st["reach"]), str(st["stratum"]),
        str(st["leap"]), _h32(st["ref_id"]), _h32(st["root_delay"]),
        _h32(st["root_disp"]), _h64(st["t4"])])


def format_sample(s):
    return " ".join([
        "sample", str(s.peer), _h64(s.offset), _h64(s.delay), _h64(s.disp), _h64(s.t4),
        str(s.stratum), str(s.leap), _h32(s.ref_id), _h32(s.root_delay),
        _h32(s.root_disp)])


def gen_events(seed=0x7A3F, n=1100):
    """A deterministic mix of events: list of ('sample', Sample) / ('miss', peer)."""
    rng = random.Random(seed)
    sec = 1 << 32
    # per-peer clock (t4); peer 3 starts just before the NTP era rollover
    t4 = [0xEC00_0000 * sec, 0xEC00_1234 * sec + 77, 0xEB00_0000 * sec,
          (1 << 64) - 600 * sec]
    base_off = [rng.randint(-50, 50) * sec // 1000 for _ in range(PEERS)]
    base_dly = [rng.randint(2, 40) * sec // 1000 for _ in range(PEERS)]
    ev = []
    for i in range(n):
        p = rng.randrange(PEERS) if rng.random() < 0.8 else (i // 40) % PEERS
        x = rng.random()
        if x < 0.10:
            ev.append(("miss", p))
            continue
        step = rng.choice([16, 32, 64, 64, 128]) * sec + rng.randrange(1 << 28)
        if rng.random() < 0.02:
            step = rng.randint(1 << 20, 1 << 21) * sec      # weeks: ages samples out
        t4[p] = (t4[p] + step) & M64
        tt = t4[p]
        if rng.random() < 0.03:
            tt = (tt - rng.choice([0, 1, 5 * sec, 100 * sec])) & M64   # not newer
        y = rng.random()
        if y < 0.03:
            off = rng.choice([(1 << 63) - 1, 1 << 63, (1 << 63) - rng.randrange(1 << 40),
                              (1 << 63) + rng.randrange(1 << 40), 0, M64, 1])
        elif y < 0.08:
            off = from_signed(rng.randint(-5, 5) * sec + rng.randrange(1 << 32))
        else:
            off = from_signed(base_off[p] + int(rng.gauss(0, 3000)) * 1000)
        dly = base_dly[p] + rng.randrange(0, 30) * (sec // 1000)
        if rng.random() < 0.15:
            dly = base_dly[p] + rng.choice([0, 1, 2]) * (sec // 1000)      # ties
        if rng.random() < 0.01:
            dly = rng.choice([0, M64, 1 << 63])
        disp = rng.randrange(1 << 12, 1 << 22)
        if rng.random() < 0.01:
            disp = rng.choice([MAXDISP, MAXDISP - 1, 1 << 40, M64])
        stratum = rng.choices([1, 2, 3, 4, 0, 15, 16, 255], [3, 8, 4, 2, 1, 1, 1, 1])[0]
        leap = rng.choices([0, 1, 2, 3], [90, 3, 3, 4])[0]
        ev.append(("sample", Sample(p, off, dly, disp, tt, stratum, leap,
                                    rng.getrandbits(32), rng.getrandbits(32) >> 8,
                                    rng.getrandbits(32) >> 8)))
    return ev


def replay(events, flt=None):
    flt = flt or ClockFilter()
    out = []
    for kind, e in events:
        out.append(flt.sample(e) if kind == "sample" else flt.miss(e))
    return out


def vector_lines(events):
    lines = []
    for (kind, e), st in zip(events, replay(events)):
        head = format_sample(e) if kind == "sample" else "miss %d" % e
        lines.append("%s -> %s" % (head, format_stat(st)))
    return lines


def write_vectors(path):
    with open(path, "w") as fh:
        fh.write(HEADER)
        for l in vector_lines(gen_events()):
            fh.write(l + "\n")


if __name__ == "__main__":
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    write_vectors(os.path.join(here, "vectors", "filter.txt"))
