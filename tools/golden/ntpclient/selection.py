"""Reference model of the clock selection unit (task T4.4, RTL `rtl/src/selection.rs`).

RFC 5905 section 11.2 (selection, clustering, combining) and Appendix A.5.5
(`clock_select`, `root_dist`, `clock_combine`), for up to 4 peers, in EXACTLY the
integer arithmetic of the RTL: the RTL is bit-exact with `Selection.process`.
Standard library only.

Units: times are 32.32 fixed point seconds ("Q32", 1 = 2^-32 s), offsets signed
(two's complement on 64 bits in PeerStat/Selected, Python ints inside),
root delay / root dispersion 16.16 (shifted left by 16 to become Q32).

State: the latest PeerStat of each of the 4 peers (none after reset), the
previous system peer (none after reset, or after an event with no selection)
and `ever` (a valid selection was made since reset). Constant input: `n_en`,
the number of enabled upstream slots (0..7, normally 1..4).
Each event (one PeerStat, plus our clock `now`) replaces the stored PeerStat of
its peer and recomputes everything below over the 4 stored peers.

1. Root distance (A.5.5.2 `root_dist`), per stored peer, exact integers:
     age  = now - t4 if now >= t4 (unsigned) else 0;  age = min(age, 2^50 - 1)
     phi  = ((age >> 16) * PHI_Q32) >> 16                  (PHI * age, floor twice)
     half = max(MINDISP_Q32, (root_delay << 16) + delay) >> 1
     rd   = half + (root_disp << 16) + disp + jitter + phi + RHO_Q32
     rd   = min(rd, 2^64 - 1)
   RHO_Q32 is our own precision (2^-24 s, docs/references.md) as a constant.
2. Eligibility (A.5.5.3 `fit`): stored, valid, reach != 0, 1 <= stratum <= 15
   (stratum 0 reads as MAXSTRAT, RFC 5905 section 7.3), leap != 3,
   rd < MAXDIST_Q32 (1.5 s).  m = number of eligible peers; m = 0: no selection.
3. Intersection (11.2.1, A.5.5.1, Marzullo): three tuples per eligible peer
   (theta - rd, lowpoint), (theta, midpoint), (theta + rd, highpoint), sorted by
   (edge, type with lowpoint < midpoint < highpoint, peer index): the intervals
   are closed and all keys distinct.  For allow = 0, 1, ... while 2 * allow < m:
   scan up (lowpoint +1, highpoint -1) to the first tuple where the count reaches
   m - allow: low = its edge; scan down (highpoint +1, lowpoint -1) likewise:
   high; found = midpoints passed by both scans before stopping.  found > allow:
   next allow; else high > low (strict): success; else next allow.  No success:
   no selection.  Truechimers (survivors) = eligible peers with
   low <= theta <= high (the midpoint test of A.5.5.1).
3b. Start-up majority (security finding F4, RFC 5905 NSANE / MINCLOCK in
   spirit): while `ever` is false, the selection also needs at least
   n_en // 2 + 1 truechimers (1 enabled slot: 1, i.e. the single source decides,
   the residual risk; 2: both; 3: 2; 4: 3); otherwise no selection (as for a
   failed intersection: the previous system peer is cleared). `ever` becomes
   true with the first valid selection and stays true until reset.
4. Clustering (11.2.2): while more than NMIN (3) survivors: for each survivor p,
   S_p = sum over survivors q of (theta_p - theta_q)^2 (exact, Q64);
   qmax = the survivor of largest S_p (strict >, lowest peer index on a tie);
   phi_min = min peer jitter of the survivors; selection jitter of qmax is
   sqrt(S_max / (n - 1)) (RMS, n survivors), so stop iff S_max < (n-1) phi_min^2,
   else drop qmax.  With 4 peers at most one is dropped.
5. System peer: best = the survivor of minimum (stratum, rd, index) (the
   A.5.5.1 metric MAXDIST * stratum + rd, ties by index).  Hysteresis: the
   previous system peer p is kept iff it is a survivor, stratum_p == stratum_best
   (the A.5.5.1 "no clock hop" rule) and rd_p <= SGATE * rd_best (our guard).
6. Combine (11.2.3, A.5.5.5), s = system peer, over the survivors i:
     W_i   = 2^48 // rd_i            (fixed-point reciprocal, 2^15 < W_i < 2^24)
     dlt_i = theta_i - theta_s       (|dlt_i| < 2^34: survivors lie in [low, high])
     N = sum W_i * dlt_i,  D = sum W_i,  J = sum W_i * dlt_i^2
     offset = theta_s + sign(N) * (|N| // D)       (truncation toward zero)
     jitter = isqrt(J // D + jitter_s^2)           (floors; system jitter =
              sqrt(selection jitter^2 + peer jitter^2), 11.2.3)
   One survivor: offset = theta_s exactly.
7. Output: Selected(valid=1, peer=s, offset, jitter, root_dist=rd_s and the
   system peer's stratum, leap, ref_id, root_delay, root_disp, delay, disp, t4),
   or Selected() all zero when there is no selection.

Run as a script to regenerate `vectors/selection.txt` (deterministic).
"""

import math
import os
import random
from dataclasses import dataclass, field, fields

PEERS = 4
M64 = (1 << 64) - 1
#: 1.5 s (RFC 5905 MAXDIST).
MAXDIST_Q32 = 3 << 31
#: 0.01 s (RFC 5905 MINDISP), 42949672.96 rounded to nearest.
MINDISP_Q32 = 42_949_673
#: 15 ppm in 2^-32 s per second (rtl/src/ntp_types.rs PHI_Q32).
PHI_Q32 = 64_425
#: Our precision, 2^-24 s, added to every root distance.
RHO_Q32 = 1 << 8
#: Ageing is clamped at 2^50 - 1 (2^18 s, PHI * age = 3.9 s > MAXDIST).
AGE_MAX = (1 << 50) - 1
NMIN = 3
SGATE = 3
#: Weights are 2^W_SHIFT // rd.
W_SHIFT = 48


def s64(v):
    """Two's complement 64-bit to Python int."""
    v &= M64
    return v - (1 << 64) if v >> 63 else v


def u64(v):
    return v & M64


@dataclass
class PeerStat:
    peer: int = 0
    valid: int = 0
    offset: int = 0  # raw 64-bit two's complement
    delay: int = 0
    disp: int = 0
    jitter: int = 0
    reach: int = 0
    stratum: int = 0
    leap: int = 0
    ref_id: int = 0
    root_delay: int = 0
    root_disp: int = 0
    t4: int = 0


@dataclass
class Selected:
    valid: int = 0
    peer: int = 0
    offset: int = 0  # raw 64-bit two's complement
    jitter: int = 0
    root_dist: int = 0
    stratum: int = 0
    leap: int = 0
    ref_id: int = 0
    root_delay: int = 0
    root_disp: int = 0
    delay: int = 0
    disp: int = 0
    t4: int = 0


STAT_FIELDS = [f.name for f in fields(PeerStat)]
SEL_FIELDS = [f.name for f in fields(Selected)]


def root_dist(p, now):
    age = (now - p.t4) if now >= p.t4 else 0
    age = min(age, AGE_MAX)
    phi = ((age >> 16) * PHI_Q32) >> 16
    half = max(MINDISP_Q32, (p.root_delay << 16) + p.delay) >> 1
    rd = half + (p.root_disp << 16) + p.disp + p.jitter + phi + RHO_Q32
    return min(rd, M64)


def eligible(p, rd):
    return bool(
        p is not None
        and p.valid
        and p.reach != 0
        and 1 <= p.stratum <= 15
        and p.leap != 3
        and rd < MAXDIST_Q32
    )


def intersect(cands):
    """cands: list of (peer, theta, rd) (ints).  Returns (low, high) or None.

    RFC 5905 A.5.5.1, literally (with the missing failure exit of the
    pseudocode: no allow value succeeds -> None)."""
    m = len(cands)
    if m == 0:
        return None
    tup = []
    for p, th, rd in cands:
        tup += [(th - rd, 0, p), (th, 1, p), (th + rd, 2, p)]
    tup.sort()
    allow = 0
    while 2 * allow < m:
        found = 0
        chime = 0
        low = None
        for e, t, _ in tup:
            chime += 1 if t == 0 else (-1 if t == 2 else 0)
            if chime >= m - allow:
                low = e
                break
            if t == 1:
                found += 1
        chime = 0
        high = None
        for e, t, _ in reversed(tup):
            chime += 1 if t == 2 else (-1 if t == 0 else 0)
            if chime >= m - allow:
                high = e
                break
            if t == 1:
                found += 1
        if found <= allow and high > low:
            return low, high
        allow += 1
    return None


def cluster(surv, theta, jit):
    """surv: sorted list of peer indexes.  Returns the kept list."""
    surv = list(surv)
    while len(surv) > NMIN:
        n = len(surv)
        smax, qmax = -1, None
        for p in surv:
            s = sum((theta[p] - theta[q]) ** 2 for q in surv)
            if s > smax:
                smax, qmax = s, p
        jmin = min(jit[p] for p in surv)
        if smax < (n - 1) * jmin * jmin:
            break
        surv.remove(qmax)
    return surv


class Selection:
    """The unit's state and behaviour, one call per PeerStat event."""

    def __init__(self, n_en=1):
        self.peers = [None] * PEERS
        self.sys = None  # previous system peer
        self.n_en = n_en  # enabled upstream slots (start-up majority, F4)
        self.ever = False  # a valid selection was made since reset
        self.info = {}

    def process(self, stat, now):
        self.peers[stat.peer] = stat
        rd = [root_dist(p, now) if p is not None else M64 for p in self.peers]
        el = [eligible(p, rd[i]) for i, p in enumerate(self.peers)]
        theta = [s64(p.offset) if p is not None else 0 for p in self.peers]
        jit = [p.jitter if p is not None else 0 for p in self.peers]
        cands = [(i, theta[i], rd[i]) for i in range(PEERS) if el[i]]
        self.info = {"rd": rd, "el": el, "inter": None, "truechimers": [], "survivors": []}
        iv = intersect(cands)
        self.info["inter"] = iv
        if iv is None:
            self.sys = None
            return Selected()
        low, high = iv
        tc = [i for i in range(PEERS) if el[i] and low <= theta[i] <= high]
        self.info["truechimers"] = tc
        if not self.ever and len(tc) < (self.n_en >> 1) + 1:
            # F4: no start-up selection without a majority of the enabled slots.
            self.info["startup_gate"] = True
            self.sys = None
            return Selected()
        self.ever = True
        surv = cluster(tc, theta, jit)
        self.info["survivors"] = surv
        best = min(surv, key=lambda i: (self.peers[i].stratum, rd[i], i))
        s = best
        o = self.sys
        if (
            o is not None
            and o in surv
            and self.peers[o].stratum == self.peers[best].stratum
            and rd[o] <= SGATE * rd[best]
        ):
            s = o
        self.sys = s
        self.info["best"] = best
        # Combine.
        num = den = jsum = 0
        for i in surv:
            w = (1 << W_SHIFT) // rd[i]
            assert (1 << 15) < w < (1 << 24)
            d = theta[i] - theta[s]
            assert abs(d) < (1 << 34)
            num += w * d
            den += w
            jsum += w * d * d
        q = abs(num) // den
        assert q < (1 << 34)
        corr = q if num >= 0 else -q
        var = jsum // den + jit[s] * jit[s]
        assert var < (1 << 70)
        ps = self.peers[s]
        return Selected(
            valid=1,
            peer=s,
            offset=u64(theta[s] + corr),
            jitter=math.isqrt(var),
            root_dist=rd[s],
            stratum=ps.stratum,
            leap=ps.leap,
            ref_id=ps.ref_id,
            root_delay=ps.root_delay,
            root_disp=ps.root_disp,
            delay=ps.delay,
            disp=ps.disp,
            t4=ps.t4,
        )


# --------------------------------------------------------------------------
# Floating-point reference of RFC 5905 (no fixed point), for the random checks.

Q = float(1 << 32)


def float_select(peers, now, prev_sys):
    """peers: list of PeerStat or None.  Returns (sel dict or None, sys)."""
    rd, el, th, jit = [], [], [], []
    for p in peers:
        if p is None:
            rd.append(float("inf"))
            el.append(False)
            th.append(0.0)
            jit.append(0.0)
            continue
        age = (now - p.t4) / Q if now >= p.t4 else 0.0
        age = min(age, AGE_MAX / Q)
        r = (
            max(0.01, p.root_delay / 65536.0 + p.delay / Q) / 2
            + p.root_disp / 65536.0
            + p.disp / Q
            + p.jitter / Q
            + PHI_Q32 / Q * age
            + 2.0**-24
        )
        rd.append(r)
        el.append(
            bool(p.valid and p.reach and 1 <= p.stratum <= 15 and p.leap != 3 and r < 1.5)
        )
        th.append(s64(p.offset) / Q)
        jit.append(p.jitter / Q)
    cands = [(i, th[i], rd[i]) for i in range(len(peers)) if el[i]]
    iv = intersect(cands)
    if iv is None:
        return None, None
    low, high = iv
    surv = [i for i in range(len(peers)) if el[i] and low <= th[i] <= high]
    while len(surv) > NMIN:
        n = len(surv)
        sj = {p: math.sqrt(sum((th[p] - th[q]) ** 2 for q in surv) / (n - 1)) for p in surv}
        qmax = max(surv, key=lambda p: (sj[p], -p))
        if sj[qmax] < min(jit[p] for p in surv):
            break
        surv.remove(qmax)
    best = min(surv, key=lambda i: (peers[i].stratum, rd[i], i))
    s = best
    o = prev_sys
    if o is not None and o in surv and peers[o].stratum == peers[best].stratum and rd[o] <= SGATE * rd[best]:
        s = o
    y = sum(1 / rd[i] for i in surv)
    z = sum(th[i] / rd[i] for i in surv)
    w = sum((th[i] - th[s]) ** 2 / rd[i] for i in surv)
    return {
        "peer": s,
        "offset": z / y,
        "jitter": math.sqrt(w / y + jit[s] ** 2),
        "root_dist": rd[s],
        "survivors": surv,
        "spread": max(abs(th[i] - th[s]) for i in surv),
    }, s


# --------------------------------------------------------------------------
# Test vectors.

HEADER = """\
# Test vectors of the clock selection unit (T4.4), generated by
# tools/golden/ntpclient/selection.py (do not edit; deterministic).
# A scenario starts from reset ("scenario <name> <n_en>", n_en = number of
# enabled upstream slots, the unit's `n_en` input, decimal).  Each event is a
# "stat" line (the PeerStat taken, with our clock `now` first) followed by the
# "sel" line expected on the output.  All numbers are hexadecimal; signed
# offsets are 64-bit two's complement.
# stat now peer valid offset delay disp jitter reach stratum leap ref_id root_delay root_disp t4
# sel valid peer offset jitter root_dist stratum leap ref_id root_delay root_disp delay disp t4
"""


def fmt_stat(now, p):
    return "stat %x " % now + " ".join("%x" % getattr(p, f) for f in STAT_FIELDS)


def fmt_sel(s):
    return "sel " + " ".join("%x" % getattr(s, f) for f in SEL_FIELDS)


def parse_vectors(text):
    """Returns [(name, n_en, [(now, PeerStat, Selected)])]."""
    out = []
    cur = None
    pending = None
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        w = line.split()
        if w[0] == "scenario":
            cur = (w[1], int(w[2]), [])
            out.append(cur)
        elif w[0] == "stat":
            v = [int(x, 16) for x in w[1:]]
            pending = (v[0], PeerStat(*v[1:]))
        elif w[0] == "sel":
            v = [int(x, 16) for x in w[1:]]
            cur[2].append((pending[0], pending[1], Selected(*v)))
            pending = None
        else:
            raise ValueError(line)
    return out


S = 1 << 32  # one second, Q32
MS = S // 1000
US = S // 1_000_000
T0 = 3_900_000_000 * S  # an NTP time in era 0


def st(peer, off_s=0.0, *, off=None, delay=10 * MS, disp=1 * MS, jitter=100 * US, reach=0xFF,
       stratum=1, leap=0, ref_id=None, rdelay=0, rdisp=0, t4=T0, valid=1):
    """PeerStat helper: offset in seconds (float) unless `off` (raw int Q32) is given."""
    o = int(round(off_s * S)) if off is None else off
    return PeerStat(peer, valid, u64(o), delay, disp, jitter, reach, stratum, leap,
                    0x47505300 + peer if ref_id is None else ref_id, rdelay, rdisp, t4)


def directed():
    """Hand-written scenarios: [(name, [(now, PeerStat)])] (n_en = 1)."""
    sc = []
    now = T0 + 2 * S
    sc.append(("single", [(now, st(0, 0.0123))]))
    sc.append(("single_negative", [(now, st(2, -0.5))]))
    sc.append(("two_agree", [(now, st(0, 0.010)), (now, st(1, 0.012, delay=30 * MS))]))
    sc.append(("two_disagree", [(now, st(0, 0.010)), (now, st(1, 0.300))]))
    sc.append(("three_one_false", [(now, st(0, 0.010)), (now, st(1, 0.011)), (now, st(2, 0.500))]))
    sc.append(("three_one_false_first", [(now, st(3, -0.700)), (now, st(1, 0.011)), (now, st(2, 0.012))]))
    sc.append(("four_two_false", [(now, st(0, 0.010)), (now, st(1, 0.011)),
                                  (now, st(2, 0.500)), (now, st(3, 0.501))]))
    sc.append(("four_one_false", [(now, st(0, 0.010)), (now, st(1, 0.011)),
                                  (now, st(2, 0.012)), (now, st(3, 0.900))]))
    sc.append(("leap3", [(now, st(0, 0.010)), (now, st(1, 0.020, leap=3))]))
    sc.append(("leap3_only", [(now, st(1, 0.020, leap=3))]))
    sc.append(("stratum16", [(now, st(0, 0.010, stratum=16)), (now, st(1, 0.020, stratum=2))]))
    sc.append(("stratum0", [(now, st(0, 0.010, stratum=0))]))
    sc.append(("unreachable", [(now, st(0, 0.010, reach=0)), (now, st(1, 0.011, reach=1))]))
    sc.append(("invalid_stat", [(now, st(0, 0.010)), (now, st(0, 0.010, valid=0))]))
    # Root distance boundaries: rd = disp + MINDISP/2 + RHO with no other term.
    base = (MINDISP_Q32 >> 1) + RHO_Q32
    sc.append(("maxdist_minus_1", [(T0, st(0, 0.25, delay=0, jitter=0, disp=MAXDIST_Q32 - 1 - base))]))
    sc.append(("maxdist_exact", [(T0, st(0, 0.25, delay=0, jitter=0, disp=MAXDIST_Q32 - base))]))
    sc.append(("high_root_disp", [(now, st(0, 0.01, rdisp=0x18000)), (now, st(1, 0.01, rdisp=0x17000))]))
    sc.append(("huge_delay", [(now, st(0, 0.01, delay=M64)), (now, st(1, 0.0, delay=M64, disp=M64, jitter=M64))]))
    # Ageing: PHI * age pushes the root distance over MAXDIST.
    d0 = st(0, 0.01, disp=S)  # rd about 1.005 s
    sc.append(("ageing", [(T0, d0), (T0 + 30000 * S, st(1, 0.01, t4=T0 + 30000 * S)),
                          (T0 + 40000 * S, st(1, 0.01, t4=T0 + 40000 * S)),
                          (T0 - 5 * S, st(1, 0.01, t4=T0))]))
    sc.append(("age_clamp", [(M64 - 1, st(1, 0.02)), (u64(-1), st(1, 0.02, t4=0))]))
    # Extremes of the offset range.
    sc.append(("offset_max", [(now, st(0, off=(1 << 63) - 1))]))
    sc.append(("offset_min", [(now, st(0, off=-(1 << 63)))]))
    sc.append(("offset_extremes", [(now, st(0, off=(1 << 63) - 1)), (now, st(1, off=-(1 << 63)))]))
    sc.append(("offset_near_max_pair", [(now, st(0, off=(1 << 63) - 1 - 5 * MS)),
                                        (now, st(1, off=(1 << 63) - 1, disp=1400 * MS))]))
    sc.append(("offset_near_min_three", [(now, st(0, off=-(1 << 63))), (now, st(1, off=-(1 << 63) + MS)),
                                         (now, st(2, off=-(1 << 63) + 2 * MS, jitter=0))]))
    # Widest survivor spread: two peers with rd just under MAXDIST, offsets ~3 s apart.
    big = MAXDIST_Q32 - 1 - base
    # Widest survivor spread: rd just under MAXDIST, survivors about 3 s apart (allow = 1).
    big = MAXDIST_Q32 - 1 - base
    sc.append(("widest_spread3", [(T0, st(0, off=-(MAXDIST_Q32 - 2), delay=0, jitter=0, disp=big)),
                                  (T0, st(1, off=MAXDIST_Q32 - 2, delay=0, jitter=0, disp=big)),
                                  (T0, st(2, off=0, delay=0, jitter=0, disp=big))]))
    sc.append(("widest_spread4", [(T0, st(0, off=-(MAXDIST_Q32 - 2), delay=0, jitter=0, disp=big)),
                                  (T0, st(1, off=MAXDIST_Q32 - 2, delay=0, jitter=0, disp=big)),
                                  (T0, st(2, off=1, delay=0, jitter=1, disp=big - 1)),
                                  (T0, st(3, off=-1, delay=0, jitter=2, disp=big - 2))]))
    jbig = MAXDIST_Q32 - base - 2
    sc.append(("widest_spread4_keep", [(T0, st(0, off=-(MAXDIST_Q32 // 2), delay=0, jitter=jbig, disp=0)),
                                       (T0, st(1, off=MAXDIST_Q32 // 2, delay=0, jitter=jbig, disp=0)),
                                       (T0, st(2, off=1, delay=0, jitter=MAXDIST_Q32 - base - 2, disp=0)),
                                       (T0, st(3, off=-1, delay=0, jitter=MAXDIST_Q32 - base - 2, disp=0))]))
    # Clustering: 4 truechimers, one far within the intersection, small jitters: dropped.
    sc.append(("cluster_drop", [(now, st(0, 0.010, jitter=10 * US, disp=100 * MS)),
                                (now, st(1, 0.011, jitter=10 * US, disp=100 * MS)),
                                (now, st(2, 0.012, jitter=10 * US, disp=100 * MS)),
                                (now, st(3, 0.040, jitter=10 * US, disp=100 * MS))]))
    # Same with large peer jitters: all four kept.
    sc.append(("cluster_keep", [(now, st(0, 0.010, jitter=50 * MS)), (now, st(1, 0.011, jitter=50 * MS)),
                                (now, st(2, 0.012, jitter=50 * MS)), (now, st(3, 0.040, jitter=50 * MS, disp=100 * MS))]))
    # Stratum dominates the metric.
    sc.append(("stratum_pref", [(now, st(0, 0.010, stratum=2, disp=1 * MS)), (now, st(1, 0.011, stratum=1, disp=200 * MS))]))
    # Ties: identical peers, the lowest index wins; the same offsets give jitter = peer jitter.
    sc.append(("tie_identical", [(now, st(2, 0.010, ref_id=7)), (now, st(1, 0.010, ref_id=7)),
                                 (now, st(3, 0.010, ref_id=7))]))
    sc.append(("tie_touching", [(T0, st(0, off=0, delay=0, jitter=0, disp=S // 4)),
                                (T0, st(1, off=2 * ((S // 4) + base), delay=0, jitter=0, disp=S // 4))]))
    sc.append(("tie_cluster", [(now, st(0, 0.000, jitter=1, disp=100 * MS)), (now, st(1, 0.010, jitter=1, disp=100 * MS)),
                               (now, st(2, 0.020, jitter=1, disp=100 * MS)), (now, st(3, 0.030, jitter=1, disp=100 * MS))]))
    # Hysteresis.
    sc.append(("hysteresis_keep", [(now, st(0, 0.010, disp=20 * MS)), (now, st(1, 0.011, disp=15 * MS)),
                                   (now, st(1, 0.011, disp=10 * MS)), (now, st(0, 0.010, disp=20 * MS))]))
    sc.append(("hysteresis_switch_far", [(now, st(0, 0.010, disp=100 * MS)), (now, st(1, 0.011, disp=200 * MS)),
                                         (now, st(1, 0.011, disp=1 * MS)), (now, st(0, 0.010, disp=100 * MS))]))
    sc.append(("hysteresis_switch_stratum", [(now, st(0, 0.010, stratum=2, disp=1 * MS)),
                                             (now, st(1, 0.011, stratum=2, disp=2 * MS)),
                                             (now, st(1, 0.011, stratum=1, disp=2 * MS))]))
    sc.append(("hysteresis_lost", [(now, st(0, 0.010, disp=5 * MS)), (now, st(1, 0.011, disp=10 * MS)),
                                   (now, st(0, 0.010, leap=3)), (now, st(0, 0.010, disp=5 * MS)),
                                   (now, st(1, 0.011, disp=10 * MS))]))
    sc.append(("hysteresis_after_none", [(now, st(0, 0.010, disp=5 * MS)), (now, st(1, 0.500, disp=10 * MS)),
                                         (now, st(1, 0.011, disp=4 * MS)), (now, st(0, 0.010, disp=5 * MS))]))
    # Interleaved updates of the four peers.
    sc.append(("interleave", [(now, st(3, 0.020)), (now + S, st(0, 0.021)), (now + 2 * S, st(3, 0.022)),
                              (now + 3 * S, st(2, -0.300)), (now + 4 * S, st(1, 0.019)),
                              (now + 5 * S, st(2, 0.020)), (now + 6 * S, st(0, 0.021, valid=0)),
                              (now + 7 * S, st(3, 0.018, stratum=3)), (now + 8 * S, st(0, 0.021))]))
    sc.append(("interleave_same_now", [(now, st(3, 0.020)), (now, st(0, 0.021)), (now, st(3, 0.022)),
                                       (now, st(2, -0.300)), (now, st(1, 0.019)), (now, st(2, 0.020)),
                                       (now, st(0, 0.021, valid=0)), (now, st(3, 0.018, stratum=3)),
                                       (now, st(0, 0.021)), (now, st(1, 0.019, leap=3))]))
    return sc


def random_scenarios(rng, count):
    sc = []
    for k in range(count):
        n_ev = rng.randint(1, 8)
        truth = rng.uniform(-2.0, 2.0) if rng.random() < 0.8 else rng.uniform(-2e9, 2e9)
        now = T0 + rng.randrange(0, 1000) * S
        evs = []
        for _ in range(n_ev):
            p = rng.randrange(PEERS)
            now += rng.randrange(0, 64 * S)
            false = rng.random() < 0.25
            off = truth + (rng.uniform(-1.0, 1.0) if false else rng.gauss(0, 0.002))
            delay = rng.randrange(0, 200 * MS)
            disp = rng.randrange(0, 400 * MS) if rng.random() < 0.9 else rng.randrange(0, 3 * S)
            jitter = rng.randrange(0, 20 * MS)
            stat = PeerStat(
                peer=p,
                valid=1 if rng.random() < 0.95 else 0,
                offset=u64(int(off * S)),
                delay=delay,
                disp=disp,
                jitter=jitter,
                reach=rng.choice([0xFF, 0xFF, 0xFF, 0x01, 0x00]) if rng.random() < 0.3 else 0xFF,
                stratum=rng.choice([1, 1, 2, 3, 15, 16, 0]) if rng.random() < 0.3 else rng.choice([1, 2]),
                leap=rng.choice([0, 1, 2, 3]) if rng.random() < 0.1 else 0,
                ref_id=rng.getrandbits(32),
                root_delay=rng.randrange(0, 0x2000),
                root_disp=rng.randrange(0, 0x3000),
                t4=now - rng.randrange(0, 4 * S),
            )
            evs.append((now, stat))
        sc.append(("random_%03d" % k, evs))
    return sc


def startup():
    """Start-up majority scenarios (F4): [(name, n_en, [(now, PeerStat)])]."""
    now = T0 + 2 * S
    sc = []
    sc.append(("startup_one_enabled", 1, [(now, st(0, 0.0123))]))
    sc.append(("startup_none_enabled", 0, [(now, st(2, 0.0123))]))
    sc.append(("startup_two_need_both", 2, [(now, st(0, 0.010)), (now, st(1, 0.011)),
                                           (now, st(1, 0.012, valid=0)), (now, st(0, 0.013))]))
    sc.append(("startup_two_disagree", 2, [(now, st(0, 0.010)), (now, st(1, 0.300)),
                                          (now, st(1, 0.011))]))
    sc.append(("startup_two_one_lies", 2, [(now, st(0, off=365 * 86400 * S)),
                                          (now + 2 * S, st(0, off=365 * 86400 * S)),
                                          (now + 4 * S, st(1, 0.001))]))
    sc.append(("startup_three", 3, [(now, st(2, 0.010)), (now, st(0, 0.500)), (now, st(1, 0.011))]))
    sc.append(("startup_four", 4, [(now, st(0, 0.010)), (now, st(1, 0.011)), (now, st(2, 0.012)),
                                  (now, st(3, 0.013)), (now, st(3, 0.013, stratum=16)),
                                  (now, st(2, 0.012, valid=0)), (now, st(1, 0.011, leap=3))]))
    sc.append(("startup_four_falseticker", 4, [(now, st(0, 0.010)), (now, st(1, 0.011)),
                                              (now, st(2, 0.900)), (now, st(3, 0.012))]))
    sc.append(("startup_gate_fails", 4, [(now, st(0, 0.010)), (now, st(1, 0.011))]))
    sc.append(("startup_four_cluster", 4, [(now, st(0, 0.010, jitter=10 * US, disp=100 * MS)),
                                          (now, st(1, 0.011, jitter=10 * US, disp=100 * MS)),
                                          (now, st(2, 0.012, jitter=10 * US, disp=100 * MS)),
                                          (now, st(3, 0.040, jitter=10 * US, disp=100 * MS))]))
    sc.append(("startup_seven", 7, [(now, st(0, 0.010)), (now, st(1, 0.011)), (now, st(2, 0.012)),
                                   (now, st(3, 0.013))]))
    return sc


SEED = 20261117
N_RANDOM = 300
SEED_EN = 20261118
N_RANDOM_EN = 120


def all_scenarios():
    """[(name, n_en, [(now, PeerStat)])]: the pre-F4 scenarios run with n_en = 1
    (exactly their earlier results), then the start-up scenarios and random
    scenarios with 2 to 4 enabled slots."""
    old = [(n, 1, evs) for n, evs in directed() + random_scenarios(random.Random(SEED), N_RANDOM)]
    rng = random.Random(SEED_EN)
    en = [("random_en_%03d" % k, rng.choice([2, 3, 4]), evs) for k, (_, evs) in
          enumerate(random_scenarios(rng, N_RANDOM_EN))]
    return old + startup() + en


def run_scenario(evs, n_en=1):
    m = Selection(n_en)
    return [(now, s, m.process(s, now)) for now, s in evs]


def render():
    out = [HEADER]
    for name, n_en, evs in all_scenarios():
        out.append("scenario %s %d\n" % (name, n_en))
        for now, s, sel in run_scenario(evs, n_en):
            out.append(fmt_stat(now, s) + "\n")
            out.append(fmt_sel(sel) + "\n")
    return "".join(out)


VECTORS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vectors", "selection.txt")


if __name__ == "__main__":
    with open(VECTORS, "w") as f:
        f.write(render())
    print("wrote", VECTORS)
