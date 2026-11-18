"""Clock discipline (servo) of the NTP client, task T4.5: reference model.

Two parts:

1. ``Servo``: the fixed-point discipline, **bit exact** with ``rtl/src/servo.rs``.
   One ``update()`` per ``Selected`` event; it returns the outputs of the unit
   after the event: ``trim`` (signed, LSB of ``DisciplinedClock.trim``),
   ``step`` (96-bit word), ``step_en``, ``state``, ``synced``. Between events
   ``outputs(now)`` gives ``trim``/``synced`` at a later local time (the phase
   slew expires one poll interval after the update).
2. ``simulate()``: a closed-loop simulation (pure Python, floating point for
   the plant, the fixed-point servo above as the controller) used by the tests
   to show the settling, the step at start-up, the frequency learning, the
   spike rejection and the holdover.

Standard library only. The algorithm, its units and its choices are described
in the header of ``rtl/src/servo.rs`` (the specification); this file is the
executable version of it.

Units (see ``rtl/src/ntp_types.rs`` and ``rtl/src/clock.rs``):

* offsets, jitters, times: 32.32 fixed point seconds (``Q32 = 2**32``),
  offsets signed (two's complement on 64 bits);
* ``trim``: 2**-64 s per 10 ns cycle, i.e. a relative rate of
  ``1e8 / 2**64`` (0.005421 ppb) per LSB, clamped to +-MAXTRIM (500 ppm);
* the unit's internal time base ``now_s`` = 32.32 time >> 16 (units of
  2**-16 s, 48 bits, modulo 2**48).

Run ``python3 tools/golden/ntpclient/servo.py`` to regenerate
``tools/golden/ntpclient/vectors/servo.txt``.
"""
import math
import os
import random

# ---------------------------------------------------------------- constants

Q32 = 1 << 32
M64 = (1 << 64) - 1
M48 = (1 << 48) - 1
M96 = (1 << 96) - 1

CLK_HZ = 100_000_000
#: Clamp of the trim (rtl/src/clock.rs MAXTRIM): floor(500 ppm * 2^64 / 1e8).
MAXTRIM = (1 << 64) // (2000 * CLK_HZ)  # 92 233 720
#: Relative rate of one trim LSB.
TRIM_LSB = CLK_HZ / float(1 << 64)

# States (numbering of RFC 5905 A.5.5.6, plus PANIC for diagnostics).
NSET, FSET, SPIK, FREQ, SYNC, PANIC = 0, 1, 2, 3, 4, 5
STATE_NAMES = {NSET: "NSET", FSET: "FSET", SPIK: "SPIK", FREQ: "FREQ", SYNC: "SYNC", PANIC: "PANIC"}

#: Step threshold STEPT = 128 ms, 32.32, rounded.
STEPT_Q = round(0.128 * Q32)  # 549 755 814
#: Panic threshold PANICT = 1000 s, 32.32.
PANICT_Q = 1000 * Q32
#: Stepout interval STEPOUT = 900 s, in now_s units (2^-16 s).
STEPOUT_S = 900 << 16
#: Spike gate: an update in SYNC is a spike if |offset| > max(SGATE * jitter, SPKMIN).
SGATE = 3
SPKMIN_Q = 4_294_967  # 1 ms, 32.32 (floor)
#: Offset -> rate: r = (|offset| * CQ) >> CS is the trim (LSB) that removes
#: |offset| in one second: 2^32 / 1e8 * 2^24 = 2^56 / 1e8, rounded.
CS = 24
CQ = round((1 << 56) / CLK_HZ)  # 720 575 940
MUL1_BITS = 30  # CQ < 2^30
MUL2_BITS = 17  # m <= 2^16
#: Loop gains, as right shifts: slew = c >> KP, integral = (c * m) >> (16 + KI).
KP = 1
KI_FREQ = 1
KI_SYNC = 3
#: Processed updates in FREQ before SYNC.
FREQ_N = 4
#: Clamp of one integral increment.
DMAX = 2 * MAXTRIM
#: Poll exponent range.
POLL_MIN, POLL_MAX = 4, 10
#: `synced` needs an accepted update less than 2^(poll + RECENT_SHIFT) s ago.
RECENT_SHIFT = 3

assert CQ < (1 << MUL1_BITS)


def clamp_trim(x: int) -> int:
    return max(-MAXTRIM, min(MAXTRIM, x))


def s64(x: int) -> int:
    """64-bit two's complement word -> signed int."""
    x &= M64
    return x - (1 << 64) if x >> 63 else x


def u64(x: int) -> int:
    return x & M64


def s48(x: int) -> int:
    x &= M48
    return x - (1 << 48) if x >> 47 else x


def ppm(trim: int) -> float:
    return trim * TRIM_LSB * 1e6


def processing_cycles(poll: int) -> int:
    """Cycles from the pop of a processed event (cycle P) to the first cycle
    where the new trim is visible: P + 1 + MUL1 + SHR + MUL2 + FIN."""
    return 1 + MUL1_BITS + poll + MUL2_BITS + 1


# ---------------------------------------------------------------- fixed-point servo


class Servo:
    """Bit-exact model of rtl/src/servo.rs (registers named as in the RTL)."""

    def __init__(self):
        self.st = NSET
        self.freq = 0  # signed trim LSB, |freq| <= MAXTRIM
        self.tr = 0  # clamp(freq + slew) of the last processed update
        self.tl = 0  # now_s of the last accepted update (48 bits)
        self.pl = 0  # poll exponent of the last accepted update (0 after reset)
        self.fresh = True  # no mu yet: the next integral term is 0
        self.slw = False  # SPIK: slewing a persistent large offset
        self.cnt = 0  # processed updates in FREQ (saturating at FREQ_N)
        self.stp = 0  # last step word (96 bits)
        # Statistics (not in the RTL).
        self.steps = 0
        self.last_kind = None

    # -- helpers shared with the RTL description

    @staticmethod
    def poll_clamp(p: int) -> int:
        return max(POLL_MIN, min(POLL_MAX, p))

    def elapsed(self, now: int) -> int:
        """el = (now >> 16) - tl, modulo 2^48 (units 2^-16 s)."""
        return ((u64(now) >> 16) - self.tl) & M48

    def outputs(self, now: int):
        """(trim, synced) of the unit while `time >> 32 == now` (held)."""
        el = self.elapsed(now)
        expired = el >= (1 << (16 + self.pl))
        trim = self.freq if expired else self.tr
        recent = el < (1 << (16 + self.pl + RECENT_SHIFT))
        synced = self.st in (FREQ, SYNC) and recent
        return trim, synced

    # -- the event

    def update(self, valid: bool, offset: int, jitter: int, now: int, poll: int) -> dict:
        """One `Selected` event, popped while `time >> 32 == now`.

        Returns the outputs once the unit is idle again (time still `now`):
        trim, step, step_en (True when this event commanded a step), state,
        synced, and `kind` (the decision: ignore/step/process).
        """
        offset, jitter, now = u64(offset), u64(jitter), u64(now)
        step_en = False
        kind = "ignore"
        if valid:
            sg = offset >> 63
            a = (-offset) & M64 if sg else offset
            big = a > STEPT_Q
            huge = a > PANICT_Q
            spk = a > max(SGATE * jitter, SPKMIN_Q)
            now_s = now >> 16
            el = (now_s - self.tl) & M48
            pn = self.poll_clamp(poll)
            m = min(el >> pn, 1 << 16)
            st = self.st
            proc = None  # (ki, integral enabled)
            if st == NSET:
                if big:
                    kind = "step"
                    self.stp = (s64(offset) << 32) & M96
                    step_en = True
                    self.steps += 1
                    self.tl = (now_s + (s64(offset) >> 16)) & M48
                    self.tr = self.freq
                    self.fresh = True
                    self.st, self.cnt, self.pl = FREQ, 0, pn
                else:
                    proc = (KI_FREQ, not self.fresh)
                    self.st, self.cnt = FREQ, 0
            elif st in (FREQ, SYNC):
                if huge:
                    self.st = PANIC
                elif big:
                    self.st = SPIK
                elif st == SYNC and spk:
                    self.st = SPIK
                elif st == SYNC:
                    proc = (KI_SYNC, not self.fresh)
                else:  # FREQ
                    proc = (KI_FREQ, not self.fresh)
                    n = 0 if spk else min(self.cnt + 1, FREQ_N)
                    self.cnt = n
                    if n >= FREQ_N:
                        self.st = SYNC
            else:  # SPIK, PANIC (FSET is never entered)
                if huge:
                    self.st = PANIC
                elif big:
                    self.st = SPIK
                    if el >= STEPOUT_S or self.slw:
                        proc = (KI_SYNC, False)
                        self.slw = True
                else:
                    proc = (KI_SYNC, not self.fresh)
                    self.st = SYNC
                    self.slw = False
            if proc is not None:
                kind = "process"
                ki, ien = proc
                self.tl, self.pl = now_s, pn
                self.fresh = False
                self._process(sg, a, m if ien else 0, pn, ki)
        self.last_kind = kind
        trim, synced = self.outputs(now)
        return {
            "trim": trim,
            "step": self.stp,
            "step_en": step_en,
            "state": self.st,
            "synced": synced,
            "kind": kind,
        }

    def _process(self, sg: int, a: int, m: int, pn: int, ki: int):
        # MUL1: r = |offset| * CQ >> CS; SHR: c = r >> pn.
        r = (a * CQ) >> CS
        c = r >> pn
        sm = min(c >> KP, MAXTRIM)
        # MUL2 + FIN: integral term (c * m) >> (16 + ki), clamped.
        dm = min((c * m) >> (16 + ki), DMAX)
        s = -sm if sg else sm
        d = -dm if sg else dm
        f1 = self.freq + d
        # Anti-windup: the integrator moves (in the direction of d) at most
        # up to the point where freq + slew reaches the clamp, and never
        # backwards; so |freq| <= MAXTRIM and the output saturates through
        # the slew only.
        lim = MAXTRIM - sm
        if not sg:
            fn = max(self.freq, lim) if f1 > lim else f1
        else:
            fn = min(self.freq, -lim) if f1 < -lim else f1
        self.freq = fn
        self.tr = clamp_trim(fn + s)


# ---------------------------------------------------------------- closed loop


def q32(x: float) -> int:
    """Seconds -> signed 32.32 word (rounded, two's complement)."""
    return round(x * Q32) & M64


class Plant:
    """The disciplined clock against true time, in floating point.

    ``x`` = local - true (seconds). The local clock runs at
    ``1 + e + trim * TRIM_LSB``; the servo's slew (``tr``) applies for 2^pl
    local seconds after an update, then ``freq``. True time starts at ``t0``
    (an integer number of seconds) so `now` is meaningful for the servo.
    """

    def __init__(self, x0: float, e_ppm: float, t0: int = 3_900_000_000):
        self.x = x0
        self.e = e_ppm * 1e-6
        self.t = 0.0  # true seconds since t0
        self.t0 = t0

    def now(self) -> int:
        """Local time, 32.32 (modulo 2^64)."""
        return (self.t0 * Q32 + round((self.t + self.x) * Q32)) & M64

    def advance(self, servo: Servo, dt: float):
        """Advance true time by dt with the servo's current outputs."""
        while dt > 0:
            el = servo.elapsed(self.now()) / 65536.0
            lim = float(1 << servo.pl)
            if el < lim:
                rate = self.e + servo.tr * TRIM_LSB
                # True time until the slew expires (local elapsed reaches lim).
                h = min(dt, (lim - el) / (1.0 + rate) + 1e-9)
            else:
                rate = self.e + servo.freq * TRIM_LSB
                h = dt
            self.x += rate * h
            self.t += h
            dt -= h

    def step(self, word: int):
        v = word & M96
        if v >> 95:
            v -= 1 << 96
        self.x += v / float(1 << 64)


def simulate(
    x0=0.0,
    e_ppm=0.0,
    poll=6,
    noise=100e-6,
    duration=6 * 3600,
    seed=1,
    spikes=None,
    outage=None,
    jitter=None,
):
    """Closed loop: one measurement per poll interval.

    spikes: {k: extra offset (s)} added to measurement k.
    outage: (k0, k1): measurements k0..k1-1 arrive with valid = 0.
    Returns a list of dicts per event: t, x (true offset of the local clock),
    theta (measured), trim, state, kind, step_en, synced.
    """
    rnd = random.Random(seed)
    servo = Servo()
    plant = Plant(x0, e_ppm)
    T = 1 << poll
    jit = q32(noise if jitter is None else jitter)
    log = []
    k = 0
    while plant.t <= duration:
        meas = -plant.x + rnd.gauss(0.0, noise)
        if spikes and k in spikes:
            meas += spikes[k]
        valid = not (outage and outage[0] <= k < outage[1])
        out = servo.update(valid, q32(meas), jit, plant.now(), poll)
        x_before = plant.x
        if out["step_en"]:
            plant.step(out["step"])
        log.append(
            dict(
                k=k,
                t=plant.t,
                x=x_before,
                theta=meas,
                trim=out["trim"],
                freq=servo.freq,
                state=out["state"],
                kind=out["kind"],
                step_en=out["step_en"],
                synced=out["synced"],
            )
        )
        plant.advance(servo, T)
        k += 1
    return servo, log


def rms(xs):
    xs = list(xs)
    return math.sqrt(sum(v * v for v in xs) / len(xs)) if xs else 0.0


def settling_time(log, bound=1e-3):
    """First time after which |x| stays below bound for the rest of the log."""
    t = None
    for e in log:
        if abs(e["x"]) >= bound:
            t = None
        elif t is None:
            t = e["t"]
    return t


# ---------------------------------------------------------------- test vectors

VECTOR_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vectors", "servo.txt")


def _vector_events():
    """Deterministic scripted scenarios: list of (name, [steps]); a step is
    ("ev", valid, offset_word, jitter_word, now, poll) or ("at", now)."""
    rnd = random.Random(4505)
    t0 = 3_900_000_000 * Q32
    scen = []

    def ev(valid, off_s, now, poll=6, jit_s=100e-6, raw_off=None):
        off = raw_off if raw_off is not None else q32(off_s)
        return ("ev", int(valid), off & M64, q32(jit_s), now & M64, poll)

    # 1. Start-up with a small offset, then a converging sequence.
    s = []
    now = t0
    for i, off in enumerate([0.050, 0.030, 0.012, 0.004, 0.0012, 0.0003, -0.0002, 0.0001]):
        s.append(ev(1, off, now))
        s.append(("at", now + (10 << 32)))
        s.append(("at", now + (63 << 32) + (Q32 >> 1)))
        s.append(("at", now + (64 << 32) + 7))
        now += (64 << 32) + rnd.randrange(Q32 // 8)
    scen.append(("startup_small", s))
    # 2. Start-up step (500 ms), then normal updates.
    s = []
    now = t0
    s.append(ev(1, 0.5, now))
    now += (64 << 32) + (Q32 >> 1)
    for off in [0.0021, 0.0035, 0.0019, 0.0008, -0.0004, 0.0002]:
        s.append(ev(1, off, now))
        now += 64 << 32
    scen.append(("startup_step", s))
    # 3. Start-up from the 1900 era: offset beyond the 32.32 signed range.
    s = []
    true = 3_913_056_000 * Q32 + 0x1234_5678
    s.append(ev(1, 0, 5 * Q32, raw_off=true - 5 * Q32))
    s.append(ev(1, 0, true + (64 << 32), raw_off=q32(0.0005)))
    scen.append(("startup_era", s))
    # 4. Negative start-up step and the -128 ms boundary.
    s = []
    s.append(ev(1, 0, t0, raw_off=-STEPT_Q - 1))
    scen.append(("step_boundary_neg", s))
    s = []
    s.append(ev(1, 0, t0, raw_off=STEPT_Q))
    s.append(ev(1, 0, t0 + (64 << 32), raw_off=-STEPT_Q))
    scen.append(("no_step_at_stept", s))
    # 5. Spikes and the SPIK/PANIC paths, at several polls.
    for poll in (4, 6, 10):
        s = []
        now = t0
        T = (1 << poll) << 32
        seq = [0.002, 0.001, 0.0005, 0.0003, 0.0001, 0.0001, -0.0001, 0.0021, 0.0001,
               -0.0025, -0.0001, 0.3, 0.0002, 0.25, 0.26, 0.24, 0.0001, 2000.0, 0.0002,
               5.0, 4.9, 0.001, -0.0002]
        for i, off in enumerate(seq):
            s.append(ev(1, off, now, poll=poll, jit_s=50e-6 + 1e-5 * i))
            gap = T if off != 0.25 else (901 << 32)
            now += gap + rnd.randrange(Q32 // 4)
        s.append(("at", now))
        scen.append(("spikes_poll%d" % poll, s))
    # 6. valid = 0 events, holdover and slew expiry.
    s = []
    now = t0
    for off in [0.04, 0.02, 0.006, 0.002, 0.0005]:
        s.append(ev(1, off, now))
        now += 64 << 32
    for i in range(6):
        s.append(ev(0, 0.9, now))
        s.append(("at", now))
        now += 100 << 32
    s.append(("at", now + (5000 << 32)))
    s.append(ev(1, 0.0003, now + (5000 << 32)))
    scen.append(("holdover", s))
    # 7. Saturation and anti-windup: a large frequency error.
    s = []
    now = t0
    x = 0.0
    for i in range(40):
        s.append(ev(1, x, now, poll=4))
        x += 0.0006 * 16  # +600 ppm of drift per interval (uncorrectable)
        x = min(x, 0.1)
        now += 16 << 32
    scen.append(("saturation", s))
    # 8. Random offsets (|offset| <= 0.2 s), polls and jitters.
    s = []
    now = t0
    for i in range(300):
        kind = rnd.random()
        if kind < 0.6:
            off = rnd.gauss(0, 0.003)
        elif kind < 0.8:
            off = rnd.uniform(-0.2, 0.2)
        elif kind < 0.9:
            off = rnd.uniform(-1500, 1500)
        else:
            off = rnd.gauss(0, 0.0001)
        poll = rnd.randrange(0, 16)
        valid = rnd.random() > 0.05
        s.append(ev(valid, off, now, poll=poll, jit_s=abs(rnd.gauss(0, 0.001))))
        if rnd.random() < 0.3:
            s.append(("at", now + rnd.randrange(2 << 42)))
        now += rnd.randrange(1 << 43)  # 0..2048 s
    scen.append(("random", s))
    # 9. Extreme words: max positive/negative offsets, huge jitter.
    s = []
    now = t0
    s.append(ev(1, 0.001, now, jit_s=0))
    for w in [(1 << 63) - 1, 1 << 63, (1 << 64) - 1, 1, PANICT_Q, PANICT_Q + 1,
              (-PANICT_Q) & M64, (-PANICT_Q - 1) & M64, STEPT_Q + 1]:
        now += 64 << 32
        s.append(("ev", 1, w, M64, now, 6))
        now += 64 << 32
        s.append(("ev", 1, w, 0, now, 6))
    now += 1000 << 32
    s.append(("ev", 1, (1 << 41) | 12345, 0, now, 6))
    s.append(("ev", 1, (-(1 << 41)) & M64, 0, now + (1000 << 32), 6))
    scen.append(("extreme", s))
    return scen


def make_vectors():
    """Lines of vectors/servo.txt.

    Format (hex without 0x, one record per line, '#' comments):
      reset <name>
      ev <valid> <offset64> <jitter64> <now64> <poll> <trim32> <step96> <step_en> <state> <synced>
      at <now64> <trim32> <synced>
    `ev`: drive `time = now << 32`, push one Selected, wait for idle; then the
    outputs must equal the rest of the line. `at`: drive `time = now << 32`
    (held) and compare trim and synced.
    """
    lines = ["# servo vectors (tools/golden/ntpclient/servo.py), format in make_vectors()"]
    for name, steps in _vector_events():
        sv = Servo()
        lines.append("reset %s" % name)
        for stp in steps:
            if stp[0] == "ev":
                _, valid, off, jit, now, poll = stp
                o = sv.update(bool(valid), off, jit, now, poll)
                lines.append(
                    "ev %d %016x %016x %016x %d %08x %024x %d %d %d"
                    % (valid, off, jit, now, poll, o["trim"] & 0xFFFFFFFF, o["step"],
                       int(o["step_en"]), o["state"], int(o["synced"]))
                )
            else:
                now = stp[1] & M64
                trim, synced = sv.outputs(now)
                lines.append("at %016x %08x %d" % (now, trim & 0xFFFFFFFF, int(synced)))
    return lines


def write_vectors(path=VECTOR_PATH):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write("\n".join(make_vectors()) + "\n")


if __name__ == "__main__":
    write_vectors()
    print("wrote", VECTOR_PATH)
