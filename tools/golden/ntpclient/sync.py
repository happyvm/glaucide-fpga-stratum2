"""Reference model of rtl/src/sync_state.rs (task T4.6), standard library only.

Pure functions of the last usable `Selected`, the clock and the servo flag;
the formulas and roundings are those of the module header of sync_state.rs
(RFC 5905 sections 7.3 and 11). Times are integers in 32.32 seconds, root
delay / dispersion in 16.16.
"""
from dataclasses import dataclass, replace
from typing import Optional

PHI_Q32 = 64425                 # 15 ppm in 2^-32 s per second (rtl/src/ntp_types.rs)
SYNC_TIMEOUT_S = 1024
SYNC_TIMEOUT_Q32 = SYNC_TIMEOUT_S << 32
MAX_SRC_STRATUM = 14
M32 = 0xFFFFFFFF
M64 = (1 << 64) - 1


@dataclass(frozen=True)
class Selected:
    valid: bool = False
    peer: int = 0
    jitter: int = 0
    stratum: int = 0
    root_delay: int = 0
    root_disp: int = 0
    delay: int = 0
    disp: int = 0
    t4: int = 0


@dataclass(frozen=True)
class State:
    """The last usable Selected (`S` of the module header) and its update time."""
    v: bool = False
    sel: Selected = Selected()
    upd: int = 0
    ref_id: int = 0       # held outputs while unsynchronised
    ref_ts: int = 0
    root_delay: int = 0
    root_disp: int = 0


@dataclass(frozen=True)
class Out:
    synced: bool
    stratum: int
    ref_id: int
    ref_ts: int
    root_delay: int
    root_disp: int
    leap: int


def sat32(x: int) -> int:
    return min(x, M32)


def ceil16(x: int) -> int:
    """32.32 to 16.16, rounding up."""
    return (x + 0xFFFF) >> 16


def ceil48(x: int) -> int:
    """2^-64 s to 16.16, rounding up."""
    return (x + (1 << 48) - 1) >> 48


def prec_floor(pr: int) -> int:
    """2^precision in 32.32; `pr` is the signed log2 as an 8-bit two's complement."""
    p = pr - 256 if pr & 0x80 else pr
    if p >= 0:
        return 1 << 32
    if p < -32:
        return 1
    return 1 << (32 + p)


def usable(sel: Selected) -> bool:
    return sel.valid and 1 <= sel.stratum <= MAX_SRC_STRATUM


def pop(state: State, sel: Selected, now: int) -> State:
    """A Selected is popped at time `now` (32.32)."""
    if not usable(sel):
        return replace(state, v=False)
    return replace(state, v=True, sel=sel, upd=now)


def fresh(state: State, now: int) -> bool:
    return now < state.upd or now - state.upd <= SYNC_TIMEOUT_Q32


def root_delay_of(sel: Selected) -> int:
    return sat32(sel.root_delay + sat32(ceil16(sel.delay)))


def root_disp_of(sel: Selected, now: int, precision: int) -> int:
    age = now - sel.t4 if now >= sel.t4 else 0
    j = max(sel.jitter, prec_floor(precision))
    total = ((sel.disp + j) << 32) + PHI_Q32 * age   # units of 2^-64 s
    return sat32(sel.root_disp + sat32(ceil48(total)))


def outputs(state: State, now: int, servo_synced: bool, precision: int, peer_refs) -> (Out, State):
    """Steady-state outputs and the new held values (ref_id, ref_ts, roots)."""
    ok = state.v and servo_synced and fresh(state, now)
    if ok:
        s = state.sel
        held = replace(state, ref_id=peer_refs[s.peer], ref_ts=state.upd,
                       root_delay=root_delay_of(s),
                       root_disp=root_disp_of(s, now, precision))
        return Out(True, s.stratum + 1, held.ref_id, held.ref_ts, held.root_delay,
                   held.root_disp, 0), held
    return Out(False, 16, state.ref_id, state.ref_ts, state.root_delay,
               state.root_disp, 3), state
