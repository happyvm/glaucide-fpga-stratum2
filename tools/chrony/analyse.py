# SPDX-License-Identifier: Apache-2.0
"""tools/chrony/analyse.py: checks the results of tools/chrony/run.sh.

    python3 -I tools/chrony/analyse.py out/chrony/results [SCENARIO...]

Reads, per scenario directory: chronyd.log, chronyc-*.txt, measurements.log
(chrony's view) and bridge.log, exchanges.csv (the bench's ground truth:
every request and reply payload, the virtual instants T1/T4 chronyd sees, the
SFD instants on the MII). Prints the findings and asserts the expected
behaviour; exits 1 if a check fails. The summary is also written to
<results>/summary.txt.

Ground truth used for the offset (scenario `offset`, chronyd `noselect`, so
its offsets are raw, never corrected by its own clock discipline):
  theta = ((T2 - T1) + (T3 - T4)) / 2       (RFC 5905, what chrony logs)
  theta_geom = ((rx_sfd - T1) + (tx_sfd - T4)) / 2
           the part due to WHERE the bench's T1/T4 sit: T1 is the instant the
           request's preamble starts, T4 the instant the reply's last nibble
           left the MII (chronyd reads its clock when the datagram arrives),
           whereas the design stamps the SFD of each frame;
  theta - theta_geom = (e2 + e3) / 2, e2 = T2 - rx_sfd, e3 = T3 - tx_sfd:
           the design's own timestamp errors, what chrony would report with
           SFD-referenced client stamps.
"""
import csv
import math
import os
import re
import statistics
import sys

OUT = []
FAILS = []


def say(s=""):
    OUT.append(s)
    print(s)


def check(cond, what):
    say(("  PASS  " if cond else "  FAIL  ") + what)
    if not cond:
        FAILS.append(what)


def read(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return ""


BAD_LOG = re.compile(r"Invalid|Unacceptable|Rejected|bad|error|failed", re.I)


def chronyd_log(d):
    lines = read(os.path.join(d, "chronyd.log")).splitlines()
    msgs = [l.split(" ", 1)[1] if " " in l else l for l in lines]
    return lines, msgs


def source_line(d):
    for l in read(os.path.join(d, "chronyc-sources.txt")).splitlines():
        if l.startswith("^"):
            f = l.split()
            return {
                "state": l[1],
                "stratum": int(f[2]),
                "poll": int(f[3]),
                "reach": f[4],
                "line": l.strip(),
            }
    return None


def ntpdata(d):
    r = {}
    for l in read(os.path.join(d, "chronyc-ntpdata.txt")).splitlines():
        if ":" in l:
            k, v = l.split(":", 1)
            r[k.strip()] = v.strip()
    return r


def measurements(d):
    rows = []
    for l in read(os.path.join(d, "measurements.log")).splitlines():
        if not l[:4].isdigit():
            continue
        f = l.split()
        rows.append(
            {
                "time": f[1],
                "leap": f[3],
                "stratum": int(f[4]),
                "tests": " ".join(f[5:8]),
                "offset": float(f[11]),
                "delay": float(f[12]),
                "disp": float(f[13]),
            }
        )
    return rows


def exchanges(d):
    p = os.path.join(d, "exchanges.csv")
    if not os.path.exists(p):
        return []
    with open(p, newline="") as f:
        return list(csv.DictReader(f))


def hexdump(d):
    """(kind, n, virtual s, port, length, payload bytes) of bridge.log."""
    r = []
    pat = re.compile(r"^(req|rep)  #(\d+) at ([\d.]+) (?:from|to) :(\d+) \((\d+) bytes\): ([0-9a-f]*)")
    for l in read(os.path.join(d, "bridge.log")).splitlines():
        m = pat.match(l)
        if m:
            r.append((m[1], int(m[2]), float(m[3]), int(m[4]), int(m[5]), bytes.fromhex(m[6])))
    return r


def bridge_done(d):
    for l in read(os.path.join(d, "bridge.log")).splitlines():
        if l.startswith("chrony_bridge: done:"):
            return l
    return ""


def stats(xs):
    return statistics.mean(xs), (statistics.stdev(xs) if len(xs) > 1 else 0.0)


# ------------------------------------------------------------ replies

GOLDEN = bytes.fromhex("0000033300000666c0000263ecc58cc000000000")


def check_replies(d, synced):
    """Every answered request: the reply is 48 bytes, byte-exact against
    the server's rules (ntp_server.rs header) but for T2/T3, T2 <= T3."""
    ex = exchanges(d)
    bad = []
    for e in ex:
        q = bytes.fromhex(e["req_hex"])
        p = bytes.fromhex(e["reply_hex"])
        vn = (q[0] >> 3) & 7
        li = 0 if synced else 3
        want = bytes([(li << 6) | (min(vn, 4) << 3) | 4, 2 if synced else 16, q[2], 0xE8])
        want += GOLDEN if synced else bytes(20)
        want += q[40:48]
        ok = len(p) == 48 and p[:32] == want and p[32:40] <= p[40:48]
        if not ok:
            bad.append((e["idx"], p.hex()))
    check(ex and not bad, f"{len(ex)} replies byte-exact (48 bytes, header fields, origin = request transmit, T2 <= T3)"
          + (f": {bad[:2]}" if bad else ""))
    return ex


def common(d, name):
    lines, msgs = chronyd_log(d)
    say(f"  bridge: {bridge_done(d).replace('chrony_bridge: done: ', '')}")
    bad = [m for m in msgs if BAD_LOG.search(m)]
    check(not bad, "no Invalid/Unacceptable/Rejected/error line in chronyd's log" + (f": {bad[:3]}" if bad else ""))
    return lines, msgs


# ------------------------------------------------------------ scenarios


def sc_fixed(d):
    common(d, "fixed")
    src = source_line(d)
    nd = ntpdata(d)
    say(f"  chronyc sources: {src['line'] if src else None}")
    check(src is not None and src["state"] == "*", "chrony selects the source ('*')")
    check(src is not None and src["reach"] == "377", "reach 377")
    check(src is not None and src["stratum"] == 2, "stratum 2 in chronyc sources")
    exp = {
        "Leap status": "Normal",
        "Version": "4",
        "Mode": "Server",
        "Stratum": "2",
        "Precision": "-24 (0.000000060 seconds)",
        "Root delay": "0.012497 seconds",
        "Root dispersion": "0.024994 seconds",
        "Reference ID": "C0000263 ()",
        "Reference time": "Mon Nov 17 12:00:00 2025",
        "NTP tests": "111 111 1111",
        "Interleaved": "No",
    }
    for k, v in exp.items():
        check(nd.get(k) == v, f"ntpdata {k}: {nd.get(k)!r} (expected {v!r})")
    for k in ("Poll interval", "Offset", "Peer delay", "Peer dispersion", "Response time", "Jitter asymmetry",
              "Total TX", "Total RX", "Total valid RX", "Total good RX"):
        say(f"         ntpdata {k}: {nd.get(k)}")
    check(nd.get("Total TX") == nd.get("Total good RX"), "every request answered and good")
    tr = read(os.path.join(d, "chronyc-tracking.txt"))
    for l in tr.splitlines():
        if l.split(":")[0].strip() in ("Reference ID", "Stratum", "System time", "Frequency", "Root delay",
                                       "Root dispersion", "Leap status"):
            say(f"         tracking {l.strip()}")
    ss = read(os.path.join(d, "chronyc-sourcestats.txt")).splitlines()
    if ss:
        say(f"         sourcestats {ss[-1].strip()}")
    ex = check_replies(d, True)
    check(len(ex) >= 30, f"{len(ex)} polls (>= 30)")


def sc_offset(d, min_n=30):
    common(d, "offset")
    ex = check_replies(d, True)
    ms = measurements(d)
    check(len(ms) == len(ex) and len(ms) >= min_n,
          f"{len(ms)} chrony samples for {len(ex)} exchanges (>= {min_n})")
    n = min(len(ms), len(ex))
    co = [ms[i]["offset"] * 1e9 for i in range(n)]
    cd = [ms[i]["delay"] * 1e9 for i in range(n)]
    bo = [float(ex[i]["offset_ns"]) for i in range(n)]
    bd = [float(ex[i]["delay_ns"]) for i in range(n)]
    geom = []
    for e in ex[:n]:
        t1, t4 = int(e["t1_ns"]), int(e["t4_ns"])
        rx, tx = int(e["rx_sfd_ns"]), int(e["tx_sfd_ns"])
        geom.append(((rx - t1) + (tx - t4)) / 2)
    design = [co[i] - geom[i] for i in range(n)]
    e2 = [float(e["e2_ns"]) for e in ex[:n]]
    e3 = [float(e["e3_ns"]) for e in ex[:n]]
    resid = [co[i] - bo[i] for i in range(n)]
    resd = [cd[i] - bd[i] for i in range(n)]
    m, s = stats(co)
    say(f"  chrony offset (theta, measurements.log): mean {m:+.1f} ns, sigma {s:.1f} ns over {n} polls "
        f"(min {min(co):+.0f}, max {max(co):+.0f})")
    m2, s2 = stats(cd)
    say(f"  chrony delay: mean {m2:.1f} ns, sigma {s2:.1f} ns")
    mg, sg = stats(geom)
    say(f"  bench geometry theta_geom = ((rx_sfd-T1)+(tx_sfd-T4))/2: mean {mg:+.1f} ns, sigma {sg:.1f} ns")
    md, sd = stats(design)
    say(f"  chrony offset - theta_geom (= design stamp error (e2+e3)/2 as chrony sees it): "
        f"mean {md:+.1f} ns, sigma {sd:.1f} ns")
    say(f"  design stamp errors (bench): e2 = T2 - rx SFD {stats(e2)[0]:+.2f} ns, e3 = T3 - tx SFD "
        f"{stats(e3)[0]:+.2f} ns (range {min(e3):+.2f}..{max(e3):+.2f})")
    check(max(abs(x) for x in resid) <= 1.5,
          f"chrony's offset = the bench's theta within its log resolution (max |diff| {max(abs(x) for x in resid):.2f} ns)")
    check(max(abs(x) for x in resd) <= 1.5,
          f"chrony's delay = the bench's delta (max |diff| {max(abs(x) for x in resd):.2f} ns)")
    check(abs(md - (stats(e2)[0] + stats(e3)[0]) / 2) <= 1.5, "offset minus geometry = (e2+e3)/2 (design error)")
    rx_part = stats([int(e["rx_sfd_ns"]) - int(e["t1_ns"]) for e in ex[:n]])[0]
    tx_part = stats([int(e["t4_ns"]) - int(e["tx_sfd_ns"]) for e in ex[:n]])[0]
    req_len = int(ex[0]["len"])
    frame = 14 + 20 + 8 + req_len + 4
    say(f"  physical: request preamble+SFD before rx SFD {rx_part:.0f} ns (15 nibbles = 600 ns); "
        f"reply after tx SFD until delivery {tx_part:.0f} ns ({frame} bytes x 80 ns = {frame * 80} ns)")
    say(f"  so a request/reply of {frame} bytes takes {frame * 80 + 640} ns on the wire each way (with preamble)")


def sc_noff(d):
    """The same measurement with every cycle simulated (no fast-forward)."""
    sc_offset(d, 8)


def sc_standalone(d):
    lines, msgs = chronyd_log(d)
    say(f"  bridge: {bridge_done(d).replace('chrony_bridge: done: ', '')}")
    src = source_line(d)
    nd = ntpdata(d)
    say(f"  chronyc sources: {src['line'] if src else None}")
    check(src is not None and src["state"] != "*" and src["state"] != "+", "chrony does NOT select the source")
    check(src is not None and src["reach"] == "0", "reach 0: chrony counts no reply of an unsynchronised server")
    for k, v in (("Leap status", "Not synchronised"), ("Stratum", "16"), ("Reference ID", "00000000 ()"),
                 ("Mode", "Server"), ("Total good RX", "0")):
        check(nd.get(k) == v, f"ntpdata {k}: {nd.get(k)!r} (expected {v!r})")
    say(f"         ntpdata NTP tests: {nd.get('NTP tests')} (test 6 fails: leap 3 / stratum 16, "
        f"'not synchronised'; tests A-D not run, so no offset/delay is computed); Total RX {nd.get('Total RX')}, valid {nd.get('Total valid RX')}")
    cs = [m for m in msgs if "Can't synchronise" in m]
    say(f"  chronyd log: {cs[0] if cs else None!r} ({len(cs)} time(s))")
    check(bool(cs) and not any("Selected source" in m for m in msgs), "log says it cannot synchronise, never selects")
    check_replies(d, False)


def window_counts(d):
    hd = hexdump(d)
    reqs = [(n, t, p) for k, n, t, _, _, p in hd if k == "req"]
    reps = {p[24:32] for k, _, _, _, _, p in hd if k == "rep"}
    answered = [(t, p[40:48] in reps) for _, t, p in reqs]
    return reqs, answered


def limiter(d, lim):
    """Requests per NTP-second window (the design's windows are aligned on
    whole seconds of ITS clock = virtual time, T0 included) vs answered."""
    reqs, answered = window_counts(d)
    t0 = None
    for l in read(os.path.join(d, "bridge.log")).splitlines():
        m = re.search(r"T0 (\d+) ns", l)
        if m:
            t0 = int(m[1])
    win = {}
    for t, ok in answered:
        w = (t0 + int(t * 1e9)) // 1_000_000_000
        a, b = win.get(w, (0, 0))
        win[w] = (a + 1, b + ok)
    over = sum(max(0, a - lim) for a, _ in win.values())
    extra = sum(max(0, b - lim) for _, b in win.values())
    under = sum(1 for a, b in win.values() if b < min(a, lim))
    tot = len(answered)
    ans = sum(ok for _, ok in answered)
    say(f"  {tot} requests in {len(win)} one-second windows, {ans} answered; {over} over the limit of {lim}/s, "
        f"of which {extra} answered by the 1/16 leak (expected {over / 16:.1f} +- {math.sqrt(over * 15 / 256):.1f})")
    check(under == 0, "within its limit every request is answered (no window answers fewer than min(n, limit))")
    return tot, ans, over, extra


def sc_ratelimit(d, lim=1):
    lines, msgs = common(d, "ratelimit")
    src = source_line(d)
    nd = ntpdata(d)
    say(f"  chronyc sources: {src['line'] if src else None}")
    limiter(d, lim)
    sel = sum("Selected source" in m for m in msgs)
    lost = sum("Can't synchronise" in m for m in msgs)
    say(f"  chronyd: 'Selected source' {sel} time(s), 'Can't synchronise' {lost} time(s); "
        f"Total TX {nd.get('Total TX')}, RX {nd.get('Total RX')}, poll {nd.get('Poll interval')}")
    reqs, _ = window_counts(d)
    if len(reqs) > 1:
        gaps = [round(reqs[i + 1][1] - reqs[i][1], 3) for i in range(min(8, len(reqs) - 1))]
        say(f"  first request intervals (virtual s): {gaps}")
    return src, msgs


def sc_rl_iburst(d):
    src, msgs = sc_ratelimit(d, 1)


def sc_burst(d):
    src, msgs = sc_ratelimit(d, 8)


def sc_xleave(d):
    common(d, "xleave")
    nd = ntpdata(d)
    src = source_line(d)
    say(f"  chronyc sources: {src['line'] if src else None}")
    check(nd.get("Interleaved") == "No", "chrony falls back to basic mode (Interleaved: No)")
    check(nd.get("Total TX") == nd.get("Total good RX"), "every request answered and good")
    hd = hexdump(d)
    reqs = [p for k, _, _, _, _, p in hd if k == "req"]
    nz = sum(1 for p in reqs if p[24:32] != bytes(8))
    say(f"  {nz}/{len(reqs)} requests carry a non-zero Origin (interleaved-mode requests: Origin = the previous "
        f"reply's Receive timestamp) - the server ignores the field (rule N9 copies Transmit only)")
    check_replies(d, True)


def sc_extfield(d):
    common(d, "extfield")
    nd = ntpdata(d)
    hd = hexdump(d)
    ql = sorted({n for k, _, _, _, n, _ in hd if k == "req"})
    rl = sorted({n for k, _, _, _, n, _ in hd if k == "rep"})
    q = next(p for k, _, _, _, _, p in hd if k == "req")
    say(f"  request lengths {ql} (extension field type {q[48:50].hex()} length {int.from_bytes(q[50:52], 'big')}), "
        f"reply lengths {rl}")
    check(ql and min(ql) > 48 and rl == [48], "requests with an extension field get a 48-byte reply")
    check(nd.get("Total TX") == nd.get("Total good RX"), "chrony accepts the replies without the field")
    check_replies(d, True)


def sc_version3(d):
    common(d, "version3")
    nd = ntpdata(d)
    hd = hexdump(d)
    q = next(p for k, _, _, _, _, p in hd if k == "req")
    r = next(p for k, _, _, _, _, p in hd if k == "rep")
    say(f"  request byte 0 = {q[0]:#04x} (VN {(q[0] >> 3) & 7}), reply byte 0 = {r[0]:#04x} (VN {(r[0] >> 3) & 7})")
    check(nd.get("Version") == "3" and nd.get("Total TX") == nd.get("Total good RX"), "NTPv3 answered in v3, accepted")
    check_replies(d, True)


def sc_version2(d):
    lines, msgs = chronyd_log(d)
    say(f"  bridge: {bridge_done(d).replace('chrony_bridge: done: ', '')}")
    nd = ntpdata(d)
    src = source_line(d)
    hd = hexdump(d)
    q = next(p for k, _, _, _, _, p in hd if k == "req")
    reps = [p for k, _, _, _, _, p in hd if k == "rep"]
    say(f"  request byte 0 = {q[0]:#04x} (VN {(q[0] >> 3) & 7}); {len(reps)} replies; "
        f"chronyc sources: {src['line'] if src else None}")
    check(not reps, "NTPv2 requests get no reply (rule N3: VN 3 or 4 only)")
    check(src is not None and src["reach"] == "0" and src["state"] != "*", "chrony: unreachable, not selected")


def sc_sync_loss(d):
    """The server is synchronised (stratum 2) for 40 virtual s, then reports stratum 16 / LI 3.
    chronyd must select it first, then stop: after the loss no source is selected, reach
    drains to 0 and chronyd's tracking does not stay on it."""
    lines, msgs = chronyd_log(d)
    say(f"  bridge: {bridge_done(d).replace('chrony_bridge: done: ', '')}")
    src = source_line(d)
    say(f"  chronyc sources: {src['line'] if src else None}")
    check(any("Selected source" in m for m in msgs), "chronyd selected the source while it was synchronised")
    check(src is not None and src["state"] not in "*+", "at the end chrony no longer selects it")
    check(src is not None and src["reach"] == "0", "reach drained to 0 (unsynchronised replies are not counted)")
    reps = [(t, p) for k, _, t, _, _, p in hexdump(d) if k == "rep"]
    before = [p for t, p in reps if t < 39]
    after = [p for t, p in reps if t > 42]
    check(len(before) >= 30 and all(p[1] == 2 and p[0] >> 6 == 0 for p in before),
          f"{len(before)} replies before the loss: stratum 2, LI 0")
    check(len(after) >= 30 and all(p[1] == 16 and p[0] >> 6 == 3 for p in after),
          f"{len(after)} replies after the loss: stratum 16, LI 3")
    sel = [m for m in msgs if "Selected source" in m]
    say(f"  chronyd log: {len(sel)} selection(s); last non-selection message: "
        f"{[m for m in msgs if 'synchronise' in m or 'No selectable' in m][-1:]}")


SCEN = {
    "fixed": sc_fixed,
    "offset": sc_offset,
    "standalone": sc_standalone,
    "ratelimit": sc_ratelimit,
    "rl-iburst": sc_rl_iburst,
    "burst": sc_burst,
    "xleave": sc_xleave,
    "extfield": sc_extfield,
    "version3": sc_version3,
    "version2": sc_version2,
    "noff": sc_noff,
    "sync-loss": sc_sync_loss,
}


def main():
    res = sys.argv[1]
    names = sys.argv[2:] or [n for n in SCEN if os.path.isdir(os.path.join(res, n))]
    for n in names:
        d = os.path.join(res, n)
        say(f"== {n}")
        if not os.path.isdir(d):
            check(False, f"{d} missing")
            continue
        SCEN[n](d)
    say()
    say("all checks passed" if not FAILS else f"{len(FAILS)} check(s) FAILED")
    with open(os.path.join(res, "summary.txt"), "w") as f:
        f.write("\n".join(OUT) + "\n")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
