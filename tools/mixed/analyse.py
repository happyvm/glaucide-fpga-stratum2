# SPDX-License-Identifier: Apache-2.0
"""tools/mixed/analyse.py: checks the results of tools/mixed/run.sh (ntpperf load
and chronyd polling the same simulated server, in the same simulation).

    python3 -I tools/mixed/analyse.py out/mixed/results [JOB...]

Per job directory: chrony.csv (the bench's ground truth for every chronyd
exchange), bridge.log, ntpperf.txt, requests.csv (ntpperf's requests),
chronyd.log, chronyc-*.txt, measurements.log, chrony.conf. Reuses the parsers of
tools/chrony/analyse.py and tools/ntpperf/analyse.py (loaded by path). Prints the
findings, asserts the expected behaviour, writes <results>/summary-mixed.txt and
exits 1 if a check fails.

What is checked
  chronyd, whatever the load of ntpperf:
    * every request is answered: bridge counters (requests = replies, 0 queue
      drops, 0 unmatched, 0 unanswered), chronyd's Total TX = good RX, every reply
      byte-exact (tools/chrony/analyse.py check_replies);
    * no reply is delayed by the load: the reply's last nibble leaves the MII
      T4 - wire_ns = 600 + 10010 + 7605 = 18215 ns after the start of its request
      on the wire (SFD at +600 ns, T3 - T2 = SFD to SFD 10010 ns, then 94 bytes);
      the only delay is the wait of the REQUEST in the sender's queue (wire_ns - T1);
    * job `select` (chronyd selecting): '*' in `chronyc sources`, reach 377,
      stratum 2, 'Selected source', NTP tests 111 111 1111; chronyd's delay per
      poll = the bench's delay (<= 1.5 ns) and = 8195 ns + queue wait;
    * job `offset` (chronyd noselect: raw offsets, never corrected by chronyd's
      own discipline): the ground-truth method of the `offset` scenario of
      tools/chrony/analyse.py: theta - theta_geom = (e2 + e3)/2 = +4.0 ns (the
      design's own stamp error) with theta_geom = ((rx_sfd - T1) + (tx_sfd - T4))/2
      INCLUDING the queue wait; chronyd's offset = the bench's within 1.5 ns.
  ntpperf: no loss, no invalid reply, response time T3 - T2 = 10020 ns sigma 0,
  T3 - TX SFD = +9.88 ns, T2 - RX SFD = -0.14 ns as without chronyd; the
  chronyd replies show the same values.
"""
import csv
import importlib.util
import os
import re
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


ca = load("chrony_analyse", os.path.join(HERE, "..", "chrony", "analyse.py"))
na = load("ntpperf_analyse", os.path.join(HERE, "..", "ntpperf", "analyse.py"))

OUT = []
FAILS = []
ca.OUT = OUT
ca.FAILS = FAILS

# Idle reference (docs/chrony-interop.md 5.3, docs/ntpperf-results.md 3.1/3.2).
IDLE_RESP = 10020.0  # T3 - T2, ns
IDLE_E2 = -0.14  # T2 - RX SFD
IDLE_E3 = 9.88  # T3 - TX SFD
IDLE_DESIGN = 4.0  # theta - theta_geom as chrony logs it
WIRE_TO_T4 = 18215  # ns, request preamble on the wire -> last nibble of the reply


def say(s=""):
    OUT.append(s)
    print(s)


def check(cond, what):
    say(("  PASS  " if cond else "  FAIL  ") + what)
    if not cond:
        FAILS.append(what)


def stats(xs):
    return statistics.mean(xs), (statistics.stdev(xs) if len(xs) > 1 else 0.0)


def chrony_rows(d):
    p = os.path.join(d, "chrony.csv")
    if not os.path.exists(p):
        return []
    with open(p, newline="") as f:
        return list(csv.DictReader(f))


# chrony's helpers read exchanges.csv: ours is chrony.csv.
ca.exchanges = chrony_rows


def bridge_text(d):
    return ca.read(os.path.join(d, "bridge.log"))


def chrony_counters(text):
    m = re.search(r"chronyd: requests (\d+), replies (\d+), unanswered (\d+), queue drops (\d+), "
                  r"unmatched replies (\d+), deliveries not taken (\d+)", text)
    return dict(zip(("requests", "replies", "unanswered", "qdrops", "unmatched", "stuck"),
                    map(int, m.groups()))) if m else None


def job(d, name):
    say(f"== {name}")
    conf = ca.read(os.path.join(d, "chrony.conf"))
    select = "noselect" not in conf
    cmd = ca.read(os.path.join(d, "cmd.txt")).strip()
    nt = cmd.split(" -- ")[1].split()[1:] if " -- " in cmd else []
    say(f"  ntpperf {' '.join(nt[6:])}; chronyd {[l for l in conf.splitlines() if l.startswith('server')][0]}")
    text = bridge_text(d)
    b = na.read_bridge(os.path.join(d, "bridge.log"))
    m = re.search(r"done \(.*?\): virt ([\d.]+) s, wall ([\d.]+) s, simulated (\d+) cycles", text)
    if m:
        say(f"  bridge: {m.group(1)} virtual s, {m.group(2)} s of wall time, {int(m.group(3)):,} simulated cycles, "
            f"skipped {b.get('skipped', 0):,}")
    check(not b.get("errors"), f"no bridge error ({b.get('errors')})")

    # ------------------------------------------------ ntpperf side
    rows, notes = na.read_table(os.path.join(d, "ntpperf.txt"))
    check(len(rows) == 1, f"ntpperf printed one rate point ({len(rows)})")
    if rows:
        r = rows[0]
        say(f"  ntpperf: rate {r['rate']} req/s, {r['clients']} clients, lost {r['lost']:.2f} %, invalid "
            f"{r['invalid']:.2f} %, response time min/mean/max/sd {r.get('min')}/{r.get('mean')}/{r.get('max')}/{r.get('sd')} ns")
        below = r["rate"] < na.LINE_RATE
        check(below and r["lost"] == 0.0 and r["invalid"] == 0.0,
              f"ntpperf: 0.00 % lost, 0.00 % invalid ({r['rate']} req/s < line rate {na.LINE_RATE:.0f})")
        check(r.get("min") == r.get("mean") == r.get("max") == int(IDLE_RESP) and r.get("sd") == 0,
              f"ntpperf: response time T3-T2 = +{IDLE_RESP:.0f} ns, sigma 0, as without chronyd")
    check(b.get("qdrops") == 0 and b.get("requests", 0) > 0 and b.get("answered") == b.get("requests"),
          f"ntpperf requests: {b.get('requests')} sent, {b.get('answered')} answered, {b.get('qdrops')} queue drops")
    check(b.get("unmatched") == 0 and b.get("other") == 0 and b.get("bad") == 0,
          "no unmatched, other or bad frame on the wire")
    s = b.get("stats", {})
    if s:
        say(f"  bridge truth (all ntpperf replies): T3-T2 {s['resp']['mean']:+.2f} ns (sd {s['resp']['sd']:.2f}), "
            f"T2-RX SFD {s['e2']['mean']:+.2f}, T3-TX SFD {s['e3']['mean']:+.2f} ns (min {s['e3']['min']:+.1f}, "
            f"max {s['e3']['max']:+.1f}); n = {s['resp']['n']}")
        check(abs(s["resp"]["mean"] - IDLE_RESP) <= 0.1 and s["resp"]["sd"] <= 0.2,
              f"ntpperf replies: T3-T2 {s['resp']['mean']:+.2f} ns (idle {IDLE_RESP:.0f})")
        check(abs(s["e3"]["mean"] - IDLE_E3) <= 0.15 and abs(s["e2"]["mean"] - IDLE_E2) <= 0.15,
              f"ntpperf replies: T2 error {s['e2']['mean']:+.2f}, T3 error {s['e3']['mean']:+.2f} ns "
              f"(idle {IDLE_E2:+.2f} / {IDLE_E3:+.2f})")

    # Same ntpperf run without chronyd (run.sh --base): is ntpperf's own log unchanged?
    bd_ = d + "-base"
    if os.path.exists(os.path.join(bd_, "requests.csv")):
        with open(os.path.join(d, "requests.csv"), newline="") as f1, \
                open(os.path.join(bd_, "requests.csv"), newline="") as f2:
            ra, rb = list(csv.DictReader(f1)), list(csv.DictReader(f2))
        diff = [i for i in range(min(len(ra), len(rb))) if ra[i] != rb[i]]
        same_cols = ("send_ns", "src_ip", "sport", "len", "bucket", "answered", "stratum", "resp_ns", "e2_ns", "e3_ns")
        shifted = [i for i in diff if any(ra[i][k] != rb[i][k] for k in same_cols)]
        say(f"  ntpperf twin without chronyd: {len(ra)} / {len(rb)} requests, {len(ra) - len(diff)} log lines "
            f"byte-identical, {len(diff)} differ (instants on the wire), {len(shifted)} differ in a send instant, "
            f"source, answer or error column")
        check(len(ra) == len(rb) and not shifted,
              "ntpperf's request log without chronyd: same requests, same answers, same T3-T2 and stamp errors")
        for i in diff[:3]:
            say(f"         line {i}: wire {rb[i]['wire_ns']} -> {ra[i]['wire_ns']}, rx SFD {rb[i]['rx_sfd_ns']} -> "
                f"{ra[i]['rx_sfd_ns']}")

    # ------------------------------------------------ chronyd side
    c = chrony_counters(text)
    ex = chrony_rows(d)
    check(c is not None, "the bridge reports chronyd's counters")
    if c:
        say(f"  chronyd exchanges: requests {c['requests']}, replies {c['replies']}, unanswered {c['unanswered']}, "
            f"queue drops {c['qdrops']}, unmatched {c['unmatched']}, deliveries not taken {c['stuck']}")
        check(c["requests"] == c["replies"] == len(ex) and c["unanswered"] == c["qdrops"] == c["unmatched"] == 0
              and c["stuck"] == 0, "every chronyd request answered, none lost, dropped or unmatched")
    lines, msgs = ca.chronyd_log(d)
    bad = [x for x in msgs if ca.BAD_LOG.search(x)]
    check(not bad, "no Invalid/Unacceptable/Rejected/error line in chronyd's log" + (f": {bad[:3]}" if bad else ""))
    ca.check_replies(d, True)
    nd = ca.ntpdata(d)
    check(nd.get("Total TX") == nd.get("Total good RX") == nd.get("Total RX"),
          f"chronyd's own counters: Total TX {nd.get('Total TX')}, RX {nd.get('Total RX')}, good RX "
          f"{nd.get('Total good RX')}")
    check(nd.get("NTP tests") == "111 111 1111" and nd.get("Leap status") == "Normal" and nd.get("Stratum") == "2",
          f"chronyd's tests: NTP tests {nd.get('NTP tests')}, Leap status {nd.get('Leap status')}, "
          f"Stratum {nd.get('Stratum')}")
    if not ex:
        return
    n = len(ex)
    t1 = [int(e["t1_ns"]) for e in ex]
    t4 = [int(e["t4_ns"]) for e in ex]
    wire = [int(e["wire_ns"]) for e in ex]
    rx = [int(e["rx_sfd_ns"]) for e in ex]
    tx = [int(e["tx_sfd_ns"]) for e in ex]
    qw = [wire[i] - t1[i] for i in range(n)]
    w4 = [t4[i] - wire[i] for i in range(n)]
    e2 = [float(e["e2_ns"]) for e in ex]
    e3 = [float(e["e3_ns"]) for e in ex]
    resp = [(int(e["t3"], 16) - int(e["t2"], 16)) * 1e9 / 2**32 for e in ex]
    bo = [float(e["offset_ns"]) for e in ex]
    bd = [float(e["delay_ns"]) for e in ex]
    gaps = [(t1[i + 1] - t1[i]) / 1e9 for i in range(n - 1)]
    say(f"  {n} chronyd polls, intervals {min(gaps):.2f}..{max(gaps):.2f} s (virtual)" if gaps else f"  {n} polls")
    say(f"  request queue wait (wire start - T1): min {min(qw)} ns, mean {statistics.mean(qw):.0f} ns, max {max(qw)} ns; "
        f"{sum(1 for x in qw if x > 40)} of {n} polls found the wire busy")
    say(f"  T4 - wire start of the request: min {min(w4)}, max {max(w4)} ns (idle: {WIRE_TO_T4})")
    check(max(abs(x - WIRE_TO_T4) for x in w4) <= 40,
          "no chronyd reply delayed: T4 - start of its request on the wire = 18215 ns +- 40 for every poll "
          "(only the request's wait in the sender's queue shifts T1)")
    say(f"  chronyd replies: T3-T2 {stats(resp)[0]:.2f} ns (sd {stats(resp)[1]:.2f}), e2 {stats(e2)[0]:+.2f}, "
        f"e3 {stats(e3)[0]:+.2f} ns (range {min(e3):+.2f}..{max(e3):+.2f})")
    check(max(abs(x - IDLE_RESP) for x in resp) <= 0.3 and abs(stats(e3)[0] - IDLE_E3) <= 0.3
          and abs(stats(e2)[0] - IDLE_E2) <= 0.3, "chronyd replies: T2/T3 errors and T3-T2 as without load")

    ms = ca.measurements(d)
    check(len(ms) == n, f"{len(ms)} chrony samples (measurements.log) for {n} exchanges")
    k = min(len(ms), n)
    co = [ms[i]["offset"] * 1e9 for i in range(k)]
    cd = [ms[i]["delay"] * 1e9 for i in range(k)]
    resd = [cd[i] - bd[i] for i in range(k)]
    say(f"  chrony delay (measurements.log): mean {stats(cd)[0]:.1f} ns, min {min(cd):.1f}, max {max(cd):.1f}; "
        f"delay - queue wait: {min(cd[i] - qw[i] for i in range(k)):.1f}..{max(cd[i] - qw[i] for i in range(k)):.1f} ns")
    check(max(abs(x) for x in resd) <= 1.5,
          f"chrony's delay = the bench's delta (max |diff| {max(abs(x) for x in resd):.2f} ns)")
    check(max(abs((cd[i] - qw[i]) - (stats([cd[j] - qw[j] for j in range(k)])[0])) for i in range(k)) <= 25,
          "chrony's delay minus the request's queue wait is constant (+-25 ns: MII phase)")

    if select:
        src = ca.source_line(d)
        say(f"  chronyc sources: {src['line'] if src else None}")
        check(src is not None and src["state"] == "*", "chrony selects the server under load ('*')")
        check(src is not None and src["reach"] == "377", "reach 377")
        check(src is not None and src["stratum"] == 2, "stratum 2 in chronyc sources")
        check(any("Selected source" in x for x in msgs), "chronyd's log: 'Selected source 127.0.0.1'")
        for kk in ("Poll interval", "Offset", "Peer delay", "Response time"):
            say(f"         ntpdata {kk}: {nd.get(kk)}")
        for l in ca.read(os.path.join(d, "chronyc-tracking.txt")).splitlines():
            if l.split(":")[0].strip() in ("Reference ID", "Stratum", "System time", "Frequency", "Leap status"):
                say(f"         tracking {l.strip()}")
        # chronyd's own discipline corrects its (virtual) clock: its offsets are cooked, not
        # compared with the bench's raw theta here (the `offset` job does).
        say(f"  chrony offsets (cooked by its discipline): {[round(x) for x in co]} ns")
        say(f"  bench theta (raw): {[round(x) for x in bo]} ns")
        check(n >= 8, f"{n} polls (>= 8 for reach 377)")
    else:
        geom = [((rx[i] - t1[i]) + (tx[i] - t4[i])) / 2 for i in range(n)]
        design = [co[i] - geom[i] for i in range(k)]
        resid = [co[i] - bo[i] for i in range(k)]
        say(f"  chrony offset (theta, measurements.log): mean {stats(co)[0]:+.1f} ns, sigma {stats(co)[1]:.1f}, "
            f"min {min(co):+.0f}, max {max(co):+.0f}")
        say(f"  bench geometry theta_geom (queue wait included): mean {stats(geom)[0]:+.1f}, sigma {stats(geom)[1]:.1f}")
        say(f"  chrony offset - theta_geom (design stamp error as chrony sees it): mean {stats(design)[0]:+.2f} ns, "
            f"min {min(design):+.1f}, max {max(design):+.1f} (idle +{IDLE_DESIGN})")
        check(max(abs(x) for x in resid) <= 1.5,
              f"chrony's offset = the bench's theta (max |diff| {max(abs(x) for x in resid):.2f} ns)")
        check(max(abs(x - IDLE_DESIGN) for x in design) <= 1.5,
              f"offset - theta_geom = +{IDLE_DESIGN} ns +-1.5 for every poll: the design's error is independent of the load")
        check(max(abs(design[i] - (e2[i] + e3[i]) / 2) for i in range(k)) <= 1.5, "offset - geometry = (e2+e3)/2")
        check(n >= 8, f"{n} polls")


def main():
    res = sys.argv[1]
    names = sys.argv[2:] or sorted(n for n in os.listdir(res) if os.path.isfile(os.path.join(res, n, "chrony.csv")))
    for n in names:
        d = os.path.join(res, n)
        if not os.path.isdir(d):
            say(f"== {n}")
            check(False, f"{d} missing")
            continue
        job(d, n)
    say()
    say("all checks passed" if not FAILS else f"{len(FAILS)} check(s) FAILED")
    with open(os.path.join(res, "summary-mixed.txt"), "w") as f:
        f.write("\n".join(OUT) + "\n")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
