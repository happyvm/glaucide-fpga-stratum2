#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""tools/ntpperf/analyse.py: checks and summarises the ntpperf load test
(task NTPPERF, docs/ntpperf-results.md).

    python3 -I tools/ntpperf/analyse.py out/ntpperf/results [JOB...]

For every job directory (written by tools/ntpperf/run.sh): ntpperf's table
(ntpperf.txt), the bridge's summary (bridge.log) and the per-request log
(requests.csv). Writes summary.md next to the jobs and exits with 1 if a
check fails. Standard library only.

Expectations checked:
  * wire: a 90-byte request takes 8 + 94 + 12 = 114 bytes of wire time, so
    the 100 Mb/s line rate is L = 10^8 / 912 = 109 649 requests/s; below it
    nothing may be lost, above it the bounded queue loses 1 - L/R;
  * response time (T3 - T2, ntpperf without -o) and TX offset (ntpperf
    -o 1e-12 -H: RX stamp = virtual SFD instant of the reply) as printed by
    ntpperf must agree with the bridge's ground truth;
  * rate limiter (RATE_LIMIT_PER_S = 8, leak 1/16, silence): the expected
    loss is recomputed from the requests' instants and the design's hash
    buckets (rl_index of ntp_server.rs, logged by the bridge): per bucket
    and one-second window, the first 8 requests are answered, every other
    one with probability 1/16; only the requests ntpperf counts (each
    client's first two are warm-up) enter the ratio;
  * KoD: ntpperf counts a KoD (stratum 0, mode 4, right origin) as a valid
    basic response: no loss, no 'invalid'.
"""
import csv
import math
import os
import re
import sys

LINE_RATE = 1e8 / (8 * (8 + 94 + 12))  # 90-byte frames, requests/s
LIMIT = 8
LEAK = 1 / 16
WARMUP = 2

ROW = re.compile(
    r"^(\d+)\s+(\d+)\s+([\d.]+)%\s+([\d.]+)%\s+([\d.]+)%\s+([\d.]+)%"
    r"(?:\s+([+-]\d+)\s+([+-]\d+)\s+([+-]\d+)\s+(\d+))?"
)


def read_table(path):
    rows, notes = [], []
    if not os.path.exists(path):
        return rows, ["no ntpperf.txt"]
    with open(path) as f:
        for line in f:
            m = ROW.match(line.strip())
            if m:
                g = m.groups()
                r = {
                    "rate": int(g[0]),
                    "clients": int(g[1]),
                    "lost": float(g[2]),
                    "invalid": float(g[3]),
                    "basic": float(g[4]),
                    "xleave": float(g[5]),
                }
                if g[6] is not None:
                    r.update(min=int(g[6]), mean=int(g[7]), max=int(g[8]), sd=int(g[9]))
                rows.append(r)
            elif line.strip() and not line.startswith((" ", "rate")):
                notes.append(line.strip())
    return rows, notes


STAT = re.compile(r"n=(\d+) min ([+-][\d.]+) mean ([+-][\d.]+) max ([+-][\d.]+) sd ([\d.]+)")


def read_bridge(path):
    b = {"stats": {}}
    if not os.path.exists(path):
        return b
    with open(path) as f:
        text = f.read()
    m = re.search(r"done \((.*?)\): virt ([\d.]+) s, wall ([\d.]+) s, simulated (\d+) cycles "
                  r"\((\d+) cycles per second of simulation\), skipped (\d+) cycles, (\d+) time advances",
                  text)
    if m:
        b.update(exit=m.group(1), virt=float(m.group(2)), wall=float(m.group(3)),
                 cycles=int(m.group(4)), speed=int(m.group(5)), skipped=int(m.group(6)),
                 advances=int(m.group(7)))
    m = re.search(r"requests (\d+), queue drops (\d+) \(max queue (\d+) B\), answered (\d+) \((\d+) KoD\), "
                  r"not answered (\d+), unmatched replies (\d+), other frames (\d+), bad frames (\d+)", text)
    if m:
        k = ["requests", "qdrops", "maxq", "answered", "kod", "unanswered", "unmatched", "other", "bad"]
        b.update(dict(zip(k, map(int, m.groups()))))
    m = re.search(r"max silence rx end -> tx SFD (\d+) cycles", text)
    if m:
        b["max_silence"] = int(m.group(1))
    for key, label in (("resp", "response time T3-T2"), ("e2", "T2 - RX SFD"), ("e3", "T3 - TX SFD"),
                       ("off", "TX SFD - T3"), ("clk", "design clock - true time")):
        m = re.search(re.escape(label) + r".*?: " + STAT.pattern, text)
        if m:
            n, mn, mean, mx, sd = m.groups()[-5:]
            b["stats"][key] = dict(n=int(n), min=float(mn), mean=float(mean), max=float(mx), sd=float(sd))
    b["errors"] = [l for l in text.splitlines() if "bad transmitted" in l or "panicked" in l
                   or "died without" in l or "max-wall" in l or "bridge exited" in l]
    return b


def read_requests(path):
    if not os.path.exists(path):
        return []
    out = []
    with open(path) as f:
        for r in csv.DictReader(f):
            out.append(r)
    return out


def segments(reqs, gap_ns=50_000_000):
    """Splits the requests of a multi-rate run into its rate points."""
    segs, cur, last = [], [], None
    for r in reqs:
        t = int(r["send_ns"])
        if last is not None and t - last > gap_ns:
            segs.append(cur)
            cur = []
        cur.append(r)
        last = t
    if cur:
        segs.append(cur)
    return segs


def measured(seg):
    """The requests ntpperf counts: each client's (source's) third and later."""
    seen = {}
    out = []
    for r in seg:
        k = r["src_ip"]
        seen[k] = seen.get(k, 0) + 1
        if seen[k] > WARMUP:
            out.append(r)
    return out


def fstats(xs):
    if not xs:
        return None
    n = len(xs)
    m = sum(xs) / n
    sd = math.sqrt(sum((x - m) ** 2 for x in xs) / (n - 1)) if n > 1 else 0.0
    return dict(n=n, min=min(xs), mean=m, max=max(xs), sd=sd)


def fmt(s, d=1):
    if not s:
        return "-"
    return f"{s['min']:+.{d}f} / {s['mean']:+.{d}f} / {s['max']:+.{d}f} / {s['sd']:.{d}f}"


def limiter_expectation(seg, lim=LIMIT, leak=LEAK):
    """Expected answered fraction of the measured requests (and its sd),
    from the instants, buckets and windows of every request of the point."""
    win = {}
    for r in seg:
        if r["qdrop"] == "1" or not r["rx_sfd_ns"]:
            continue
        # Decision ~1 us after the SFD; the design's clock is 10 ns behind
        # the true time; T0 is a whole second.
        w = (int(r["rx_sfd_ns"]) + 1000 - 10) // 1_000_000_000
        win.setdefault((r["bucket"], w), []).append(r)
    p = {}
    for lst in win.values():
        lst.sort(key=lambda r: int(r["rx_sfd_ns"]))
        for i, r in enumerate(lst):
            p[r["idx"]] = 1.0 if i < lim else leak
    m = measured(seg)
    e = sum(p.get(r["idx"], 0.0) for r in m)
    var = sum(p.get(r["idx"], 0.0) * (1 - p.get(r["idx"], 0.0)) for r in m)
    got = sum(1 for r in m if r["answered"] == "1")
    sources = {r["src_ip"]: r["bucket"] for r in seg}
    per_bucket = {}
    for b in sources.values():
        per_bucket[b] = per_bucket.get(b, 0) + 1
    shared = sum(1 for b in sources.values() if per_bucket[b] > 1)
    return dict(n=len(m), expected=e, sd=math.sqrt(var), got=got, sources=len(sources),
                buckets=len(per_bucket), shared=shared,
                maxload=max(per_bucket.values()) if per_bucket else 0)


def main():
    res = sys.argv[1] if len(sys.argv) > 1 else "out/ntpperf/results"
    only = sys.argv[2:]
    jobs = sorted(d for d in os.listdir(res) if os.path.isdir(os.path.join(res, d)))
    if only:
        jobs = [j for j in jobs if j in only]
    lines = ["# ntpperf load test: summary (generated by tools/ntpperf/analyse.py)", ""]
    fails = []
    speed = []

    def check(ok, what):
        if not ok:
            fails.append(what)
        return "ok" if ok else "**FAIL**"

    for j in jobs:
        d = os.path.join(res, j)
        rows, notes = read_table(os.path.join(d, "ntpperf.txt"))
        b = read_bridge(os.path.join(d, "bridge.log"))
        reqs = read_requests(os.path.join(d, "requests.csv"))
        cmd = open(os.path.join(d, "cmd.txt")).read().strip() if os.path.exists(os.path.join(d, "cmd.txt")) else ""
        wall = open(os.path.join(d, "wall_s.txt")).read().strip() if os.path.exists(os.path.join(d, "wall_s.txt")) else "?"
        offset_mode = " -o " in cmd
        limited = "--rate-limit 8" in cmd
        kod = "--kod" in cmd
        nt = cmd.split(" -- ")[1].split()[1:] if " -- " in cmd else []
        bo = cmd.split(" -- ")[0].split()[1:] if " -- " in cmd else []
        bo = [x for k, x in enumerate(bo) if x not in ("--shim", "--shm", "--log")
              and (k == 0 or bo[k - 1] not in ("--shim", "--shm", "--log"))]
        lines += [f"## {j}", "", f"`ntpperf {' '.join(nt)}`  ",
                  f"bridge: `{' '.join(bo)}`; wall time {wall} s", ""]
        if "cycles" in b:
            speed.append((j, b["cycles"], b["speed"], b["skipped"], b["virt"], b["wall"]))
            lines.append(f"simulated {b['cycles']:,} cycles at {b['speed']:,} cycles/s, skipped {b['skipped']:,}, "
                         f"{b['virt']:.3f} virtual s, {b.get('advances', 0):,} time advances")
            lines.append("")
        lines.append("```")
        lines += [l.rstrip() for l in open(os.path.join(d, "ntpperf.txt"))] if os.path.exists(os.path.join(d, "ntpperf.txt")) else ["(no output)"]
        lines.append("```")
        lines.append("")
        if b.get("errors"):
            lines += ["bridge errors: " + "; ".join(b["errors"]), ""]
            check(False, f"{j}: bridge errors")
        if b.get("bad", 0) or b.get("unmatched", 0) or b.get("other", 0):
            check(False, f"{j}: bad/unmatched/other frames")
        if not rows:
            check(False, f"{j}: no ntpperf table ({'; '.join(notes)})")
            continue
        segs = segments(reqs)
        if len(segs) != len(rows):
            lines.append(f"(warning: {len(segs)} request segments for {len(rows)} rate points)")
        lines.append("| rate | lost (ntpperf) | expected lost | queue drops | ntpperf min/mean/max/sd (ns) | bridge truth min/mean/max/sd (ns) | check |")
        lines.append("|---|---|---|---|---|---|---|")
        for i, row in enumerate(rows):
            seg = segs[i] if i < len(segs) else []
            m = measured(seg)
            n = len(m)
            qd = sum(1 for r in m if r["qdrop"] == "1")
            truth = None
            if offset_mode:
                xs = [-float(r["e3_ns"]) for r in m if r["answered"] == "1" and r["stratum"] != "0" and r["e3_ns"]]
            else:
                xs = [float(r["resp_ns"]) for r in m if r["answered"] == "1" and r["resp_ns"]]
            truth = fstats(xs)
            got = sum(1 for r in m if r["answered"] == "1")
            sim_lost = 100.0 * (n - got) / n if n else float("nan")
            verdict = []
            if limited:
                e = limiter_expectation(seg)
                exp = 100.0 * (1 - e["expected"] / e["n"]) if e["n"] else float("nan")
                tol = 100.0 * (4 * e["sd"] + 1) / e["n"] if e["n"] else 1
                exp_s = f"{exp:.2f} % ± {100.0 * e['sd'] / e['n']:.2f} (exact, see below)"
                if kod:
                    nk = sum(1 for r in m if r["stratum"] == "0")
                    exp_s = (f"0 % (KoD instead of silence: {100.0 * nk / n:.2f} % of the counted replies "
                             f"are KoD, {exp:.2f} % expected)")
                    ok = row["lost"] <= 0.5 and row["invalid"] == 0.0 and abs(100.0 * nk / n - exp) <= tol
                    verdict.append(check(ok, f"{j} {row['rate']}: KoD run lost/invalid"))
                else:
                    ok = abs(row["lost"] - exp) <= tol
                    verdict.append(check(ok, f"{j} {row['rate']}: limiter loss {row['lost']} vs {exp:.2f}"))
            else:
                exp = max(0.0, 100.0 * (1 - LINE_RATE / row["rate"]))
                exp_s = f"{exp:.2f} %"
                if row["rate"] < LINE_RATE:
                    ok = row["lost"] == 0.0 and row["invalid"] == 0.0
                else:
                    ok = abs(row["lost"] - exp) <= 1.5 and row["invalid"] == 0.0
                verdict.append(check(ok, f"{j} {row['rate']}: loss {row['lost']} vs {exp:.2f}"))
            ntp_s = f"{row['min']:+d} / {row['mean']:+d} / {row['max']:+d} / {row['sd']}" if "mean" in row else "-"
            if "mean" in row and truth and not kod:
                ok = abs(row["mean"] - truth["mean"]) <= 2 and abs(row["min"] - truth["min"]) <= 2 \
                    and abs(row["max"] - truth["max"]) <= 2
                verdict.append(check(ok, f"{j} {row['rate']}: ntpperf stamps vs bridge truth"))
            if abs(sim_lost - row["lost"]) > 0.05 and n:
                verdict.append(check(False, f"{j} {row['rate']}: ntpperf lost {row['lost']} vs bridge {sim_lost:.2f}"))
            lines.append(f"| {row['rate']} | {row['lost']:.2f} % | {exp_s} | {qd} | {ntp_s} | {fmt(truth)} | {' '.join(verdict)} |")
        lines.append("")
        if limited:
            for i, row in enumerate(rows):
                seg = segs[i] if i < len(segs) else []
                e = limiter_expectation(seg)
                lines.append(
                    f"limiter, rate {row['rate']}: {e['sources']} sources in {e['buckets']} buckets "
                    f"({e['shared']} sources share a bucket, at most {e['maxload']} per bucket); measured "
                    f"requests {e['n']}, answered {e['got']} (bridge), expected {e['expected']:.1f} ± {e['sd']:.1f}; "
                    f"lone-source formula: lost = (10 - 8)/10 x (1 - 1/16) = 18.75 %")
            lines.append("")
        if kod and reqs:
            nk = sum(1 for r in reqs if r["stratum"] == "0")
            lines.append(f"KoD replies (stratum 0): {nk} of {len(reqs)} requests; ntpperf invalid "
                         f"{', '.join(str(r['invalid']) for r in rows)} %")
            lines.append("")
        if b.get("stats"):
            s = b["stats"]
            lines.append("bridge ground truth (all replies, warm-up included): "
                         + "; ".join(f"{k} {v['min']:+.1f}/{v['mean']:+.2f}/{v['max']:+.1f}/{v['sd']:.2f}"
                                     for k, v in s.items()))
            lines.append("")
    if speed:
        lines += ["## Simulation speed", "", "| job | simulated cycles | cycles/s | skipped cycles | virtual s | wall s |",
                  "|---|---|---|---|---|---|"]
        for j, c, sp, sk, v, w in speed:
            lines.append(f"| {j} | {c:,} | {sp:,} | {sk:,} | {v:.3f} | {w:.0f} |")
        lines.append("")
    lines += ["## Checks", ""]
    lines += [f"- FAIL: {f}" for f in fails] if fails else ["all checks passed"]
    out = "\n".join(lines) + "\n"
    with open(os.path.join(res, "summary.md"), "w") as f:
        f.write(out)
    print(out)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
