#!/usr/bin/env python3
"""Turn the files written by the Vivado scripts into Markdown tables (no Vivado needed).

  summarize.py run  <dir>   analyze.tcl results: summary.json, unit_timing.csv, paths_setup.csv,
                            utilization_hier.csv, skipped reports   -> <dir>/SUMMARY.md
  summarize.py ooc  <dir>   ooc.sh results (one result.csv per unit) -> <dir>/OOC.md
  summarize.py sweep <dir>  sweep.sh results                         -> <dir>/SWEEP.md
"""
import csv, glob, json, os, sys


def f(x, nd=3):
    try:
        return f"{float(x):.{nd}f}"
    except (TypeError, ValueError):
        return x or ""


def table(head, rows):
    out = ["| " + " | ".join(head) + " |", "|" + "|".join("---" for _ in head) + "|"]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def short(pin, n=48):
    return pin if len(pin) <= n else "…" + pin[-(n - 1):]


def run(d):
    md = [f"# Vivado run: {os.path.basename(os.path.abspath(d))}", ""]
    opts = os.path.join(d, "run_options.txt")
    if os.path.exists(opts):
        md += ["```", open(opts).read().strip(), "```", ""]
    sj = os.path.join(d, "summary.json")
    if os.path.exists(sj):
        s = json.load(open(sj))
        wns, whs = s.get("wns_ns"), s.get("whs_ns")
        ok = wns is not None and whs is not None and wns >= 0 and whs >= 0
        md += [f"**Timing {'MET' if ok else 'NOT MET'}**: WNS {f(wns)} ns, TNS {f(s.get('tns_ns'))} ns, "
               f"WHS {f(whs)} ns, THS {f(s.get('ths_ns'))} ns.", ""]
        md += ["## Clock domains", "", table(["clock", "period (ns)", "worst setup slack (ns)"],
               [(k, f(v.get("period_ns")), f(v.get("wns_ns"))) for k, v in s.get("clocks", {}).items()]), ""]
        md += ["## Resources (primitives)", "", table(["LUT", "FF", "CARRY4", "DSP", "BRAM", "LUT-RAM", "BUFG", "MMCM"],
               [[s["cells"].get(k) for k in ("lut", "ff", "carry4", "dsp", "bram", "lutram", "bufg", "mmcm")]]), ""]
    else:
        md += ["`summary.json` missing: the run did not reach the report step.", ""]
    units = read_csv(os.path.join(d, "reports", "unit_timing.csv"))
    if units:
        units.sort(key=lambda r: float(r["slack_ns"] or 1e9))
        md += ["## Worst setup path ending in each unit (sorted by slack)", "",
               table(["unit", "slack", "levels", "logic ns", "route ns", "from", "to"],
                     [(r["unit"], f(r["slack_ns"]), r["logic_levels"], f(r["logic_ns"]), f(r["net_ns"]),
                       short(r["startpoint"]), short(r["endpoint"])) for r in units]), ""]
    paths = read_csv(os.path.join(d, "reports", "paths_setup.csv"))
    if paths:
        md += ["## 25 worst setup paths", "",
               table(["slack", "levels", "logic ns", "route ns", "clock", "from", "to"],
                     [(f(r["slack_ns"]), r["logic_levels"], f(r["logic_ns"]), f(r["net_ns"]), r["end_clock"],
                       short(r["startpoint"]), short(r["endpoint"])) for r in paths[:25]]), ""]
        md += ["Route share of the 25 worst paths: "
               + f"{100 * sum(float(r['net_ns'] or 0) for r in paths[:25]) / max(1e-9, sum(float(r['datapath_ns'] or 0) for r in paths[:25])):.0f} %", ""]
    util = read_csv(os.path.join(d, "reports", "utilization_hier.csv"))
    if util:
        cols = [c for c in util[0].keys()]
        key = [c for c in cols if "LUT" in c and "Total" in c]
        if key:
            util.sort(key=lambda r: -float(r[key[0]] or 0))
            show = [c for c in cols if c in ("Instance", "Module") or any(t in c for t in ("Total LUTs", "FFs", "DSP", "RAMB"))]
            md += ["## Utilization by instance (largest 25)", "",
                   table(show, [[r[c] for c in show] for r in util[:25]]), ""]
    sk = os.path.join(d, "reports", "_skipped.log")
    if os.path.exists(sk):
        md += ["## Reports that could not be produced", "", "```", open(sk).read().strip(), "```", ""]
    md += ["## Files", "", ", ".join(sorted(os.path.basename(p) for p in glob.glob(os.path.join(d, "reports", "*")))), ""]
    open(os.path.join(d, "SUMMARY.md"), "w").write("\n".join(md))
    print(f"wrote {os.path.join(d, 'SUMMARY.md')}")


def ooc(d):
    rows = []
    for p in glob.glob(os.path.join(d, "*", "result.csv")):
        rows += read_csv(p)
    rows.sort(key=lambda r: float(r["wns_ns"] or 1e9))
    head = ["unit", "impl", "LUT", "FF", "DSP", "BRAM", "LUT-RAM", "WNS ns", "levels", "path ns", "from", "to"]
    md = ["# Out-of-context units (own clock, I/O not timed)", "",
          "WNS is against the period asked (impl=0: after synthesis, no routing delay).", "",
          table(head, [(r["module"].replace("arty_top_core_", ""), r["impl"], r["lut"], r["ff"], r["dsp"], r["bram"],
                        r["lutram"], f(r["wns_ns"]), r["logic_levels"], f(r["datapath_ns"]),
                        short(r["startpoint"], 36), short(r["endpoint"], 36)) for r in rows]), ""]
    open(os.path.join(d, "OOC.md"), "w").write("\n".join(md))
    print(f"wrote {os.path.join(d, 'OOC.md')} ({len(rows)} units)")


def sweep(d):
    rows = []
    for p in glob.glob(os.path.join(d, "*", "result.csv")):
        rows += read_csv(p)
    rows.sort(key=lambda r: -float(r["wns_ns"] or -1e9))
    md = ["# Implementation strategy sweep", "", table(
        ["opt", "place", "physopt", "route", "WNS ns", "TNS ns", "WHS ns", "THS ns", "s"],
        [(r["opt"], r["place"], r["physopt"], r["route"], f(r["wns_ns"]), f(r["tns_ns"]), f(r["whs_ns"]),
          f(r["ths_ns"]), r["seconds"]) for r in rows]), ""]
    if rows:
        w = [float(r["wns_ns"]) for r in rows if r["wns_ns"]]
        md += [f"Spread of WNS over {len(w)} runs: {min(w):.3f} to {max(w):.3f} ns "
               f"(a wide spread means the failure is placement-sensitive).", ""]
    open(os.path.join(d, "SWEEP.md"), "w").write("\n".join(md))
    print(f"wrote {os.path.join(d, 'SWEEP.md')} ({len(rows)} runs)")


if __name__ == "__main__":
    if len(sys.argv) != 3 or sys.argv[1] not in ("run", "ooc", "sweep"):
        sys.exit(__doc__)
    {"run": run, "ooc": ooc, "sweep": sweep}[sys.argv[1]](sys.argv[2])
