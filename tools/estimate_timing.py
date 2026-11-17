#!/usr/bin/env python3
"""Structural timing estimate per generated module (no Vivado needed, task T5.3).

For every Verilog file (default: out/*.v, plus out/top*/arty_top.v when present) this
runs Yosys `synth_xilinx -family xc7 -flatten` and walks the mapped netlist to find, per
clock domain, the longest register-to-register path: LUT levels, CARRY4 cells and
MUXF7/F8 on it, its start and end registers, and a rough delay.

Needs a NATIVE Yosys with ABC (pyosys), as tools/estimate_resources.py:
    python3 -m venv v && v/bin/pip install pyosys
    PATH=v/lib/python3.13/site-packages/pyosys:$PATH v/bin/python tools/estimate_timing.py
(the pyosys directory holds `yosys-abc`, which synth_xilinx calls). Run tools/gen_verilog.sh
first; for the board top, `cargo run -p ntp_fpga --bin top_gen -- out/top_server` and
`... -- --with-client out/top_client`.

Why not Yosys `ltp`: `ltp -noff` only knows Yosys's internal flip-flop types ($dff...),
so on a synth_xilinx netlist it walks through FDRE cells and reports clock-to-output
chains. This script reads the netlist (write_json) instead and stops at every sequential
primitive.

Delay model (an indicator, NOT a timing analysis; no placement, routing or fanout):
    delay = 1.0 ns (clock-to-q + setup) + 0.6 ns per LUT level (logic + average net)
            + 0.1 ns per CARRY4 + 0.3 ns per MUXF7/MUXF8
            (+ 1.5 ns from a block-RAM output, + 3.0 ns through an unregistered DSP48E1)
LUT-RAM / SRL read (address -> data) counts as one LUT level. Input ports start at 0 ns
and output ports end paths (reported apart: in the full design they continue in the
neighbouring module, which is why the board top is analysed too).

Usage: estimate_timing.py [-n N] [--json FILE] [files...]
    -n N         also print the N worst endpoint registers per file (default 3)
    --json FILE  write all results as JSON
"""
import argparse, glob, json, os, re, sys, tempfile
from collections import defaultdict

T_FIXED, T_LUT, T_CARRY, T_MUXF, T_BRAM, T_DSP = 1.0, 0.6, 0.1, 0.3, 1.5, 3.0
MACC = False  # --yosys-macc: let Yosys merge chained additions (optimistic vs Vivado)
NEG = -1e9  # arrival of a startpoint that is not in the launching domain being analysed
BUFS = ("BUFG", "BUFGCE", "BUFH", "BUFR", "IBUF", "IBUFG", "BUFIO", "BUFHCE", "BUFGCTRL")


def synth_json(path, top=None):
    from pyosys import libyosys as ys
    src = open(path).read()
    # Yosys refuses the 4096-step memory init loop of tx_path; the init is irrelevant here.
    src = re.sub(r"^\s*initial for \(mem_i.*$", "", src, flags=re.M)
    if top is None:
        top = re.search(r"^module\s+(\w+)", src, re.M).group(1)
    with tempfile.TemporaryDirectory() as d:
        v, js, log = os.path.join(d, "m.v"), os.path.join(d, "m.json"), os.path.join(d, "log")
        open(v, "w").write(src)
        design = ys.Design()
        # Yosys logs to stdout: silence file descriptor 1 while it runs.
        sys.stdout.flush()
        saved, null = os.dup(1), os.open(os.devnull, os.O_WRONLY)
        os.dup2(null, 1)
        try:
            ys.run_pass(f"read_verilog {v}", design)
            sx = f"synth_xilinx -family xc7 -top {top} -flatten -noiopad -noclkbuf"
            if MACC:
                ys.run_pass(sx, design)
            else:
                # The `coarse` step of synth_xilinx, with `alumacc -macc-only`: every `+`
                # stays its own carry chain, as Vivado builds it (plain `alumacc` merges a
                # chain of additions into one carry-save tree, which Vivado does not do).
                ys.run_pass(sx + " -run :coarse", design)
                for c in ("techmap -map +/cmp2lut.v -map +/cmp2lcu.v -D LUT_WIDTH=6",
                          "alumacc -macc-only", "share", "opt", "memory -nomap", "opt_clean"):
                    ys.run_pass(c, design)
                ys.run_pass(sx + " -run map_memory:", design)
            ys.run_pass(f"write_json {js}", design)
        finally:
            os.dup2(saved, 1)
            os.close(null)
            os.close(saved)
        return top, json.load(open(js))["modules"][top]


def kind(t):
    if re.fullmatch(r"LUT[1-6]|INV|BUF", t):
        return "lut" if t.startswith("LUT") else "wire"
    if t == "CARRY4":
        return "carry"
    if t in ("MUXF7", "MUXF8"):
        return "muxf"
    if t.startswith(("FD", "LD")):
        return "ff"
    if t.startswith(("SRL", "RAM32", "RAM64", "RAM128", "RAM256")):
        return "lutram"
    if t.startswith("RAMB"):
        return "bram"
    if t == "DSP48E1":
        return "dsp"
    if t in BUFS:
        return "buf"
    return "box"


def analyse(mod):
    """All endpoint paths, computed once per launching domain (so that a long path from
    another domain or from a port does not hide a shorter same-domain one)."""
    doms = None
    out, path_of = [], None
    first = _analyse(mod, None)
    doms = first[2]
    for d in sorted(doms) + ["port"]:
        paths, po, _ = _analyse(mod, d)
        out += paths
        path_of = path_of or {}
        path_of[d] = po
    return out, lambda p: path_of[p["start_dom"]](p["bit"])


def _analyse(mod, active):
    names = {}  # bit -> best net name (public names preferred)
    for n, w in mod["netnames"].items():
        hidden = w.get("hide_name", 0) or n.startswith("$")
        for i, b in enumerate(w["bits"]):
            if isinstance(b, str):
                continue
            nm = n if len(w["bits"]) == 1 else f"{n}[{i + w.get('offset', 0)}]"
            if b not in names or (names[b][0] and not hidden):
                names[b] = (hidden, nm)
    nm = lambda b: names.get(b, (1, f"bit{b}"))[1]

    driver = {}  # bit -> (cell name, cell)
    for cn, c in mod["cells"].items():
        for p, bits in c["connections"].items():
            if c["port_directions"].get(p) == "output":
                for b in bits:
                    if not isinstance(b, str):
                        driver[b] = (cn, c)

    # Clock domain of every sequential cell: trace its clock pin back through buffers.
    def clk_of(c):
        conn = c["connections"]
        pin = next((p for p in ("C", "CLK", "WCLK", "CLKARDCLK", "CLKBWRCLK") if p in conn), None)
        if pin is None:
            return "?"
        b = conn[pin][0]
        for _ in range(8):
            if isinstance(b, str) or b not in driver:
                break
            dc = driver[b][1]
            if kind(dc["type"]) not in ("buf", "wire"):
                break
            b = next(x for p, x in dc["connections"].items() if dc["port_directions"][p] == "input")[0]
        return nm(b) if not isinstance(b, str) else b

    # Startpoints: (arrival, luts, carries, muxf, start label, start domain)
    arr = {}
    for pn, p in mod["ports"].items():
        if p["direction"] == "input":
            for i, b in enumerate(p["bits"]):
                if not isinstance(b, str):
                    arr[b] = (0.0 if active == "port" else NEG, 0, 0, 0, f"in:{pn}", "port", None)

    def startpoint(b, c, extra=0.0):
        d = clk_of(c)
        arr[b] = (extra if active == d else NEG, 0, 0, 0, nm(b), d, None)

    seq_cells = []
    for cn, c in mod["cells"].items():
        k = kind(c["type"])
        if k in ("ff", "lutram", "bram", "dsp", "box"):
            seq_cells.append((cn, c, k))
            for p, bits in c["connections"].items():
                if c["port_directions"].get(p) != "output":
                    continue
                for b in bits:
                    if isinstance(b, str):
                        continue
                    if k == "ff" or k == "box" or (k == "lutram" and p.startswith("Q") and c["type"].startswith("SRL")):
                        startpoint(b, c)
                    elif k == "bram":
                        startpoint(b, c, T_BRAM)
                    elif k == "dsp":
                        pass  # handled below
    # DSP48E1: registered if any of AREG/BREG/MREG/PREG is set; otherwise combinational.
    def dsp_regd(c):
        pr = c.get("parameters", {})
        return any(int(str(pr.get(k, "0")), 2) if set(str(pr.get(k, "0"))) <= {"0", "1"} else int(pr.get(k, 0)) for k in ("MREG", "PREG"))
    for cn, c, k in seq_cells:
        if k == "dsp" and dsp_regd(c):
            for p, bits in c["connections"].items():
                if c["port_directions"].get(p) == "output":
                    for b in bits:
                        if not isinstance(b, str):
                            startpoint(b, c)

    sys.setrecursionlimit(1000000)

    def comb_inputs(c, k, out_port):
        conn, dirs = c["connections"], c["port_directions"]
        ins = [p for p in conn if dirs.get(p) == "input"]
        if k == "lutram":
            # asynchronous read: address pins -> data out
            ins = [p for p in ins if re.match(r"(A\d*|ADDR[A-D]|DPRA\d*)$", p)]
        return [b for p in ins for b in conn[p] if not isinstance(b, str)]

    def get(b):
        # iterative DFS to compute arrival of bit b
        stack = [b]
        while stack:
            x = stack[-1]
            if x in arr:
                stack.pop()
                continue
            if x not in driver:
                arr[x] = (NEG, 0, 0, 0, "const/undriven", "port", None)
                stack.pop()
                continue
            cn, c = driver[x]
            k = kind(c["type"])
            if k in ("ff", "bram", "box") or (k == "dsp" and dsp_regd(c)):
                d = clk_of(c)
                arr[x] = (0.0 if active == d else NEG, 0, 0, 0, nm(x), d, None)
                stack.pop()
                continue
            ins = comb_inputs(c, k, None)
            pend = [i for i in ins if i not in arr]
            if pend:
                stack.extend(pend)
                continue
            best = None
            for i in ins:
                a = arr[i]
                if best is None or (a[0], a[1]) > (best[0][0], best[0][1]):
                    best = (a, i)
            if best is None:
                a, i = (NEG, 0, 0, 0, "const", "port", None), None
            else:
                a, i = best
            t, l, cy, mx = a[0], a[1], a[2], a[3]
            if k in ("lut", "lutram"):
                t, l = t + T_LUT, l + 1
            elif k == "carry":
                t, cy = t + T_CARRY, cy + 1
            elif k == "muxf":
                t, mx = t + T_MUXF, mx + 1
            elif k == "dsp":
                t = t + T_DSP
            for ob in (bb for p, bits in c["connections"].items() if c["port_directions"].get(p) == "output" for bb in bits):
                if not isinstance(ob, str):
                    arr[ob] = (t, l, cy, mx, a[4], a[5], i)
            stack.pop()
        return arr[b]

    # Endpoints: every input pin of a sequential cell (except clocks), output ports.
    ends = []  # (endpoint label, domain, bit)
    for cn, c, k in seq_cells:
        dom = clk_of(c)
        for p, bits in c["connections"].items():
            if c["port_directions"].get(p) != "input" or p in ("C", "CLK", "WCLK", "CLKARDCLK", "CLKBWRCLK"):
                continue
            if k == "dsp" and not dsp_regd(c):
                continue
            for b in bits:
                if isinstance(b, str):
                    continue
                q = next((x for pp, xs in c["connections"].items() if c["port_directions"].get(pp) == "output" for x in xs if not isinstance(x, str)), None)
                label = nm(q) if q is not None else cn
                ends.append((f"{label}.{p}" if k != "ff" else label, dom, b))
    for pn, p in mod["ports"].items():
        if p["direction"] == "output":
            for i, b in enumerate(p["bits"]):
                if not isinstance(b, str):
                    ends.append((f"out:{pn}", "port", b))

    def path_of(b):
        out = []
        while b is not None:
            out.append(nm(b))
            b = arr[b][6]
        return out[::-1]

    paths = []
    doms = {clk_of(c) for cn, c, k in seq_cells}
    if active is None:
        return [], None, doms
    for label, dom, b in ends:
        a = get(b)
        if a[0] < 0:
            continue
        paths.append(dict(delay=round(a[0] + T_FIXED, 2), luts=a[1], carry=a[2], muxf=a[3],
                          start=a[4], start_dom=a[5], end=label, end_dom=dom, bit=b))
    return paths, path_of, doms


def base(s):
    return re.sub(r"\[\d+\]", "", s)


def summarise(paths):
    """Split paths into: same-domain reg->reg (per domain), port paths, crossings."""
    regs, ports, cross = defaultdict(list), [], []
    for p in paths:
        if p["start_dom"] == "port" or p["end_dom"] == "port":
            ports.append(p)
        elif p["start_dom"] == p["end_dom"]:
            regs[p["end_dom"]].append(p)
        else:
            cross.append(p)
    key = lambda p: (p["delay"], p["luts"])
    return ({d: sorted(v, key=key, reverse=True) for d, v in regs.items()},
            sorted(ports, key=key, reverse=True), sorted(cross, key=key, reverse=True))


def verdict(p, budget=10.0):
    if budget > 10.0:
        return "OK" if p["delay"] <= 0.85 * budget else "critique"
    if p["delay"] > 8.5 or p["luts"] > 11:
        return "critique"
    if p["delay"] >= 7.0:
        return "a surveiller"
    return "OK"


def worst_per_end(lst, n):
    seen, out = set(), []
    for p in lst:
        k = (base(p["start"]), base(p["end"]))
        if base(p["end"]) in seen:
            continue
        seen.add(base(p["end"]))
        out.append(p)
        if len(out) >= n:
            break
    return out


def fmt(p):
    return (f"{p['delay']:5.1f} ns  {p['luts']:2} LUT {p['carry']:2} CARRY4 {p['muxf']} MUXF  "
            f"{p['start']} -> {p['end']}")


def one_file(f, a):
    global MACC
    MACC = a.yosys_macc
    out = []
    top, mod = synth_json(f, "arty_top" if f.endswith("arty_top.v") else None)
    paths, path_of = analyse(mod)
    regs, ports, cross = summarise(paths)
    parent = os.path.basename(os.path.dirname(os.path.abspath(f)))
    label = (f"{parent}/" if parent.startswith("top") else "") + os.path.basename(f)[:-2]
    res = {"domains": {}, "ports": ports[:a.n], "cross": cross[:a.n]}
    for dom, lst in sorted(regs.items()):
        budget = 40.0 if re.search(r"rx|tx", dom) else 10.0
        w = lst[0]
        out.append(f"{label:26}{dom:10}{w['delay']:6.1f}{w['luts']:5}{w['carry']:5}{w['muxf']:5}  "
                   f"{verdict(w, budget):13} {w['start']} -> {w['end']}")
        for p in worst_per_end(lst, a.n)[1:]:
            out.append(f"{'':36}{fmt(p)}")
        if a.path:
            out.append("      path: " + " > ".join(path_of(w)))
        res["domains"][dom] = worst_per_end(lst, a.n)
        if a.end:
            for p in worst_per_end([p for p in lst if re.search(a.end, p["end"])], a.n):
                out.append(f"{'':26}{'(--end)':10}{'':>6}  {fmt(p)}")
    if ports:
        out.append(f"{'':26}{'(ports)':10}{'':>6}  worst port path: {fmt(ports[0])}")
    for c in worst_per_end(cross, a.n):
        out.append(f"{'':26}{'(cross)':10}{'':>6}  crossing: {fmt(c)} [{c['start_dom']} -> {c['end_dom']}]")
    return "\n".join(out) + "\n", (label, res)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", type=int, default=3)
    ap.add_argument("--json")
    ap.add_argument("--path", action="store_true", help="print the nets of the worst path")
    ap.add_argument("--yosys-macc", action="store_true",
                    help="let Yosys merge chained additions into carry-save trees")
    ap.add_argument("--end", help="also print the worst path to endpoints matching this regex")
    ap.add_argument("-j", type=int, default=1, help="files analysed in parallel")
    ap.add_argument("files", nargs="*")
    a = ap.parse_args()
    global MACC
    MACC = a.yosys_macc
    files = a.files or (sorted(glob.glob("out/*.v")) + sorted(glob.glob("out/top*/arty_top.v")))
    print(f"{'module':26}{'domain':10}{'ns':>6}{'LUT':>5}{'CY4':>5}{'MUXF':>5}  verdict       worst path")
    allres = {}
    if a.j > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(a.j) as ex:
            for f, (text, res) in zip(files, ex.map(one_file, files, [a] * len(files))):
                print(text, end="", flush=True)
                allres[res[0]] = res[1]
    else:
        for f in files:
            text, res = one_file(f, a)
            print(text, end="", flush=True)
            allres[res[0]] = res[1]
    if a.json:
        json.dump(allres, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
