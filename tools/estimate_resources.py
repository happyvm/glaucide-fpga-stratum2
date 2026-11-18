#!/usr/bin/env python3
"""Rough Artix-7 resource estimate per generated module (no Vivado needed).

Runs Yosys `synth_xilinx -family xc7` on every out/*.v and prints LUT / LUT-RAM / FF / BRAM / DSP.
Needs a NATIVE Yosys with ABC: `python3 -m venv v && v/bin/pip install pyosys`, then
    v/bin/python tools/estimate_resources.py
(the YoWASP build used for the formal proofs cannot run ABC). Run tools/gen_verilog.sh first.
LUT-RAM cost in LUTs: RAM64M/RAM32M = 4, RAM64X1S/RAM32X1S = 1, RAM64X1D = 2, RAM128X1S = 2.
Estimates are per module (no cross-module optimisation, no top level, no timing): +-15 % vs Vivado.
"""
import glob, os, re, subprocess, sys, tempfile
from pyosys import libyosys as ys

COST = {"RAM64M": 4, "RAM32M": 4, "RAM64X1S": 1, "RAM32X1S": 1, "RAM64X1D": 2, "RAM128X1S": 2, "RAM256X1S": 4}

def synth(path):
    src = open(path).read()
    # Yosys refuses the 4096-step memory init loops of tx_path and the rate limiter; the init is irrelevant here.
    src = re.sub(r"^\s*initial for \((?:mem_i|rl_i).*$", "", src, flags=re.M)
    top = re.search(r"^module\s+(\w+)", src, re.M).group(1)
    with tempfile.TemporaryDirectory() as d:
        v, st = os.path.join(d, "m.v"), os.path.join(d, "m.stat")
        open(v, "w").write(src)
        design = ys.Design()
        ys.run_pass(f"read_verilog {v}", design)
        ys.run_pass(f"synth_xilinx -family xc7 -top {top} -flatten", design)
        ys.run_pass(f"tee -o {st} stat", design)
        text = open(st).read()
    blk = text[text.rfind("==="):]
    cells = {}
    for m in re.finditer(r"^\s+(\d+)\s+([A-Z][A-Z0-9_]*)\s*$", blk, re.M):
        cells[m.group(2)] = cells.get(m.group(2), 0) + int(m.group(1))
    lut = sum(v for k, v in cells.items() if re.fullmatch(r"LUT[1-6]", k))
    ram = sum(v * COST.get(k, 0) for k, v in cells.items())
    ff = sum(v for k, v in cells.items() if k.startswith(("FDRE", "FDSE", "FDCE", "FDPE")))
    bram = sum(v for k, v in cells.items() if k.startswith("RAMB"))
    return lut, ram, ff, bram, cells.get("DSP48E1", 0)

def main():
    files = sorted(glob.glob("out/*.v"))
    tot = [0] * 5
    print(f"{'module':18}{'LUT':>7}{'LUT-RAM':>9}{'FF':>7}{'BRAM':>6}{'DSP':>5}")
    for f in files:
        r = synth(f)
        print(f"{os.path.basename(f)[:-2]:18}{r[0]:>7}{r[1]:>9}{r[2]:>7}{r[3]:>6}{r[4]:>5}")
        tot = [a + b for a, b in zip(tot, r)]
    print(f"{'TOTAL':18}{tot[0]:>7}{tot[1]:>9}{tot[2]:>7}{tot[3]:>6}{tot[4]:>5}")
    lut = tot[0] + tot[1]
    print(f"\nLUT incl. LUT-RAM: {lut} / 63400 = {100*lut/63400:.1f} %   FF: {tot[2]} / 126800 = {100*tot[2]/126800:.1f} %")

if __name__ == "__main__":
    main()
