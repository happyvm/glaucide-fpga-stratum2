#!/usr/bin/env bash
# Formal proof of one unit's check!/assume!/cover! statements.
#   tools/formal/prove.sh <unit> [prove|bmc|cover] [depth]
# <unit> is a name known to rtl/src/bin/formal_gen.rs.
# Needs `pip install yowasp-yosys z3-solver` (or native sby, yosys, z3 on PATH).
# prove = unbounded k-induction (smtbmc, z3); bmc = bounded search; cover = reach cover points.
# UNKNOWN or FAIL exits non-zero: a proof must be a PASS.
set -euo pipefail
cd "$(dirname "$0")/../.."
unit=$1 mode=${2:-prove} depth=${3:-30}
# Native tools win; otherwise shim the YoWASP ones under the names sby expects.
shim=$PWD/out/formal/bin; mkdir -p "$shim"
for pair in sby:yowasp-sby yosys:yowasp-yosys yosys-smtbmc:yowasp-yosys-smtbmc yosys-witness:yowasp-yosys-witness; do
  n=${pair%%:*}; w=${pair##*:}
  command -v "$n" >/dev/null 2>&1 || { command -v "$w" >/dev/null && ln -sf "$(command -v "$w")" "$shim/$n"; }
done
export PATH="$shim:$PATH"
SBY=sby
work=out/formal/$unit
mkdir -p "$work"
cargo run -q -p ntp_fpga --bin formal_gen -- "$unit" > "$work/$unit.v"
# PDR (mode `pdr`) needs a native yosys-abc: YOSYS_ABC=/path/to/yosys-abc (e.g. from
# `pip install pyosys` in a venv: <venv>/lib/pythonX/site-packages/pyosys/yosys-abc).
if [ -n "${YOSYS_ABC:-}" ]; then ln -sf "$YOSYS_ABC" "$shim/yosys-abc"; fi
case $mode in
  pdr)   engine="abc pdr"; smtmode=prove; aig="aigsmt none" ;;
  prove) engine="smtbmc z3"; smtmode=prove ;;
  bmc)   engine="smtbmc z3"; smtmode=bmc ;;
  cover) engine="smtbmc z3"; smtmode=cover ;;
esac
cat > "$work/$unit.sby" <<SBYEOF
[options]
mode $smtmode
depth $depth
${aig:-}
[engines]
$engine
[script]
read_verilog -formal -DFORMAL $unit.v
prep -top $unit
[files]
$unit.v
SBYEOF
( cd "$work" && "$SBY" -f "$unit.sby" )
