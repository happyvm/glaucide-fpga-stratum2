#!/usr/bin/env bash
# Out-of-context synthesis of every unit of a generated netlist, N in parallel, then a table.
#   tools/vivado/ooc.sh <top_gen dir> [out dir] [period ns] [impl 0|1] [parallel]
# Example: tools/vivado/ooc.sh out/top_server out/vivado_ooc_server 10 1 4
set -euo pipefail
cd "$(dirname "$0")/../.."
SRC=${1:?usage: ooc.sh <top_gen dir> [out] [period] [impl] [parallel]}
OUT=${2:-out/vivado_ooc}
PERIOD=${3:-10.0}
IMPL=${4:-0}
PAR=${5:-4}
mkdir -p "$OUT"
# every module of the generated netlist except the board-level wrappers
grep -E '^module arty_top_' "$SRC/arty_top.v" | sed -E 's/^module (\w+).*/\1/' | sort -u > "$OUT/modules.txt"
echo "$(wc -l < "$OUT/modules.txt") units"
run_one() {
    m=$1; SRC=$2; OUT=$3; PERIOD=$4; IMPL=$5
    vivado -mode batch -nojournal -log "$OUT/$m.log" -source tools/vivado/ooc.tcl \
        -tclargs src="$SRC" module="$m" period="$PERIOD" impl="$IMPL" out="$OUT" jobs=1 > /dev/null 2>&1 \
        || echo "FAILED $m (see $OUT/$m.log)"
}
export -f run_one
xargs -P "$PAR" -I{} bash -c 'run_one {} "$@"' _ "$SRC" "$OUT" "$PERIOD" "$IMPL" < "$OUT/modules.txt"
python3 tools/vivado/summarize.py ooc "$OUT"
