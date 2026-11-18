#!/usr/bin/env bash
# Archive of everything useful (reports, CSV, JSON, logs, constraints used) without the
# checkpoints and bitstreams, which are large.  Output: out/vivado_results.tar.gz
set -euo pipefail
cd "$(dirname "$0")/../.."
OUT=${1:-out/vivado_results.tar.gz}
FILES=$(ls -d out/vivado_* 2>/dev/null | tr '\n' ' ')
[ -n "$FILES" ] || { echo "no out/vivado_* directory"; exit 1; }
git rev-parse HEAD > out/vivado_commit.txt 2>/dev/null || true
tar --exclude='*.dcp' --exclude='*.bit' --exclude='*.sdf' --exclude='*timesim.v' \
    --exclude='.Xil' -czf "$OUT" $FILES out/vivado_commit.txt constraints tools/vivado/top_timing.xdc
ls -lh "$OUT"
