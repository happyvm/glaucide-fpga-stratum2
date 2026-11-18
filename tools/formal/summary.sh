#!/usr/bin/env bash
# Markdown table of formal results, one row per unit.
#   tools/formal/summary.sh [dir...]     (default: out/formal/*/)
# Each dir is a unit's output of prove.sh. The status comes from the
# `DONE (STATUS, rc=N)` line of run.log (stdout of prove.sh, saved by the
# workflow); without it, from the marker file sby writes in its work dir
# (out/formal/<unit>/<unit>/PASS, FAIL, ...). No marker either: NO_RESULT,
# typically a timeout killed sby before it finished.
# This script only reports: the job fails through prove.sh's exit code.
# Relative paths are relative to the repository root.
set -uo pipefail
cd "$(dirname "$0")/../.."

dirs=("$@")
[ ${#dirs[@]} -eq 0 ] && dirs=(out/formal/*/)

echo "| Unit | Result | Detail |"
echo "|---|---|---|"
for d in "${dirs[@]}"; do
  d=${d%/}
  unit=$(basename "$d")
  # Skip non-unit dirs such as out/formal/bin (prove.sh's tool shims).
  [ -f "$d/run.log" ] || [ -f "$d/$unit.sby" ] || continue
  status=""
  detail=""

  if [ -f "$d/run.log" ]; then
    line=$(grep -o 'DONE ([A-Z]*, rc=[0-9]*)' "$d/run.log" | tail -n 1 || true)
    if [ -n "$line" ]; then
      status=$(sed -E 's/^DONE \(([A-Z]+), rc=([0-9]+)\)$/\1/' <<<"$line")
      detail="rc=$(sed -E 's/^DONE \(([A-Z]+), rc=([0-9]+)\)$/\2/' <<<"$line")"
    fi
  fi

  if [ -z "$status" ]; then
    for m in PASS FAIL UNKNOWN TIMEOUT ERROR CANCELLED; do
      if [ -f "$d/$unit/$m" ]; then
        status=$m
        detail="sby marker file"
        break
      fi
    done
  fi

  if [ -z "$status" ]; then
    status=NO_RESULT
    detail="no DONE line (timeout or crash before sby finished)"
  fi

  echo "| \`$unit\` | **$status** | $detail |"
done
