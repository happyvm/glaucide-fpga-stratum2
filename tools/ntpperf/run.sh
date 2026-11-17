#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# tools/ntpperf/run.sh: load test of the server-only design with ntpperf
# (task NTPPERF, docs/ntpperf-results.md). Run tools/ntpperf/build.sh first.
#
#   tools/ntpperf/run.sh [--long] [--jobs N] [JOB...]
#
# Every job is one ntpperf invocation under the bridge (one full simulation
# of the design); jobs run N at a time (default 4), the most expensive
# first. Results in out/ntpperf/results/<job>/: ntpperf.txt (ntpperf's
# table), bridge.log (status lines and the bridge's own summary: ground
# truth of every stamp), requests.csv (one line per request), cmd.txt.
# Then tools/ntpperf/analyse.py checks everything and writes
# out/ntpperf/results/summary.md (exit code 1 if a check fails).
#
# Default set (about 15 minutes of wall time on 4 cores): ntpperf's own
# sampling interval is set to its minimum, 0.2 s of VIRTUAL time (each rate
# point lasts 0.5 virtual s with ntpperf's warm-up), 1.0 s for the
# rate-limiter jobs (whole one-second windows). --long adds full sweeps
# (several hours) and the validation of the idle-cycle skipping.
#
# Jobs (name: bridge options | ntpperf options):
#   a-sweep    no limiter | -B -r 1000-11390 -x 1.5   response time, 7 rates
#   a-25k      no limiter | -B -r 25000
#   a-60k      no limiter | -B -r 60000
#   a-105k     no limiter | -B -r 105000             just under the line rate
#   a-125k     no limiter | -B -r 125000             over the line rate
#   b-sweep    no limiter | -B -o 1e-12 -H -r 1000-3375 -x 1.5   TX offset
#   b-105k     no limiter | -B -o 1e-12 -H -r 105000
#   b-125k     no limiter | -B -o 1e-12 -H -r 125000
#   c-1k       --rate-limit 8 | -B -t 1.0 -r 1000    default limiter, 100 sources
#   c-10k      --rate-limit 8 | -B -t 1.0 -r 10000   1 000 sources
#   d-kod-1k   --rate-limit 8 --kod | -B -t 1.0 -r 1000
#   d-kod-o-1k --rate-limit 8 --kod | -B -t 1.0 -o 1e-12 -H -r 1000
#   x-xleave-1k no limiter | -I -t 0.2 -r 1000       interleaved-mode requests
# --long adds:
#   la-full    no limiter | -B -r 1000-200000 -x 1.5   ntpperf's default sweep
#   lb-full    no limiter | -B -o 1e-12 -H -r 1000-200000 -x 1.5
#   lc-30k     --rate-limit 8 | -B -t 1.0 -r 30000     3 000 sources
#   lc-80k     --rate-limit 8 | -B -t 1.0 -r 80000     8 000 sources
#   ld-kod-10k --rate-limit 8 --kod | -B -t 1.0 -r 10000
#   l-noff     no limiter, --no-ff | -B -r 1000          every cycle simulated
#   l-ff       no limiter          | -B -r 1000          the same with skipping
#
# -o 1e-12: ntpperf prints the TX timestamp offset only for a non-zero
# correction; 1 ps changes no printed digit. -H: the RX time stamp of a
# reply is the virtual instant of its SFD on the TX pins (no PHY, no cable).
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/../.." && pwd)
OUT="$ROOT/out/ntpperf"
RES="$OUT/results"
BRIDGE="$ROOT/target/release/ntpperf_bridge"
NTPPERF="$OUT/src/ntpperf"
SHIM="$OUT/ntpperf_vtime.so"
JOBS=4
LONG=0
SEL=()
while [ $# -gt 0 ]; do
    case "$1" in
    --long) LONG=1 ;;
    --jobs) JOBS=$2; shift ;;
    -h|--help) sed -n '3,45p' "$0"; exit 0 ;;
    *) SEL+=("$1") ;;
    esac
    shift
done
for f in "$BRIDGE" "$NTPPERF" "$SHIM"; do
    [ -e "$f" ] || { echo "missing $f: run tools/ntpperf/build.sh" >&2; exit 1; }
done

NET="-i sim0 -m 02:00:00:00:00:01 -d 192.168.123.1 -s 172.18.0.0/16"
# name|bridge options|ntpperf options|estimated cost (10^6 simulated cycles)
specs() {
    cat <<EOF
a-sweep|--rate-limit 0|-B -t 0.2 -r 1000-11390 -x 1.5|36
a-25k|--rate-limit 0|-B -t 0.2 -r 25000|28
a-60k|--rate-limit 0|-B -t 0.2 -r 60000|50
a-105k|--rate-limit 0|-B -t 0.2 -r 105000|50
a-125k|--rate-limit 0|-B -t 0.2 -r 125000|50
b-sweep|--rate-limit 0|-B -t 0.2 -o 1e-12 -H -r 1000-3375 -x 1.5|9
b-105k|--rate-limit 0|-B -t 0.2 -o 1e-12 -H -r 105000|50
b-125k|--rate-limit 0|-B -t 0.2 -o 1e-12 -H -r 125000|50
c-1k|--rate-limit 8|-B -t 1.0 -r 1000|3
c-10k|--rate-limit 8|-B -t 1.0 -r 10000|29
d-kod-1k|--rate-limit 8 --kod|-B -t 1.0 -r 1000|3
d-kod-o-1k|--rate-limit 8 --kod|-B -t 1.0 -o 1e-12 -H -r 1000|3
x-xleave-1k|--rate-limit 0|-I -t 0.2 -r 1000|1
EOF
    if [ "$LONG" = 1 ]; then
        cat <<EOF
la-full|--rate-limit 0|-B -t 0.2 -r 1000-200000 -x 1.5|500
lb-full|--rate-limit 0|-B -t 0.2 -o 1e-12 -H -r 1000-200000 -x 1.5|500
lc-30k|--rate-limit 8|-B -t 1.0 -r 30000|87
lc-80k|--rate-limit 8|-B -t 1.0 -r 80000|130
ld-kod-10k|--rate-limit 8 --kod|-B -t 1.0 -r 10000|29
l-noff|--rate-limit 0 --no-ff|-B -t 0.2 -r 1000|50
l-ff|--rate-limit 0|-B -t 0.2 -r 1000|1
EOF
    fi
}

selected() {
    [ ${#SEL[@]} -eq 0 ] && return 0
    local s
    for s in "${SEL[@]}"; do [ "$s" = "$1" ] && return 0; done
    return 1
}

run_one() {
    local name=$1 bopts=$2 nopts=$3
    local R="$RES/$name"
    rm -rf "$R"
    mkdir -p "$R"
    local t0=$SECONDS
    # shellcheck disable=SC2086
    echo "$BRIDGE --shim $SHIM --shm $R/vtime.shm $bopts --log $R/requests.csv -- $NTPPERF $NET $nopts" >"$R/cmd.txt"
    # shellcheck disable=SC2086
    "$BRIDGE" --shim "$SHIM" --shm "$R/vtime.shm" $bopts --log "$R/requests.csv" \
        --status 30 --max-wall 7200 -- "$NTPPERF" $NET $nopts >"$R/ntpperf.txt" 2>"$R/bridge.log" ||
        echo "bridge exited with $?" >>"$R/bridge.log"
    rm -f "$R/vtime.shm"
    echo "$((SECONDS - t0))" >"$R/wall_s.txt"
    echo "== $name done in $((SECONDS - t0)) s"
}
export -f run_one
export RES BRIDGE NTPPERF SHIM NET

mkdir -p "$RES"
T0=$SECONDS
# Most expensive first (longest-processing-time scheduling on N workers).
specs | sort -t'|' -k4,4nr | while IFS='|' read -r name b n cost; do
    selected "$name" && printf '%s\0%s\0%s\0' "$name" "$b" "$n"
done | xargs -0 -n 3 -P "$JOBS" bash -c 'run_one "$0" "$1" "$2"'
echo "all jobs done in $((SECONDS - T0)) s" | tee "$RES/wall_total.txt"
python3 -I "$ROOT/tools/ntpperf/analyse.py" "$RES"
