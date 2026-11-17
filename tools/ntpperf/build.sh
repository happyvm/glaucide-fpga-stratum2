#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# tools/ntpperf/build.sh: builds everything the ntpperf load test needs
# (task NTPPERF, docs/ntpperf-results.md). Idempotent.
#
#  1. our replacement libpcap (tools/ntpperf/pcap_shim/pcap.c + pcap/pcap.h,
#     Apache-2.0) as a static library out/ntpperf/pcap/libpcap.a;
#  2. the time shim out/ntpperf/ntpperf_vtime.so (pcap_shim/vtime.c, LD_PRELOAD);
#  3. ntpperf itself: cloned (git, pinned commit) into out/ntpperf/src and
#     built with its own unmodified Makefile, pointed at our header and
#     library through make variables only:
#       make CPPFLAGS="-D_GNU_SOURCE -I<our include dir>" \
#            LDFLAGS="-L<our lib dir> -lpcap -lm"
#     (its Makefile links with `$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)`, LDFLAGS
#     being `-lpcap -lm`). ntpperf is GPL-2.0-or-later: it is downloaded,
#     built and run here, never committed nor modified (no patch needed);
#  4. the simulation bridge (cargo, release).
#
# Downloaded material is UNTRUSTED data: it stays under out/ntpperf/ (ignored
# by git); only its own Makefile runs, in its own directory.
set -euo pipefail

REPO=https://github.com/mlichvar/ntpperf.git
# Commit tested (2024-..., "fix aliasing issue breaking IP checksum").
PIN=${NTPPERF_PIN:-aee5446}
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
OUT="$ROOT/out/ntpperf"
SHIM_SRC="$ROOT/tools/ntpperf/pcap_shim"
mkdir -p "$OUT/pcap"

# 1. libpcap replacement
if [ ! -f "$OUT/pcap/libpcap.a" ] || [ "$SHIM_SRC/pcap.c" -nt "$OUT/pcap/libpcap.a" ] ||
    [ "$SHIM_SRC/pcap/pcap.h" -nt "$OUT/pcap/libpcap.a" ] || [ "$SHIM_SRC/simlink.h" -nt "$OUT/pcap/libpcap.a" ]; then
    gcc -O2 -Wall -Wextra -c -I"$SHIM_SRC" -o "$OUT/pcap/pcap.o" "$SHIM_SRC/pcap.c"
    rm -f "$OUT/pcap/libpcap.a"
    ar rcs "$OUT/pcap/libpcap.a" "$OUT/pcap/pcap.o"
fi
echo "libpcap (replacement): $OUT/pcap/libpcap.a"

# 2. time shim
if [ ! -f "$OUT/ntpperf_vtime.so" ] || [ "$SHIM_SRC/vtime.c" -nt "$OUT/ntpperf_vtime.so" ] ||
    [ "$SHIM_SRC/simlink.h" -nt "$OUT/ntpperf_vtime.so" ]; then
    gcc -O2 -Wall -Wextra -shared -fPIC -I"$SHIM_SRC" -o "$OUT/ntpperf_vtime.so" "$SHIM_SRC/vtime.c" -ldl
fi
echo "time shim: $OUT/ntpperf_vtime.so"

# 3. ntpperf
if [ ! -d "$OUT/src/.git" ]; then
    rm -rf "$OUT/src"
    git clone --quiet "$REPO" "$OUT/src"
fi
(
    cd "$OUT/src"
    if [ "$(git rev-parse --short=7 HEAD)" != "${PIN:0:7}" ]; then
        git fetch --quiet --depth 50 origin || true
        git -c advice.detachedHead=false checkout --quiet "$PIN" ||
            { echo "ntpperf: commit $PIN not found" >&2; exit 1; }
    fi
    if [ ! -x ntpperf ] || [ "$OUT/pcap/libpcap.a" -nt ntpperf ]; then
        make --quiet clean >/dev/null
        make --quiet CPPFLAGS="-D_GNU_SOURCE -I$SHIM_SRC" LDFLAGS="-L$OUT/pcap -lpcap -lm" >"$OUT/make.log" 2>&1 ||
            { cat "$OUT/make.log" >&2; exit 1; }
    fi
)
echo "ntpperf: $OUT/src/ntpperf (commit $(cd "$OUT/src" && git rev-parse --short HEAD))"

# 4. bridge
(cd "$ROOT" && CARGO_INCREMENTAL=0 cargo build --release -p ntp_fpga --bin ntpperf_bridge)
echo "bridge: $ROOT/target/release/ntpperf_bridge"
