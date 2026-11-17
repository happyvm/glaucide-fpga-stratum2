#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# tools/chrony/build.sh: downloads and builds chrony (chronyd + chronyc) into
# out/chrony/, builds the virtual-time shim (vtime_shim.so) and the bridge
# (cargo, release). Idempotent: every step is skipped when its product exists.
#
# Downloaded material is UNTRUSTED data: it stays under out/chrony/ (ignored
# by git), is never committed, and only its own `configure`/`make` run, in
# its own directory.
#
# chrony is configured for this co-simulation:
#   --disable-timestamping  no SO_TIMESTAMPING (kernel software TX/RX stamps
#                           are wall-clock values; the shim also refuses
#                           SO_TIMESTAMP/SO_TIMESTAMPNS for the same reason)
#   --disable-ipv6          the container kernel has no AF_INET6 sockets
#   --disable-privdrop --without-seccomp --disable-scfilter
#                           runs as root in a private directory, no syscall
#                           filter (the shim adds no syscalls but keeps it simple)
#   --disable-nts --without-nettle --without-gnutls --without-readline
#   --without-editline      no crypto/readline dependencies
# cmdmon stays enabled: chronyc queries chronyd over a UNIX socket.
set -euo pipefail

VER=4.9
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
OUT="$ROOT/out/chrony"
SRC="$OUT/src"
INST="$OUT/inst"
mkdir -p "$OUT"

if [ ! -x "$INST/sbin/chronyd" ] || [ ! -x "$INST/bin/chronyc" ]; then
    if [ ! -f "$OUT/chrony-$VER.tar.gz" ]; then
        curl -fsSL --retry 3 -o "$OUT/chrony-$VER.tar.gz.part" \
            "https://chrony-project.org/releases/chrony-$VER.tar.gz"
        mv "$OUT/chrony-$VER.tar.gz.part" "$OUT/chrony-$VER.tar.gz"
    fi
    rm -rf "$SRC"
    mkdir -p "$SRC"
    tar -xzf "$OUT/chrony-$VER.tar.gz" -C "$SRC" --strip-components=1 --no-same-owner
    (
        cd "$SRC"
        ./configure --prefix="$INST" --disable-ipv6 --without-readline --without-editline \
            --disable-privdrop --without-seccomp --disable-scfilter --without-nettle \
            --without-gnutls --disable-nts --disable-timestamping >"$OUT/configure.log" 2>&1
        make -j2 chronyd chronyc >"$OUT/make.log" 2>&1
    )
    mkdir -p "$INST/sbin" "$INST/bin"
    cp "$SRC/chronyd" "$INST/sbin/chronyd"
    cp "$SRC/chronyc" "$INST/bin/chronyc"
fi
echo "chrony: $("$INST/sbin/chronyd" -v)"

if [ ! -f "$OUT/vtime_shim.so" ] || [ "$ROOT/tools/chrony/vtime_shim.c" -nt "$OUT/vtime_shim.so" ]; then
    gcc -O2 -Wall -shared -fPIC -o "$OUT/vtime_shim.so" "$ROOT/tools/chrony/vtime_shim.c" -ldl
fi
echo "shim: $OUT/vtime_shim.so"

(cd "$ROOT" && CARGO_INCREMENTAL=0 cargo build --release -p ntp_fpga --bin chrony_bridge)
echo "bridge: $ROOT/target/release/chrony_bridge"
