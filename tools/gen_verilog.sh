#!/usr/bin/env bash
# Generate Verilog for every module into out/.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p out
cargo run -q -p ntp_fpga --bin clock_sim -- --verilog > out/ntp_clock.v
