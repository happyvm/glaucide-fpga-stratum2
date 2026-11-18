#!/usr/bin/env bash
# Everything, in one go, for the server board and the stratum 2 (client) board.
#   tools/vivado/run_all.sh [quick|full]
# quick: the two full analyses + per-unit out-of-context synthesis + a small strategy sweep.
# full : same with place-and-route of every unit and the large sweep (hours).
# Results in out/vivado_<board>/ ; the archive out/vivado_results.tar.gz (without checkpoints)
# is what to send back.
set -euo pipefail
cd "$(dirname "$0")/../.."
MODE=${1:-quick}
command -v vivado >/dev/null || { echo "vivado not in PATH (source settings64.sh)"; exit 1; }
cargo run -q -p ntp_fpga --bin top_gen out/top_server
cargo run -q -p ntp_fpga --bin top_gen -- --with-client out/top_client
for b in server client; do
  vivado -mode batch -nojournal -log out/vivado_$b.log -source tools/vivado/analyze.tcl \
      -tclargs src=out/top_$b out=out/vivado_$b jobs=4 simnetlist=0
  python3 tools/vivado/summarize.py run out/vivado_$b
done
if [ "$MODE" = full ]; then IMPL=1; SW=full; else IMPL=0; SW=quick; fi
for b in server client; do
  tools/vivado/ooc.sh out/top_$b out/vivado_ooc_$b 10 $IMPL 4
  tools/vivado/sweep.sh out/vivado_$b/post_synth.dcp out/vivado_sweep_$b 3 $SW
done
tools/vivado/pack.sh
