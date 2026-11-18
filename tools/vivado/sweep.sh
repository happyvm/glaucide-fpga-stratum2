#!/usr/bin/env bash
# Implementation-strategy sweep: the same synthesis checkpoint implemented with several
# directive combinations in parallel (Vivado has no seed: directives are how its placer
# and router are varied). Shows how much of a timing failure is placement luck.
#   tools/vivado/sweep.sh <post_synth.dcp> [out dir] [parallel] [quick|full]
# Example: tools/vivado/sweep.sh out/vivado_server/post_synth.dcp out/vivado_sweep_server 4 quick
set -euo pipefail
cd "$(dirname "$0")/../.."
DCP=${1:?usage: sweep.sh <post_synth.dcp> [out] [parallel] [quick|full]}
OUT=${2:-out/vivado_sweep}
PAR=${3:-3}
MODE=${4:-quick}
mkdir -p "$OUT"
if [ "$MODE" = full ]; then
  PLACE="Default Explore ExtraNetDelay_high ExtraPostPlacementOpt AltSpreadLogic_high AltSpreadLogic_medium EarlyBlockPlacement WLDrivenBlockPlacement"
  PHYS="Explore AggressiveExplore AggressiveFanoutOpt AlternateFlowWithRetiming"
  ROUTE="Default Explore NoTimingRelaxation MoreGlobalIterations HigherDelayCost AggressiveExplore"
else
  PLACE="Default Explore ExtraNetDelay_high AltSpreadLogic_high"
  PHYS="Explore AggressiveExplore"
  ROUTE="Default Explore NoTimingRelaxation"
fi
: > "$OUT/jobs.txt"
for p in $PLACE; do for f in $PHYS; do for r in $ROUTE; do echo "$p $f $r" >> "$OUT/jobs.txt"; done; done; done
echo "$(wc -l < "$OUT/jobs.txt") implementations, $PAR in parallel"
xargs -P "$PAR" -L1 bash -c '
  vivado -mode batch -nojournal -log "'"$OUT"'/$0_$1_$2.log" -source tools/vivado/sweep.tcl \
     -tclargs dcp="'"$DCP"'" place="$0" physopt="$1" route="$2" out="'"$OUT"'" jobs=1 >/dev/null 2>&1 \
     || echo "FAILED $0 $1 $2"' < "$OUT/jobs.txt"
python3 tools/vivado/summarize.py sweep "$OUT"
