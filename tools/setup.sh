#!/usr/bin/env bash
# Clone TxHDL at the pinned commit into third_party/hdl-txhdl (not committed).
set -euo pipefail
cd "$(dirname "$0")/.."
PIN=cacb24130d7a4b81234ac415b96d7ce154675f0e
if [ ! -d third_party/hdl-txhdl/.git ]; then
  mkdir -p third_party
  git clone https://github.com/filmil/hdl-txhdl.git third_party/hdl-txhdl
fi
git -C third_party/hdl-txhdl fetch -q origin "$PIN" 2>/dev/null || true
git -C third_party/hdl-txhdl checkout -q "$PIN"
echo "TxHDL @ $PIN"
