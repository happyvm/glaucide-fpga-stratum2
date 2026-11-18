# Placement/routage indicatif (nextpnr-xilinx, sans Vivado)

Indication seulement : nextpnr-xilinx place et route moins bien que Vivado et son modèle de délais est approximatif.
Mesure du cœur `arty_top` (sans MMCM, BUFG, E/S réelles), toutes horloges visées à 100 MHz.

1. Outil : `nix build nixpkgs#nextpnr-xilinx` (contient `bbasm` et la base prjxray).
2. Chipdb XC7A100T (≈ 2 min) : `bbaexport.py --device xc7a100tcsg324-1 --bba xc7a100t.bba --constids constids.inc` (avec `PYTHONPATH` sur `share/nextpnr/python`), puis `bbasm --l xc7a100t.bba xc7a100t.bin`.
3. Netlist : `cargo run -p ntp_fpga --bin top_gen [-- --with-client] <dossier>`.
4. Synthèse : `python tools/pnr/synth.py <dossier>/arty_top.v out.json` (pyosys ; DSP mappés en LUT, boucles d'init de mémoire retirées, `RAM256X1S` éclaté en `RAM64M` + LUT car non géré), puis `python tools/pnr/fixdir.py out.json` (corrige le sens des ports `RAM64M`).
5. `nextpnr-xilinx --chipdb xc7a100t.bin --xdc tools/pnr/core.xdc --json out.json --freq 100 --seed N` (4 à 10 min).
