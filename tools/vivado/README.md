# Construction Vivado (Arty A7-100T)

Script non interactif (tâche T0.5) : synthèse, placement-routage, bitstream et rapports, en flot
« non-projet ». **Il n'a pas été exécuté** : Vivado n'est pas disponible dans l'environnement où il a été
écrit. Seule la partie Verilog a été vérifiée (yosys : `hierarchy -check`, `proc`, `check -assert`, et
une synthèse `synth_xilinx` sans erreur jusqu'au mappage ABC, qui ne s'exécute pas avec le yosys WebAssembly
disponible : environ 14 900 bascules et 343 primitives de RAM distribuée `RAM64M`/`RAM32M` avant
optimisation par Vivado, aucun nombre de LUT fiable).

## Utilisation

```sh
cargo run -p ntp_fpga --bin top_gen                          # écrit out/top/
vivado -mode batch -source tools/vivado/build.tcl            # résultats dans out/vivado/
vivado -mode batch -source tools/vivado/build.tcl -tclargs /chemin/vers/build
```

Composant : `xc7a100tcsg324-1`. Top : `arty_top_board` (`rtl/hdl/arty_top_board.v`, copié dans
`out/top/` par `top_gen`).

## Entrées

| Fichier | Rôle |
|---|---|
| `out/top/files.f` | liste des sources dans l'ordre : `arty_top.v` (généré), `arty_top_board.v` (enveloppe écrite à la main) |
| `constraints/arty_a7_100.xdc` | broches, horloges primaires, délais d'E/S MII |
| `tools/vivado/top_timing.xdc` | horloge transmise `eth_ref_clk`, groupes asynchrones incluant les horloges du MMCM, faux chemins MDIO/MDC/reset PHY |

`top_gen -- --with-client` produit la variante phase B (`ArtyBoardWithClient`) au même endroit.

## Sorties (`out/vivado/`)

- `arty_ntp.bit` ;
- `post_synth.dcp`, `post_route.dcp` ;
- `reports/` : `timing_summary.rpt`, `utilization.rpt`, `utilization_hier.rpt`, `clock_utilization.rpt`,
  `clock_interaction.rpt`, `cdc.rpt`, `methodology.rpt`, `drc.rpt`, `io.rpt`, plus les rapports
  post-synthèse.

Le script échoue (code non nul) si WNS ou WHS est négatif.

Précision : le bitstream est écrit **avant** le contrôle de WNS/WHS. Après un échec, `arty_ntp.bit` existe
mais correspond à un design qui ne respecte pas le timing : ne le chargez pas sur la carte.

La suite (programmation, lecture des rapports, vérifications sur la carte) est dans
[../../docs/bringup.md](../../docs/bringup.md).

## Points à vérifier au premier passage (T5.3)

- `IOB TRUE` sur les entrées MII : `mii_rx` n'a pas de bascule directement sur `eth_rxd`/`eth_rx_dv`/
  `eth_rx_er` (la logique de décodage précède le registre), Vivado signalera que la propriété ne peut pas
  être honorée. Les sorties `eth_txd`/`eth_tx_en` sont des registres de `mii_tx` sans autre charge :
  elles doivent être placées dans l'IOB.
- Horloges : 7 tampons globaux (BUFG retour MMCM, `clk_sys`, `clk_25`, copies libres `eth_rx_clk` /
  `eth_tx_clk`, deux `BUFGCE`). `eth_rx_clk` (F15) et `eth_tx_clk` (H16) arrivent sur des broches
  d'horloge ; chacune attaque un BUFG et un BUFGCE.
- `report_cdc` : les passages attendus sont ceux de `rx_path` / `tx_path` (Gray, poignée de main) et les
  synchroniseurs à deux bascules `ASYNC_REG` de l'enveloppe ; tout autre passage est une erreur.
- Chemins larges à surveiller (en-têtes des unités) : somme `pseudo_v6` de `icmpv6` (19 additions
  enchaînées), sommes de contrôle 128 bits, comparateurs 64 bits de `ntp_server`.

## Analyse complète (extraction maximale d'informations)

Scripts ajoutés pour un premier passage sous Vivado qui ne s'arrête pas sur un échec de timing et
produit des fichiers lisibles sans Vivado (CSV, JSON, Markdown). **Non exécutés** (Vivado indisponible
là où ils ont été écrits) : le déroulement de chaque script a été vérifié avec des commandes Vivado
simulées (aucune erreur de Tcl), pas avec le vrai outil ; chaque rapport est protégé par un `catch`
(un rapport inconnu de votre version est consigné dans `reports/_skipped.log` et le reste continue).

| Script | Rôle |
|---|---|
| `run_all.sh [quick\|full]` | tout, pour la carte serveur et la carte client, puis l'archive à renvoyer |
| `analyze.tcl` | flot complet (synthèse, implémentation, tous les rapports) ; options `key=value` : répertoires, directives de synthèse/placement/routage, `retiming`, `clk_sys_mhz=125` (marge à une fréquence plus haute), `simnetlist=1` (netlist + SDF post-routage), `bitstream=1` |
| `synth_only.tcl` | synthèse seule (quelques minutes) : surface, profondeur de logique, fanout |
| `ooc.sh` / `ooc.tcl` | synthèse (et routage avec `impl=1`) de **chaque unité isolément** avec sa propre horloge : surface et fréquence estimée par unité |
| `sweep.sh` / `sweep.tcl` | la même synthèse implémentée avec plusieurs directives, en parallèle : mesure la part de chance du placement |
| `report_dcp.tcl` | relance tous les rapports sur un point de contrôle existant |
| `summarize.py run\|ooc\|sweep <dir>` | tableaux Markdown (`SUMMARY.md`, `OOC.md`, `SWEEP.md`) |
| `pack.sh` | archive `out/vivado_results.tar.gz` (rapports, CSV, journaux, sans les `.dcp`/`.bit`) |

Premier passage conseillé :

```sh
source /opt/Xilinx/Vivado/<version>/settings64.sh
tools/vivado/run_all.sh quick        # 1 à 3 h selon la machine ; renvoyer out/vivado_results.tar.gz
```

ou, pour la seule carte serveur : `cargo run -p ntp_fpga --bin top_gen out/top_server`, puis
`vivado -mode batch -source tools/vivado/analyze.tcl -tclargs src=out/top_server out=out/vivado_server`
et `python3 tools/vivado/summarize.py run out/vivado_server`.

Ce que contient `out/vivado_<carte>/` :

- `summary.json` : WNS/TNS/WHS/THS, pire marge par horloge, décompte des primitives ;
- `reports/unit_timing.csv` : pire chemin se terminant dans chaque unité (marge, niveaux de LUT, délai de
  logique contre délai de routage) : montre **quelle unité limite l'horloge** ;
- `reports/paths_setup.csv`, `paths_hold.csv` : les 500 et 200 pires chemins ;
- `timing_summary.rpt`, `timing_setup_paths.rpt`, `timing_hold_paths.rpt`, `timing_to_clock_*.rpt`,
  `timing_inputs.rpt`, `timing_outputs.rpt`, `exceptions.rpt`, `check_timing.rpt`, `pulse_width.rpt`,
  `bus_skew.rpt`, `clock_interaction.rpt`, `clock_networks.rpt`, `clock_utilization.rpt` ;
- `utilization*.rpt/.csv` (par hiérarchie), `ram_utilization.rpt`, `carry_chains.rpt`, `control_sets.rpt` ;
- `fanout.rpt`, `route_status.rpt`, `design_complexity.rpt`, `design_timing.rpt`, `logic_levels.rpt`,
  `congestion.rpt`, `qor_summary.rpt`, `qor_suggestions.rpt` ;
- `cdc.rpt`, `cdc_summary.rpt`, `methodology.rpt`, `drc.rpt`, `io.rpt`, `power.rpt`, `power_hier.rpt`,
  `config_timing.rpt` ;
- les points de contrôle `post_synth/opt/place/physopt/route.dcp` (à ouvrir dans Vivado pour inspecter
  un chemin, non inclus dans l'archive).

À regarder en premier : `SUMMARY.md` (marge par horloge, unités classées par marge), `cdc_summary.rpt`
(tout passage hors `rx_path`/`tx_path`/synchroniseurs de l'enveloppe est une erreur), `methodology.rpt`,
puis la part routage/logique des pires chemins (si le routage domine, c'est le placement ou le fanout).
