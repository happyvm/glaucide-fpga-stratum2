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
