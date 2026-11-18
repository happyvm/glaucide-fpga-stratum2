# Consommation de ressources (estimation)

Cible : XC7A100T = **63 400 LUT**, 126 800 bascules, 135 BRAM36, 240 DSP.
Mesure : Yosys natif `synth_xilinx -family xc7` (sans Vivado), **estimation à ±15 %**, sans analyse de timing. Mise à jour au commit `0cc611f` (limiteur de débit, KoD, client complet, filtre, sélection, servo inclus).

## Cartes complètes (niveau supérieur aplati, E/S et canaux compris)

| Carte | LUT logiques | LUT-RAM | **LUT total** | Bascules | BRAM | DSP | CARRY4 |
|---|---:|---:|---:|---:|---:|---:|---:|
| `ArtyBoard` (serveur seul, 45 modules) | 16 258 | 3 420 | **19 678 (31,0 %)** | 15 441 (12,2 %) | 0 | 0 | 499 |
| `ArtyBoardWithClient` (stratum 2, 53 modules) | 35 130 | 4 060 | **39 190 (61,8 %)** | 30 313 (23,9 %) | 0 | 12 (5 %) | 1 765 |

Reproduction : `cargo run -p ntp_fpga --bin top_gen [-- --with-client] <dossier>`, puis synthèse de `arty_top.v` (sommet `arty_top`).

## Par module (synthèse isolée, sans optimisation inter-modules)

| Module | LUT | LUT-RAM | Bascules | DSP |
|---|---:|---:|---:|---:|
| selection | 5 094 | 0 | 3 213 | 4 |
| icmpv6 | 2 811 | 0 | 812 | 0 |
| ntp_client | 2 433 | 104 | 955 | 4 |
| clock_filter | 1 928 | 556 | 977 | 0 |
| ntp_server | 1 767 | 2 048 | 1 198 | 0 |
| servo | 1 279 | 0 | 440 | 0 |
| nbr_cache | 1 129 | 0 | 710 | 0 |
| arp | 1 035 | 0 | 771 | 0 |
| sync_state | 675 | 0 | 970 | 4 |
| udp6_tx | 663 | 24 | 495 | 0 |
| ipv6_rx | 618 | 0 | 475 | 0 |
| udp6_rx | 508 | 0 | 474 | 0 |
| tx_path | 503 | 768 | 274 | 0 |
| rx_path | 394 | 556 | 286 | 0 |
| ipv6_tx | 374 | 0 | 361 | 0 |
| udp4_tx | 354 | 24 | 251 | 0 |
| ipv4_tx | 323 | 0 | 200 | 0 |
| disciplined_clock | 279 | 0 | 97 | 0 |
| udp4_rx | 273 | 0 | 283 | 0 |
| eth_tx | 195 | 0 | 160 | 0 |
| ipv4_rx | 182 | 0 | 269 | 0 |
| tx_arb | 156 | 0 | 3 | 0 |
| icmp_echo | 155 | 0 | 189 | 0 |
| csum | 151 | 0 | 56 | 0 |
| phy_init | 145 | 0 | 124 | 0 |
| eth_rx | 115 | 0 | 175 | 0 |
| frame_crc | 95 | 0 | 32 | 0 |
| fcs_check | 84 | 0 | 153 | 0 |
| mii_tx | 49 | 0 | 36 | 0 |
| mii_rx | 25 | 0 | 31 | 0 |
| autres (compteurs, démux, horloge libre) | 28 | 0 | 114 | 0 |
| **Somme** | **23 820** | **4 080** | **14 584** | **12** |

La somme des modules isolés ne vaut pas le niveau supérieur : elle compte une seule fois chaque module, alors que la carte instancie par exemple deux voies UDP/IP et des canaux registrés. Les chiffres de référence sont ceux des cartes complètes ci-dessus.

## Évolution depuis la première mesure (T2.3)
- Serveur seul : 9 760 LUT (15,4 %) en T2.3, sans niveau supérieur → **19 678 LUT (31,0 %)** avec niveau supérieur, IPv6, limiteur de débit et KoD.
- `ntp_server` : le limiteur de débit (table de 4 096 × 32 bits, `rl`) coûte **2 048 LUT-RAM** à lui seul, soit plus de la moitié de la LUT-RAM du serveur.
- Avec client : `selection` (5 094 LUT, 3 213 bascules) est le plus gros module ; les 12 DSP viennent de `selection` (4), `ntp_client` (4) et `sync_state` (4).
- Aucun BRAM n'est utilisé : toutes les mémoires sont en RAM distribuée.

## Où gagner si besoin
- Les mémoires de `rl` (4 096 mots), `tx_path` (4 096 octets), `rx_path` (2 048 mots) et `clock_filter` sont en **RAM distribuée** (≈ 3 400 LUT-RAM sur la carte serveur). Un passage en BRAM36 libérerait ces LUT ; il demande une lecture registrée et une refonte des preuves associées.
- `selection` : ses 5 094 LUT et ses 3 213 bascules dominent la carte client ; un calcul séquentiel plutôt que parallèle réduirait la surface au prix de cycles (le chemin n'est pas critique).
- `icmpv6` (2 811 LUT) : comparaisons d'adresses 128 bits et graine de checksum à une seule étape (risque de timing, voir `docs/timing.md`).

## Limites
- Un placement/routage indicatif (nextpnr-xilinx) donne des chiffres d'utilisation et de Fmax différents : voir `docs/timing.md`, section « Placement/routage indicatif ».
- Ni MMCM, ni tampons d'horloge BUFG/BUFGCE, ni contraintes : le comptage final dépend de Vivado (`tools/vivado/README.md`).
- Les marges de timing à 100 MHz ne sont pas mesurées ici.
