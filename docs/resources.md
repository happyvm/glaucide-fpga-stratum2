# Consommation de ressources (estimation)

Cible : XC7A100T = **63 400 LUT**, 126 800 bascules, 135 BRAM36, 240 DSP.
Mesure : `tools/estimate_resources.py` (Yosys natif `synth_xilinx`, module par module, sans Vivado). **Estimation à ±15 %**, sans niveau supérieur ni analyse de timing. Photographie au commit de fusion de T2.3.

| Module | LUT | LUT-RAM | Bascules |
|---|---:|---:|---:|
| icmpv6 | 1 974 | 0 | 638 |
| ntp_server | 1 224 | 32 | 866 |
| arp | 886 | 0 | 738 |
| tx_path | 478 | 768 | 261 |
| rx_path | 403 | 556 | 286 |
| ipv6_rx | 392 | 0 | 474 |
| ipv6_tx | 374 | 0 | 361 |
| udp4_tx | 354 | 24 | 251 |
| ipv4_tx | 322 | 0 | 200 |
| disciplined_clock | 279 | 0 | 97 |
| udp4_rx | 262 | 0 | 283 |
| eth_tx | 198 | 0 | 160 |
| ipv4_rx | 193 | 0 | 268 |
| tx_arb | 157 | 0 | 3 |
| csum | 151 | 0 | 56 |
| icmp_echo | 151 | 0 | 189 |
| phy_init | 141 | 0 | 124 |
| eth_rx | 117 | 0 | 175 |
| frame_crc | 95 | 0 | 32 |
| fcs_check | 84 | 0 | 153 |
| mii_tx | 49 | 0 | 36 |
| mii_rx | 25 | 0 | 31 |
| autres (horloge libre, config, compteur de test) | 17 | 0 | 112 |
| **Total** | **≈ 8 380** | **1 380** | **5 794** |

**≈ 9 760 LUT (15,4 %)**, **5 794 bascules (4,6 %)**, **0 BRAM, 0 DSP**.

## Ce qui n'est pas compté
- Niveau supérieur (T1.11) : MMCM, tampons E/S, synchroniseurs de reset, **canaux registrés entre unités** (une centaine de liaisons valid/ready), à estimer à quelques milliers de LUT/bascules.
- Pas encore écrits : UDP IPv6 (T3.4), client NTP, filtre, sélection, servo (T4.2 à T4.6) : le client est probablement du même ordre que `ntp_server`.
- Aucune optimisation inter-modules.

## Où gagner si besoin
- `tx_path` (4096 octets) et `rx_path` (2048 mots) sont en **RAM distribuée** (1 324 LUT) : un passage en BRAM36 (2 blocs) libérerait ces LUT. Demande une lecture registrée et une refonte de leurs preuves.
- `icmpv6` (1 974 LUT) est le plus gros module : comparaisons d'adresses 128 bits et graine de checksum à une seule étape (risque de timing, voir T5.3).
