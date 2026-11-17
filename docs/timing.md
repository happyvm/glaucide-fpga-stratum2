# Analyse temporelle (estimation structurelle, tâche T5.3)

Cible : Arty A7-100T, `xc7a100tcsg324-1` (vitesse -1), `clk_sys` = 100 MHz (10 ns), horloges MII
`clk_rx` / `clk_tx` = 25 MHz (40 ns). **Vivado n'est pas disponible ici** : ce document ne remplace
pas le rapport de timing de Vivado (`tools/vivado/build.tcl`, rapports `report_timing_summary`), il
repère les chemins longs par la structure de la netlist et décrit ce qui a été raccourci.

## 1. Méthode

Outil : `tools/estimate_timing.py` (Yosys natif via `pyosys`, comme `tools/estimate_resources.py`) :

```sh
tools/gen_verilog.sh
cargo run -p ntp_fpga --bin top_gen -- out/top_server
cargo run -p ntp_fpga --bin top_gen -- --with-client out/top_client
PATH=<venv>/lib/python3.13/site-packages/pyosys:$PATH <venv>/bin/python tools/estimate_timing.py -j 3 --json t.json
<venv>/bin/python tools/estimate_timing.py --compare avant.json apres.json   # tableau Markdown
```

Pour chaque `out/*.v` et pour les deux variantes du top (`out/top*/arty_top.v`, module `arty_top`,
sans l'enveloppe `arty_top_board`), l'outil lance `synth_xilinx -family xc7 -flatten`, lit la
netlist placée sur primitives (`write_json`) et calcule, **par domaine d'horloge**, le plus long
chemin registre → registre : nombre de niveaux de LUT, de `CARRY4`, de `MUXF7/F8`, registres de
départ et d'arrivée (`--path` donne la suite des cellules). Les chemins qui partent d'un port
d'entrée ou arrivent sur un port de sortie du module sont listés à part (dans le design complet ils
se prolongent dans l'unité voisine : c'est ce que montre l'analyse du top). Les chemins entre
domaines sont listés à part (« crossing »).

`ltp` de Yosys n'est pas utilisé : `ltp -noff` ne connaît que les bascules internes de Yosys
(`$dff`…) et, sur une netlist `synth_xilinx`, traverse les `FDRE` (essai sur un exemple de 3
registres : chemin rapporté de l'entrée `clk` à la sortie, à travers deux bascules).

### Modèle de délai (indicatif)

```
délai ≈ 1,0 ns (clock-to-q + setup) + 0,6 ns par niveau de LUT (logique + net moyen)
        + 0,1 ns par CARRY4 + 0,3 ns par MUXF7/MUXF8
        (+ 1,5 ns en sortie de BRAM ; DSP48E1 : A/B → M 2,0 ns, M/C/PCIN → P 1,3 ns,
         chaque étage coupé par son registre AREG/BREG, MREG, PREG)
```

Une lecture de LUT-RAM (adresse → donnée) compte pour un niveau de LUT ; la donnée stockée est
lancée par l'horloge d'écriture (une lecture dans un autre domaine est un croisement).

Seuils pour `clk_sys` (10 ns) : **critique** si l'estimation dépasse 8,5 ns ou si le chemin a plus
de 11 niveaux de LUT ; **à surveiller** entre 7 et 8,5 ns ; **OK** en dessous. Domaines MII :
budget 40 ns (critique au-delà de 34 ns).

### Limites (à lire avant les chiffres)

* Pas de placement, pas de routage, pas d'éventail (fanout) : le 0,6 ns par LUT est une moyenne.
  Un chemin de 8 LUT très chargé peut être plus lent qu'un chemin de 10 LUT local. Chemin
  « topologique », pas une analyse temporelle : ni setup/hold réels, ni incertitude d'horloge, ni
  skew (sauf la marge forfaitaire de 1 ns).
* **Additions en chaîne** : Yosys (`alumacc`) regroupe `a + b + c + …` en un arbre carry-save,
  Vivado garde une chaîne de retenue par `+`. L'outil lance l'étape `coarse` avec
  `alumacc -macc-only` pour reproduire le comportement de Vivado (`--yosys-macc` : l'arbre de
  Yosys, plus optimiste). Exemple : 19 additions de 16 bits en chaîne, 18 LUT + 23 CARRY4
  (14,1 ns) ; avec l'arbre de Yosys, 7 LUT + 6 CARRY4.
* **ABC9 privilégie la surface** : une égalité de 128 bits sort en chaîne de LUT5 (6 niveaux au
  lieu d'environ 4 avec Vivado) ; une soustraction de 64 bits suivie d'une comparaison peut sortir
  en LUT au lieu de la chaîne de retenue ; dans le top aplati (plus de 20 000 LUT), le même chemin
  interne d'une unité prend souvent 30 à 40 % de niveaux en plus que dans le module seul
  (`eth_tx` : `cnt → crc` 9 LUT seul, 13 LUT dans le top). **Les chiffres par module sont la
  référence** ; ceux du top servent à voir les chemins **entre** unités (canal registré → logique
  de l'unité → canal suivant), qui n'existent pas dans un module seul.
* Les DSP48E1 : Yosys décompose un produit large en tranches cascadées (PCIN) ; le modèle de délai
  du DSP est grossier (valeurs typiques de la vitesse -1).
* `rst` est un signal synchrone ordinaire (même domaine que chaque module) : il est compté.

## 2. Résultats par module (`clk_sys`, avant / après T5.3)

Pire chemin registre → registre de chaque module généré par `tools/gen_verilog.sh` ; en gras les
modules modifiés par T5.3. « Avant » : commit `3d4d1e7` (début de T5.3). Format : délai estimé
(niveaux de LUT / CARRY4).

| module | domaine | avant | après | verdict après | port → / → port, après |
|---|---|---|---|---|---|
| arp | clk | 5,8 (8/0) | 5,8 (8/0) | OK | 5,2 (7/0) |
| **clock_filter** | clk | 8,4 (6/38) | 7,4 (8/16) | à surveiller | 3,4 (4/0) |
| csum | clk | 5,0 (6/4) | 5,0 (6/4) | OK | 7,8 (10/8) |
| disciplined_clock | clk | 4,7 (2/25) | 4,7 (2/25) | OK | 6,2 (4/28) |
| eth_rx | clk | 4,6 (6/0) | 4,6 (6/0) | OK | 4,6 (6/0) |
| eth_tx | clk | 6,4 (9/0) | 6,4 (9/0) | OK | 5,2 (7/0) |
| fcs_check | clk | 4,0 (5/0) | 4,0 (5/0) | OK | 4,0 (5/0) |
| frame_counter | clk | 2,0 (1/4) | 2,0 (1/4) | OK | 1,6 (1/0) |
| frame_crc | clk | 4,0 (5/0) | 4,0 (5/0) | OK | 4,6 (6/0) |
| **icmp_echo** | clk | 7,0 (10/0) | 6,4 (9/0) | OK | 6,4 (9/0) |
| **icmpv6** | clk | 13,7 (17/22) | 7,1 (9/7) | à surveiller | 7,1 (9/7) |
| **ipv4_rx** | clk | 6,6 (7/14) | 6,4 (8/0) | OK | 6,4 (8/0) |
| ipv4_tx | clk | 7,1 (10/1) | 7,1 (10/1) | à surveiller | 7,7 (11/1) |
| ipv6_rx | clk | 5,8 (8/0) | 5,8 (8/0) | OK | 5,8 (8/0) |
| ipv6_tx | clk | 3,6 (4/2) | 3,6 (4/2) | OK | 3,6 (4/2) |
| mii_rx | clk_rx | 3,4 (4/0) | 3,4 (4/0) | OK | 3,4 (4/0) |
| mii_tx | clk_tx | 3,4 (4/0) | 3,4 (4/0) | OK | 2,8 (3/0) |
| nbr_cache | clk | 5,2 (7/0) | 5,2 (7/0) | OK | 5,2 (7/0) |
| **ntp_client** | clk | 8,8 (1/0) | 7,2 (10/2) | à surveiller | 8,4 (12/2) |
| ntp_clock | clk | 3,4 (0/24) | 3,4 (0/24) | OK | 1,0 (0/0) |
| ntp_demux | clk | 2,2 (2/0) | 2,2 (2/0) | OK | 2,2 (2/0) |
| ntp_server | clk | 5,0 (6/4) | 5,0 (6/4) | OK | 4,4 (5/4) |
| phy_init | clk | 4,0 (5/0) | 4,0 (5/0) | OK | 1,6 (1/0) |
| rx_path | clk_rx | 4,7 (6/1) | 4,7 (6/1) | OK | 4,0 (5/0) |
| rx_path | clk | 4,2 (5/2) | 4,2 (5/2) | OK | 4,0 (5/0) |
| **selection** | clk | 15,3 (23/5) | 7,5 (1/13) | à surveiller | 3,4 (4/0) |
| **servo** | clk | 10,0 (12/18) | 7,9 (10/9) | à surveiller | 7,9 (10/9) |
| **sync_state** | clk | 8,2 (0/0) | 7,6 (5/36) | à surveiller | 5,7 (4/23) |
| tx_arb | clk | 2,8 (3/0) | 2,8 (3/0) | OK | 2,8 (3/0) |
| **tx_path** | clk | 6,0 (8/2) | 5,2 (7/0) | OK | 4,6 (6/0) |
| **tx_path** | clk_tx | 5,2 (7/0) | 4,6 (6/0) | OK | 4,6 (6/0) |
| **udp4_rx** | clk | 8,8 (13/0) | 5,2 (7/0) | OK | 8,2 (11/6) |
| **udp4_tx** | clk | 7,2 (8/14) | 5,8 (8/0) | OK | 4,6 (6/0) |
| **udp6_rx** | clk | 9,2 (11/16) | 6,1 (7/9) | OK | 7,9 (10/9) |
| **udp6_tx** | clk | 8,0 (9/16) | 6,7 (7/15) | OK | 4,8 (5/8) |

Dernière colonne : pire chemin qui part d'un port d'entrée ou arrive sur un port de sortie du module
(il se prolonge dans l'unité voisine).

Chemins des modules modifiés, avant → après :

| module | avant : départ → arrivée | après : départ → arrivée |
|---|---|---|
| clock_filter | 8,4 (6/38) `sbit[0]` → `sres[76]` | 7,4 (8/16) `cp[0]` → `sres[0]` |
| icmp_echo | 7,0 (10/0) `pos[2]` → `st[1]` | 6,4 (9/0) `pos[3]` → `st[1]` |
| icmpv6 | 13,7 (17/22) `mdst[2]` → `acc[18]` | 7,1 (9/7) `odd` → `gd` |
| ipv4_rx | 6,6 (7/14) `acc[0]` → `f_icmp` | 6,4 (8/0) `pos[3]` → `dst[0]` |
| ntp_client | 8,8 (1/0) `rd1[63]` → `prod[51].PREG` | 7,2 (10/2) `st[3]` → `pos[0]` |
| selection | 15,3 (23/5) `st[2]` → `lv0` | 7,5 (1/13) `DSP.AREG` → `mp[65]` |
| servo | 10,0 (12/18) `mp[38]` → `tr[0]` | 7,9 (10/9) `tl[0]` → `sx` |
| sync_state | 8,2 (0/0) `DSP.AREG` → `p2_prod[51].PREG` | 7,6 (5/36) `p2_plo[0]` → `o_rds[0]` |
| tx_path | 6,0 (8/2) `rg2[12]` → `cptr[0]` | 5,2 (7/0) `rg2[11]` → `rb[0]` |
| udp4_rx | 8,8 (13/0) `pos[3]` → `st[1]` | 5,2 (7/0) `st[0]` → `st[1]` |
| udp4_tx | 7,2 (8/14) `ip_req_data[0]` → `hd[0]` | 5,8 (8/0) `len[11]` → `abort` |
| udp6_rx | 9,2 (11/16) `mdst[16]` → `acc[28]` | 6,1 (7/9) `odd` → `acc[28]` |
| udp6_tx | 8,0 (9/16) `acc[0]` → `hd[0]` | 6,7 (7/15) `acc[0]` → `hd[0]` |

## 3. Chemins entre unités (top aplati, `clk_sys`)

Pire chemin par instance d'unité (registre d'arrivée), top serveur `ArtyBoard` et top client
`ArtyBoardWithClient` ; les canaux registrés entre unités (`*_chan`) apparaissent sous leur nom.
Rappel : dans le top, ABC allonge les chemins de 30 à 40 % par rapport au module seul (§ 1).

**top_server/arty_top** (unités dont le pire chemin dépasse 8 ns après T5.3, ou dépassait 8,5 ns avant) :

| unité / canal | avant | après | verdict | pire chemin après |
|---|---|---|---|---|
| `core.i4p_chan` | 10,8 (14/14) | 10,2 (14/8) | **critique** | `core.v4p_chan.head[2]` → `core.i4p_chan.tail_v` |
| `core.u4p_chan` | 10,8 (14/14) | 10,2 (14/8) | **critique** | `core.v4p_chan.head[2]` → `core.u4p_chan.tail_v` |
| `core.ipv4_rx` | 10,8 (14/14) | 10,2 (14/8) | **critique** | `core.v4p_chan.head[2]` → `core.ipv4_rx.meta_valid` |
| `core.v4m_chan` | 10,8 (14/14) | 10,2 (14/8) | **critique** | `core.v4p_chan.head[2]` → `core.v4m_chan.head[0]` |
| `core.i4m_chan` | 10,8 (14/14) | 10,2 (14/8) | **critique** | `core.v4p_chan.head[2]` → `core.i4m_chan.tx_data[120]` |
| `core.n6p_chan` | 13,2 (17/20) | 10,1 (14/7) | **critique** | `core.udp6_rx.odd` → `core.n6p_chan.head[0]` |
| `core.v4p_chan` | 10,2 (13/14) | 9,6 (13/8) | **critique** | `core.v4p_chan.head[2]` → `core.v4p_chan.head[0]` |
| `core.n4p_chan` | 13,3 (16/27) | 9,6 (11/20) | **critique** | `core.u4m_chan.head[144]` → `core.n4p_chan.h1[0]` |
| `core.u4m_chan` | 9,6 (12/14) | 9,0 (12/8) | **critique** | `core.v4p_chan.head[2]` → `core.u4m_chan.tail_v` |
| `core.ie4p_chan` | 10,0 (13/12) | 8,9 (12/7) | **critique** | `core.i4p_chan.head[2]` → `core.ie4p_chan.head[0]` |
| `core.eth_tx` | 9,4 (14/0) | 8,8 (13/0) | **critique** | `core.eth_tx.cnt[8]` → `core.eth_tx.crc[15]` |
| `core.i6m_chan` | 8,8 (13/0) | 8,8 (13/0) | **critique** | `core.v6p_chan.head[3]` → `core.i6m_chan.tx_data[121]` |
| `core.p1_chan` | 8,3 (12/1) | 8,3 (12/1) | **critique** | `core.ipv4_tx.n[0]` → `core.p1_chan.head[0]` |
| `core.ie6p_chan` | 9,3 (12/11) | 8,3 (11/7) | à surveiller | `core.i6p_chan.head[2]` → `core.ie6p_chan.head[0]` |
| `core.i6p_chan` | 8,2 (12/0) | 8,2 (12/0) | **critique** | `core.v6p_chan.head[3]` → `core.i6p_chan.tail_v` |
| `core.u6p_chan` | 8,2 (12/0) | 8,2 (12/0) | **critique** | `core.v6p_chan.head[3]` → `core.u6p_chan.tail_v` |
| `core.ipv6_rx` | 8,2 (12/0) | 8,2 (12/0) | **critique** | `core.v6p_chan.head[3]` → `core.ipv6_rx.meta_valid` |
| `core.v6m_chan` | 8,2 (12/0) | 8,2 (12/0) | **critique** | `core.v6p_chan.head[3]` → `core.v6m_chan.head[0]` |
| `core.udp6_tx` | 8,3 (10/13) | 8,0 (9/16) | à surveiller | `core.udp6_tx.acc[0]` → `core.udp6_tx.hd[0]` |
| `core.icmp6` | 14,0 (18/22) | 7,7 (10/7) | à surveiller | `core.i6p_chan.head[2]` → `core.icmp6.gd` |
| `ram:core.tx_path.mem.0` | 11,4 (17/2) | 6,6 (9/2) | OK | `core.tx_path.wptr[0]` → `ram:core.tx_path.mem.0.0.WE` |
| `core.tx_path` | 10,2 (15/2) | 7,6 (11/0) | à surveiller | `core.tx_path.rg2[11]` → `core.tx_path.rb[0]` |
| `core.ef_chan` | 10,2 (15/2) | 7,0 (10/0) | à surveiller | `core.eth_tx.cnt[8]` → `core.ef_chan.head[3]` |
| `core.udp6_rx` | 8,6 (10/16) | 6,0 (7/8) | OK | `core.udp6_rx.odd` → `core.udp6_rx.acc[28]` |

**top_client/arty_top** (unités dont le pire chemin dépasse 8 ns après T5.3, ou dépassait 8,5 ns avant) :

| unité / canal | avant | après | verdict | pire chemin après |
|---|---|---|---|---|
| `core.sync.filter` | 14,4 (22/2) | 13,8 (21/2) | **critique** | `core.sync.filter.cp[0]` → `core.sync.filter.it[0]` |
| `ram:core.client.m_tm.0` | 11,7 (17/5) | 11,7 (17/5) | **critique** | `core.client.st[2]` → `ram:core.client.m_tm.0.0.WE` |
| `core.client` | 10,5 (15/5) | 11,1 (16/5) | **critique** | `core.client.st[2]` → `core.client.st[2]` |
| `ram:core.client.m_fl.0` | 11,1 (16/5) | 11,1 (16/5) | **critique** | `core.client.st[2]` → `ram:core.client.m_fl.0.0.DIA` |
| `core.cm_chan` | 10,5 (15/5) | 10,5 (15/5) | **critique** | `core.client.st[2]` → `core.cm_chan.head[0]` |
| `core.n6p_chan` | 13,1 (17/19) | 9,9 (13/11) | **critique** | `core.n6m_chan.tx_data[272]` → `core.n6p_chan.head[0]` |
| `core.ipv4_rx` | 11,3 (15/13) | 9,7 (13/9) | **critique** | `core.v4p_chan.head[2]` → `core.ipv4_rx.meta_valid` |
| `core.v4m_chan` | 11,3 (15/13) | 9,7 (13/9) | **critique** | `core.v4p_chan.head[2]` → `core.v4m_chan.head[0]` |
| `core.i4m_chan` | 11,3 (15/13) | 9,7 (13/9) | **critique** | `core.v4p_chan.head[2]` → `core.i4m_chan.tx_data[120]` |
| `core.i4p_chan` | 11,3 (15/13) | 9,7 (13/9) | **critique** | `core.v4p_chan.head[2]` → `core.i4p_chan.tail_v` |
| `core.u4p_chan` | 11,3 (15/13) | 9,7 (13/9) | **critique** | `core.v4p_chan.head[2]` → `core.u4p_chan.tail_v` |
| `core.n4p_chan` | 13,3 (16/27) | 9,5 (10/19) | **critique** | `core.u4m_chan.head[144]` → `core.n4p_chan.head[0]` |
| `core.v4p_chan` | 10,7 (14/13) | 9,1 (12/9) | **critique** | `core.v4p_chan.head[2]` → `core.v4p_chan.head[0]` |
| `core.ie4p_chan` | 10,0 (13/12) | 8,9 (12/7) | **critique** | `core.i4p_chan.head[2]` → `core.ie4p_chan.head[0]` |
| `core.i6m_chan` | 8,8 (13/0) | 8,8 (13/0) | **critique** | `core.v6p_chan.head[2]` → `core.i6m_chan.tx_data[121]` |
| `core.eth_tx` | 10,0 (15/0) | 8,8 (13/0) | **critique** | `core.eth_tx.cnt[8]` → `core.eth_tx.crc[19]` |
| `core.u4m_chan` | 10,1 (13/13) | 8,5 (11/9) | à surveiller | `core.v4p_chan.head[2]` → `core.u4m_chan.tail_v` |
| `core.p1_chan` | 8,3 (12/1) | 8,3 (12/1) | **critique** | `core.ipv4_tx.n[0]` → `core.p1_chan.head[0]` |
| `core.ie6p_chan` | 9,9 (13/11) | 8,3 (11/7) | à surveiller | `core.i6p_chan.head[2]` → `core.ie6p_chan.head[0]` |
| `core.ipv6_rx` | 8,2 (12/0) | 8,2 (12/0) | **critique** | `core.v6p_chan.head[2]` → `core.ipv6_rx.rem_rw[0]` |
| `core.i6p_chan` | 8,2 (12/0) | 8,2 (12/0) | **critique** | `core.v6p_chan.head[2]` → `core.i6p_chan.tail_v` |
| `core.u6p_chan` | 8,2 (12/0) | 8,2 (12/0) | **critique** | `core.v6p_chan.head[2]` → `core.u6p_chan.tail_v` |
| `core.v6m_chan` | 8,2 (12/0) | 8,2 (12/0) | **critique** | `core.v6p_chan.head[2]` → `core.v6m_chan.head[0]` |
| `core.udp6_tx` | 8,3 (10/13) | 8,0 (9/16) | à surveiller | `core.udp6_tx.acc[0]` → `core.udp6_tx.hd[0]` |
| `core` | 6,3 (3/35) | 8,0 (6/34) | à surveiller | `core.sync.state.p2_plo[0]` → `core.root_disp[0]` |
| `core.sync.selection` | 14,8 (21/12) | 7,5 (1/13) | à surveiller | `DSP.AREG` → `core.sync.selection.mp[65]` |
| `core.icmp6` | 14,0 (18/22) | 7,9 (11/3) | à surveiller | `core.i6p_chan.head[2]` → `core.icmp6.st[0]` |
| `ram:core.tx_path.mem.0` | 11,4 (17/2) | 6,6 (9/2) | OK | `core.tx_path.wptr[0]` → `ram:core.tx_path.mem.0.0.WE` |
| `core.sync.servo` | 10,5 (13/17) | 7,7 (9/13) | à surveiller | `core.sync.servo.ki3` → `core.sync.servo.fb[0]` |
| `core.ef_chan` | 10,2 (15/2) | 7,0 (10/0) | à surveiller | `core.eth_tx.cnt[8]` → `core.ef_chan.head[6]` |
| `core.tx_path` | 10,2 (15/2) | 7,6 (11/0) | à surveiller | `core.tx_path.rg2[12]` → `core.tx_path.rb[0]` |
| `core.sync.sel_chan` | 9,7 (12/15) | 3,5 (4/1) | OK | `core.sync.selection.el0` → `core.sync.sel_chan.tx_data[492]` |
| `core.udp6_rx` | 8,6 (10/16) | 5,8 (6/12) | OK | `core.n6m_chan.tx_data[272]` → `core.udp6_rx.acc[28]` |

## 4. Domaines MII (`clk_rx`, `clk_tx`, 25 MHz, 40 ns)

| logique | domaine | pire chemin estimé | verdict |
|---|---|---|---|
| `mii_rx` | `clk_rx` | 3,4 ns (4 LUT) | OK |
| `rx_path` moitié ClkRx (`ingress`, écriture de `data`, `ev`, `cred`) | `clk_rx` | 4,7 ns (6 LUT, 1 CARRY4) | OK |
| `mii_tx` | `clk_tx` | 3,4 ns (4 LUT) | OK |
| `tx_path` moitié ClkTx (lecture, `rgray`, `cpt`, `hq*`, événements SFD) | `clk_tx` | 5,2 ns (7 LUT) | OK |
| top serveur / client, tout `clk_rx` | `clk_rx` | 5,2 ns (7 LUT) | OK |
| top serveur / client, tout `clk_tx` | `clk_tx` | 6,4 / 5,8 ns (9 / 8 LUT) | OK |

Marge très large (budget 40 ns). Les entrées `eth_rxd`/`eth_rx_dv`/`eth_rx_er` vont directement
dans un registre de `mii_rx` et les sorties `eth_txd`/`eth_tx_en` sortent de registres de `mii_tx`
(IOB TRUE dans la XDC) : les chemins d'E/S sont contraints par les `set_input_delay` /
`set_output_delay` de `constraints/arty_a7_100.xdc` (10/30 ns en entrée, 10/0 ns en sortie).

## 5. Croisements d'horloge et contraintes

Croisements présents (relevés par l'outil dans le top et dans `rx_path` / `tx_path`) :

| croisement | mécanisme | registres |
|---|---|---|
| pointeur d'écriture `rx_path` (ClkRx → `clk_sys`) et pointeur de lecture (retour) | Gray, 2 bascules `ASYNC_REG` | `wgray_r1/r2`, `rgray_w1/w2` (`data`, `ev`, `cred`) |
| données de la mémoire de `rx_path` (écrite en ClkRx, lue en `clk_sys`) | protocole : lecture seulement derrière le pointeur Gray synchronisé | LUT-RAM `data.mem`, `ev.mem` |
| pointeur de lecture `tx_path` (ClkTx → `clk_sys`) | Gray, 2 bascules `ASYNC_REG` (`rg1`, `rg2`), puis binaire registré `rb` (T5.3) | `rgray → rg1 → rg2 → rb` |
| pointeur engagé `tx_path` (`clk_sys` → ClkTx) | poignée de main : `cx` stable pendant l'échange, requête `hreq` → `hq1/hq2` (2 bascules), `cpt` chargé depuis `cx`, accusé `hq3 → ha1/ha2` | `cx → cpt` (multi-bits, **faux chemin**) |
| données de la mémoire de `tx_path` (écrite en `clk_sys`, lue en ClkTx) | protocole : lecture seulement dans `[rptr, cpt)`, octets écrits avant l'engagement | LUT-RAM `mem` (chemin `mem → sent_last/busy/txf_chan`, 6-7 LUT) |
| événements d'horodatage TX (SFD, fin bonne / mauvaise, ClkTx → `clk_sys`) | bascule de basculement (toggle) + 2 bascules `ASYNC_REG` + détection de front | `sf1/sf2/sf3`, `ok1..3`, `bd1..3` |
| validation des horloges MII (`clk_sys` → `clk_rx`/`clk_tx`, enveloppe) | 2 bascules `ASYNC_REG` sur l'horloge libre (BUFG), puis BUFGCE | `rx_ce_s`, `tx_ce_s` |
| `locked` du MMCM → `clk_sys` | 2 bascules, remise à zéro asynchrone | enveloppe |
| `rst` vers les domaines MII | relâché pendant que les horloges MII sont arrêtées (séquenceur de l'enveloppe) | enveloppe |

Contraintes (lues dans `constraints/arty_a7_100.xdc` et `tools/vivado/top_timing.xdc`) :

* **Couvert** : `set_clock_groups -asynchronous` entre `clk_100` (et ses horloges générées :
  `clk_sys`, `clk_25`, `eth_ref_clk_fwd`), `eth_rx_clk` et `eth_tx_clk` (version de
  `top_timing.xdc`, qui inclut les horloges générées ; celle de la XDC carte ne nomme que
  `clk_100`). Tous les croisements ci-dessus sont donc exclus de l'analyse setup/hold, y compris
  `cx → cpt` (faux chemin voulu, documenté dans `tx_path.rs`) et les lectures de LUT-RAM d'un
  domaine à l'autre. Horloges primaires `clk_100` (10 ns), `eth_rx_clk`, `eth_tx_clk` (40 ns) ;
  `eth_ref_clk` en horloge générée sur l'ODDR. Gestion du PHY (`eth_mdc`, `eth_mdio`,
  `eth_rstn`) en faux chemin. `ASYNC_REG` posé par TxHDL (`#[async_reg]`, 26 registres dans le
  top serveur) et par l'enveloppe (4).
* **Ouvert** :
  1. `set_clock_groups` supprime toute contrainte, donc **aucune borne sur le décalage
     (skew) entre les bits des bus Gray** (`rgray`, `wgray`) **ni sur `cx → cpt`**. La pratique
     recommandée est de remplacer, pour ces bus, le groupe asynchrone par
     `set_max_delay -datapath_only` (une période de l'horloge de destination, voire moins) et
     `set_bus_skew` (une période de l'horloge la plus rapide). Même remarque pour les lectures de
     LUT-RAM entre domaines : le protocole suppose que la donnée est stable bien avant que le
     pointeur la rende lisible (au moins deux cycles de l'horloge de destination), ce qu'une
     contrainte `set_max_delay -datapath_only` sur ces chemins rendrait vérifiable.
  2. Front de capture TX du PHY (TODO de la XDC : figure 5-4 de SNLS250E), skew de carte et
     latence d'horloge côté PHY non modélisés.
  3. Le domaine `clk_25` (MMCM) ne sert que l'ODDR de `eth_ref_clk` : pas de chemin de données.
  4. Rien de tout cela n'a été passé dans Vivado : `report_cdc` et `report_methodology`
     (TIMING-*, CDC-*) restent à lancer.

## 6. Modifications (T5.3) et latences

Toutes les modifications gardent le comportement **bit à bit** (vecteurs de référence, modèles
Python, tests système inchangés et verts) ; certaines ajoutent des cycles à des séquenceurs qui
ont du temps libre (événements rares).

| unité | changement | latence |
|---|---|---|
| `csum.rs` | `pseudo_v4`, `pseudo_v6` et nouveau `sum8` en **arbres équilibrés** d'additions (profondeur 3 et 5 au lieu de chaînes de 6 et 19) ; nouveau `csum_ok(x)` = « `csum_fold(x) == 0xFFFF` » avec un seul repli (`x1` = 0xFFFF ou 0x1FFFE) | aucune |
| `udp6_rx` (`words8_into`), `udp6_tx`, `udp4_tx` | graines en arbres (via `sum8` / `pseudo_v4`) | aucune |
| `udp4_rx`, `udp6_rx` | verdict de somme du dernier octet calculé depuis `acc` seul (la graine n'est ajoutée que dans l'en-tête : même valeur là où il sert), `csum_ok` ; décodage de la position d'en-tête sur 3 bits (`pos` ≤ 7 dans l'état HDR) | aucune |
| `ipv4_rx`, `icmp_echo` | `csum_ok` | aucune |
| `icmpv6` | graines `pseudo_v6` en arbre ; comparaisons d'adresses 128 bits (cible locale, destination, sollicité, source nulle, multicast) **registrées** et lues un cycle plus tard (nouveaux états `CK0`, `RS0`) ; verdict du dernier octet du NS/NA registré (`gd`, appliqué en `CK0`) ; `mlen ≥ 8 / ≥ 24` registrés ; décodage des positions 0-3 sur 2 bits | **+1 cycle** avant la NA (« environ 7 cycles » après le dernier octet du NS) et avant la NS générée (environ 7 cycles après `resolve`) ; un NS refusé revient en GATHER un cycle plus tard |
| `selection` | comparateur de l'intersection en **pipeline à deux étages** (bords `e_*` registrés, comparaison `c_*` registrée ; compteurs mis à jour deux cycles après l'émission de la paire ; les paires de décision passent par le même pipeline, états `FINA`, `FINB`) ; distance racine : somme partielle registrée `rdx` (tout sauf `phi`, en arbre), `phi` ajouté en phase 3 | **+4 cycles** par événement : `event_cycles` et `MAX_EVENT_CYCLES` = **545** (au lieu de 541) |
| `clock_filter` | pointeur de tête du pair en cours dans un registre `hd` (copie de `p_head[cp]`, écrite à la prise) au lieu de la lecture de LUT-RAM ; racine carrée : `sres + sbit` remplacé par `sres | sbit` (`sres` est un multiple de `2·sbit` : aucune retenue) | aucune |
| `servo` | état `FIN` découpé : `FIN` écrit `freq`, **`FIN2`** écrit `tr = clamp(freq + s)` depuis le `freq` registré | **+1 cycle** : `process_cycles(p)` = 50 + p (54..60), nouveau `trim` en P + 50 + pn (test `cycles_per_event` mis à jour) |
| `sync_state` | `PHI × age` en deux produits partiels 32 × 16 (`p2_plo`, `p2_phi`, deux tranches DSP au lieu de quatre) ; étage 3 : `ceil48(T)` = `(dj + phi + ((plo + 2^48 − 1) >> 32)) >> 16` (exact) | aucune (toujours `E+3`) |
| `ntp_client` | `prod` = `iv × PHI_Q32` en deux produits partiels (`plo`, `phi`), `(prod + 2^32 − 1) >> 32 = phi + ((plo + 2^32 − 1) >> 32)` | aucune |
| `tx_path` | pointeur de lecture synchronisé converti en binaire **dans un registre** (`rb`) | le côté `clk_sys` voit la place libérée un cycle plus tard (`full` plus prudent) ; **latence TX inchangée** |

Pas de recalibrage : aucun changement ne touche la latence du chemin d'émission (`TX_LATENCY_V4/V6`,
règle T1 du client, formules de `udp4_tx`/`udp6_tx`) ni celle du chemin de réception
(`RX_STAMP_BIAS`, `TX_STAMP_BIAS`). Mesures après T5.3 (tests de mesure, MII à 25 MHz réels) :
`top.rs` `real_ratio_timestamp_error_over_eight_phases` : erreur T2 de 0 à +5,1 ns, erreur T3 de
+4,9 à +10,0 ns ; `top_client.rs` : T1 de −20,0 à +10,0 ns, T3 de −20,0 à +10,0 ns, toutes dans la
fenêtre −25..+10,5 ns des tests.

`disciplined_clock` : l'addition de 96 bits par cycle fait 2 LUT + 25 CARRY4, **4,7 ns** estimées
(le chemin `trim → acc` à travers le clamp : 6,2 ns). La chaîne de retenue de 96 bits tient
largement à 100 MHz (≈ 0,1 ns par CARRY4) : **pas de découpage**, `time` et ses propriétés
formelles sont inchangés.

Propriétés formelles : aucune `check!`/`assume!`/`cover!` modifiée. À re-prouver par
l'orchestrateur : `csum` (fonctions modifiées, pas l'unité), `ipv4_hdr_equiv` (inchangé, mais
`ipv4_hdr_cksum` n'a pas été touché), `tx_path` (`rs` est maintenant le registre `rb`, un cycle
plus ancien que la conversion de `rg2` : les `check!` qui lisent `rs` gardent leur sens, « le
pointeur de lecture synchronisé est en retard sur le vrai », mais l'induction peut demander un
lien entre `rb` et `rg2`).

## 7. Ce qui reste

Par ordre d'importance :

1. **Chemins entre unités dans le top** (12 à 14 niveaux de LUT estimés, 8 à 10 ns) : canal
   registré → décision de l'unité → canal suivant, surtout `ipv4_rx` (octet 19 : vérification de
   l'en-tête, somme, adresse, puis émission de la méta et poussée dans les canaux ICMP/UDP),
   `ipv6_rx`, le verdict du dernier octet de `udp4_rx` / `udp6_rx` (bit `err` du `last`, qui passe
   par la somme de l'octet courant), `icmp_echo`, `eth_tx` (`cnt → crc`). **Les mêmes unités
   seules restent sous 6,5 ns (8 à 9 LUT)** ; l'écart vient pour partie des canaux (2 à 3 niveaux
   de plus) et pour partie de la cartographie d'ABC dans le top aplati (§ 1). À confirmer dans
   Vivado. Si Vivado les confirme : (a) `ipv4_rx` : registrer la décision de l'octet 19 et émettre
   la méta un cycle plus tard (sans effet sur les horodatages, portés par la méta) ; (b) verdict
   UDP : tenir un accumulateur déjà replié (17 bits) à côté de `acc`, ou retarder la sortie d'un
   octet ; (c) solution générale : canaux TxHDL avec `ready` registré (tampon « skid ») sur les
   liaisons longues.
2. **Client, état SCAN** (top client) : lecture des LUT-RAM `m_fl` / `m_tm` du créneau → soustraction
   de 32 bits → comparaisons (échéances, intervalle de poll) → décision → écritures. 7,2 ns dans le
   module seul, 10,5 à 11,7 ns dans le top (les entrées de configuration s'y ajoutent). Découpage
   possible en deux cycles par créneau (lecture registrée, puis décision) sans effet sur T1 (pris
   à la construction de l'octet 40). Non fait ici : unité de 900 lignes, risque de régression
   pour un gain à confirmer.
3. **`clock_filter` dans le top client** (13,8 ns, 21 LUT, `cp → it`, état FIN : `t40 − tlast`
   lu en LUT-RAM puis test « plus récent ») : **artefact de l'outil**. Le chemin (`--path`) montre
   la soustraction de 64 bits cartographiée par ABC en une chaîne de 16 LUT5 au lieu de la chaîne
   de retenue (16 CARRY4) qu'il a dans le module seul (7,4 ns, 8 LUT + 16 CARRY4) ; Vivado utilise
   la chaîne de retenue. Attendu : environ 7 ns.
4. **À surveiller** (7 à 8,5 ns par module) : `servo` (`tl → sx`, comparaison de `el` = `now − tl`
   avec le seuil du poll, 7,9 ns), `sync_state` (étage 3, `p2_plo → o_rds`, 7,6 ns),
   `selection` (multiplieur 34 × 34 en DSP cascadés, 7,5 ns ; chemin du diviseur `st → dx`,
   7,3 ns), `clock_filter` (`cp → sres`, 7,4 ns), `ntp_client` (7,2 ns), `icmpv6` (verdict du
   dernier octet vers `gd`, 7,1 ns), `ipv4_tx` (`n → st`, 7,1 ns, non modifié).
5. **Croisements** : les points ouverts du § 5 (bornes de skew des bus Gray et de `cx → cpt`,
   lectures de LUT-RAM entre domaines, front de capture TX du PHY).
6. **Vivado** : lancer `tools/vivado/build.tcl`, puis comparer `report_timing_summary`
   (WNS / TNS, 10 pires chemins de `clk_sys`) à ce document ; `report_cdc` et
   `report_methodology` pour les croisements.

Ressources : les changements de T5.3 ne modifient pas la consommation de façon notable
(`tools/estimate_resources.py` sur les modules modifiés : `selection` −52 LUT / +219 bascules
pour le pipeline du comparateur et `rdx`, les autres à ±70 LUT et ±20 bascules) ;
`docs/resources.md` n'est pas mis à jour.

## 8. Vérifications faites après les modifications

`cargo test -p ntp_fpga` (tout, profil par défaut) vert ; `python3 -m unittest discover
tools/golden` et `tools/golden/ntpclient` verts ; `python3 tools/formal/contracts.py` : PASS
(seuls les numéros de ligne de `csum.rs` changent) ; `tools/gen_verilog.sh` sans erreur ;
`top_gen` (serveur et `--with-client`) : Yosys `read_verilog` + `hierarchy -check -top
arty_top_board` + `proc` + `check -assert` sans problème. Preuves formelles non relancées
(l'orchestrateur les refait, voir § 6).
