# Contraintes Arty A7-100T (`arty_a7_100.xdc`)

## Source et vérification

- Référence : `Arty-A7-100-Master.xdc` du dépôt `github.com/Digilent/digilent-xdc` (branche `master`), récupéré le 2026-11-17. Le fichier maître n'est pas versionné ici (licence Digilent).
- Datasheet PHY : TI DP83848x, `SNLS250E` (rév. E, avril 2015), tables T2.4.x et T2.5.x. Le numéro `SNLS266` cité dans la tâche ne correspond pas à ce document ; c'est `SNLS250E` qui couvre le DP83848J et qui a servi de référence.
- Vivado n'est pas disponible dans cet environnement : le fichier n'est **pas** vérifié par l'outil. Seule une vérification de syntaxe Tcl (accolades, crochets, guillemets) a été faite.

## Table des broches

Les lignes « maître » sont les lignes du `Arty-A7-100-Master.xdc` (toutes commentées dans le maître : ce sont des modèles à décommenter).

| Signal | Broche | Ligne maître | Remarque |
|---|---|---|---|
| `clk_100` | E3 | 7 | identique |
| `eth_rstn` | C16 | 186 | identique |
| `eth_ref_clk` | G18 | 185 | identique (sortie FPGA vers PHY) |
| `eth_mdc` | F16 | 183 | **corrigée** (était M16) |
| `eth_mdio` | K13 | 184 | identique |
| `eth_rx_clk` | F15 | 187 | identique |
| `eth_rx_dv` | G16 | 188 | identique |
| `eth_rx_er` | C17 | 193 | identique ; le maître l'appelle `eth_rxerr`, on garde `eth_rx_er` (nom utilisé par le RTL) |
| `eth_rxd[0]` | D18 | 189 | identique |
| `eth_rxd[1]` | E17 | 190 | identique |
| `eth_rxd[2]` | E18 | 191 | identique |
| `eth_rxd[3]` | G17 | 192 | identique |
| `eth_tx_clk` | H16 | 194 | identique |
| `eth_tx_en` | H15 | 195 | identique |
| `eth_txd[0]` | H14 | 196 | identique |
| `eth_txd[1]` | J14 | 197 | identique |
| `eth_txd[2]` | J13 | 198 | identique |
| `eth_txd[3]` | H17 | 199 | identique |
| `eth_col` | D17 | 181 | **ajoutée** (entrée optionnelle, non utilisée) |
| `eth_crs` | G14 | 182 | **ajoutée** (entrée optionnelle, non utilisée) |

Toutes les broches utilisent `IOSTANDARD LVCMOS33`, comme le maître. Le maître ne contient ni `CONFIG_VOLTAGE` ni `CFGBVS` : aucune propriété de ce type n'est ajoutée.

**Différences trouvées et corrigées :** `eth_mdc` (M16 vers F16) ; ajout de `eth_col` (D17) et `eth_crs` (G14). Le reste était déjà conforme.

## Hypothèses de timing (MII, 100 Mb/s, horloges 25 MHz)

- `eth_rx_clk` et `eth_tx_clk` : `create_clock` 40 ns, fournis par le PHY.
- **PHY vers FPGA** (`eth_rx_dv`, `eth_rx_er`, `eth_rxd[*]`), relatif à `eth_rx_clk` : délai RX_CLK vers données de 10 ns min et 30 ns max (T2.5.2). Ce sont les valeurs de la datasheet, et non les 0/10 ns prévus au départ.
- **FPGA vers PHY** (`eth_tx_en`, `eth_txd[*]`), relatif à `eth_tx_clk` : setup de 10 ns et hold de 0 ns (T2.4.2 / T2.4.3), donc `set_output_delay -max 10` et `-min 0`.
- `set_clock_groups -asynchronous` entre `clk_100`, `eth_rx_clk` et `eth_tx_clk`.
- `IOB TRUE` sur les ports RX et TX : n'a d'effet que si le haut niveau place des registres sur ces ports.

## TODO

1. **Vérifier avec Vivado** (non disponible ici) : syntaxe et matching des ports. Le fichier ne doit pas être utilisé en synthèse avant cette étape.
2. **Vérifier le front de capture TX** sur la figure 5-4 de SNLS250E. La note 12 de la datasheet mentionne un échantillonnage sur le front descendant de TX_CLK, mais elle est rattachée à la section 10 Mb/s. Si le PHY échantillonne sur le front descendant en 100 Mb/s, les `set_output_delay` doivent être recalculés.
3. **`eth_ref_clk`** : `create_generated_clock` sur la sortie du MMCM (25 MHz depuis `clk_100`). Le nom de l'instance MMCM n'est pas encore connu ; un commentaire TODO est dans le fichier.
4. **`eth_col` et `eth_crs`** : doivent être déclarés comme ports dans le top-level, sinon `get_ports` ne trouve rien et Vivado signale une erreur. Non utilisés par le RTL actuel.
5. **Délais de piste et latence d'horloge** du PCB : non modélisés.
6. **`eth_mdc` / `eth_mdio`** (interface de gestion, lente, bidirectionnelle) : pas de contrainte de timing pour l'instant. À traiter avec la FSM MDIO (faux chemins ou règle dédiée).
7. **`IOB TRUE`** : effectif uniquement si le top-level enregistre les ports concernés.
