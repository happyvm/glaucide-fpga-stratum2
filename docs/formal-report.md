# Suivi des preuves formelles

Légende : **Auteur** = propriétés écrites par l'auteur du RTL (avant la décision « Partage des rôles », voir `docs/formal.md`) ; **Revu** = relues et complétées par l'orchestrateur à partir du catalogue/RFC ; ✅ prouvé (k-induction non bornée) ; 🔶 partiel (voir note).

| Module | Propriétés | Revue | Preuve | Notes |
|---|---|---|---|---|
| `crc32::crc32_step` | orchestrateur (F1.1) | indépendant | ✅ `crc_equiv` | équivalence avec la matrice GF(2) calculée en Python, 2^40 entrées ; mutation détectée |
| `crc32::FrameCrc` | auteur | à faire | ✅ | init après `last` |
| `fcs_check` | auteur + orchestrateur | **Revu** | ✅ | ajout d'un témoin de position symbolique `pick` : l'octet sorti à la position p est l'octet entré à la position p, pour tout p ; mutation « un bit changé sur le 8e octet » détectée (l'ancienne propriété ne contraignait que le premier octet) |
| `mii_rx` | auteur | à faire | ✅ | troncature à 1518 octets : simulation seule |
| `mii_tx` | auteur | à faire | ✅ | mutation détectée seulement en `bmc` ; troncature : simulation seule |
| `eth_rx` | auteur | à faire | ✅ | hypothèse `local_mac` unicast (garant T4.1) |
| `eth_tx` | auteur | à faire | ✅ | 3 couvertures profondes : simulation seule |
| `tx_arb` | auteur | à faire | ✅ | borne de non-famine prouvée |
| `tx_path` | auteur | à faire | 🔶 | une seule horloge dans le prouveur ; rapports d'horloges, bornes de timestamp : simulation seule |
| `rx_path` | auteur | à faire | 🔶 | idem ; `rx_egress` prouvé à profondeur 8 |
| `csum` | auteur | à faire | ✅ | référence de repli à deux étages, pas une congruence mod 0xFFFF |
| `phy_init` | auteur | à faire | ✅ | instance raccourcie, mêmes logique ; 4 couvertures : simulation seule |
| `clock::DisciplinedClock` | auteur | à faire | ✅ | accumulateur 96 bits complet |
