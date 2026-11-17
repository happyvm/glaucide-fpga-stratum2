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
| `arp` | auteur + orchestrateur | **Revu** | ✅ | refonte sécurité (F5/F6) : apprentissage uniquement depuis la réponse à notre propre résolution, prouvé (`learn` ⇒ résolution en cours, `op` = 2, `spa` = adresse résolue, trame sans erreur, MAC source ni de groupe ni nul). **Défaut trouvé par la preuve** : une réponse sollicitée avec MAC source nul était apprise ; corrigé (`sha != 0`) et couvert par le test `solicited_reply_with_null_or_group_mac_is_not_learned` (échoue sans la correction) |
| `selection` | orchestrateur | **Revu** | 🔶 | 8 itérations de renforcement d'induction (chaque contre-exemple = un état inatteignable à exclure). Prouvé : pair utilisable ⇒ strate 1..15 et leap ≠ 3 ; éligible ⇒ utilisable et distance racine < MAXDIST ; sélection valide ⇒ `ever` ; survivant éligible ; le clustering ne tourne qu'avec 4 survivants et en laisse au moins 3 ; recherche du pair système (meilleur trouvé = survivant, tout survivant examiné a positionné `bv`) ; pair système survivant, éligible, de distance racine < MAXDIST. Mutation « `keep` sans `sv_o` » détectée par « le pair système est un survivant ». **Hypothèse non prouvée** (moniteur `m_ne`) : l'intersection laisse au moins un survivant (raisonnement sur les rangs de Marzullo ; vérifié par simulation et par les vecteurs dorés seulement). Non prouvés : exactitude arithmétique (offset, jitter), majorité de démarrage F4 (comptage des truechimers), liveness (≤ 545 cycles) : simulation |
| `icmp_echo` | orchestrateur | **Revu** | ✅ | une seule réponse par requête acceptée, aucun octet hors réponse, type et code 0, exactement un `last`, message non-echo sans réponse ; mutation (code attendu 1) détectée, 5 couvertures atteintes. Non prouvé : la correction arithmétique du checksum incrémental (simulation + modèle doré) |
| `ipv4_rx` | orchestrateur | **Revu** | ✅ | le verdict est reformulé à partir des octets bruts (moniteurs `m_b0`, `m_b6`, `m_b7`, `m_mc`), indépendamment de `ok`/`accept` : version/IHL, pas de groupe (I0, E1), pas de fragment, TTL ≠ 0, longueur > 20, destination = `local_ip`, source unicast (0/8, 127/8, 224/4, 240/4, soi-même) ; protocole 1 → ICMP, 17 → UDP ; méta avant tout octet ; paquet terminé avec la trame ; un paquet sans erreur porte exactement longueur totale − 20 octets. 3 mutations (127/8, source ≠ `local_ip`, `tl > 19`) détectées. Hypothèses : trame ≤ 1518 octets, `local_ip` constante (configuration). Non prouvé : le checksum d'en-tête (`csum`) |
| `udp4_rx` | orchestrateur | **Revu** | ✅ | transmis seulement vers le port 123 avec longueur UDP = longueur IP > 8 (U2) ; un datagramme qui finit dans l'en-tête n'émet rien ; un datagramme abandonné n'émet rien ; un datagramme sans `err` porte exactement longueur UDP − 8 octets ; `err` seulement avec `last` ; méta avant le premier octet. 3 mutations (`> 7`, `end_len` décalé, `err` sans écart de longueur) détectées. Non prouvé : le checksum UDP (pseudo-en-tête) |

