# Guide d'utilisation

Ce guide va du clonage du dépôt aux fichiers Verilog prêts pour Vivado, puis décrit les tests, les
preuves et l'emplacement de chaque module. La mise en service sur la carte est dans
[bringup.md](bringup.md), la configuration réseau dans [config-guide.md](config-guide.md).

**Ce qui a été exécuté et ce qui ne l'a pas été.** Tout ce qui précède la synthèse (génération du Verilog,
tests, preuves, estimation de ressources) a été exécuté dans l'environnement de développement. Vivado ne
l'a jamais été, et la carte non plus : les étapes Vivado de ce guide sont écrites d'après
`tools/vivado/build.tcl` et ne sont pas vérifiées.

## 1. Prérequis

| Outil | Pour quoi | Remarque |
|---|---|---|
| `git`, accès sortant vers `github.com` | cloner TxHDL (`tools/setup.sh`) | obligatoire |
| Chaîne Rust (`cargo`) | simulation, génération du Verilog | obligatoire ; la CI utilise la version `stable`, les commandes de ce guide ont été exécutées avec `cargo 1.97.0` |
| Python 3 | modèles de référence, vérificateur de contrats | obligatoire pour les tests Python ; bibliothèque standard seulement (`scapy` est facultatif, un test de recoupement est ignoré s'il est absent) ; essayé avec Python 3.13, la CI utilise 3.12 |
| `pip install yowasp-yosys z3-solver` | preuves formelles (`tools/formal/prove.sh`) | facultatif ; `sby`, `yosys` et `z3` natifs sur le `PATH` sont préférés s'ils existent |
| Yosys natif avec ABC (`pip install pyosys` dans un venv) | estimation de ressources (`tools/estimate_resources.py`) | facultatif ; la version WebAssembly utilisée pour les preuves ne sait pas exécuter ABC |
| Vivado | synthèse, placement-routage, bitstream | facultatif pour le développement, obligatoire pour la carte ; version et édition non déterminées (le composant est `xc7a100tcsg324-1`) |

Bazel n'est pas nécessaire : Cargo remplace la chaîne Bazel de TxHDL pour simuler et générer le Verilog.

## 2. Du clone à `out/top/`

```sh
git clone <url-du-dépôt> glaucide-fpga-stratum2
cd glaucide-fpga-stratum2
tools/setup.sh                                          # clone TxHDL dans third_party/hdl-txhdl, au commit épinglé
cargo run -p ntp_fpga --bin top_gen                     # carte serveur seul   -> out/top/
cargo run -p ntp_fpga --bin top_gen -- --with-client    # carte stratum 2      -> out/top/ (mêmes noms de fichiers)
```

`top_gen` écrit dans `out/top/` (dossier ignoré par git) :

| Fichier | Contenu |
|---|---|
| `arty_top.v` | arbre de modules généré (un module par unité, plus `arty_top_txhdl_chan`, le canal registré) |
| `arty_top_board.v` | copie de `rtl/hdl/arty_top_board.v` : enveloppe de carte écrite à la main |
| `files.f` | les deux fichiers ci-dessus, dans l'ordre de compilation |

Sortie observée : `ArtyBoard` donne 45 modules (7 642 lignes), `ArtyBoardWithClient` 52 modules (10 976 lignes).
Un argument sans tiret remplace le dossier de sortie (`... --bin top_gen -- /chemin`) ; les deux variantes
écrivent les mêmes noms de fichiers, donc **la seconde exécution écrase la première** : utilisez deux
dossiers si vous voulez garder les deux.

La suite (Vivado) : `vivado -mode batch -source tools/vivado/build.tcl`, détaillée dans
[bringup.md](bringup.md) et [../tools/vivado/README.md](../tools/vivado/README.md).

`tools/gen_verilog.sh` est un autre chemin : il génère un fichier Verilog **par module** dans `out/*.v`
(la liste est codée dans le script, elle ne contient pas le top). Il sert à l'estimation de ressources et à
l'inspection, pas à la construction de la carte.

## 3. Quelle carte construire

| | `ArtyBoard` | `ArtyBoardWithClient` |
|---|---|---|
| Commande | `top_gen` | `top_gen -- --with-client` |
| Rôle | serveur seul | serveur et client NTP |
| Synchronisation | **jamais** : l'horloge court librement, démarre à 0 (1900) | le client interroge les amonts de `UPSTREAM`, le servo discipline l'horloge |
| Annonce | stratum 16, LI 3 | stratum 16 / LI 3 tant que non synchronisé, puis **stratum 2** (stratum de l'amont + 1), LI 0 |
| Modules | 45 | 52 |
| Bascules (Yosys, avant optimisation Vivado) | 14 950 | 29 255 |
| Usage | vérifier le lien, ARP, ping, ICMPv6, le format des réponses | le but du projet |

Construisez **`ArtyBoard` d'abord** pour la mise en service du réseau (un client NTP standard voit un
serveur non synchronisé : c'est le comportement attendu), puis `ArtyBoardWithClient`. Pour un serveur de
production il n'y a que la seconde carte : seule elle peut annoncer stratum 2.

Les deux utilisent `config::Board` pour les adresses et `phy_init::Board` pour les temporisations du PHY ;
voir [config-guide.md](config-guide.md) pour changer les adresses. Ne confondez pas les deux `Board` : le
premier est un jeu d'adresses, le second des temporisations (reset, attente, MDC).

## 4. Tests

Tous les tests Rust sont dans `rtl/tests/` (un fichier par unité ou par famille) ; il n'y a pas de tests
dans `rtl/src/`.

```sh
cargo test -p ntp_fpga                          # tout (long en mode debug : plusieurs minutes)
cargo test -p ntp_fpga --test ping4             # un seul fichier (9 tests, ~17 s ici)
cargo test -p ntp_fpga --test config            # 15 tests, quasi instantané
cargo test -p ntp_fpga --test top standalone_answers_unsynchronised   # un seul test (~0,4 s une fois compilé)
python3 -m unittest discover tools/golden       # 59 tests : modèle de référence du protocole
python3 -m unittest discover tools/golden/ntpclient   # 81 tests : filtre, sélection, servo, client
python3 tools/formal/contracts.py               # vérificateur garanties/hypothèses
python3 -m unittest discover tools/formal       # tests du vérificateur (20 tests)
```

Les familles de tests :

| Famille | Fichiers | Ce qu'elle couvre |
|---|---|---|
| **Unitaire** | un fichier par unité : `mii_rx.rs`, `mii_tx.rs`, `crc32.rs`, `fcs_check.rs`, `eth_rx.rs`, `eth_tx.rs`, `tx_arb.rs`, `arp.rs`, `ipv4.rs`, `ping4.rs`, `udp4.rs`, `ipv6.rs`, `icmpv6.rs`, `udp6.rs`, `csum.rs`, `ntp_server.rs`, `ntp_client.rs`, `ntp_demux.rs`, `clock_filter.rs`, `selection.rs`, `servo.rs`, `sync_state.rs`, `refid.rs`, `clock_disc.rs`, `phy_init.rs`, `config.rs`, `stream.rs`, `nd_client.rs` | comportement de chaque unité avec un banc minimal ; chaque `check!`/`assume!` de l'unité est aussi évalué à chaque cycle |
| **Vecteurs de référence** (*golden*) | `tools/golden/vectors/*.hex` (ARP, ping v4/v6, NDP, NTP v4/v6, NTP v3, non synchronisé, trames à rejeter) lus par `top.rs`, `ntp_server.rs`, `arp.rs`, ... | octet pour octet contre le modèle Python `tools/golden/` (règles N1-N10, A1-A5, I0-I6, ... : [../tools/golden/README.md](../tools/golden/README.md)) |
| **Système, sur broches MII simulées** | `top.rs` (serveur seul), `top_client.rs` (stratum 2) ; tests `real_ratio_*` | le top complet, quartets MII en entrée et en sortie, horloges `clk_rx`/`clk_tx` à 25 MHz réels (rapport 4:1 avec `clk_sys`, plusieurs phases) : ARP, ping, NDP, NTP, trafic mélangé, FCS faux, erreur des horodatages contre l'instant SFD ; pour le client : requêtes ARP/NS puis NTP octet pour octet, réponse falsifiée rejetée, synchronisation de bout en bout (le servo fait un pas de l'horloge réelle, puis le serveur répond stratum amont + 1) |
| **Chemin d'horloge MII** | `rx_path.rs`, `tx_path.rs` | passages d'horloge, appariement trame / horodatage, latence et erreur d'horodatage selon la phase |
| **Boucle fermée, modèles Python** | `tools/golden/ntpclient/` (`servo.py` : `simulate()`, `filter.py`, `selection.py`, `sync.py`, `client.py`) et leurs tests | comportement du servo (saut, pente, dérive de 50 ppm), bits identiques aux unités Rust sur des vecteurs (`ntpclient/vectors/`) |
| **Formelle** | `tools/formal/prove.sh`, `rtl/src/bin/formal_gen.rs`, `tools/formal/contracts.py` | voir ci-dessous |

Les tests de `top_client.rs` utilisent une horloge accélérée (`WarpClock`, **simulation seulement**, absente
des cartes) pour atteindre les délais du client (iburst à 2 s, échéance 1 à 2 s) ; les valeurs absolues des
horodatages n'y valent rien. Ils ne démontrent donc pas la convergence du servo sur des intervalles de
poll réels (64 s) : cela n'est couvert que par le modèle Python et par les tests unitaires du servo.

### Preuves formelles

```sh
pip install yowasp-yosys z3-solver
tools/formal/prove.sh formal_smoke prove       # k-induction non bornée ; doit finir par PASS
tools/formal/prove.sh <unité> bmc 40           # recherche bornée (contre-exemple en VCD)
tools/formal/prove.sh <unité> cover 40         # atteindre chaque cover!
```

`<unité>` est un nom connu de `rtl/src/bin/formal_gen.rs` (par exemple `arp`, `ipv4_rx`, `ntp_server`,
`ntp_client`, `selection`, `servo`, `mii_rx`, `tx_path`, `disciplined_clock`, `phy_init` ; la liste complète
est dans ce fichier). Un résultat UNKNOWN n'est pas une preuve : `prove.sh` sort avec un code non nul sauf
sur PASS. Les résultats sont dans `out/formal/<unité>/`. Ce que les preuves couvrent et ne couvrent pas :
[formal.md](formal.md) ; l'état unité par unité : [formal-report.md](formal-report.md). Vérifié ici :
`formal_smoke` donne PASS en une seconde environ. Les autres unités ne sont pas relancées pour ce guide.

La CI (`.github/workflows/formal.yml`) prouve les unités de sa matrice (neuf au moment de l'écriture) ;
une unité enregistrée dans `formal_gen.rs` n'est pas forcément dans cette matrice.

### Estimation de ressources

```sh
tools/gen_verilog.sh                                          # out/<module>.v
python3 -m venv /chemin/v && /chemin/v/bin/pip install pyosys
/chemin/v/bin/python tools/estimate_resources.py              # LUT / LUT-RAM / bascules par module
```

Méthode : Yosys natif, `synth_xilinx -family xc7` sur chaque `out/*.v`, module par module, sans
optimisation inter-modules, sans niveau supérieur ni analyse de timing ; précision annoncée ±15 %. Résultats
(cartes complètes serveur et client, mises à jour) dans [resources.md](resources.md). Seuls les dénombrements de
bascules de la section 3 (`top_gen` suivi de Yosys) viennent de [top-client-hooks.md](top-client-hooks.md).

## 5. Où est chaque module

Chemins sous `rtl/src/` ; tests sous `rtl/tests/`.

| Module | Fichier | Rôle | Tests |
|---|---|---|---|
| `mii_rx` | `mii_rx.rs` | quartets MII vers octets, préambule/SFD, impulsion `sfd` | `mii_rx.rs` |
| `mii_tx` | `mii_tx.rs` | octets vers quartets, préambule, IFG, impulsion `sfd` | `mii_tx.rs` |
| `rx_path` | `rx_path.rs` | passage `clk_rx` vers `clk_sys`, horodatage RX au SFD | `rx_path.rs` |
| `tx_path` | `tx_path.rs` | mémoire de trame (4096 octets), passage `clk_sys` vers `clk_tx`, horodatage TX | `tx_path.rs` |
| `crc32`, `frame_crc` | `crc32.rs` | CRC-32 Ethernet 8 bits/cycle | `crc32.rs` |
| `fcs_check` | `fcs_check.rs` | retire et vérifie le FCS | `fcs_check.rs` |
| `eth_rx` | `eth_rx.rs` | filtre MAC, classification ARP/IPv4/IPv6 | `eth_rx.rs` |
| `eth_tx` | `eth_tx.rs` | en-tête Ethernet, bourrage à 60 octets, FCS | `eth_tx.rs` |
| `tx_arb` | `tx_arb.rs` | arbitre de sortie (tourniquet) | `tx_arb.rs` |
| `phy_init` | `phy_init.rs` | reset du PHY, MDIO (ID, auto-négociation, lien) | `phy_init.rs` |
| `arp` | `arp.rs` | réponse et résolution ARP, expiration | `arp.rs` |
| `ipv4_rx`, `ipv4_tx` | `ipv4_rx.rs`, `ipv4_tx.rs` | analyse et construction IPv4 | `ipv4.rs` |
| `icmp_echo` | `icmp_echo.rs` | réponse au ping IPv4 | `ping4.rs` |
| `udp4_rx`, `udp4_tx` | `udp4_rx.rs`, `udp4_tx.rs` | UDP IPv4, port 123 | `udp4.rs` |
| `ipv6_rx`, `ipv6_tx` | `ipv6_rx.rs`, `ipv6_tx.rs` | IPv6 (adresses locales, pas d'en-têtes d'extension) | `ipv6.rs` |
| `icmpv6` | `icmpv6.rs` | echo, NS/NA | `icmpv6.rs` |
| `nbr_cache` | `nbr_cache.rs` | cache de voisins IPv6 (carte avec client) | `nd_client.rs` |
| `udp6_rx`, `udp6_tx` | `udp6_rx.rs`, `udp6_tx.rs` | UDP IPv6, somme de contrôle obligatoire | `udp6.rs` |
| `csum` | `csum.rs` | somme de contrôle Internet (pseudo-en-têtes) | `csum.rs` |
| `ntp_server` | `ntp_server.rs` | réponses mode 4, limiteur de débit, KoD (désactivé dans les tops) | `ntp_server.rs` |
| `ntp_reply_route` | `ntp_reply_route.rs` | route la réponse vers UDP v4/v6, constantes `TX_LATENCY_*` | `top.rs`, `ntp_server.rs` |
| `ntp_demux` | `ntp_demux.rs` | sépare requêtes (serveur) et réponses (client) | `ntp_demux.rs` |
| `ntp_client` | `ntp_client.rs` | requêtes mode 3 vers 4 amonts, T1..T4 | `ntp_client.rs` |
| `clock_filter` | `clock_filter.rs` | filtre d'horloge RFC 5905 (8 échantillons) | `clock_filter.rs` |
| `selection` | `selection.rs` | sélection et regroupement des amonts | `selection.rs` |
| `servo` | `servo.rs` | boucle PI : trim et saut | `servo.rs` |
| `sync_state` | `sync_state.rs`, `refid.rs` | stratum (amont + 1), LI, Reference ID, dispersion | `sync_state.rs`, `refid.rs` |
| `DisciplinedClock` | `clock.rs` | horloge 32.32 s, trim ±500 ppm, saut | `clock_disc.rs` |
| `Config` | `config.rs` | constantes statiques (adresses, amonts, poll, limite) | `config.rs` |
| `ip_arb`, `udp_arb` | `ip_arb.rs`, `udp_arb.rs` | arbitres IP et UDP (réponses serveur / requêtes client) | `top.rs`, `top_client.rs` |
| top serveur | `top.rs` (`ArtyBoard`) | assemblage et en-tête décrivant la hiérarchie, les canaux, le reset | `top.rs` |
| top stratum 2 | `top_client.rs` (`ArtyBoardWithClient`) | assemblage avec le client | `top_client.rs` |
| enveloppe de carte | `../hdl/arty_top_board.v` | MMCM, tampons d'horloge, séquenceur de reset, E/S du PHY | aucun test de simulation ; à vérifier sous Vivado |

L'en-tête de chaque fichier source est la spécification de l'unité (ports, timing, contrats de flux).

## 6. Intégration continue

Voir [../README.md](../README.md#ci). Les commandes de la CI sont celles de la section 4.
