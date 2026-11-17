# Roadmap — découpage pour agents

Chaque tâche est autonome : un agent peut la prendre avec seulement ce document, `docs/architecture.md` et les contrats de la section « Conventions ».

**Modèle recommandé**
- **Haiku** : mécanique, bien spécifié, peu de raisonnement (tables, fixtures, scripts, vérifications).
- **Sonnet** : module RTL standard avec spécification claire et test.
- **Opus** : conception délicate (CDC, timestamps, asservissement, filtre NTP) ou revue d'intégration.

Le modèle est une recommandation, pas une contrainte. Si un agent bloque, il le dit et la tâche monte d'un cran.

## Conventions (valables pour toutes les tâches)

1. **Une tâche = un fichier source principal** dans `rtl/src/`, déclaré dans `rtl/src/lib.rs`, plus un test dans `rtl/tests/` (`cargo test -p ntp_fpga`). Ne pas toucher aux fichiers des autres tâches, sauf `lib.rs` (ajout d'une ligne `pub mod`).
2. **Définition de « fini »** : (a) `cargo test -p ntp_fpga` passe, (b) `tools/gen_verilog.sh` produit le Verilog du module sans erreur, (c) le critère d'acceptation de la tâche est vérifié par un test, (d) **la partie formelle est faite par l'orchestrateur après fusion (voir `docs/formal.md`, « Partage des rôles ») : les agents de codage ne rédigent pas les propriétés de leur module ; leur en-tête de module décrit précisément le comportement, le timing des ports et les contrats de flux dont dépendent les propriétés**, (e) un commit par tâche, message `T<id>: <titre>`. Une propriété hors de portée de l'induction est prouvée en `bmc` borné, avec la borne justifiée dans le commit.
3. **Branches** : une branche `task/T<id>` par tâche, fusion par l'humain ou par la tâche d'intégration. Pas de force-push.
4. **Interfaces** : les modules se parlent uniquement par les types de `rtl/src/stream.rs` (tâche T0.1). Pas de port ad hoc.
5. **Référence** : tout comportement protocolaire est comparé au modèle Python `tools/golden/` (tâche T0.3) ou à un vecteur de test réel (pcap hexadécimal).
6. **Pas de secrets, pas d'adresses réelles** dans les fixtures : utiliser `192.0.2.0/24`, `2001:db8::/32`, MAC `02:00:00:00:00:xx`.
7. **Horloges** : `clk_sys` 100 MHz, `clk_rx` 25 MHz (PHY), `clk_tx` 25 MHz (PHY). Tout passage entre domaines passe par `ChanCdc` de TxHDL ou un module de la tâche T1.4.
8. **Canaux** : deux modules ne se connectent jamais port à port en Verilog (boucle combinatoire valid/ready) : passer par un canal registré, voir la note « Netlist » de `rtl/src/stream.rs`.
9. **Bloqué ?** Écrire la question en tête du fichier `docs/questions.md` (une entrée par tâche) et s'arrêter.

## Phase 0 — Fondations

| ID | Tâche | Modèle | Dépend de | Livrable | Critère d'acceptation |
|---|---|---|---|---|---|
| T0.1 ✅ | Types d'interface : flux d'octets de trame (`data: U<8>`, `valid`, `last`, `err`), timestamp (`U<64>`), métadonnées de paquet | Opus | — | `rtl/src/stream.rs` | Compile, exemple d'un producteur/consommateur en simulation, documenté en tête du fichier |
| T0.2 | Vérifier le XDC contre le XDC maître Digilent Arty A7-100 (broches PHY, horloge), corriger, ajouter contraintes I/O MII (input/output delay, `IOB`) | Haiku | — | `constraints/arty_a7_100.xdc` | Diff expliqué ligne par ligne dans le message de commit, broches identiques au XDC officiel |
| T0.3 | Modèle de référence Python : construction/analyse Ethernet, ARP, IPv4, IPv6, ICMP/ICMPv6, UDP, NTP v4 (checksums inclus) + générateur de vecteurs hexadécimaux | Sonnet | — | `tools/golden/*.py`, `tools/golden/vectors/*.hex` | `python3 -m unittest` passe ; vecteurs vérifiés avec `scapy` ou comparaison manuelle RFC |
| T0.4 | CI : `cargo fmt --check`, `clippy`, `cargo test`, `python3 -m unittest` | Haiku | — | `.github/workflows/ci.yml` | Workflow vert sur la branche |
| T0.5 | Script Vivado non interactif (projet, synthèse, P&R, bitstream, rapports timing/utilisation) | Sonnet | T0.2 | `tools/vivado/build.tcl`, `tools/vivado/README.md` | Script relu ; non exécutable ici (Vivado absent), documenté pour la machine de l'utilisateur |
| T0.6 | Banc de test partagé : lecture de vecteurs `.hex`, injection octet par octet, capture de sortie, comparaison | Sonnet | T0.1, T0.3 | `rtl/tests/common/mod.rs` | Utilisé par au moins un test existant |

## Phase 1 — MAC Ethernet MII et ping IPv4

| ID | Tâche | Modèle | Dépend de | Livrable | Critère d'acceptation |
|---|---|---|---|---|---|
| T1.1 | `mii_rx` (domaine `ClkRx`) : préambule/SFD, nibbles → octets, `rx_dv`/`rx_er`, impulsion `sfd` ; sortie = trame brute FCS inclus | Sonnet | T0.1 | `rtl/src/mii_rx.rs` | Trame de 64 et 1518 octets reçue à l'identique ; trame avec `rx_er` marquée en erreur |
| T1.2 | `mii_tx` (domaine `ClkTx`) : préambule, SFD, octets → nibbles, IFG de 96 bits, impulsion `sfd` ; entrée = trame complète (rembourrée, FCS inclus) | Sonnet | T0.1 | `rtl/src/mii_tx.rs` | Sortie conforme octet par octet, IFG respecté entre deux trames |
| T1.3 | CRC-32 Ethernet (8 bits/cycle) : fonction `crc32_step` + unité `FrameCrc` | Haiku | T0.1 | `rtl/src/crc32.rs` | Vecteur `123456789` → `0xCBF43926` ; FCS d'une trame connue |
| T1.4 | Timestamp au SFD + passage de domaine `clk_rx`/`clk_tx` → `clk_sys` | Opus | T0.1, T1.1, T1.2 | `rtl/src/ts_capture.rs` | Pas de métastabilité par construction (Gray/handshake), erreur de timestamp ≤ 1 cycle `clk_sys` + latence fixe documentée |
| T1.5 | Filtre MAC et classification : unicast, broadcast, multicast IPv6 `33:33:ff:xx:xx:xx`, EtherType → ARP/IPv4/IPv6 | Sonnet | T0.1 | `rtl/src/eth_rx.rs` | Test table : chaque classe de trame routée vers la bonne sortie, trames étrangères jetées |
| T1.6 | Constructeur de trame Ethernet + padding 60 octets + FCS | Sonnet | T0.1, T1.3 | `rtl/src/eth_tx.rs` | Trame courte rembourrée à 60 + FCS 4 |
| T1.7 | Arbitre de sortie : sélectionne ARP/ICMP/UDP v4/v6 vers `eth_tx`, sans interruption de trame | Sonnet | T0.1 | `rtl/src/tx_arb.rs` | Deux demandes simultanées servies l'une après l'autre sans corruption |
| T1.8 | Initialisation PHY : reset, MDIO (lecture ID, auto-négociation, état du lien) | Haiku | T0.1 | `rtl/src/mdio.rs` | Lecture du registre 2/3 simulée avec un PHY modèle |
| T1.9 | ARP : réponse + requête (résolution), cache de quelques entrées | Sonnet | T1.5, T1.6, T1.7, T0.3 | `rtl/src/arp.rs` | Réponse ARP identique au vecteur de référence ; entrée mise en cache |
| T1.12 | `fcs_check` : retire et vérifie le FCS d'un flux de trame brute (délai de 4 octets), `err` sur `last` si FCS faux | Sonnet | T1.1, T1.3 | `rtl/src/fcs_check.rs` | Trame valide : FCS retiré, pas d'erreur ; un bit modifié : `err` |
| T1.10 | IPv4 : parsing, checksum header, rejet fragments/options ; ICMP echo | Sonnet | T1.5, T1.6, T0.3 | `rtl/src/ipv4.rs`, `rtl/src/icmp.rs` | Ping de référence → réponse identique octet par octet |
| T1.11 | Top phase 1 + bitstream ping | Opus | T1.1–T1.10, T0.5 | `rtl/src/top.rs`, `rtl/src/bin/top_gen.rs` | Banc de test : ARP puis ping en boucle complète en simulation ; Verilog généré |

## Phase 2 — Serveur NTP IPv4 (horloge libre)

| ID | Tâche | Modèle | Dépend de | Livrable | Critère d'acceptation |
|---|---|---|---|---|---|
| T2.1 | Horloge disciplinable : entrée d'incrément (trim ppb) et d'ajustement de phase (saut/slew) | Opus | `rtl/src/clock.rs` existant | `rtl/src/clock.rs` | Trim de ±500 ppm vérifié sur 1 s simulée à 1 ppm près ; saut appliqué sans retard d'un cycle |
| T2.2 | UDP IPv4 : parsing, démultiplexage port 123, checksum (ou 0 accepté en v4), construction de réponse | Sonnet | T1.10 | `rtl/src/udp4.rs` | Datagramme de référence → réponse correcte avec checksum valide |
| T2.3 | Serveur NTP : paquet mode 3 → mode 4, 48 octets, T1/T2/T3 corrects, Reference ID, stratum, LI, root delay/dispersion, poll | Opus | T1.4, T2.1, T2.2, T0.3 | `rtl/src/ntp_server.rs` | Réponse conforme au modèle Python, Transmit ≥ Receive, Origin = Transmit client |
| T2.4 | Politique : paquets non client ignorés, version 3/4 acceptées, KoD `RATE` si limite dépassée, `LI=3`/stratum 16 si non synchronisé | Sonnet | T2.3 | `rtl/src/ntp_policy.rs` | Table de cas testée |
| T2.7 | Champ *precision* : valeur −24 par défaut (timestamp MII 40 ns), registre configurable ; décision tracée dans `docs/references.md` | Haiku | T2.3 | `rtl/src/ntp_server.rs` | Champ présent dans la réponse, testé |
| T2.5 | Limiteur de débit par source (table hachée petite) | Sonnet | T2.4 | `rtl/src/ratelimit.rs` | Rafale d'une même source limitée, autres sources inchangées |
| T2.8 | Test de débit : 100 000 requêtes IPv4 à pleine cadence ligne, aucune perte jusqu'au débit ligne, réponses toutes correctes | Sonnet | T2.3 | `rtl/tests/throughput.rs` | ≥ 109 000 paquets/s traités en simulation cycle par cycle (IPv6 : ≥ 93 000) |
| T2.6 | Test d'interopérabilité : fixtures `ntpdate -q`/`chronyc` rejouées en simulation | Haiku | T2.3 | `rtl/tests/ntp_interop.rs` | Réponses acceptées par le parseur Python de référence |

## Phase 3 — IPv6

| ID | Tâche | Modèle | Dépend de | Livrable | Critère d'acceptation |
|---|---|---|---|---|---|
| T3.1 | IPv6 : parsing en-tête fixe, adresse link-local EUI-64 + adresse statique configurable, rejet extension headers non gérés | Sonnet | T1.5, T0.3 | `rtl/src/ipv6.rs` | Paquets de référence acceptés/rejetés comme attendu |
| T3.2 | Checksum pseudo-en-tête (IPv6 UDP/ICMPv6), commun | Haiku | T0.1 | `rtl/src/csum.rs` | Vecteurs RFC, utilisé par T3.3–T3.5 |
| T3.3 | ICMPv6 : echo, NDP NS/NA (solicited-node), DAD minimal | Sonnet | T3.1, T3.2 | `rtl/src/icmpv6.rs` | `ping6` et NS de référence → NA identique |
| T3.4 | UDP IPv6 : checksum obligatoire, port 123 | Sonnet | T3.1, T3.2 | `rtl/src/udp6.rs` | Datagramme v6 de référence → réponse valide |
| T3.5 (optionnelle, hors v1) | Router Advertisement (SLAAC) en réception : préfixe /64, adresse dérivée | Sonnet | T3.3 | `rtl/src/slaac.rs` | RA de référence → adresse configurée |
| T3.6 | Intégration NTP IPv6 : `ntp_server` partagé v4/v6, Reference ID selon la famille | Opus | T2.3, T3.4 | `rtl/src/ntp_server.rs` (extension) | Mêmes tests que T2.3 en v6 |

## Phase 4 — Client NTP et stratum 2

| ID | Tâche | Modèle | Dépend de | Livrable | Critère d'acceptation |
|---|---|---|---|---|---|
| T4.1 | Registres de configuration : serveurs amont (v4/v6, statiques, pas de DNS en v1), MAC, adresses, poll ; **prouve** `local_mac` unicast (garant de l'hypothèse de T1.5, voir `docs/formal.md`) | Haiku | T0.1 | `rtl/src/config.rs` | Valeurs par défaut documentées, accès par un bus simple |
| T4.1b | `config` : ajouter `GATEWAY_IP6` (passerelle IPv6 statique, link-local ou globale) et le fil `gateway6` ; test et vérification (pas de DHCP, voir `docs/architecture.md`) | Haiku | T4.1 | `rtl/src/config.rs` | Test du fil, adresse non multicast vérifiée à la compilation |
| T4.2 | Client NTP : émission de requêtes mode 3 vers N amonts, collecte de T1..T4 (offset, delay) | Opus | T2.3, T3.4, T4.1 | `rtl/src/ntp_client.rs` | Échanges simulés avec un serveur modèle, offset/delay exacts à 1 LSB |
| T4.3 | Filtre d'horloge RFC 5905 (8 échantillons, meilleur delay, dispersion, jitter) | Opus | T4.2 | `rtl/src/clock_filter.rs` | Comparaison au modèle Python sur 1000 échantillons aléatoires |
| T4.4 | Sélection et regroupement (clustering) des amonts, rejet des *falsetickers* | Opus | T4.3 | `rtl/src/selection.rs` | Cas : 1 amont faux sur 3 écarté |
| T4.5 | Servo PI : écart → trim ppb et sauts (`>128 ms`), anti-windup, états SYNC/SPIK/FREQ | Opus | T2.1, T4.4 | `rtl/src/servo.rs` | Simulation : écart initial 50 ms rattrapé, erreur résiduelle < 1 ms, dérive de 50 ppm compensée |
| T4.6 | État de synchronisation : calcul stratum (amont + 1), LI, root delay/dispersion, reference ID/time | Sonnet | T4.4, T2.3 | `rtl/src/sync_state.rs` | Stratum 2 si un stratum 1 valide est sélectionné, 16 sinon |
| T4.7 | Banc de bout en bout : amont simulé + client + servo + serveur, clients de test | Opus | T4.1–T4.6 | `rtl/tests/e2e_stratum2.rs` | Serveur synchronisé en < 10 min simulées, sert stratum 2 |

## Phase 5 — Robustesse et mise en service

| ID | Tâche | Modèle | Dépend de | Livrable | Critère d'acceptation |
|---|---|---|---|---|---|
| T5.1 | Sortie de diagnostic UART : état de synchro, offset, compteurs | Haiku | T4.6 | `rtl/src/uart_diag.rs` | Trame texte simulée conforme |
| T5.2 | Compteurs (paquets, erreurs, KoD, trames FCS invalides) | Haiku | T1.11 | `rtl/src/counters.rs` | Incrément vérifié par événement |
| T5.3 | Analyse temporelle : revue du timing Vivado, CDC, rapport des chemins critiques | Opus | T0.5, T1.11 | `docs/timing.md` | Pas de violation à 100 MHz, chemins inter-horloges déclarés |
| T5.4 | Procédure de test réelle (chrony/ntpd/`ntpq`) sur carte, mesures d'offset et de jitter | Sonnet | T4.7 | `docs/bringup.md` | Procédure pas à pas, tableau de résultats à remplir |
| T5.5 | Revue de sécurité : amplification, spoofing, taille des réponses, KoD | Opus | T2.5, T4.7 | `docs/security.md` | Liste de risques et décisions |
| T5.6 | Documentation utilisateur finale | Haiku | tout | `README.md`, `docs/` | Un lecteur reproduit la génération et le flash |

## Piste F — Preuves formelles

Voir `docs/formal.md` (méthode, catalogue de propriétés, règles anti-vacuité). Les tâches RTL prouvent leurs propres propriétés (définition de « fini », point 2d) ; cette piste construit l'outillage et prouve les propriétés transverses.

| ID | Tâche | Modèle | Dépend de | Livrable | Critère d'acceptation |
|---|---|---|---|---|---|
| F0.1 | Outillage de preuve (fait) : `prove.sh`, `formal_gen`, unité `formal_smoke` | — | — | `tools/formal/prove.sh`, `rtl/src/formal_smoke.rs` | PASS sur `formal_smoke` ; propriété fausse → FAIL |
| F0.2 | Étendre `formal_gen` : registre d'unités, prise en charge des unités composées (vérifier que TxHDL génère l'`ifdef FORMAL` pour les sous-unités) | Sonnet | T0.1 | `rtl/src/bin/formal_gen.rs` | Une unité composée de deux sous-unités prouvée |
| F0.3 | Vérificateur de garantie/hypothèse : chaque `assume!` a un `check!` de même message ailleurs, sinon échec | Haiku | F0.2 | `tools/formal/contracts.py` | Détecte une hypothèse orpheline sur un cas de test |
| F0.4 | Job CI : preuves de toutes les unités + `cover`, temps limité, résumé PASS/FAIL/UNKNOWN | Haiku | T0.4, F0.2 | `.github/workflows/formal.yml` | UNKNOWN fait échouer le job |
| F0.5 | Moniteurs de flux : unité réutilisable portant les propriétés « flux d'octets » du catalogue | Opus | T0.1, F0.2 | `rtl/src/monitors/stream_mon.rs` | Branchée sur un producteur correct : PASS ; sur un producteur mutant : FAIL |
| F1.1 ✅ | Équivalence CRC-32 : le module de T1.3 égale un modèle bit à bit (fait : référence matricielle GF(2) générée en Python, `rtl/src/crc_equiv.rs`) | orchestrateur | T1.3, F0.2 | `rtl/src/formal/crc_equiv.rs` | Preuve non bornée PASS sur le CRC 8 bits/cycle |
| F1.2 | Propriétés MII RX/TX (préambule/SFD, IFG, fidélité des octets) | Opus | T1.1, T1.2, F0.5 | dans `mii_rx.rs`, `mii_tx.rs` | Tableau « MII » du catalogue entièrement couvert |
| F1.3 | Propriétés Ethernet/ARP/IPv4/ICMP (champs, checksums, rejets) | Sonnet | T1.5–T1.10, F0.5 | dans chaque module | Tableaux correspondants couverts, tous les `cover!` atteints |
| F2.1 | Propriétés UDP et checksum v4/v6 (équivalence avec modèle RFC 1071) | Sonnet | T2.2, T3.2, F0.2 | `rtl/src/formal/csum_equiv.rs` | Preuve PASS |
| F2.2 | Propriétés du serveur NTP (origin/transmit, stratum, LI, taille, pas d'amplification) | Opus | T2.3, T2.4, F0.5 | dans `ntp_server.rs`, `ntp_policy.rs` | Ligne « NTP serveur » et « Politique/KoD » couvertes |
| F3.1 | Propriétés IPv6 / ICMPv6 / NDP | Sonnet | T3.1–T3.5, F0.5 | dans chaque module | Lignes correspondantes couvertes |
| F4.1 | Propriétés de l'horloge : monotonie, bornes de trim, absence de dépassement (invariant) | Opus | T2.1, F0.2 | dans `clock.rs` | Preuve non bornée PASS, ou invariant documenté et prouvé |
| F4.2 | Propriétés du client, filtre, sélection, servo, état de synchro | Opus | T4.2–T4.6 | dans chaque module | Lignes « Client », « Filtre », « Servo », « État » couvertes |
| F5.1 | Top formel de bout en bout : hypothèses sur l'environnement seulement, vivacité bornée (réponse en ≤ N cycles) | Opus | F2.2, F3.1, T1.11 | `rtl/src/formal/top_formal.rs` | PASS ; borne N justifiée |
| F5.2 | Rapport de couverture des preuves : propriétés du catalogue ↔ preuves ↔ `cover!` ; trous listés | Haiku | tout F | `docs/formal-report.md` | Chaque ligne du catalogue est cochée ou justifiée |

## Graphe critique

`T0.1 → (T1.1, T1.2, T1.5, T1.6) → T1.4 → T1.11 → T2.3 → T3.6 → T4.2 → T4.5 → T4.7`

Parallélisables dès T0.1 : T0.2, T0.3, T0.4, T1.3, T1.8 (Haiku) et T1.1, T1.2, T1.5, T1.6 (Sonnet).

## Références

Voir `docs/references.md` : Fakernet (NTP Arty A7, stratum 1, IPv4) et Netnod (NTP/NTS matériel en production). Inspiration uniquement, pas de copie de code.

## Risques à surveiller

- **Précision des timestamps** : prise au SFD MII (25 MHz, 40 ns de granularité) ; la latence du PHY DP83848J est fixe mais à mesurer sur carte.
- **API TxHDL** : certaines constructions (affectation inconditionnelle, constantes larges) exigent des formes précises (`when!`). Les tâches doivent s'inspirer de `third_party/hdl-txhdl/lib/examples/`.
- **Pas de Vivado ici** : le timing réel n'est vérifiable que sur la machine de l'utilisateur (T0.5, T5.3).
