# Roadmap — découpage pour agents

Chaque tâche est autonome : un agent peut la prendre avec seulement ce document, `docs/architecture.md` et les contrats de la section « Conventions ».

**Modèle recommandé**
- **Haiku** : mécanique, bien spécifié, peu de raisonnement (tables, fixtures, scripts, vérifications).
- **Sonnet** : module RTL standard avec spécification claire et test.
- **Opus** : conception délicate (CDC, timestamps, asservissement, filtre NTP) ou revue d'intégration.

Le modèle est une recommandation, pas une contrainte. Si un agent bloque, il le dit et la tâche monte d'un cran.

## Conventions (valables pour toutes les tâches)

1. **Une tâche = un fichier source principal** dans `rtl/src/`, déclaré dans `rtl/src/lib.rs`, plus un test dans `rtl/tests/` (`cargo test -p ntp_fpga`). Ne pas toucher aux fichiers des autres tâches, sauf `lib.rs` (ajout d'une ligne `pub mod`).
2. **Définition de « fini »** : (a) `cargo test -p ntp_fpga` passe, (b) `tools/gen_verilog.sh` produit le Verilog du module sans erreur, (c) le critère d'acceptation de la tâche est vérifié par un test, (d) un commit par tâche, message `T<id>: <titre>`.
3. **Branches** : une branche `task/T<id>` par tâche, fusion par l'humain ou par la tâche d'intégration. Pas de force-push.
4. **Interfaces** : les modules se parlent uniquement par les types de `rtl/src/stream.rs` (tâche T0.1). Pas de port ad hoc.
5. **Référence** : tout comportement protocolaire est comparé au modèle Python `tools/golden/` (tâche T0.3) ou à un vecteur de test réel (pcap hexadécimal).
6. **Pas de secrets, pas d'adresses réelles** dans les fixtures : utiliser `192.0.2.0/24`, `2001:db8::/32`, MAC `02:00:00:00:00:xx`.
7. **Horloges** : `clk_sys` 100 MHz, `clk_rx` 25 MHz (PHY), `clk_tx` 25 MHz (PHY). Tout passage entre domaines passe par `ChanCdc` de TxHDL ou un module de la tâche T1.4.
8. **Bloqué ?** Écrire la question en tête du fichier `docs/questions.md` (une entrée par tâche) et s'arrêter.

## Phase 0 — Fondations

| ID | Tâche | Modèle | Dépend de | Livrable | Critère d'acceptation |
|---|---|---|---|---|---|
| T0.1 | Types d'interface : flux d'octets de trame (`data: U<8>`, `valid`, `last`, `err`), timestamp (`U<64>`), métadonnées de paquet | Opus | — | `rtl/src/stream.rs` | Compile, exemple d'un producteur/consommateur en simulation, documenté en tête du fichier |
| T0.2 | Vérifier le XDC contre le XDC maître Digilent Arty A7-100 (broches PHY, horloge), corriger, ajouter contraintes I/O MII (input/output delay, `IOB`) | Haiku | — | `constraints/arty_a7_100.xdc` | Diff expliqué ligne par ligne dans le message de commit, broches identiques au XDC officiel |
| T0.3 | Modèle de référence Python : construction/analyse Ethernet, ARP, IPv4, IPv6, ICMP/ICMPv6, UDP, NTP v4 (checksums inclus) + générateur de vecteurs hexadécimaux | Sonnet | — | `tools/golden/*.py`, `tools/golden/vectors/*.hex` | `python3 -m unittest` passe ; vecteurs vérifiés avec `scapy` ou comparaison manuelle RFC |
| T0.4 | CI : `cargo fmt --check`, `clippy`, `cargo test`, `python3 -m unittest` | Haiku | — | `.github/workflows/ci.yml` | Workflow vert sur la branche |
| T0.5 | Script Vivado non interactif (projet, synthèse, P&R, bitstream, rapports timing/utilisation) | Sonnet | T0.2 | `tools/vivado/build.tcl`, `tools/vivado/README.md` | Script relu ; non exécutable ici (Vivado absent), documenté pour la machine de l'utilisateur |
| T0.6 | Banc de test partagé : lecture de vecteurs `.hex`, injection octet par octet, capture de sortie, comparaison | Sonnet | T0.1, T0.3 | `rtl/tests/common/mod.rs` | Utilisé par au moins un test existant |

## Phase 1 — MAC Ethernet MII et ping IPv4

| ID | Tâche | Modèle | Dépend de | Livrable | Critère d'acceptation |
|---|---|---|---|---|---|
| T1.1 | `mii_rx` : détection préambule/SFD, nibbles → octets, `rx_dv`/`rx_er` | Sonnet | T0.1 | `rtl/src/mii_rx.rs` | Trame de 64 et 1518 octets reçue à l'identique ; trame avec `rx_er` marquée en erreur |
| T1.2 | `mii_tx` : préambule, SFD, octets → nibbles, IFG de 96 bits | Sonnet | T0.1 | `rtl/src/mii_tx.rs` | Sortie conforme octet par octet, IFG respecté entre deux trames |
| T1.3 | CRC-32 Ethernet (génération et vérification, 8 bits/cycle) | Haiku | T0.1 | `rtl/src/crc32.rs` | Vecteur `123456789` → `0xCBF43926` ; FCS d'une trame connue |
| T1.4 | Timestamp au SFD + passage de domaine `clk_rx`/`clk_tx` → `clk_sys` | Opus | T0.1, T1.1, T1.2 | `rtl/src/ts_capture.rs` | Pas de métastabilité par construction (Gray/handshake), erreur de timestamp ≤ 1 cycle `clk_sys` + latence fixe documentée |
| T1.5 | Filtre MAC et classification : unicast, broadcast, multicast IPv6 `33:33:ff:xx:xx:xx`, EtherType → ARP/IPv4/IPv6 | Sonnet | T0.1 | `rtl/src/eth_rx.rs` | Test table : chaque classe de trame routée vers la bonne sortie, trames étrangères jetées |
| T1.6 | Constructeur de trame Ethernet + padding 60 octets + FCS | Sonnet | T0.1, T1.3 | `rtl/src/eth_tx.rs` | Trame courte rembourrée à 60 + FCS 4 |
| T1.7 | Arbitre de sortie : sélectionne ARP/ICMP/UDP v4/v6 vers `eth_tx`, sans interruption de trame | Sonnet | T0.1 | `rtl/src/tx_arb.rs` | Deux demandes simultanées servies l'une après l'autre sans corruption |
| T1.8 | Initialisation PHY : reset, MDIO (lecture ID, auto-négociation, état du lien) | Haiku | T0.1 | `rtl/src/mdio.rs` | Lecture du registre 2/3 simulée avec un PHY modèle |
| T1.9 | ARP : réponse + requête (résolution), cache de quelques entrées | Sonnet | T1.5, T1.6, T1.7, T0.3 | `rtl/src/arp.rs` | Réponse ARP identique au vecteur de référence ; entrée mise en cache |
| T1.10 | IPv4 : parsing, checksum header, rejet fragments/options ; ICMP echo | Sonnet | T1.5, T1.6, T0.3 | `rtl/src/ipv4.rs`, `rtl/src/icmp.rs` | Ping de référence → réponse identique octet par octet |
| T1.11 | Top phase 1 + bitstream ping | Opus | T1.1–T1.10, T0.5 | `rtl/src/top.rs`, `rtl/src/bin/top_gen.rs` | Banc de test : ARP puis ping en boucle complète en simulation ; Verilog généré |

## Phase 2 — Serveur NTP IPv4 (horloge libre)

| ID | Tâche | Modèle | Dépend de | Livrable | Critère d'acceptation |
|---|---|---|---|---|---|
| T2.1 | Horloge disciplinable : entrée d'incrément (trim ppb) et d'ajustement de phase (saut/slew) | Opus | `rtl/src/clock.rs` existant | `rtl/src/clock.rs` | Trim de ±500 ppm vérifié sur 1 s simulée à 1 ppm près ; saut appliqué sans retard d'un cycle |
| T2.2 | UDP IPv4 : parsing, démultiplexage port 123, checksum (ou 0 accepté en v4), construction de réponse | Sonnet | T1.10 | `rtl/src/udp4.rs` | Datagramme de référence → réponse correcte avec checksum valide |
| T2.3 | Serveur NTP : paquet mode 3 → mode 4, 48 octets, T1/T2/T3 corrects, Reference ID, stratum, LI, root delay/dispersion, poll | Opus | T1.4, T2.1, T2.2, T0.3 | `rtl/src/ntp_server.rs` | Réponse conforme au modèle Python, Transmit ≥ Receive, Origin = Transmit client |
| T2.4 | Politique : paquets non client ignorés, version 3/4 acceptées, KoD `RATE` si limite dépassée, `LI=3`/stratum 16 si non synchronisé | Sonnet | T2.3 | `rtl/src/ntp_policy.rs` | Table de cas testée |
| T2.5 | Limiteur de débit par source (table hachée petite) | Sonnet | T2.4 | `rtl/src/ratelimit.rs` | Rafale d'une même source limitée, autres sources inchangées |
| T2.6 | Test d'interopérabilité : fixtures `ntpdate -q`/`chronyc` rejouées en simulation | Haiku | T2.3 | `rtl/tests/ntp_interop.rs` | Réponses acceptées par le parseur Python de référence |

## Phase 3 — IPv6

| ID | Tâche | Modèle | Dépend de | Livrable | Critère d'acceptation |
|---|---|---|---|---|---|
| T3.1 | IPv6 : parsing en-tête fixe, adresse link-local EUI-64 + adresse statique configurable, rejet extension headers non gérés | Sonnet | T1.5, T0.3 | `rtl/src/ipv6.rs` | Paquets de référence acceptés/rejetés comme attendu |
| T3.2 | Checksum pseudo-en-tête (IPv6 UDP/ICMPv6), commun | Haiku | T0.1 | `rtl/src/csum.rs` | Vecteurs RFC, utilisé par T3.3–T3.5 |
| T3.3 | ICMPv6 : echo, NDP NS/NA (solicited-node), DAD minimal | Sonnet | T3.1, T3.2 | `rtl/src/icmpv6.rs` | `ping6` et NS de référence → NA identique |
| T3.4 | UDP IPv6 : checksum obligatoire, port 123 | Sonnet | T3.1, T3.2 | `rtl/src/udp6.rs` | Datagramme v6 de référence → réponse valide |
| T3.5 | Router Advertisement (SLAAC) en réception : préfixe /64, adresse dérivée | Sonnet | T3.3 | `rtl/src/slaac.rs` | RA de référence → adresse configurée |
| T3.6 | Intégration NTP IPv6 : `ntp_server` partagé v4/v6, Reference ID selon la famille | Opus | T2.3, T3.4 | `rtl/src/ntp_server.rs` (extension) | Mêmes tests que T2.3 en v6 |

## Phase 4 — Client NTP et stratum 2

| ID | Tâche | Modèle | Dépend de | Livrable | Critère d'acceptation |
|---|---|---|---|---|---|
| T4.1 | Registres de configuration : serveurs amont (v4/v6, statiques, pas de DNS en v1), MAC, adresses, poll | Haiku | T0.1 | `rtl/src/config.rs` | Valeurs par défaut documentées, accès par un bus simple |
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

## Graphe critique

`T0.1 → (T1.1, T1.2, T1.5, T1.6) → T1.4 → T1.11 → T2.3 → T3.6 → T4.2 → T4.5 → T4.7`

Parallélisables dès T0.1 : T0.2, T0.3, T0.4, T1.3, T1.8 (Haiku) et T1.1, T1.2, T1.5, T1.6 (Sonnet).

## Risques à surveiller

- **Précision des timestamps** : prise au SFD MII (25 MHz, 40 ns de granularité) ; la latence du PHY DP83848J est fixe mais à mesurer sur carte.
- **API TxHDL** : certaines constructions (affectation inconditionnelle, constantes larges) exigent des formes précises (`when!`). Les tâches doivent s'inspirer de `third_party/hdl-txhdl/lib/examples/`.
- **Pas de Vivado ici** : le timing réel n'est vérifiable que sur la machine de l'utilisateur (T0.5, T5.3).
