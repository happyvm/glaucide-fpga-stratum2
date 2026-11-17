# Mise en service sur la carte et mesures (T5.4)

Procédure pas à pas pour construire le bitstream, le charger sur une Arty A7-100T, vérifier le lien,
les réponses du serveur, la synchronisation en stratum 2, puis mesurer la précision.

**Avertissement : rien de ce document n'a été exécuté sur une carte.** Le projet est simulé de bout en
bout ; le script Vivado, le timing à 100 MHz, les croisements d'horloge, le comportement du PHY et les
mesures de précision n'ont jamais été vérifiés. Les « observations attendues » viennent de la simulation et de
la lecture du code ; chaque point non vérifié est marqué **NON VÉRIFIÉ** ou « à vérifier ». Lorsqu'un
résultat réel diffère de celui annoncé ici, c'est le document qui a tort : corrigez-le.

Lecture préalable : [user-guide.md](user-guide.md) (construction), [config-guide.md](config-guide.md) (adresses),
[status.md](status.md) (limites). La revue de timing (T5.3, `docs/timing.md`) et la revue de sécurité (T5.5,
`docs/security.md`) sont **à venir** ; ne présumez pas de leur contenu.

## 0. Matériel et préparation

- Une carte Arty A7-100T, un câble micro-USB (alimentation et JTAG), un câble Ethernet.
- Un hôte Linux avec une interface Ethernet **directement reliée** à la carte (pas de commutateur pour les
  mesures : un commutateur ajoute des délais variables et asymétriques). L'hôte négocie 100 Mb/s duplex
  intégral avec le PHY ; si l'auto-négociation échoue, essayez `sudo ethtool -s eth0 speed 100 duplex full autoneg on`
  (`eth0` est l'interface de l'hôte dans tout ce document).
- Pour la section 5 : une carte supplémentaire (référence), une carte réseau d'hôte avec horodatage matériel
  (`ethtool -T eth0`), `chrony` ou `ntpd`.
- Outils : `arping` (iputils), `ping`, `ndisc6`, `tcpdump`, `ntpdate` (ou `sntp`, ou `ntpdig` de ntpsec),
  `chronyc`/`ntpq`, `ethtool`.

Désactivez la gestion automatique de l'interface par NetworkManager ou `systemd-networkd` pour ce test, puis
configurez-la à la main. Avec les adresses de `config::Board` :

```sh
sudo ip link set eth0 up
sudo ip addr add 192.0.2.1/24 dev eth0
sudo ip -6 addr add 2001:db8:0:1::1/64 dev eth0
```

Adaptez si vous avez changé la configuration ([config-guide.md](config-guide.md)). **Construisez d'abord
`ArtyBoard`** (serveur seul, sections 1 à 4), puis `ArtyBoardWithClient` (section 6).

## 1. Construction Vivado

```sh
cargo run -p ntp_fpga --bin top_gen                       # ou ... -- --with-client
vivado -mode batch -source tools/vivado/build.tcl         # résultats dans out/vivado/
vivado -mode batch -source tools/vivado/build.tcl -tclargs /chemin/vers/build
```

### Entrées et sorties

Entrées : `out/top/files.f` (donc `arty_top.v` et `arty_top_board.v`), `constraints/arty_a7_100.xdc` (broches,
horloges primaires, délais d'E/S MII), `tools/vivado/top_timing.xdc` (horloge transmise `eth_ref_clk`,
groupes asynchrones, faux chemins MDIO/MDC/reset). Composant `xc7a100tcsg324-1`, top `arty_top_board`.

Sorties dans `out/vivado/` : `arty_ntp.bit`, `post_synth.dcp`, `post_route.dcp`, et `reports/` :
`timing_summary.rpt`, `utilization.rpt`, `utilization_hier.rpt`, `clock_utilization.rpt`,
`clock_interaction.rpt`, `cdc.rpt`, `methodology.rpt`, `drc.rpt`, `io.rpt`, `utilization_synth.rpt`,
`timing_synth.rpt`.

### Ordre de lecture des rapports

1. La fin de la console : `WNS = ... ns, WHS = ... ns` puis `Bitstream: ...`. Le script s'arrête avec
   `timing not met (WNS x ns, WHS y ns): see .../timing_summary.rpt` si WNS ou WHS est négatif.
   **Attention : le bitstream est écrit avant ce verdict** (`write_bitstream` précède le contrôle). Un fichier
   `arty_ntp.bit` existe donc même quand le timing échoue ; ne le chargez pas dans ce cas.
2. `timing_summary.rpt` : WNS, WHS, TNS ; « unconstrained paths » (doit être vide : sinon une contrainte
   manque) ; horloges non propagées.
3. `cdc.rpt` : seuls passages attendus : ceux de `rx_path` et `tx_path` (codes de Gray, poignées de main) et
   les synchroniseurs à deux bascules `ASYNC_REG` de l'enveloppe (autorisations d'horloge MII, `locked` du
   MMCM). **Tout autre passage est une erreur de conception.**
4. `clock_interaction.rpt` : aucun passage non déclaré entre `clk_sys` (100 MHz), `clk_25`, `eth_rx_clk`,
   `eth_tx_clk` ; ils sont placés en groupes asynchrones par `top_timing.xdc`.
5. `methodology.rpt` et `drc.rpt` : chaque avertissement sur un croisement d'horloge, un reset ou une E/S est
   à comprendre.
6. `utilization.rpt` : à comparer à l'estimation ([resources.md](resources.md), environ 15 % des LUT pour le serveur
   seul ; 29 255 bascules sur 126 800 pour la carte avec client d'après Yosys). Un écart important signale un
   problème de génération.
7. `clock_utilization.rpt` : sept tampons globaux attendus (retour MMCM, `clk_sys`, `clk_25`, copies libres
   `eth_rx_clk`/`eth_tx_clk`, deux `BUFGCE`).
8. `io.rpt` : placement des broches contre le XDC.

### Points NON VÉRIFIÉS à regarder au premier passage

Ce sont les hypothèses du projet que Vivado n'a jamais confrontées à la réalité (liste de
`tools/vivado/README.md` et de [status.md](status.md)) :

- **Croisements d'horloge** : tous les passages sont censés être dans `rx_path`, `tx_path` et les
  synchroniseurs de l'enveloppe. Rien ne l'a jamais confirmé par `report_cdc`.
- **BUFG et BUFGCE sur les horloges du PHY** : `eth_rx_clk` (F15) et `eth_tx_clk` (H16) alimentent chacune un
  `BUFG` (copie libre, uniquement pour deux bascules de synchronisation) et un `BUFGCE` (l'horloge du domaine
  MII, coupée autour de la levée du reset). Cette structure n'a pas été placée ; une erreur de placement liée
  au chemin d'horloge dédié (propriété `CLOCK_DEDICATED_ROUTE`) ou au `BUFGCE` est possible. L'argument de
  sûreté du reset ([../rtl/hdl/arty_top_board.v](../rtl/hdl/arty_top_board.v), en-tête) est un raisonnement, pas un résultat
  d'outil.
- **`IOB TRUE`** : sur les entrées MII, `mii_rx` n'a pas de bascule directement sur les broches ; Vivado
  signalera vraisemblablement que la propriété ne peut pas être honorée. Les sorties `eth_txd`/`eth_tx_en` sont des
  registres de `mii_tx` sans autre charge et devraient être placées en IOB : à vérifier dans `io.rpt` ou dans le
  DCP final.
- **Front de capture TX du PHY** : les `set_output_delay` supposent que le PHY échantillonne TXD sur le front
  montant de `TX_CLK` à 100 Mb/s. La note 12 de la datasheet (SNLS250E, figure 5-4) mentionne un échantillonnage sur
  le front descendant ; elle est rattachée au 10 Mb/s mais le point n'est **pas tranché**
  ([../constraints/README.md](../constraints/README.md), TODO 2). Symptôme possible si c'est faux : aucune trame de la carte n'est
  reçue ou toutes ont un FCS faux, alors que la réception fonctionne.
- **Chemins larges** : somme `pseudo_v6` d'`icmpv6` (19 additions enchaînées), sommes de contrôle 128 bits,
  comparateurs 64 bits et multiplieurs de `ntp_server`, `selection`, `servo`.
- Le fichier XDC lui-même n'a jamais été lu par Vivado (syntaxe vérifiée seulement par relecture).

### Que faire si le timing échoue

1. Identifiez le chemin le plus long (`timing_summary.rpt`, section des pires chemins) et l'unité d'origine (le
   nom hiérarchique est celui des modules générés, p. ex. `core_icmp6_...`).
2. Si le chemin est un passage d'horloge non déclaré : c'est un défaut de conception, pas un réglage
   (`cdc.rpt`, `clock_interaction.rpt`). Ne le masquez pas par un faux chemin sans comprendre.
3. Si c'est un chemin large synchrone à `clk_sys` : le remède est de le découper en étages dans le RTL
   (modification des unités concernées, hors de ce document). Pour un essai rapide on peut relancer le flot avec
   d'autres directives de placement-routage dans `build.tcl` (à essayer, non testé : le script ne les utilise pas).
4. Ne changez pas la fréquence de `clk_sys` (le diviseur du MMCM dans `arty_top_board.v`) sans reprendre le
   code : l'incrément de l'horloge, le diviseur MDC, les délais du PHY et les latences calibrées supposent 100 MHz.
5. Conservez le rapport et notez-le dans le tableau de la section 8 : c'est une entrée de la revue de timing T5.3.

## 2. Programmation de la carte

La configuration chargée en SRAM est volatile : **après toute coupure d'alimentation, il faut reprogrammer**.
`build.tcl` ne produit pas de fichier pour la mémoire flash ; la programmation permanente n'est pas traitée ici.

### Vivado Hardware Manager (méthode de référence, NON VÉRIFIÉE)

Branchez le micro-USB de l'Arty. Dans Vivado : *Open Hardware Manager*, *Open target*, *Auto Connect*, clic
droit sur le composant `xc7a100t_0`, *Program Device*, choisissez `out/vivado/arty_ntp.bit`, *Program*. Sous
Linux, le câble Digilent peut demander l'installation de règles `udev` fournies avec Vivado (à vérifier
sur votre installation). En ligne de commande, la même opération passe par un script Tcl
(`open_hw_manager`, `connect_hw_server`, `open_hw_target`, `program_hw_devices`) : non fourni, non testé.

### openFPGALoader (à vérifier)

`openFPGALoader` sait en principe programmer l'Arty A7-100T avec un fichier `.bit`, par exemple
`openFPGALoader -b arty_a7_100t out/vivado/arty_ntp.bit`. Le nom de carte et les options ne sont **pas
vérifiés** ici (l'outil n'est pas installé dans l'environnement de développement) : consultez `openFPGALoader --help`
et sa liste de cartes. N'utilisez pas d'option d'écriture en flash sans l'avoir lue.

## 3. Lien et PHY

### Ce que fait la carte au démarrage (`phy_init`, simulé)

1. L'enveloppe maintient le reset des modules jusqu'au verrouillage du MMCM puis environ 190 cycles de
   `clk_sys` ; les horloges MII sont coupées autour de la levée du reset.
2. `eth_rstn` est maintenu **bas 10 ms**, puis relâché.
3. Attente de **200 ms** (le DP83848J a besoin d'environ 167 ms après reset avant l'accès MDIO).
4. Lecture de l'identifiant par MDIO (adresse PHY 1, MDC 1 MHz) : registre 2 = `0x2000`, registre 3 = `0x5C9x` (les 4 bits
   de poids faible, révision du silicium, sont ignorés). En cas d'écart, nouvelle lecture toutes les 100 ms.
5. Écriture de BMCR = `0x1200` (auto-négociation activée et relancée).
6. Interrogation de BMSR toutes les 100 ms pour l'état du lien.

Le lien ne peut donc apparaître que **plus de 0,2 s après la programmation**, plus la durée de l'auto-négociation
(de l'ordre de la seconde ; à vérifier). Quand le PHY ne répond pas (mauvaise adresse, pas d'horloge de référence,
broche MDIO), `id_ok` reste bas, BMCR n'est jamais écrit et la carte continue de lire l'identifiant : rien
d'autre ne se produit.

### Pas de LED, pas de sortie de diagnostic

**Aucune LED d'état n'est câblée** : les signaux `link_up`, `id_ok` et `phy_ready` de `phy_init` sortent du
module `arty_top` mais l'enveloppe ne les relie à aucune broche (le XDC ne contient pas de LED), et il n'y a pas
d'UART de diagnostic (T5.1 non fait). L'état interne n'est pas observable sans ajouter un analyseur logique
interne (ILA) ou un câblage de LED, qui ne sont pas fournis. Si la prise RJ45 de votre carte a des LED, elles
sont pilotées par le PHY et non par le FPGA (à vérifier sur la fiche de la carte). Les moyens d'observation
minimaux sont donc ceux de l'hôte :

```sh
ethtool eth0                   # Link detected: yes ; Speed: 100Mb/s ; Duplex: Full  (attendu une fois le lien monté)
ip -s link show eth0           # compteurs RX/TX : des paquets reçus de la carte prouvent qu'elle émet
sudo tcpdump -n -e -i eth0     # toutes les trames, avec adresses MAC
```

- **Pas de lien** (`Link detected: no`) : câble ; la carte n'est pas programmée ; **horloge `eth_ref_clk`** (le
  PHY a besoin des 25 MHz fournis par le FPGA, via l'`ODDR` de l'enveloppe) ; broches du XDC. Sans lien rien
  ne sera visible, y compris sur `ArtyBoard`.
- **Lien monté mais aucun trafic** : jouez ARP (section 4). Si rien ne répond : problème au niveau du MII
  (horloges coupées par `BUFGCE`, `IOB`, front de capture) ; revoyez les points non vérifiés de la section 1.
- **Lien présent puis perdu** à intervalles réguliers : BMCR/BMSR (cycle de relance) ; notez les dates.
- Les trames reçues par la carte avec un FCS faux sont silencieusement jetées (aucun compteur : T5.2 non fait).

## 4. Vérifications fonctionnelles (carte `ArtyBoard`, adresses de `Board`)

Valeurs de `Board` : MAC `02:4e:54:50:00:01`, IPv4 `192.0.2.10`, globale `2001:db8:0:1::50:1`, lien-local
`fe80::4e:54ff:fe50:1`. Les « sorties attendues » ci-dessous sont des formes, non des copies d'une exécution réelle.

| # | Commande | Attendu |
|---|---|---|
| 1 | `sudo arping -I eth0 -c 3 192.0.2.10` | `Unicast reply from 192.0.2.10 [02:4E:54:50:00:01]` à chaque essai ; 3 réponses sur 3 |
| 2 | `ping -c 4 192.0.2.10` | 4 réponses, `ttl=64`, aucune perte |
| 3 | `ip neigh show dev eth0` | `192.0.2.10 lladdr 02:4e:54:50:00:01 REACHABLE` (ou `STALE`) |
| 4 | `ndisc6 2001:db8:0:1::50:1 eth0` | une annonce de voisin avec `Target link-layer address: 02:4E:54:50:00:01` |
| 5 | `ip -6 neigh show dev eth0` | entrée `lladdr 02:4e:54:50:00:01` |
| 6 | `ping -6 -c 4 2001:db8:0:1::50:1` | 4 réponses, aucune perte |
| 7 | `ping -6 -c 4 -I eth0 fe80::4e:54ff:fe50:1` | 4 réponses |
| 8 | `ntpdate -q 192.0.2.10` | une ligne `server 192.0.2.10, stratum 16, offset ..., delay ...` puis un avis « no server suitable » : normal, le serveur est non synchronisé |
| 9 | `ntpdate -q 2001:db8:0:1::50:1` | idem en IPv6 (la prise en charge de l'adresse IPv6 littérale dépend de la version de `ntpdate`) |
| 10 | `sntp 192.0.2.10` (suite ntp ; `ntpdig` pour ntpsec) | réponse reçue, serveur signalé non synchronisé / stratum 16 |

Pendant les tests 8 à 10, capturez : `sudo tcpdump -n -vv -i eth0 'udp port 123'`. Attendu pour la réponse :
mode Server, version égale à celle de la requête (3 ou 4), `Leap indicator: clock unsynchronized` (LI 3),
`Stratum 16`, `poll` recopié de la requête, `precision -24`, réponse de 48 octets, source = l'adresse à laquelle la requête
a été envoyée. L'« offset » affiché par `ntpdate` n'a pas de sens : l'horloge de la carte est libre, elle démarre à
0 (1900-01-01) à la mise sous tension.

À savoir :

- La carte **ne répond qu'aux requêtes de mode 3**. `ntpq -p 192.0.2.10` ou `ntpq -c rv 192.0.2.10` (mode 6)
  restent sans réponse : c'est normal, ce n'est pas une panne.
- Pas de réponse à un ping vers l'adresse de diffusion, ni aux trames non conformes (ARP à destination d'une autre
  adresse, etc.) : le modèle de référence `tools/golden/` énumère les règles ([../tools/golden/README.md](../tools/golden/README.md)).
- **Limiteur de débit** : 8 réponses par seconde et par source (`RATE_LIMIT_PER_S`). Au-delà, **silence** (le KoD
  est désactivé). Pour le vérifier : `for i in $(seq 20); do sntp -t 1 192.0.2.10; done` doit montrer des
  réponses puis des délais d'attente dans la même seconde (comportement de la simulation ; NON MESURÉ sur carte).

## 5. Mesure de la précision

### 5.1 Ce qui est attendu, d'après la simulation

Erreur de l'horodatage matériel par rapport à l'instant SFD **sur les broches MII**, à 25 MHz réels, huit
phases ([status.md](status.md), `rtl/tests/top.rs`) :

| Horodatage | Simulation (ArtyBoard) | Remarque |
|---|---|---|
| T2 (réception) | 0 à +5 ns | par rapport au SFD sur le MII |
| T3 (émission du serveur) | +5 à +10 ns | idem ; la fenêtre vaut -10 à +10 ns sur cinq phases seulement |
| T1 (émission du client) et T3 du serveur, carte avec client | -25 à +10 ns | `rtl/src/top_client.rs`, mêmes huit phases |

Ces chiffres **excluent la latence propre du PHY** (DP83848J, fixe, non mesurée) et supposent un chemin
d'émission libre : une trame qui attend derrière une autre (réponse serveur derrière une requête du client, ARP,
NS) part plus tard que son T3 ne l'annonce (environ 9 µs par trame de 90 octets à 100 Mb/s). Le budget
documenté (`rx_path.rs`) est de ±10 ns pour le croisement d'horloge plus 40 ns de granularité du MII, donc
|erreur| ≤ 50 ns, ce qui justifie le champ *precision* −24 (≈ 60 ns). Rien de cela n'a été mesuré sur carte.

### 5.2 Constantes à ajuster

| Constante | Fichier | Rôle | Valeur actuelle |
|---|---|---|---|
| `RX_STAMP_LATENCY_PS` (donne `RX_STAMP_BIAS`) | `rtl/src/rx_path.rs` | retranchée de l'horloge pour estimer l'instant du SFD sur le MII : `stamp = (time >> 32) - RX_STAMP_BIAS` | 115 ns (`2 * 40 ns + 3,5 * 10 ns`), soit 494 unités de 2^-32 s |
| `TX_LATENCY_PS_V4`, `TX_LATENCY_PS_V6` (donnent `TX_LATENCY_V4/V6`) | `rtl/src/ntp_reply_route.rs` | constante K ajoutée au temps échantillonné pour T3 du serveur (par la route) et pour T1 du client (`CLIENT_TX_LATENCY_*`) | 1 885 000 ps (IPv4) ; IPv6 = IPv4 + 200 000 ps |
| `SERVER_ARB_ADJ` | `rtl/src/top_client.rs` | +10 ns ajoutés au K du serveur dans la carte avec client | 10 ns |
| `TX_STAMP_BIAS_NS` | `rtl/src/tx_path.rs` | latence interne du chemin d'horodatage d'émission de `tx_path` | 60 ns |

Remarques :

- `TX_LATENCY_V4/V6` de `ntp_server.rs` sont des valeurs par défaut du banc d'unité : **la carte utilise celles de
  `ntp_reply_route.rs`**.
- `TX_STAMP_BIAS` n'agit que sur les horodatages produits par `tx_path`, qui sont dans les deux cartes envoyés vers une
  unité d'évacuation (`Drain`) : aucun effet sur NTP aujourd'hui. Un test (`rtl/tests/tx_path.rs`) le fige à 60 ns.
- Ces constantes sont dans le RTL : toute modification impose de régénérer le Verilog et de **refaire le
  bitstream** (et de repasser la revue de timing). Les tests de calibration comparent le résultat à l'instant SFD sur
  les broches simulées, sans PHY : si vous ajoutez la latence du PHY, ils verront un écart égal à cette latence et
  devront être adaptés. Ce changement n'a pas été essayé.

### 5.3 Ce que l'on peut et ne peut pas mesurer avec NTP (analyse, à confirmer)

Soit `L_rx` la latence du PHY de la broche RJ45 au MII en réception, `L_tx` celle du MII à la broche en émission.
Sans compensation, T2 est pris trop tard de `L_rx` (erreur +`L_rx`) et T3, pris à l'instant MII, trop tôt de `L_tx`
(erreur −`L_tx`). Pour un client C et un serveur S, l'erreur de décalage mesuré est `(a2 - a1 + a3 - a4) / 2`, où
`a1..a4` sont les erreurs de T1..T4. Conséquences :

- Un serveur de carte interrogé par un hôte dont les horodatages sont exacts : erreur d'offset
  `(L_rx - L_tx) / 2`, erreur de délai `L_rx + L_tx`. Les deux latences étant de même ordre dans le DP83848J, l'effet sur
  l'offset est a priori petit, et le **délai mesuré** est le meilleur indicateur de leur somme.
- Une carte à la fois client de l'hôte et serveur pour lui s'asservit de façon que l'hôte voie un offset nul,
  **quelle que soit** la compensation : NTP ne voit pas une asymétrie constante. Mesurer l'offset d'une carte
  synchronisée par l'hôte ne permet donc pas de régler `RX_STAMP_BIAS` ni `TX_LATENCY_*`.
- Entre deux cartes identiques (même PHY, mêmes constantes), la part commune des latences s'annule : la mesure
  carte à carte ne montre que les écarts non communs (calibration IPv4/IPv6, phase du croisement d'horloge,
  gigue).

La détermination de `L_rx` et `L_tx` doit donc venir d'ailleurs : les valeurs de la datasheet (SNLS250E, à lire :
non reproduites ici), ou une mesure physique (oscilloscope ou analyseur logique sur les signaux MII du PHY face au signal
de ligne), ou une référence absolue (sortie PPS : **absente** du design actuel). Cette limite est une conclusion
d'analyse, non une mesure.

### 5.4 Procédure

1. **Hôte** : `ethtool -T eth0` doit lister `hardware-transmit`, `hardware-receive` et `hardware-raw-clock`, avec
   un filtre de réception qui couvre le NTP (par exemple `all` ou `ntp-all`). Sans horodatage matériel, les mesures sont
   limitées par la pile logicielle de l'hôte (de l'ordre de la dizaine de µs), donc inutilisables pour juger des 50 ns.
2. **Chrony** (hôte), fichier de configuration minimal :

```
server 192.0.2.10 minpoll 0 maxpoll 0
hwtimestamp eth0
```

   (`minpoll 0` est une requête par seconde : sous le seuil de 8 par seconde de la carte. N'activez pas le mode
   entrelacé `xleave` : le serveur ne le gère pas.) Voir la documentation de chrony pour `rxfilter`.
   Suivez : `chronyc ntpdata 192.0.2.10`, `chronyc sources -v`, `chronyc sourcestats -v`. Avec `ArtyBoard`
   (strate 16) chrony n'utilise pas le serveur : lisez le **délai** et sa gigue sur `ntpdata` ou sur une capture,
   pas l'offset.
3. **Capture** à horodatage d'interface : `sudo tcpdump -n -i eth0 --time-stamp-precision=nano -j adapter_unsynced -w mesure.pcap 'udp port 123'`
   (options à vérifier sur votre version de tcpdump et de libpcap). T1 et T4 viennent de la capture (côté hôte),
   T2 et T3 des champs *Receive* et *Transmit* de la réponse ; `offset = ((T2-T1)+(T3-T4))/2`, `delay = (T4-T1)-(T3-T2)`.
4. **Mesure carte à carte** : A = carte de référence (`ArtyBoardWithClient`, synchronisée sur un amont de stratum
   1, p. ex. l'hôte avec `local stratum 1`), B = carte sous test (`ArtyBoardWithClient` dont `UPSTREAM[0]` est
   l'adresse de A : un jeu de configuration distinct par carte, [config-guide.md](config-guide.md)). B annonce alors
   le stratum 3. Un hôte tiers interroge A et B avec horodatage matériel ; la différence des offsets est l'écart
   A-B. Avec deux cartes identiques, voir 5.3 pour ce que cette mesure ne montre pas.
5. **Mesure de la gigue de délai** sur plusieurs milliers de requêtes (une par seconde au plus, ou une rafale
   espacée) : l'écart-type du délai et des offsets est une borne de la gigue des horodatages (croisements d'horloge
   et phase).
6. **Itération sur les constantes** : (a) partez des valeurs actuelles et relevez délai moyen, gigue, offset ; (b)
   corrigez `RX_STAMP_LATENCY_PS` d'une valeur `+L_rx` et `TX_LATENCY_PS_V4` d'une valeur `+L_tx` (signes : T3 doit désigner le départ sur la ligne, donc plus tard que
   l'instant MII, d'où `+L_tx` dans K ; T2 doit désigner l'arrivée sur la ligne, donc plus tôt que l'instant MII, d'où
   `+L_rx` dans la latence retranchée) ; (c) régénérez, refaites le bitstream ;
   (d) mesurez à nouveau : le délai mesuré doit baisser d'environ `L_rx + L_tx`, l'offset d'une carte serveur changer de
   `(L_rx - L_tx)/2`. Notez chaque itération dans le tableau.
7. **Précision annoncée** : ne baissez pas `PRECISION_LOG2` (−24) sans mesure ; si les mesures montrent une
   erreur supérieure à 50 ns, relevez-la au contraire.

## 6. Stratum 2 (carte `ArtyBoardWithClient`)

### Préparation de l'amont

La configuration par défaut interroge `192.0.2.1` et `2001:db8::1` ; il faut des **serveurs de stratum 1
réels** à ces adresses, ou changer la configuration. Pour un essai en laboratoire, l'hôte lui-même peut servir : avec
chrony, `local stratum 1` et `allow 192.0.2.0/24` dans `chrony.conf` (l'heure n'est alors pas une référence
réelle : c'est un essai de fonctionnement, pas de précision). Autorisez UDP 123 dans le pare-feu de l'hôte. L'amont IPv6 par défaut (`2001:db8::1`) n'est pas l'adresse de
l'hôte de la section 0 (`2001:db8:0:1::1`) : l'amont IPv4 suffit pour un premier essai, ou adaptez la configuration.
Ne pointez pas la carte vers un serveur public sans son accord : le client n'applique **aucun recul** sur KoD
(il compte seulement la requête comme manquée) et interroge toutes les 2^`POLL_LOG2` secondes après sa
salve initiale.

**Boucle de référence** : n'utilisez pas l'hôte qui sert d'amont comme client de la carte (la carte annonce
l'identifiant de l'amont comme Reference ID, et un client NTP rejette une source dont la référence est lui-même). Pour
observer la carte, utilisez un hôte tiers, ou des interrogations qui ne règlent pas l'horloge (`ntpdate -q`,
`chronyc ntpdata`).

### Séquence attendue (simulation, `rtl/tests/top_client.rs` et en-tête de `ntp_client.rs`)

Capture : `sudo tcpdump -n -e -i eth0 'arp or icmp6 or udp port 123'`. Après le démarrage du PHY (section 3) :

1. Une requête **ARP** (`who-has 192.0.2.1 tell 192.0.2.10`) pour l'amont IPv4 ; pour l'amont IPv6, une
   sollicitation de voisin (NS) vers le groupe solicited-node de la cible (ou de la passerelle `fe80::1` si l'amont est
   hors du /64 : le cas de `Board`, voir [config-guide.md](config-guide.md)).
2. Après la réponse ARP/NA, une requête NTP `192.0.2.10.123 > 192.0.2.1.123`, 48 octets, version 4 mode 3,
   `poll` = `POLL_LOG2`, précision −24. Le créneau d'amont *i* démarre à `t0 + i/4 s` : la requête IPv6 suit
   d'un quart de seconde.
3. **Salve initiale (iburst)** : quatre requêtes par amont à 0, 2, 4 et 6 s de son démarrage, puis une requête
   toutes les 64 s (`POLL_LOG2 = 6`), mesurée depuis le début de l'interrogation précédente du même créneau.
4. Une requête sans réponse expire 1 à 2 s après T1 (comptée comme manquée).
5. Après quatre réponses d'un amont, la simulation montre le **premier pas** du servo (l'horloge saute de
   0 à l'heure de l'amont, quel que soit l'écart), puis le serveur répond en **stratum amont + 1, LI 0**.

Interrogez ensuite la carte depuis un hôte tiers :

```sh
ntpdate -q 192.0.2.10
sudo tcpdump -n -vv -i eth0 'udp port 123'
```

Attendu : `stratum 2` pour un amont de stratum 1, `Leap indicator: (0)`, *Reference ID* = l'adresse IPv4 de
l'amont (`192.0.2.1`) ou, pour un amont IPv6, quatre octets de son MD5 (affichés comme une adresse IPv4
quelconque), *Reference Timestamp* récent, *Root Delay* et *Root Dispersion* faibles, mais **croissants** tant qu'aucune
mise à jour ne vient (environ 15 µs de dispersion par seconde d'âge).

Avec un client NTP configuré sur la carte (sur un hôte tiers) :

- chrony : `chronyc ntpdata 192.0.2.10` doit afficher `Stratum : 2`, un `Leap status : Normal`, un `Reference ID`
  égal à celui de l'amont ; `chronyc sources -v` liste la carte avec la colonne `Stratum` à 2.
- ntpd : `ntpq -p` puis `ntpq -c "rv <associd>"` sur l'hôte client : `stratum=2`, `leap=00`, `refid`, `rootdelay`, `rootdisp`.

**Durée de la sélection par le client de l'hôte** : un pair n'est retenu que lorsque sa dispersion est assez
faible. Les entrées vides d'un filtre à huit échantillons comptent comme dispersion maximale (16 s) ; la
dispersion ne passe sous 1 s qu'après environ **cinq échantillons utilisables**. Avec une requête par seconde cela prend
quelques secondes, avec un poll de 64 s plusieurs minutes. Chrony et ntpd peuvent afficher la carte comme
non sélectionnée (`^?`, ou `x`/`-`) pendant ce temps : ce n'est pas une panne. Le délai de la première
synchronisation de la carte elle-même (salve de quatre requêtes espacées de 2 s, soit environ 6 à 8 s) vient de la
simulation ; **non mesuré**. Les premières minutes de stabilisation du servo (états FREQ puis SYNC, quelques mises à
jour à 64 s) sont de l'ordre de quatre à cinq minutes d'après l'en-tête du servo : NON VÉRIFIÉ.

### Modes de défaillance

| Observation | Causes probables |
|---|---|
| Requêtes ARP/NS répétées, aucune requête NTP | passerelle ou amont muet à l'ARP/NS (jusqu'à 8 essais, 15,6 à 31,3 ms d'écart, par interrogation) ; adresse hors masque mais passerelle fausse |
| Requêtes NTP sans réponse | amont injoignable, pare-feu, adresse d'amont erronée |
| Réponses reçues mais la carte reste en stratum 16, LI 3 | la réponse est **rejetée** : stratum 0 (KoD), 16 ou LI 3 de l'amont (amont non synchronisé) ; adresse source de la réponse différente de l'adresse configurée (amont multi-adresses : le client n'accepte que l'adresse exacte et le port 123) ; mode ou version invalides ; champ *Origin* différent de la requête (anti-usurpation) |
| Amont KoD (`RATE`, `DENY`) | la carte compte la requête comme manquée et continue au rythme normal : **réduisez la charge**, ne l'ignorez pas |
| Pair non sélectionné | distance de racine ≥ 1,5 s (amont lointain ou imprécis) ; moins de quatre échantillons utiles |
| Stratum 2 annoncé puis retour à 16 | plus d'échantillon valide depuis 1024 s (`SYNC_TIMEOUT_S`) ou plus de mise à jour récente du servo |
| Après synchronisation, écart > 1000 s | le servo passe en PANIC et ne réajuste qu'en glissement ; **d'après l'en-tête du servo, il n'y a de saut qu'à partir de l'état NSET** : reprogrammez la carte pour la recaler d'un coup (NON VÉRIFIÉ sur carte) |

Les erreurs de configuration les plus fréquentes sont dans [config-guide.md](config-guide.md#8-pièges).
La carte ne garde pas l'heure : chaque reprogrammation ou reset redémarre à 0 et repasse par la salve initiale.

## 7. Sécurité

**N'exposez pas la carte à Internet avant la revue de sécurité** (`docs/security.md`, **à venir**, T5.5). Points
déjà connus, non revus :

- **Réflexion** : le serveur répond à l'adresse source de la requête, qui peut être falsifiée. L'effet est borné :
  réponse de 48 octets, jamais plus grande que la requête (au moins 48 octets), et limiteur de débit par source.
- **Limiteur** : 8 réponses par seconde et par source par défaut, par IPv4 ou par préfixe IPv6 /64, dans une table à 64
  entrées (des collisions limitent plus tôt, jamais plus tard) ; plafond global de 64 × la limite par seconde.
  `RATE_LIMIT_PER_S = 0` le désactive. Les requêtes limitées restent sans réponse (KoD désactivé).
- Pas d'authentification (NTS, MAC), pas de mode 6/7 (pas de `monlist`), pas d'extensions en sortie.
- Ne mettez pas le client de la carte sur un serveur public sans autorisation (voir section 6).
- Placez la carte sur un réseau de laboratoire pour les essais de ce document.

## 8. Tableau de résultats à remplir

Une ligne par mesure ou par itération de constantes. Conservez le commit, le fichier `.bit` et les rapports.

| Date | Commit | Carte (`ArtyBoard` / `WithClient`) | Config | `RX_STAMP_LATENCY_PS` / `TX_LATENCY_PS_V4` | Lien (direct, commutateur) / NIC hôte | WNS / WHS (ns) | Lien monté (oui/non, durée) | Délai moyen (ns) | Gigue du délai (ns) | Offset moyen (ns) | Offset écart-type (ns) | Pertes (%) | Notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|   |   |   |   |   |   |   |   |   |   |   |   |   |   |

Mesures de la mise en service :

| Étape | Résultat attendu | Observé (date, valeur) |
|---|---|---|
| Vivado : WNS / WHS | ≥ 0 / ≥ 0 | |
| `report_cdc` : passages hors `rx_path`/`tx_path`/enveloppe | aucun | |
| `Link detected` après programmation | oui, 100 Mb/s, duplex intégral | |
| `arping`, `ping`, `ping -6`, `ndisc6` | 100 % de réponses | |
| `ntpdate -q` (`ArtyBoard`) | stratum 16, LI 3 | |
| Premier pas du client (`WithClient`) | quelques secondes après la salve initiale | |
| Stratum annoncé après synchronisation | amont + 1, LI 0, Reference ID de l'amont | |
| Erreur T2 / T3 mesurée | à comparer à 0..+5 ns / +5..+10 ns (hors PHY) | |
| `L_rx`, `L_tx` du PHY (datasheet / mesure) | | |
