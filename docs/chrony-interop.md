# Interopérabilité avec chrony (co-simulation, tâche CHRONY)

Le serveur seul (`ArtyCore`, unités MII réelles à 25 MHz, une graduation de simulation = 5 ns) est simulé au cycle
près et interrogé par un **chronyd 4.9 réel, non modifié**, à travers un pont UDP. Ce document décrit la méthode, les
résultats observés (avec les preuves : lignes de log, sorties de `chronyc`) et les constats.

**En bref** : chrony accepte toutes nos réponses (aucune ligne `Invalid`/`Unacceptable`/`Rejected`, `NTP tests :
111 111 1111`), sélectionne le serveur en strate 2 (`^*`, reach 377), lit exactement les champs que le design est
censé envoyer, ne sélectionne pas le serveur en strate 16, accepte NTPv3, le champ d'extension expérimental F323 et
retombe proprement en mode de base quand il demande le mode entrelacé. L'offset mesuré par chrony (−3,486 µs,
σ ≈ 1 ns) s'explique entièrement par la géométrie du banc (−3,490 µs) plus l'erreur propre des horodatages du design
(+4 ns ≈ (T2 + T3)/2, cohérent avec les +0/+10 ns mesurés par `rtl/tests/top.rs`). Le limiteur de débit se comporte
comme spécifié face aux rafales de chrony. Dix constats, aucun défaut du design ; deux portent sur la documentation de mise en service (§6).
**IPv6 n'a pas pu être testé** (pas de sockets IPv6 dans le conteneur, §7).

## 1. Reproduire

```
tools/chrony/build.sh     # télécharge et compile chrony 4.9 dans out/chrony/, le shim, le pont (cargo --release)
tools/chrony/run.sh       # tous les scénarios, puis tools/chrony/analyse.py (code de sortie 1 si un contrôle échoue)
tools/chrony/run.sh fixed offset   # seulement certains scénarios
```

Durées mesurées (4 cœurs partagés) : `build.sh` compile chrony en moins d'une minute (`make -j2`), la compilation
Rust en `--release` de la bibliothèque et du pont prend ≈ 1 min sur un `target/` vide ; `run.sh` complet
**≈ 4 min 45 s** (mesuré : `build.sh && run.sh` sur `out/chrony/` vidé, `target/` déjà compilé), dont ≈ 4 min pour
le seul scénario `noff` (sans saut de cycles inactifs), les autres prenant 1 à 17 s chacun. Résultats dans `out/chrony/results/<scénario>/` (ignoré par
git) et le résumé de l'analyse dans `out/chrony/results/summary.txt`.

Fichiers ajoutés : `rtl/src/bin/chrony_bridge.rs` (le pont), `tools/chrony/vtime_shim.c` (le shim de temps),
`tools/chrony/build.sh`, `tools/chrony/run.sh`, `tools/chrony/analyse.py`. Aucun fichier existant n'est modifié ;
le binaire est découvert par Cargo dans `rtl/src/bin/` (pas de changement de `rtl/Cargo.toml`, pas de dépendance).
Le code téléchargé (chrony) reste sous `out/chrony/` et n'est jamais versionné ; seuls son `configure` et son `make`
sont exécutés, dans son propre répertoire.

## 2. Le pont (`chrony_bridge`)

- Chaque datagramme reçu de chronyd sur 127.0.0.1:`--port` devient une trame Ethernet/IPv4/UDP (sommes de contrôle
  IP et UDP correctes, FCS) d'un client fictif du sous-réseau de la carte (02:00:00:00:00:02, 192.0.2.2, port source =
  le vrai port source de chronyd) vers la carte (adresses de `config::Test` : 02:00:00:00:00:01, 192.0.2.1, port 123),
  envoyée sur `rxd`/`rx_dv` un quartet par front de `clk_rx` (préambule de 7 octets + SFD, écart inter-trames de
  12 octets si des trames se suivent).
- Chaque trame sortant sur `txd`/`tx_en` est décodée (FCS, sommes IPv4 et UDP vérifiées) et sa charge UDP est
  renvoyée à chronyd **depuis la socket du pont**, donc avec l'adresse et le port que chronyd interroge. Le serveur
  répond à l'adresse MAC source de la requête : aucun ARP n'est nécessaire (une requête ARP du design pour le client
  fictif recevrait une réponse ; il n'y en a eu aucune). Toute autre trame est journalisée (`other frames 0` dans
  tous les scénarios).
- Options : `--port`, `--sync fixed|standalone`, `--rate-limit 0|1|2|4|8`, `--duration` (secondes virtuelles),
  `--log` (CSV, une ligne par échange), `--hexdump` (chaque charge utile, avec son instant virtuel), `--no-ff`,
  `--status`, `--max-wall`. Ligne d'état périodique : temps simulé, temps réel, cycles simulés et sautés, requêtes,
  réponses, écart horloge du design − temps vrai (`clk_err`), état d'attente de chronyd.
- **Source de synchronisation** : `BridgeSync`, unité de simulation (pas de netlist) qui sort exactement les valeurs
  de `top::FixedSync` (synchronisé, strate 2, champs de référence « golden ») ou de `top::Standalone` (strate 16,
  LI 3, champs nuls), plus l'entrée de saut d'horloge du « fast-forward » (§3). Les autres unités sont celles de
  `ArtyTopClk<ClkRx25, ClkTx25, Test', BridgeSync, phy_init::Sim>`, `Test'` étant `config::Test` à la constante
  `RATE_LIMIT_PER_S` près (option `--rate-limit`). `kod_enable` reste câblé à 0 dans `ArtyCore` : le pont ne peut
  pas activer le KoD.

## 3. Le couplage des horloges

chrony horodate T1 et T4 avec son horloge système ; le design horodate T2 et T3 avec son `DisciplinedClock`, qui
avance de 10 ns par cycle simulé, environ 800 à 1 000 fois plus lentement que le temps réel (vitesse mesurée :
**95 000 à 137 000 cycles `clk_sys` par seconde de simulation**, en `--release`, selon la charge de la machine).

### Variantes évaluées

| Variante | Principe | Verdict |
|---|---|---|
| (A) chrony au ralenti, facteur fixe | shim LD_PRELOAD : temps de chronyd = temps réel / A | Écartée sous cette forme : la vitesse du simulateur varie de ±20 % (machine partagée), donc un A fixe fait dériver l'horloge virtuelle de chronyd par rapport à celle du design (erreur de fréquence apparente de l'ordre de 10⁵ ppm, offsets dénués de sens) ; et les attentes deviennent A fois plus longues (un sondage de 1 s ≈ 15 min). |
| **(A') temps virtuel piloté par le simulateur** (retenue) | même shim, mais l'horloge de chronyd **est** le temps vrai de la simulation, publié par le pont ; le pont n'avance la simulation que quand chronyd est bloqué dans `select()` | Exacte par construction (pas de facteur à estimer), déterministe, et rapide grâce au saut des cycles inactifs. |
| (B) horloge du design « déformée » (`WarpClock`) | chrony au temps réel, horloge du design A fois plus rapide | Écartée : toutes les durées physiques (trame de 7,5 µs, latence K = 1,885 µs de `ntp_reply_route.rs`, biais du SFD) seraient A fois trop longues en temps du design : avec A ≈ 1 000, biais de l'ordre de (A − 1) × 1,885 µs ≈ 1,9 ms sur T3 et de (A − 1) × 3,5 µs ≈ 3,5 ms sur l'offset (§5.3), plus la gigue de cadencement du pont (ms) : seul un test protocolaire serait possible. |
| (C) pas de couplage | offsets sans signification | Inutile, (A') fonctionne. |

### Mise en œuvre de (A')

- **Temps vrai** : `T0 + 5 ns × graduations + temps sauté`, T0 étant l'heure murale de l'hôte au lancement du pont.
  Le `DisciplinedClock` du design est chargé, avant le premier front, avec l'heure NTP de T0 (même méthode que
  `start_clock` des tests système : accumulateur et registres fantômes d'une horloge déjà en marche), puis il tourne
  librement (`trim` = 0). **L'écart entre chronyd et le design part donc de zéro** ; le pont le vérifie en continu :
  `clk_err` (horloge du design lue au front `clk_sys` − temps vrai à la graduation de livraison) vaut −10,0 à −10,5 ns
  dans tous les scénarios, constant sur 300 s virtuelles (le −10 ns est le décalage d'un cycle entre le registre et
  l'instant de lecture ; la dérive de 0,5 ns sur 300 s est la troncature de `INCR` = ⌊2⁶⁴/10⁸⌋, 5·10⁻¹³).
- **Shim** (`tools/chrony/vtime_shim.c`, chargé par `LD_PRELOAD` dans chronyd seulement) : `clock_gettime` (toutes
  horloges), `gettimeofday`, `time` lisent le temps publié dans une page partagée (`mmap` d'un fichier) ;
  `select`/`pselect`/`poll`/`ppoll`/`nanosleep`/`clock_nanosleep`/`sleep` publient une échéance virtuelle et un numéro
  d'attente, puis interrogent les vrais descripteurs par tranches de 100 µs réelles jusqu'à ce qu'un descripteur soit
  prêt (réponse livrée, requête `chronyc`) ou que le temps virtuel atteigne l'échéance ; `clock_settime` et
  `settimeofday` sont refusés (chronyd tourne en plus avec `-x` : l'horloge de l'hôte n'est jamais touchée).
  `setsockopt(SO_TIMESTAMP, SO_TIMESTAMPNS, SO_TIMESTAMPING)` est refusé : ces horodatages du noyau sont en temps
  mural ; chronyd retombe sur sa propre lecture d'horloge au retour de `select()` (`RX timestamping : Daemon`), qui
  est exactement l'instant de livraison. chrony est en outre compilé avec `--disable-timestamping`.
  `strace` confirme que chronyd n'attend que dans `select()` (boucle de `sched.c`) ; c'est le seul point de blocage.
- **Pont** : avance la simulation seulement quand chronyd attend, jusqu'à l'échéance de l'attente ou jusqu'à la
  livraison d'une réponse, puis s'arrête jusqu'à la prochaine attente. Le calcul de chronyd prend donc un temps
  virtuel nul (hôte infiniment rapide). T1 (lecture de chronyd avant `sendmsg`) est l'instant où le pont prend la
  requête et commence le préambule ; T4 est l'instant où le dernier quartet de la réponse a quitté `txd`.
- **Deux réglages de chronyd imposés par le temps figé** : `clockprecision 0.000000001` (sinon chronyd mesure la
  résolution de son horloge en la lisant en boucle, ce qui ne finit jamais sur une horloge figée) et une attente
  minimale de 1 µs virtuelle par appel bloquant (chronyd arrondit son délai à la µs : un temporisateur à moins de 1 µs
  donne `select()` avec un délai nul, qui ne laisserait jamais le temps avancer ; sur un vrai hôte l'appel système
  dure à peu près autant).
- **Saut des cycles inactifs** (« fast-forward », désactivable par `--no-ff`) : entre deux sondages le design est au
  repos. Quand rien n'a bougé sur les broches depuis 3 000 cycles et que l'échéance est à plus de 2 100 cycles, le
  pont demande à `BridgeSync` un saut d'horloge de exactement J × `INCR` (J multiple de 4 cycles pour garder les
  phases MII) et ajoute J × 10 ns au temps vrai. Ce qui n'est pas simulé pendant un saut : l'état libre qui ne sert
  qu'en présence de trafic (générateur de la fuite du limiteur, scrutation MDIO). Validation : le scénario `noff`
  simule chacun des 3·10⁷ cycles de 0,3 s virtuelle et donne le même résultat que `offset` à 0,1 ns près (§5.3).
- Résultat : une seconde virtuelle de sondage coûte ≈ 5 000 cycles simulés (deux trames + marge de repos) et
  ≈ 20 ms réelles ; 300 s virtuelles avec 138 sondages en ≈ 5 s.

## 4. Scénarios

| Scénario | Pont | `server 127.0.0.1 port 12300 …` | Durée virtuelle |
|---|---|---|---|
| `fixed` | strate 2, limite 8/s | `minpoll 0 maxpoll 2 iburst` | 300 s |
| `offset` | strate 2, limite 8/s | `minpoll 0 maxpoll 0 noselect` (offsets bruts : chronyd ne se discipline pas sur une source `noselect`) | 64 s |
| `standalone` | strate 16, LI 3 | `minpoll 0 maxpoll 2 iburst` | 60 s |
| `ratelimit` | strate 2, limite **1/s** | `minpoll -2 maxpoll -2 iburst` (4 sondages/s) | 60 s |
| `rl-iburst` | strate 2, limite **1/s** | `minpoll 0 maxpoll 2 iburst` | 30 s |
| `burst` | strate 2, limite 8/s | `minpoll -4 maxpoll -4` (16 sondages/s) | 30 s |
| `xleave` | strate 2 | `minpoll 0 maxpoll 0 xleave` | 30 s |
| `extfield` | strate 2 | `minpoll 0 maxpoll 0 extfield F323` | 30 s |
| `version3` / `version2` | strate 2 | `… version 3` / `… version 2` | 30 s / 20 s |
| `noff` | strate 2, **sans saut** | `minpoll -7 maxpoll -7 noselect` | 0,3 s |

Configuration commune : `clockprecision 0.000000001`, `cmdport 0`, socket de commande UNIX dans un répertoire privé
(`mktemp -d`, mode 700), `log measurements statistics tracking rawmeasurements`, `chronyd -x -d -f <conf>`.

## 5. Résultats

Chiffres de l'exécution complète de référence (`tools/chrony/build.sh && tools/chrony/run.sh` sur une copie
propre, 4 min 45 s, « all checks passed ») ; les numéros de port, les dates et les horodatages varient d'une
exécution à l'autre, pas les conclusions.

### 5.1 a. Serveur strate 2 (`fixed`, 300 s virtuelles, 101 sondages)

Aucune ligne `Invalid`/`Unacceptable`/`Rejected` ; le journal de chronyd ne contient que :

```
2026-11-17T12:42:36Z Selected source 127.0.0.1
```

`chronyc sources` (fin de course) :

```
MS Name/IP address         Stratum Poll Reach LastRx Last sample
^* localhost                     2   2   377     4     +0ns[   +0ns] +/-   31ms
```

`chronyc ntpdata`, comparé à ce que le design doit envoyer (`ntp_server.rs`, en-tête ; champs de `FixedSync`,
`top.rs`) :

| Champ (chrony) | Vu par chrony | Attendu du design | |
|---|---|---|---|
| Leap status | Normal | LI = 0 (synchronisé) | ✔ |
| Version | 4 | VN de la requête (4) | ✔ |
| Mode | Server | 4 | ✔ |
| Stratum | 2 | `stratum` = 2 | ✔ |
| Poll interval | 2 (4 seconds) | octet 2 de la requête (N7) : chrony a envoyé 2 | ✔ |
| Precision | -24 (0.000000060 seconds) | `PRECISION_LOG2` = −24 (0xE8) | ✔ |
| Root delay | 0.012497 seconds | 0x0000_0333 / 65536 = 0,0124969 s | ✔ |
| Root dispersion | 0.024994 seconds | 0x0000_0666 / 65536 = 0,0249939 s | ✔ |
| Reference ID | C0000263 () | 192.0.2.99 (`GOLDEN_REF_ID`) | ✔ (affiché en hexadécimal) |
| Reference time | Mon Nov 17 12:00:00 2025 | `GOLDEN_REF_TS` = 0xECC58CC0.00000000 | ✔ (constat C8) |
| Offset | +0.000000000 seconds | — (chronyd a déjà corrigé son horloge virtuelle, voir `tracking`) | |
| Peer delay | 0.000008220 seconds | 8,2 µs : deux trames de 94 octets sur le fil, §5.3 | ✔ |
| Peer dispersion | 0.000000061 seconds | précision 2⁻²⁴ + bruit nul | ✔ |
| Response time | 0.000010020 seconds | T3 − T2 = fin de la requête (7,5 µs après son SFD) + traitement + K | ✔ (constat C10) |
| Jitter asymmetry | +0.00 | | |
| NTP tests | 111 111 1111 | tous les tests passent | ✔ |
| Interleaved | No | mode de base | ✔ |
| Total TX / RX / valid RX / good RX | 101 / 101 / 101 / 101 | aucune perte (8/s ≫ 1/s) | ✔ |

`chronyc tracking` : `Reference ID : 7F000001 (localhost)`, `Stratum : 3`, `System time : 0.000003485 seconds fast
of NTP time`, `Frequency : 0.000 ppm`, `Root delay : 0.012505168 seconds` (0,012497 + 8,2 µs), `Leap status :
Normal`. `chronyc sourcestats` : `localhost 6 4 20 -0.000 0.000 -0ns 1ns` (fréquence et dérive nulles : l'horloge du
design avance exactement au rythme vrai). En plus, `analyse.py` vérifie octet par octet les 101 réponses : 48 octets,
octets 0 à 31 conformes aux règles N4 à N9 (LI/VN/mode, strate, poll copié, précision, délai et dispersion racine,
Reference ID, Reference Timestamp, Origin = Transmit de la requête), T2 ≤ T3.

### 5.2 b. Serveur non synchronisé (`standalone`, strate 16, LI 3)

chrony **ne sélectionne pas** le serveur :

```
^? localhost                     0   2     0     -     +0ns[   +0ns] +/-    0ns
2026-11-17T12:42:4xZ Can't synchronise: no selectable sources (1 unreachable sources)
```

`ntpdata` : `Leap status : Not synchronised`, `Stratum : 16`, `Reference ID : 00000000 ()`, `NTP tests : 111 101
0000`, `Total RX 29`, `Total valid RX 29`, `Total good RX 0`, `Peer delay : 0.000000000`. Le test 6 de chrony
(`leap != non synchronisé`, `ntp_core.c`) échoue ; les réponses sont *valides* (tests 1, 2, 3, 5, 7) mais pas
*bonnes*, chrony ne calcule alors ni offset ni délai (`measurements.log` : `? 16 111 101 0000 … 0.000e+00
0.000e+00 …`) et ne met pas à jour le registre d'accessibilité (reach 0). Les 29 réponses sont conformes octet par
octet (LI 3, strate 16, champs de référence nuls). Voir constats C1 et C2.

### 5.3 c. Offset et délai mesurés (`offset`, `noselect`, 64 sondages d'une seconde ; `noff`)

Avec `noselect` chronyd ne corrige pas son horloge : chaque ligne de `measurements.log` est la mesure brute θ.

```
2026-11-17 12:42:40 127.0.0.1  N  2 111 111 1111  0  0 0.00 -3.496e-06  8.200e-06  9.704e-08  1.250e-02  2.499e-02 C0000263 4B D D
2026-11-17 12:42:41 127.0.0.1  N  2 111 111 1111  0  0 0.00 -3.486e-06  8.220e-06  9.708e-08  1.250e-02  2.499e-02 C0000263 4B D D
```

| Grandeur (64 sondages) | Moyenne | σ |
|---|---|---|
| θ de chrony (offset, convention RFC 5905 : négatif = horloge locale en avance) | **−3 486,2 ns** | **1,2 ns** (min −3 496, max −3 486 : le premier sondage seul diffère, autre phase) |
| δ de chrony (délai) | 8 219,7 ns | 2,5 ns |
| θ_géom = ((SFD_rx − T1) + (SFD_tx − T4)) / 2, vérité du banc | −3 490,2 ns | 1,2 ns |
| **θ − θ_géom** (erreur d'horodatage du design vue par chrony) | **+4,0 ns** | 0,0 ns |
| e2 = T2 − instant SFD reçu / e3 = T3 − instant SFD émis (banc) | −0,17 ns / +9,88 ns | — |

Lecture :

- **La quasi-totalité de l'offset est la géométrie du banc**, pas une erreur du design : chrony prend T1 quand le
  préambule de la requête commence (le SFD, que le design horodate, arrive 15 quartets = 600 ns plus tard, 625 ns en
  moyenne avec l'alignement sur le front de `clk_rx`) et T4 quand le dernier quartet de la réponse a quitté le fil
  (7 605 ns après son SFD : 94 octets × 80 ns = 7 520 ns, plus la détection de fin). θ_géom = (625 − 7 605)/2 ≈
  −3 490 ns ; δ = 625 + 7 605 − (T3 − T2 mesuré sur le design, ≈ 0) ≈ 8 220 ns. Autrement dit : une requête ou une
  réponse NTP IPv4 de 94 octets occupe **8,16 µs** de fil chacune (7,52 µs de trame + 0,64 µs de préambule), et un
  client qui horodate début d'émission et fin de réception voit un offset de −(7,52 − 0,6)/2 ≈ −3,46 µs (constat C9).
- **Le reste, +4,0 ns, est l'erreur d'horodatage du design** : (e2 + e3)/2 = (−0,2 + 9,9)/2 = +4,9 ns, retrouvé par
  chrony à moins de 1 ns (sa conversion NTP → ns tronque ; `analyse.py` vérifie |θ_chrony − θ_banc| ≤ 1,5 ns, mesuré
  0,99 ns, et |δ_chrony − δ_banc| ≤ 0,15 ns). C'est cohérent avec les fenêtres mesurées par `rtl/tests/top.rs` (T2 0
  à +5 ns, T3 +5 à +10 ns) : ici une seule paire de phases MII, donc une valeur constante (σ = 0).
- **Limite de résolution, honnêtement** : ces nanosecondes ne sont visibles que parce que l'horloge virtuelle de
  chronyd a une résolution de 1 ns et aucun bruit (`clockprecision 1e-9`, pas de pile réseau). Sur un vrai hôte,
  chrony annonce une précision de quelques dizaines de ns et ses horodatages logiciels ont une gigue de l'ordre de la
  µs : l'erreur de +5 ns du design y est invisible ; même avec horodatage matériel (8 ns de résolution sur beaucoup
  de cartes) elle est à la limite. `measurements.log` n'imprime que 4 chiffres significatifs (1 ns à l'échelle de
  la µs).
- **Validation du saut de cycles** (`noff`, chaque cycle simulé, 8 sondages à 7,8 ms) : θ − θ_géom = +4,0 ns,
  σ 0,0 ; e2 = −0,14 ns, e3 = +9,87 ns ; `clk_err` −10,2 à −10,0 ns : identique au scénario avec saut. (Les 5
  requêtes sans réponse de `noff` sont le limiteur : 13 requêtes en 0,1 s, limite 8/s, 8 réponses.)
- `chronyc tracking` du scénario `fixed` : `System time : 0.000003485 seconds fast of NTP time` : chronyd estime
  l'écart de son horloge à −θ = 3,485 µs, la même valeur ; fréquence estimée 0,000 ppm.

### 5.4 d. Limiteur de débit contre le rythme de chrony

Le limiteur (`ntp_server.rs`, « Rate limiter ») autorise `RATE_LIMIT_PER_S` réponses par source et par fenêtre d'une
seconde NTP (alignée sur les secondes de l'horloge du design), puis répond quand même avec une probabilité 1/16.
`analyse.py` recompte, à partir des vidages hexadécimaux horodatés, les requêtes par fenêtre et les réponses.

| Scénario | Requêtes | Réponses | Au-delà de la limite | Fuites (1/16) | chronyc sources (fin) | Journal chronyd |
|---|---|---|---|---|---|---|
| `fixed` / `offset` : 8/s, ≤ 1 sondage/s (iburst : 4/s) | 101 / 64 | 101 / 64 | 0 | — | `^*`, 377 | `Selected source` une fois |
| `rl-iburst` : 1/s, `minpoll 0 iburst` | 26 | 20 | 6 | 0 (attendu 0,4) | `^*`, 377, poll 1 | `Selected source` une fois |
| `ratelimit` : 1/s, `minpoll -2` (4/s) | 243 | 73 | 182 | 12 (attendu 11,4 ± 3,3) | `^*`, reach 21, poll −2 | `Can't synchronise … (0 unreachable sources)` puis `Selected source` |
| `burst` : 8/s, `minpoll -4` (16/s) | 473 | 256 | 233 | 16 (attendu 14,6 ± 3,7) | `^*`, reach 300, poll −4 | **14 × `Selected source`, 13 × `Can't synchronise: no selectable sources (1 unreachable sources)`** en 30 s |

Dans chaque fenêtre, tant que la limite n'est pas atteinte, toute requête reçoit sa réponse (contrôle « no window
answers fewer than min(n, limit) »). Les fuites observées (12 + 16 + 0 = 28 pour 421 requêtes au-delà de la limite,
attendu 26 ± 5) sont conformes à 1/16 ; remarque : pendant les sauts de cycles le générateur pseudo-aléatoire de la
fuite n'avance pas, sa séquence n'est donc pas celle du matériel (sa loi reste la même).

Comportement de chrony : avec la limite par défaut (8/s) et `minpoll 0`, **aucune perte** : `iburst` envoie ses
premières requêtes à 0,25 s d'intervalle (4/s, sous la limite), puis une par seconde. Quand des réponses manquent,
chrony **ne ralentit pas** : le poll reste à `minpoll` (`Poll interval -2`, `-4`), seul le registre d'accessibilité
se vide ; chrony ne recule que sur un KoD `RATE` (`ntp_core.c` : `delay_time += 4 * poll` et arrêt de la rafale), que
le design n'envoie jamais (`kod_enable` = 0, câblé dans `ArtyCore` : non testable ici sans modifier le top). Voir
constats C3 et C4.

### 5.5 d2. Paquets réellement envoyés par chrony, versions, extensions, mode entrelacé

Requête typique de chronyd 4.9 (48 octets, vidage du pont) et réponse du design :

```
req  #1 at 0.201125000 from :48195 (48 bytes): 23000020 00000000 00000000 00000000 00000000 00000000 0000000000000000 0000000000000000 d608dadd3eabe18a
rep  #1 at 0.201143220 to :48195 (48 bytes):   240200e8 00000333 00000666 c0000263 ecc58cc000000000 d608dadd3eabe18a eea6ca3c42ead640 eea6ca3c42eb7e5b
```

- Requête : LI 0, **VN 4**, mode 3 (0x23), strate 0, poll 0, précision 0x20 (constante de chrony en mode client),
  tous les autres champs à zéro et un **Transmit aléatoire** (chrony ne publie pas son heure : minimisation des
  données) ; **pas de champ d'extension** par défaut (ni NTS ni MAC). Réponse : 48 octets, 0x24 (LI 0, VN 4, mode 4),
  strate 2, poll copié, précision 0xE8, Origin = ce Transmit aléatoire, T2 < T3 : octet pour octet ce qu'exige
  `ntp_server.rs`.
- `version 3` : requêtes 0x1b (VN 3), réponses 0x1c (VN 3), chrony `Version : 3`, 30/30 acceptées.
- `version 2` : requêtes 0x13 (VN 2), **aucune réponse** (règle N3 : VN 3 ou 4 seulement, `docs/security.md`),
  chrony : `^? … reach 0`.
- `extfield F323` (champ d'extension expérimental de chrony) : requêtes de **76 octets** (champ de type 0xF323,
  longueur 28) ; le design ignore les octets au-delà du 48e et répond **48 octets** ; chrony accepte les 30 réponses
  (`Total good RX` = `Total TX`), il n'utilise le champ que si le serveur le renvoie. Pas d'amplification.
- `xleave` : 29 requêtes sur 30 portent un Origin non nul (requêtes du mode entrelacé) ; le design, en mode de base,
  y répond avec Origin = Transmit de la requête ; chrony le reconnaît (`Interleaved : No`) et utilise l'échange de
  base, 30/30 bonnes réponses, offset identique (constat C5).


## 6. Constats

Aucun défaut du design n'a été trouvé : tous les champs, toutes les règles d'acceptation et le limiteur se
comportent comme spécifié face à chrony. Les constats portent sur la documentation et sur des choix de conception.

**C1 — `docs/bringup.md` donne un conseil faux pour `ArtyBoard` (strate 16).** Le § « Chrony (hôte) » dit :
« Avec `ArtyBoard` (strate 16) chrony n'utilise pas le serveur : lisez le **délai** et sa gigue sur `ntpdata` ».
Or chrony 4.9 ne calcule ni offset ni délai pour une réponse LI 3 / strate 16 : `ntpdata` affiche `Peer delay :
0.000000000 seconds`, `Response time : 0`, et `measurements.log` des zéros (preuve : §5.2 ; source :
`ntp_core.c`, `sample.offset = sample.peer_delay = … = 0.0` quand le test 6 échoue). *Reproduction* :
`tools/chrony/run.sh standalone`. *Correction proposée* (texte) : « Avec `ArtyBoard` (strate 16) chrony ne calcule
rien (`Peer delay` reste à 0) : mesurez le délai avec `ntpdate -q` / `sntp`, ou sur une capture (point 3), ou
utilisez une carte synchronisée (`ArtyBoardWithClient`). »

**C2 — Un serveur non synchronisé apparaît « injoignable » dans chrony, pas « non synchronisé ».** `chronyc
sources` affiche `^?`, strate 0 et reach 0, et le journal dit `(1 unreachable sources)`, alors que toutes les
réponses arrivent (`Total RX` = `Total valid RX` = `Total TX`). Implication pour la mise en service : face à
`ArtyBoard`, ne pas conclure à un problème réseau ; lire `chronyc ntpdata` (`Leap status : Not synchronised`,
`Stratum : 16`, `Total RX`). À ajouter au tableau de diagnostic de `docs/bringup.md`.

**C3 — `iburst` envoie à ¼ du poll minimal.** chronyd 4.9 espace ses requêtes de rafale de 0,25 s avec `minpoll 0`
et de 1/16 s avec `minpoll -2` (intervalles mesurés : 0,250 à 0,254 s ; 0,063 s). Avec la limite par défaut (8/s)
c'est sans conséquence pour tout réglage raisonnable (`minpoll` ≥ −3 avec ou sans `iburst`) et la configuration
conseillée par `docs/bringup.md` (`minpoll 0`) ne perd rien. Avec une limite ≤ 3/s, une partie de la rafale initiale
est perdue (`rl-iburst` : 6 requêtes sur 26, chrony sélectionne quand même la source). Ne pas configurer
`RATE_LIMIT_PER_S` sous 4.

**C4 — Client plus rapide que la limite : rafales puis silence, et chrony ne ralentit pas.** Les fenêtres fixes
d'une seconde donnent, à 16 requêtes/s contre 8/s, huit réponses en début de seconde puis le silence (hors fuite) :
le registre d'accessibilité oscille et chrony alterne `Selected source` / `Can't synchronise: no selectable sources
(1 unreachable sources)` (13 fois en 30 s, reach final 300). Comme le design ne renvoie jamais de KoD
(`kod_enable` = 0), chrony n'a aucun moyen d'apprendre qu'il doit ralentir : son poll reste à `minpoll`. Ce n'est
pas un défaut (le silence est le comportement par défaut de ntpd et le choix documenté du projet), mais une
conséquence à connaître. *Proposition* (non appliquée, hors périmètre) : faire de `kod_enable` une constante de
`ConfigSet` (au lieu du `tie(Bit::Zero)` de `ArtyCore`), pour pouvoir activer le KoD `RATE` (48 octets, jamais plus
gros que la requête, donc sans amplification) : chrony recule alors de 4 × poll et arrête sa rafale
(`ntp_core.c`). Un limiteur à fenêtre glissante ou à seau percé étalerait aussi les pertes.

**C5 — Mode entrelacé : repli propre.** chrony `xleave` envoie des requêtes entrelacées (Origin non nul) ; le
serveur, en mode de base, renvoie toujours Origin = Transmit de la requête ; chrony détecte que le serveur ne gère
pas le mode (`Interleaved : No`) et mesure normalement. Comme T3 est déjà une prédiction de l'instant SFD (latence
K), le mode entrelacé n'apporterait presque rien au design. Conforme à `docs/bringup.md` (« N'activez pas le mode
entrelacé »), qui peut être nuancé : l'activer ne casse rien.

**C6 — Champs d'extension : acceptés et ignorés, réponse de 48 octets.** Vérifié avec le champ expérimental F323
de chrony (requêtes de 76 octets) : réponse de 48 octets acceptée par chrony. Aucune amplification.

**C7 — La requête de chrony ne porte aucune heure.** Transmit aléatoire, précision 0x20 constante, autres champs
nuls. Le serveur n'en a pas besoin (il copie le Transmit dans l'Origin, ce que chrony vérifie). Conséquence pour le
limiteur : la seule entropie liée au client que le design peut mêler à son générateur reste l'instant d'arrivée
(ce que fait déjà `ntp_server.rs`).

**C8 — Champs de référence de la simulation.** `FixedSync` annonce le Reference Timestamp « golden » du 17 novembre
2025 12:00:00, vieux d'un an, et une dispersion racine fixe de 25 ms (distance racine affichée ± 31 ms) : chrony ne
teste ni l'âge de la référence ni ne s'en plaint, il l'affiche tel quel. Sur la carte, `ClientSync`/`SyncState`
fournissent une référence et une dispersion vivantes (non couvert ici). Le Reference ID d'un serveur de strate ≥ 2
(adresse IPv4 192.0.2.99) est affiché par chrony en hexadécimal, `C0000263 ()`, ce qui est normal.

**C9 — L'offset vu par un client à horodatage logiciel contient l'asymétrie de longueur de trame.** Un client qui
horodate le début de son émission et la fin de sa réception voit ici −3,49 µs (§5.3) alors que le design est juste à
5 ns près ; chronyd, s'il disciplinait réellement l'horloge, la décalerait de 3,485 µs (`System time : 0.000003485
seconds fast`). Ce n'est pas une erreur du serveur : c'est l'écart entre la référence SFD (celle du design, d'IEEE
1588 et de l'horodatage matériel) et des horodatages pris en début/fin de trame. Sur la carte, il faut donc des
horodatages matériels côté hôte (`hwtimestamp`, et la correction de longueur de trame de chrony) avant d'interpréter
un offset de quelques µs ; avec des horodatages logiciels, seule la stabilité est significative.

**C10 — Temps de réponse de 10,02 µs.** T3 − T2 (SFD à SFD) = 7,52 µs de réception complète de la requête (le
serveur attend la fin de la trame, FCS et somme UDP, avant de répondre) + ≈ 2,5 µs de traitement et d'émission.
chrony l'affiche (`Response time`) sans conséquence.


## 7. Ce qui n'est pas testé

- **IPv6** : le noyau du conteneur n'a pas de sockets IPv6 (`socket(AF_INET6)` échoue, errno 97), chrony est
  compilé avec `--disable-ipv6`. L'interopérabilité IPv6 avec chrony reste **non testée** ; le pont ne fabrique que
  des trames IPv4 (le chemin IPv6 du design est couvert par `rtl/tests/top.rs` et les vecteurs « golden »).
- Une seule paire de phases MII (`ClkRx25`/`ClkTx25`, fronts aux graduations 1 et 3) : T2 et T3 ont donc une erreur
  constante ici (≈ 0 et ≈ +10 ns) ; le balayage des phases est fait par `rtl/tests/top.rs` (`ClkRxP`/`ClkTxP`).
- Les latences du PHY (DP83848J) et de la pile de l'hôte : le banc représente un client qui lit son horloge à
  l'instant exact où la trame commence (T1) et finit (T4) sur le fil ; chronyd n'a ici que ses horodatages
  « daemon » (pas d'horodatage noyau ni matériel, `hwtimestamp`, ni des compensations `rxcomp`/`txcomp`).
- Le KoD `RATE` (`kod_enable` câblé à 0 dans `ArtyCore`, non modifiable sans toucher au top).
- La carte avec client (`ArtyBoardWithClient`, horloge réellement disciplinée, `ref_ts` et dispersion vivants) : le
  pont utilise le top serveur seul, avec les champs de référence fixes de `FixedSync`.
- NTS / authentification (non pris en charge par le design ; chrony compilé sans).
