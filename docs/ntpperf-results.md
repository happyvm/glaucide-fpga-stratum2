# Test de charge avec ntpperf (tâche NTPPERF)

Le serveur seul (le cœur de `ArtyBoard` : `ArtyCore` avec `mii_rx`/`mii_tx` réels aux horloges MII de 25 MHz, une
graduation de simulation = 5 ns) est simulé au cycle près et chargé par **ntpperf** (Miroslav Lichvar, commit
`aee5446`, **non modifié**), l'outil de mesure de performance des serveurs NTP : il simule des milliers de clients
NTP de sources différentes, envoie des trames Ethernet brutes à débit croissant via libpcap et imprime, par débit,
la fraction de réponses perdues/invalides et le temps de réponse T3 − T2 du serveur ou l'écart entre son T3 et
l'horodatage de réception local. ntpperf tourne entièrement dans le **temps virtuel** de la simulation, sur une
**libpcap de remplacement** (notre code) dont l'« interface » est le pont de simulation.

**En bref** — RESUME

## 1. Reproduire

```
tools/ntpperf/build.sh     # libpcap de remplacement, shim de temps, clone + make de ntpperf, pont (cargo --release)
tools/ntpperf/run.sh       # 12 tâches, 4 en parallèle, puis tools/ntpperf/analyse.py (code 1 si un contrôle échoue)
tools/ntpperf/run.sh c-10k d-kod-1k     # seulement certaines tâches
tools/ntpperf/run.sh --long             # + balayages complets de ntpperf, limiteur à 30 k et 80 req/s, validation du saut
```

DUREES

Fichiers ajoutés : `rtl/src/bin/ntpperf_bridge.rs` (le pont), `tools/ntpperf/pcap_shim/` (`pcap.h`, `pcap/pcap.h`,
`pcap.c` : la libpcap de remplacement ; `vtime.c` : le shim de temps ; `simlink.h` : le contrat avec le pont),
`tools/ntpperf/build.sh`, `run.sh`, `analyse.py`, ce document. Aucun fichier existant n'est modifié (le binaire est
découvert par Cargo dans `rtl/src/bin/`, pas de dépendance). ntpperf (GPL-2.0-or-later) est cloné dans
`out/ntpperf/src/` (ignoré par git), compilé par **son propre Makefile sans aucune modification** — seules les
variables de make sont remplacées : `make CPPFLAGS="-D_GNU_SOURCE -I tools/ntpperf/pcap_shim" LDFLAGS="-L
out/ntpperf/pcap -lpcap -lm"` (son Makefile lie par `$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)`) — et n'est jamais versionné ;
aucun correctif n'est nécessaire. La vraie libpcap n'est pas installée (ni `flex` pour la construire) : notre
`libpcap.a` (Apache-2.0) n'implémente que l'API qu'utilise ntpperf.

## 2. Méthode

### 2.1 Ce que fait ntpperf

ntpperf est fait de deux processus : le **processus de mesure** (`perf.c`) calcule pour chaque client simulé
(`rate / 10` clients, au plus 16 384, adresses source `réseau ^ index` dans le réseau `-s`) l'instant d'émission de
ses requêtes (une toutes les 0,1 s par client), les écrit 25 ms à l'avance dans un tube vers le **processus
d'émission** (`sender.c`, créé par `fork`), puis lit les réponses par `pcap_next_ex` non bloquant ; l'émetteur attend
l'instant de chaque requête **en tournant** sur `clock_gettime(CLOCK_MONOTONIC)` puis fabrique la trame (MAC source =
celle de l'interface, port source aléatoire, Transmit aléatoire) et l'envoie par `sendmmsg` sur `pcap_fileno()`. Par
débit : 0,2 s d'échauffement (les deux premières requêtes de chaque client ne comptent pas), `-t` secondes de mesure
(minimum 0,2), 0,1 s d'attente des dernières réponses ; une réponse est valide si elle est en mode 4 depuis le port
123 et que son Origin vaut le Transmit de la requête du client. Sans `-o`, ntpperf imprime T3 − T2 (temps de
réponse) ; avec `-o C` (C ≠ 0), l'écart (horodatage de réception local − T3 − C) ; `-H` demande des horodatages
« adaptateur » (`PCAP_TSTAMP_ADAPTER_UNSYNCED`). ntpperf ne connaît qu'IPv4.

### 2.2 La libpcap de remplacement et l'interface simulée

`pcap.c` : `pcap_create`, les accesseurs `pcap_set_*` (enregistrent snaplen, type et précision d'horodatage),
`pcap_activate` (se connecte au pont : descripteur hérité `NTPPERF_SIM_FD`, une extrémité d'une paire de sockets
`AF_UNIX SOCK_SEQPACKET` créée par le pont, et lui envoie un message `HELO` de 12 octets avec le type d'horodatage
demandé), `pcap_set_datalink`, `pcap_setdirection`, `pcap_setnonblock`, `pcap_fileno`, `pcap_next_ex` (un message du
pont = un en-tête {horodatage virtuel en ns, longueur} + la trame sans FCS ; précision ns ou µs), `pcap_inject`,
`pcap_sendpacket`, `pcap_statustostr`, `pcap_geterr`, `pcap_close`. Les constantes sont celles de libpcap. L'adresse
MAC de l'interface (`ioctl(SIOCGIFHWADDR)` sur `sim0`) est fournie par le shim de temps : 02:00:00:00:00:02.

### 2.3 Le couplage des temps : ntpperf tourne en boucle, le temps virtuel n'avance que quand il attend

Le problème : ntpperf ne se bloque jamais, il tourne sur `clock_gettime`. Le modèle retenu reprend celui de
`chrony_bridge` (le pont possède le temps ; le client calcule en un temps virtuel nul, comme sur un hôte infiniment
rapide) en remplaçant « chronyd est bloqué dans `select()` » par « chaque processus de ntpperf est **prêt** » :

- **Créneaux** : chaque processus de ntpperf a un créneau dans la page partagée (`simlink.h`), alloué paresseusement
  au premier appel intercepté ; `fork()` alloue celui de l'enfant **dans le parent, avant** de forker (état
  « calcule »), si bien que le temps ne peut pas avancer pendant le démarrage de l'émetteur. Un destructeur marque
  la sortie ; le pont détecte aussi un processus mort sans prévenir (`/proc/<pid>/stat`).
- **Une lecture d'horloge qui ne peut rien changer devient une attente** (`vtime.c`, `maybe_wait`) :
  - l'**émetteur** : son échéance est connue **exactement** : c'est l'instant `when` de la prochaine requête non
    encore envoyée. Le shim lit les `struct sender_request` (40 octets, `sender.h` ; un `timespec` monotone en tête)
    au moment où l'émetteur les lit dans son tube, et les retire quand `sendmmsg` les déclare envoyées. Une lecture
    d'horloge avec une requête déjà due rend la main tout de suite ; sinon le processus attend `when`. Chaque
    requête part donc **exactement** à l'instant virtuel prévu par ntpperf (`requests.csv` : à 1 000 req/s,
    `send_ns` = 0, 1 000 000, 2 000 000… ns) ;
  - tout autre processus (le **processus de mesure**) : une lecture d'horloge est une attente quand rien d'utile
    ne s'est passé depuis la précédente (pas de trame reçue, pas de requête écrite dans le tube) ; il attend alors
    `vt + 1 ms` (quantum, `--quantum-ns`). C'est sans effet sur les mesures : il planifie 25 ms à l'avance et lit des
    trames qui portent leur propre horodatage virtuel ; tant qu'il travaille (planification par paquets de 102
    requêtes, lecture des réponses), il n'attend pas.
- **Tubes** : les descripteurs créés par `pipe`/`pipe2` sont suivis ; les octets écrits sont « en vol » jusqu'à leur
  lecture. Un lecteur bloqué n'est prêt que si rien n'est en vol ; un écrivain bloqué (tube plein : à plus de
  ≈ 65 000 req/s les 25 ms d'avance ne tiennent pas dans les 1 632 requêtes du tube) est prêt, puisque seul le
  lecteur peut le débloquer et qu'il attend le temps.
- **Avance du temps** (pont) : quand tous les créneaux sont prêts — un état figé : aucun processus ne calcule, donc
  aucun ne peut en réveiller un autre — le pont vide la socket (trames émises jusque-là, estampillées du temps
  courant, qui est leur instant d'émission puisque le temps n'avançait pas), simule le design jusqu'à la **plus
  proche échéance**, livre les trames émises par le design, publie le nouveau temps et réveille les processus
  (futex). Les changements d'état sont suivis d'un incrément d'un compteur (futex sur lequel le pont dort) ; le pont
  lit le compteur, le nombre d'octets en vol, puis les états, puis le compteur à nouveau, et n'accepte qu'un
  instantané sans changement. L'ordre des écritures du shim (un lecteur quitte l'état « bloqué » avant de décrémenter
  les octets en vol ; un écrivain les incrémente avant d'écrire) garantit qu'un instantané accepté « tout le monde
  est prêt » l'est réellement.
- **Horloges** : `CLOCK_REALTIME`/`TAI` = temps virtuel (époque Unix) ; les autres (`MONOTONIC`…) = temps virtuel −
  T0 + 1 s ; `gettimeofday`, `time` suivent (ntpperf initialise son générateur aléatoire par `time()` : comme T0 est
  fixe, **les exécutions sont reproductibles à l'octet près**).

Coût : un réveil par requête (l'émetteur) et un par milliseconde virtuelle (le processus de mesure), quelques µs
réelles chacun, négligeable devant la simulation (≈ 7 µs réelles par cycle de 10 ns).

### 2.4 Le pont Ethernet (`ntpperf_bridge`)

- **Fil et file d'attente** : chaque trame de ntpperf (90 octets) reçoit sa FCS et entre dans un modèle de la file
  d'émission de la carte réseau / du port de commutateur côté serveur : une FIFO d'au plus `--queue-bytes` octets
  de trames (FCS comprise ; **64 Kio** par défaut, soit 697 requêtes, 5,2 ms de fil), **rejet en queue** quand elle
  est pleine, servie au débit de la ligne : préambule + SFD, trame, écart inter-trames de 12 octets, un quartet par
  front de `clk_rx` (25 MHz). Une requête occupe 8 + 94 + 12 = 114 octets de fil = 9,12 µs : **débit de ligne
  L = 10⁸ / 912 = 109 649 requêtes/s**. Choix : c'est ce que fait un commutateur (ou la file `qdisc` + anneau de la
  carte) devant un lien à 100 Mb/s ; la taille n'influe que sur le délai d'attente (au plus 5,2 ms, invisible dans
  les grandeurs mesurées : T3 − T2 et T3 contre le SFD de la réponse) et sur la perte transitoire au début d'une
  surcharge.
- **Retour** : chaque trame sur `txd`/`tx_en` est vérifiée (FCS, sommes IPv4/UDP) et rendue à ntpperf sans FCS avec
  un horodatage virtuel : avec `-H` (`PCAP_TSTAMP_ADAPTER*`) **l'instant de son SFD sur les broches TX** (comme
  `RealBench` de `rtl/tests/top.rs` et `chrony_bridge`), sinon l'instant où son dernier quartet a quitté les broches.
  Pas de PHY ni de câble : l'écart mesuré avec `-o -H` est directement l'erreur de T3 du design (au signe près).
- **Configuration** : `config::Test` sauf l'adresse IPv4 192.168.123.1/24 (ntpperf refuse les réseaux non privés,
  et 192.0.2.1 de `Test` en est un) ; MAC 02:00:00:00:00:01, `RATE_SECRET` de `Test`. `--rate-limit N` (0 = sans
  limite, 1, 2, 4, 8, 16), `--kod` (`KOD_ENABLE`, avec 1 ou 8), `--sync fixed` (= `FixedSync`, strate 2, champs de
  référence « golden » ; défaut) ou `standalone`. Clients : réseau `-s 172.18.0.0/16`, MAC 02:00:00:00:00:02 ; le
  serveur répond à la MAC source de la requête (aucun ARP).
- **Saut des cycles inactifs** (comme `chrony_bridge`) : quand rien n'a bougé sur les broches depuis
  `--idle-cycles` = **400** cycles et qu'aucune trame n'est due, le pont saute jusqu'à la prochaine trame ou
  échéance par un pas d'horloge exact du design (J × `INCR`, J multiple de 4). Le silence le plus long observé entre
  la fin d'une requête et le SFD de sa réponse est de 253 cycles (le seuil de 1 000 cycles d'abord utilisé coûtait
  25 % de cycles en plus à faible débit). **Validation** (`run.sh --long l-noff l-ff`, 1 000 req/s, 0,5 s
  virtuelle) : sans saut (50,1 M cycles simulés, 384 s) et avec saut (0,89 M cycles, 6 s), le journal par requête
  `requests.csv` (instants d'émission, de SFD, T2 et T3 bruts de chacune des 401 réponses) est **identique octet pour
  octet**, ainsi que la table de ntpperf. Ce qui n'est pas simulé pendant un saut : le générateur de la fuite du
  limiteur (sa loi est inchangée) et la scrutation MDIO.
- **Sorties** : la table de ntpperf (stdout), un résumé (stderr : requêtes, rejets de la file, réponses, KoD,
  statistiques exactes de T3 − T2, T2 − SFD reçu, T3 − SFD émis, horloge du design − temps vrai) et `requests.csv`
  (une ligne par requête : instants d'émission, d'entrée sur le fil, de SFD, source, port, case du limiteur
  `rl_index`, réponse, T2/T3 bruts, SFD émis, erreurs).

### 2.5 Vitesse de simulation et coût d'un point de mesure

VITESSE

## 3. Résultats

RESULTATS

## 4. Constats

CONSTATS

## 5. Ce qui n'est pas couvert

NONCOUVERT
