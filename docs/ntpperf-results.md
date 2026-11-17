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

Mesuré (release, 3 à 4 ponts en parallèle sur 4 cœurs) : **121 000 à 140 000 cycles `clk_sys` par seconde réelle**
par pont (≈ 7,5 µs réelles par cycle de 10 ns : ≈ 750 fois plus lent que le matériel), la coordination avec
ntpperf coûtant moins de 1 % (a-105k : 40,0 M cycles simulés en 316 s de simulation pour 317 s au total ; 42 118
avances du temps pour 40 001 requêtes). Une requête isolée coûte ≈ 2 225 cycles simulés (préambule et requête :
816 cycles sur `rxd`, 253 cycles jusqu'au SFD de la réponse, 760 de réponse, 400 de seuil d'inactivité), le reste
est sauté. Au-delà de ≈ 45 000 req/s le fil n'est jamais au repos : un point de ntpperf (0,5 s virtuelle avec son
échauffement, dont les 0,1 s finales d'attente sont sautées) coûte alors **4·10⁷ cycles ≈ 5 à 5,5 min**, quel
que soit le débit. D'où le choix de l'intervalle d'échantillonnage minimal de ntpperf (`-t 0.2`), d'un point par
processus pour les débits élevés (exécutés en parallèle) et d'un balayage de ntpperf lui-même (`-r 1000-11390 -x
1.5`, 7 points en une exécution) pour les faibles débits. Le balayage complet par défaut de ntpperf (`-r
1000-200000 -x 1.5`, 15 points) prend ≈ LAWALL en un seul processus : il est dans `--long`.

## 3. Résultats

Chiffres de l'exécution de référence (`tools/ntpperf/build.sh && tools/ntpperf/run.sh`, puis `run.sh --long`),
« all checks passed ». Les exécutions sont déterministes (T0 fixe, graine de ntpperf tirée de l'heure virtuelle) :
a-105k et b-105k simulent exactement le même nombre de cycles (40 000 175). Résumé complet produit par
`analyse.py` : `out/ntpperf/results/summary.md`.

### 3.1 a. Débit brut sans limiteur (`--rate-limit 0`, `FixedSync` strate 2)

Commandes (`-t 0.2` : 0,2 s virtuelle de mesure par débit ; réseau `NET` = `-i sim0 -m 02:00:00:00:00:01 -d
192.168.123.1 -s 172.18.0.0/16`) :

```
ntpperf_bridge --shim out/ntpperf/ntpperf_vtime.so --shm F --rate-limit 0 --log requests.csv -- out/ntpperf/src/ntpperf NET -B -t 0.2 -r 1000-11390 -x 1.5
ntpperf_bridge ... --rate-limit 0 ... -- ntpperf NET -B -t 0.2 -r 25000      (puis 60000, 105000, 125000)
```

Tables brutes de ntpperf (cinq exécutions mises bout à bout) :

```
               |          responses            |        response time (ns)
rate   clients |  lost invalid   basic  xleave |    min    mean     max stddev
1000       100   0.00%   0.00% 100.00%   0.00%   +10020  +10020  +10020      0
1500       150   0.00%   0.00% 100.00%   0.00%   +10020  +10020  +10020      0
2250       225   0.00%   0.00% 100.00%   0.00%   +10020  +10020  +10020      0
3375       337   0.00%   0.00% 100.00%   0.00%   +10020  +10020  +10020      0
5062       506   0.00%   0.00% 100.00%   0.00%   +10020  +10020  +10020      0
7593       759   0.00%   0.00% 100.00%   0.00%   +10020  +10020  +10020      0
11389     1138   0.00%   0.00% 100.00%   0.00%   +10020  +10020  +10020      0
25000     2500   0.00%   0.00% 100.00%   0.00%   +10020  +10020  +10020      0
60000     6000   0.00%   0.00% 100.00%   0.00%   +10020  +10020  +10020      0
105000   10500   0.00%   0.00% 100.00%   0.00%   +10020  +10020  +10020      0
125000   12500  12.28%   0.00%  87.72%   0.00%   +10020  +10020  +10020      0
```

Balayage complet de ntpperf (`--long`, tâche `la-full`, un seul processus, `-B -t 0.2 -r 1000-200000 -x 1.5`) :

```
LA_TABLE
```

Interprétation :

- **Aucune perte jusqu'au débit de ligne** : 0,00 % perdu, 0,00 % invalide jusqu'à 105 000 req/s (96 % de L) ;
  au-delà, la perte est **exactement celle de la file d'émission** : 1 − L/R = 12,28 % à 125 000 req/s (3 070
  requêtes mesurées rejetées par la file, aucune requête parvenue au design sans réponse), 15,66 % mesuré à
  130 000 req/s pour 15,65 % attendus (essai préliminaire). Le design répond à **toutes** les requêtes qui
  arrivent sur `rxd`, trames dos à dos avec l'écart minimal de 12 octets compris : **c'est le lien à 100 Mb/s qui
  limite, pas le design**. Plus haut débit sans perte : le débit de ligne, 109 649 req/s (le point de ntpperf
  immédiatement inférieur, 86 497 ou 105 000, est sans perte ; le suivant, 125 000 ou 129 746, perd 1 − L/R).
- **Marge du design** : le serveur consomme n + 50 = 98 cycles par requête de 48 octets (`ntp_server.rs`,
  « Throughput », test `throughput_back_to_back`) pour 912 cycles disponibles entre deux requêtes au débit de ligne
  (9,12 µs) : ≈ 11 % d'occupation, marge ≈ 9,3. Côté émission la réponse a exactement la taille de la requête
  (90 octets) : le lien TX est saturé exactement quand le lien RX l'est, et `mii_tx` tient le débit de ligne
  (sinon le temps de réponse croîtrait et la FIFO de 2 048 octets de `rx_path` finirait par déborder : rien de tel).
- **Temps de réponse** (T3 − T2 calculé par ntpperf) : **+10 020 ns, σ 0, à tous les débits**, surcharge comprise ;
  la vérité du banc donne 10 019,9 à 10 020,1 ns (σ 0,11 ns, 40 001 réponses à 105 000 req/s). T3 − T2 = SFD à SFD :
  7,52 µs de réception de la fin de la requête (94 octets), puis traitement et K = 1,885 µs de latence d'émission
  prédite (même valeur que chronyd, constat C10 de `docs/chrony-interop.md`). **Pas de file d'attente dans le
  serveur** : avec des requêtes sérialisées par le fil (au moins 9,12 µs d'écart) et des réponses de même taille,
  chaque réponse trouve `udp4_tx` et `tx_path` vides ; le temps de réponse ne dépend pas de la charge.

### 3.2 b. Exactitude de l'horodatage d'émission sous charge (`-o 1e-12 -H`)

`-o` n'active l'affichage de l'écart que pour une correction non nulle : 10⁻¹² s ne change aucun chiffre. `-H` :
l'horodatage de réception local d'une réponse est l'instant virtuel de son SFD sur les broches TX (pas de PHY ni de
câble). ntpperf affiche donc **SFD émis − T3**, soit l'opposé de l'erreur de T3.

```
               |          responses            |     TX timestamp offset (ns)
rate   clients |  lost invalid   basic  xleave |    min    mean     max stddev
1000       100   0.00%   0.00% 100.00%   0.00%       -9      -9      -9      0
1500       150   0.00%   0.00% 100.00%   0.00%       -9      -9      -9      0
2250       225   0.00%   0.00% 100.00%   0.00%       -9      -9      -9      0
3375       337   0.00%   0.00% 100.00%   0.00%       -9      -9      -9      0
105000   10500   0.00%   0.00% 100.00%   0.00%       -9      -9      -9      0
125000   12500  12.28%   0.00%  87.72%   0.00%       -9      -9      -9      0
```

Balayage complet (`--long`, `lb-full`, `-B -t 0.2 -o 1e-12 -H -r 1000-200000 -x 1.5`) :

```
LB_TABLE
```

| Débit (req/s) | ntpperf min / moyenne / max / σ (ns) | vérité du banc SFD − T3 : min / moyenne / max / σ (ns) |
|---|---|---|
| 1 000 à 3 375 | −9 / −9 / −9 / 0 | −10,0 / −9,9 / −9,8 / 0,1 |
| 105 000 | −9 / −9 / −9 / 0 | −10,0 / −9,9 / −9,8 / 0,1 |
| 125 000 (surcharge) | −9 / −9 / −9 / 0 | −10,0 / −9,9 / −9,8 / 0,1 |

- **L'erreur de T3 est de +9,88 ns (σ 0,12 ns, min +9,8, max +10,0) contre le SFD émis, indépendante de la
  charge**, de 1 000 req/s à la saturation du lien : aucune attente dans `tx_path` ni dans l'arbitre (voir 3.1).
  C'est exactement la valeur des tests de `rtl/tests/top.rs` (T3 +5 à +10 ns selon la phase MII) et de
  `chrony_bridge` (e3 = +9,88 ns), avec la latence d'émission calibrée K de `ntp_reply_route.rs`
  (TX_CYCLES_V4 = 108 cycles + 790 ns de `tx_path`). Une seule paire de phases MII est simulée (`ClkRx25`/`ClkTx25`
  synchrones de `clk_sys`), d'où σ ≈ 0.
- ntpperf affiche −9 et non −9,88 : il convertit T3 en ns en **tronquant** (`convert_ntp_ts`), ce qui ajoute
  jusqu'à 1 ns à l'écart ; l'écart entre horloge du design et temps vrai (−10,1 ns, un cycle) est déjà inclus dans
  la vérité du banc (`clk_err`, stable de −10,2 à −10,0 ns).
- Sur une vraie carte s'ajoutent les latences TX du PHY et la résolution de l'horodatage de la carte réseau de
  l'hôte ntpperf ; cette co-simulation mesure l'erreur propre du design, à 0,1 ns près.

### 3.3 c. Limiteur par défaut (`RATE_LIMIT_PER_S` = 8, fuite 1/16, silence)

```
ntpperf_bridge ... --rate-limit 8 ... -- ntpperf NET -B -t 1.0 -r 1000     (puis 10000 ; --long : 30000, 80000)
```

`-t 1.0` : chaque client émet 12 requêtes en 1,2 s (une toutes les 0,1 s), les 2 premières d'échauffement ; T0 est
une seconde entière, les fenêtres d'une seconde du limiteur commencent donc avec la mesure. **Formule** pour une
source seule dans sa case qui envoie n requêtes dans une fenêtre : réponses = min(n, 8) + (n − 8)⁺/16 ; avec
ntpperf n = 10 par seconde (`rate / 10` clients à 10 req/s chacun), d'où **perte = (10 − 8)/10 × (1 − 1/16) =
18,75 %** (dans la fenêtre 1 : requêtes 1 à 10, dont 2 d'échauffement ; fenêtre 2 : requêtes 11-12 toutes
servies ; sur les 10 requêtes comptées, 2 sont au-delà de la limite, servies avec probabilité 1/16). `analyse.py`
recalcule l'espérance exacte à partir des instants, des fenêtres et des **cases réelles du design** (`rl_index`
de `ntp_server.rs`, journalisée par le pont et recalculée en Python : identiques).

```
               |          responses            |        response time (ns)
rate   clients |  lost invalid   basic  xleave |    min    mean     max stddev
1000       100  18.58%   0.00%  81.42%   0.00%   +10020  +10020  +10020      0
10000     1000  18.95%   0.00%  81.05%   0.00%   +10020  +10020  +10020      0
(--long)
30000     3000  56.49%   0.00%  43.51%   0.00%   +10020  +10020  +10020      0
80000     8000  76.63%   0.00%  23.37%   0.00%   +10020  +10020  +10020      0
```

| Débit | Sources | Cases occupées | Sources partageant une case | Perte ntpperf | Espérance exacte (hachage du design) | Une case par source | Hachage aléatoire uniforme |
|---|---|---|---|---|---|---|---|
| 1 000 | 100 | 100 | 0 | 18,58 % | 18,73 % ± 0,34 | 18,73 % | — |
| 10 000 | 1 000 | 1 000 | 0 | 18,95 % | 18,75 % ± 0,11 | 18,75 % | 27,3 % |
| 30 000 (`--long`) | 3 000 | 1 498 | 2 818 | **56,49 %** | 56,30 % ± 0,11 | 18,75 % | 40,7 % |
| 80 000 (`--long`) | 8 000 | 1 826 | 7 818 | **76,63 %** | 76,63 % ± 0,08 | (18,75 %, impossible : 8 000 > 4 096) | 60,8 % |

- **La perte mesurée par ntpperf est celle que prévoit le modèle du limiteur, à tous les débits** : la formule
  18,75 % tant que chaque source a sa case (100 et 1 000 sources), et l'espérance exacte calculée avec les cases
  réelles quand des sources les partagent (30 000 et 80 000 req/s : 56,49 % pour 56,30 ± 0,11 % ; 76,63 % pour
  76,63 ± 0,08 %). Contrôle direct dans `requests.csv` : **toutes** les requêtes sous la limite de leur case et de
  leur fenêtre ont une réponse (29 000 sur 29 000 pour c-1k, c-10k, lc-30k), et au-delà la fuite vaut 1 187 / 20 217
  = 5,87 % ± 0,17 pour 6,25 % attendus (−2,2 σ ; le générateur seul, rejoué en Python sur 4·10⁶ cycles, donne
  6,247 %, −0,25 σ : pas de biais propre du générateur ; l'écart vient vraisemblablement de l'échantillonnage
  périodique, à confirmer sur une plus longue série — sans conséquence, la fuite n'est qu'une atténuation).
- À 80 000 req/s, 8 000 sources ne peuvent pas avoir chacune leur case (4 096) : le limiteur plafonne alors le
  volume total servi à ≈ 4 096 × 8 + N/16 réponses/s ; avec les cases réelles (1 826 occupées) le plafond tombe à
  18 700 réponses/s.
- **Les sources consécutives de ntpperf ne se répartissent pas comme des clés aléatoires** : le hachage
  multiplicatif `(m·k + b) >> 52` sur des clés consécutives est une suite de Weyl (k·m/2⁶⁴ mod 1) dont la
  répartition dépend du développement en fraction continue de m/2⁶⁴ (`RATE_SECRET`). Jusqu'à 1 000 sources : aucune
  collision (mieux que le hasard, qui en aurait 23 %) ; au-delà, avec le secret de `Test`, les sources s'entassent
  dans peu de cases (tableau ci-dessous) : voir le constat **N1**.

| N sources consécutives (172.18.0.0 + i) | cases distinctes, secret `Test` | secret `Board` | fonction aléatoire (espérance) |
|---|---|---|---|
| 100 | 100 | 100 | 99 |
| 1 000 | 1 000 | 929 | 887 |
| 2 500 | 1 461 | 2 103 | 1 871 |
| 3 000 | 1 498 | 2 495 | 2 127 |
| 4 096 | 1 566 | 3 351 | 2 589 |
| 8 000 | 1 826 | 4 096 | 3 515 |
| 16 384 | 2 374 | 4 096 | 4 021 |

(ntpperf plafonne à 16 384 clients, `MAX_CLIENTS` : la taille du réseau `-s` au-delà ne change rien ; un /16 en
donne 65 536 adresses mais ntpperf n'en utilise que `min(rate/10, 16 384)`.)

### 3.4 d. KoD activé (`--rate-limit 8 --kod`)

```
ntpperf_bridge ... --rate-limit 8 --kod ... -- ntpperf NET -B -t 1.0 -r 1000
               |          responses            |        response time (ns)
rate   clients |  lost invalid   basic  xleave |    min    mean     max stddev
1000       100   0.00%   0.00% 100.00%   0.00%       +0   +8178  +10020   3884
ntpperf_bridge ... --rate-limit 8 --kod ... -- ntpperf NET -B -t 1.0 -o 1e-12 -H -r 1000
               |          responses            |     TX timestamp offset (ns)
rate   clients |  lost invalid   basic  xleave |    min    mean     max stddev
1000       100   0.00%   0.00% 100.00%   0.00%  -237268110785876256 +339910485709810496 +3955530061237428224 887712134519796864
ntpperf_bridge ... --rate-limit 8 --kod ... -- ntpperf NET -B -t 1.0 -r 10000      (--long)
10000     1000   0.00%   0.00% 100.00%   0.00%       +0   +8133  +10020   3917
```

- **ntpperf compte un KoD comme une réponse valide en mode de base**, ni perdue ni « invalid » : sa validation
  (`process_response`) ne regarde que le mode (4), le port source (123) et l'Origin (= Transmit de la requête),
  jamais la strate ni le Reference ID. 184 des 1 201 réponses sont des KoD `RATE` (strate 0) ; parmi les réponses
  comptées, 18,38 % de KoD pour 18,73 % attendus (requêtes au-delà de la limite moins les fuites, qui reçoivent une
  réponse normale).
- Le KoD du design porte Receive = Transmit = Transmit de la requête (comme ntpd) : ntpperf calcule pour lui
  T3 − T2 = 0, d'où le minimum +0 et la moyenne abaissée à 8 178 ns = 10 020 × (1 − 0,1838) ; avec `-o`, il
  soustrait de son horodatage local un « T3 » qui est son propre Transmit aléatoire : écarts de l'ordre de 10¹⁷ ns,
  dénués de sens. Le design se comporte comme spécifié (`ntp_server.rs`, « KoD ») ; c'est ntpperf qui ne filtre
  pas les KoD (constat N3).
- Avec le KoD, **chaque requête valide reçoit une réponse** de même taille : le limiteur ne réduit plus le volume
  réfléchi (déjà noté par `docs/security.md`, recommandation R4).

### 3.5 Requêtes en mode entrelacé (`-I`)

```
ntpperf_bridge ... --rate-limit 0 ... -- ntpperf NET -I -t 0.2 -r 1000
               |          responses            |        response time (ns)
rate   clients |  lost invalid   basic  xleave |    min    mean     max stddev
1000       100   0.00%   0.00% 100.00%   0.00%
```

Le design ne connaît que le mode de base : il répond à chaque requête « entrelacée » de ntpperf (Receive ≠ 0) par
une réponse de base valide (Origin = Transmit de la requête), comptée `basic` ; ntpperf n'imprime d'écart que pour
les réponses entrelacées, d'où les colonnes vides. Comportement attendu, identique à celui observé avec chrony
`xleave` (`docs/chrony-interop.md`).

### 3.6 e. IPv6

**Non couvert par ntpperf** : il ne fabrique que des requêtes IPv4 (`sender.c` : en-tête IPv4 codé en dur, option
`-s` IPv4 seulement) ; le chemin IPv6 du design (`udp6_rx`, `udp6_tx`, limiteur indexé par /64) n'est pas exercé
par ce test.

## 4. Constats

Aucun défaut de débit, de latence ni d'horodatage : pas de perte imputable au design, pas de croissance de la
latence, pas de dérive de T3, aucune trame erronée (FCS, sommes IPv4/UDP vérifiées sur chaque trame émise), aucun
blocage, aucune réponse sans requête. Un constat porte sur le limiteur (N1), les autres sont des observations.

**N1 — Le hachage du limiteur répartit mal des sources consécutives pour certains secrets (basse à moyenne).**
`rl_index` = bits de poids fort de `m·k + b` (multiplication-décalage) : universel *en moyenne sur m* (deux clés
données collisionnent avec une probabilité ≤ 2/4096), mais pour un `RATE_SECRET` **fixé**, des clés en progression
arithmétique (un bloc d'adresses consécutives : les clients de ntpperf, un réseau de campus derrière un pool de NAT,
un /20 de centre de données) forment une suite de Weyl dont la répartition dépend de la fraction continue de
m/2⁶⁴. Reproduction : `run.sh --long lc-30k lc-80k` (tableau 3.3), ou en Python (`analyse.py`, `rl_index`) :
- secret de `Test` : 3 000 sources consécutives n'occupent que 1 498 cases (2 818 partagent une case ; une fonction
  aléatoire en occuperait 2 127), 16 384 sources 2 374 cases ; mesuré à 30 000 req/s : **56,49 % de perte au lieu des 18,75 %** d'une case par source (40,7 % avec une fonction
  aléatoire) ; à 80 000 req/s, 76,63 % au lieu de 60,8 % ;
- sur 300 secrets tirés au hasard, 4 096 sources consécutives occupent de **120** à 4 045 cases (10 % des secrets :
  moins de 1 572 ; médiane 3 012 ; fonction aléatoire : 2 589). Le secret de `Board` est bon (3 351 ; 16 384
  sources sur les 4 096 cases).
Effet : des clients légitimes d'un même bloc partagent la limite de 8 réponses/s de leur case bien plus souvent que
ne l'annonce la borne 2/4096 ; dans le pire des cas observé (120 cases), 4 096 clients consécutifs n'ont droit
ensemble qu'à 960 réponses/s (+ 1/16 du reste). Un attaquant n'en tire rien de plus (il ne connaît pas m), et
jusqu'à ≈ 1 000 sources consécutives la suite de Weyl fait au contraire mieux que le hasard (0 collision).
Correctifs proposés (non appliqués, `rtl/src` hors du périmètre de cette tâche) : (a) sans matériel : un contrôle
de compilation dans `config::Checks` qui refuse un `RATE_SECRET` dont 4 096 clés consécutives occupent moins de,
par exemple, 2 500 cases (celui de `Test` serait refusé, celui de `Board` accepté) — ne couvre que le pas 1 ;
(b) robuste : un second tour de mélange `x = m·k + b ; x ^= x >> 32 ; case = (m·x) >> 52`, qui rend la
répartition indépendante du secret et de la structure des clés (2 523 à 2 643 cases pour 4 096 clés consécutives
sur les 300 secrets, comme une fonction aléatoire) ; coût : un second produit 64 × 64, à faire en 16 cycles (4 bits
par cycle) ou sur DSP48 pour rester avant le cycle de décision (le premier tour finit au cycle 32, la décision a
lieu au plus tôt au cycle 49).

**N2 — T3 ne dépend pas de la charge parce que requêtes et réponses ont la même taille (observation).** La
latence d'émission K est calibrée sur un chemin TX libre ; une réponse qui attendrait dans `tx_path` derrière une
autre trame aurait un T3 trop tôt de ce temps d'attente (limite documentée de K, `ntp_server.rs`). Avec ntpperf ce
cas est inatteignable : les requêtes de 90 octets arrivent au plus au débit de ligne et chaque réponse de 90 octets
part avant que la suivante soit prête (vérifié : erreur de T3 +9,88 ns, σ 0,12, à 105 000 et 125 000 req/s). Il le
deviendrait avec un trafic mêlé (écho ICMP de grande taille, ARP, NA émis juste avant une réponse NTP) : non testé
ici.

**N3 — ntpperf compte les KoD comme des réponses valides (outil, pas design).** Strate 0 non vérifiée : avec
`--kod`, 0 % perdu et 0 % « invalid », temps de réponse minimal 0 et écarts `-o` absurdes (10¹⁷ ns) ; pour mesurer
un serveur dont le KoD est activé, lire la proportion de KoD dans le journal du pont (`requests.csv`, `stratum` = 0).

**N4 — Le débit maximal sans perte est le débit de la ligne à 100 Mb/s (observation).** 109 649 requêtes NTP IPv4
par seconde, soit environ 9 fois moins que ce que le cœur traiterait (98 cycles par requête). Le PHY de l'Arty
(10/100) est donc le goulot ; un serveur sur un lien gigabit (exemple de ntpperf : pas de perte jusqu'à
≈ 200 000 req/s) n'est pas comparable.

**N5 — Méthode : deux pièges de ntpperf en temps virtuel, contournés sans le modifier.** `-o 0` désactive l'affichage
de l'écart (il faut une correction non nulle : 1e-12) ; et l'intervalle `-t` minimal est 0,2 s, ce qui fixe le
coût d'un point à 0,5 s virtuelle (5 minutes de simulation au-delà de 45 000 req/s).

## 5. Ce qui n'est pas couvert

- **IPv6** (ntpperf est IPv4 seulement), PTP (`-D`, `-N`), NTS (`-S`, ntpperf compilé sans gnutls ; le design ne fait pas NTS).
- La carte avec client (`ArtyBoardWithClient`, cœur `ArtyCoreClient`) : seul le serveur seul (`ArtyCore`, celui
  d'`ArtyBoard`) est chargé ; sa synchronisation est `FixedSync` (strate 2), pas le servo.
- Plusieurs processus d'émission (`-p`), la distribution exponentielle des départs (`-e`) : les requêtes restent
  sérialisées par le fil, le design ne verrait pas de rafale plus dense qu'au débit de ligne.
- Le trafic mêlé (ICMP, ARP, IPv6 en même temps que la charge NTP), qui seul pourrait faire attendre une réponse
  dans `tx_path` (N2).
- Les latences du PHY et du câble, la gigue de l'oscillateur, d'autres phases MII (une seule paire simulée, comme
  dans `chrony_bridge`), le limiteur sur une durée longue (fenêtres glissant sur 65 536 s) et des clients au-delà
  de 16 384 (plafond de ntpperf).
- Pendant les cycles sautés, le générateur de la fuite n'avance pas : la séquence des fuites n'est pas celle du
  matériel, leur loi (1/16) l'est (vérifiée en 3.3).
