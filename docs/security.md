# Revue de sécurité (T5.5)

Serveur NTP stratum 2 sur Arty A7-100T (`ArtyBoard` : serveur seul ; `ArtyBoardWithClient` : serveur + client NTP). Revue faite sur le RTL et les tests de la branche à la date de T5.5, **sans modifier `rtl/src/`** : chaque problème qui demande un changement de code est un **constat** (F1 à F11) avec une correction proposée précise, à appliquer par l'orchestrateur. Les propriétés qui tiennent sont **démontrées par des tests** (section « Tests de démonstration »), pas seulement affirmées.

## Résumé

Ce qui tient (démontré par test) :
- **Aucune amplification** : une réponse NTP fait toujours 48 octets de charge, pour une requête d'au moins 48 octets ; sur le fil, la trame émise n'est jamais plus longue que la trame reçue (facteur ≤ 1, exactement 1 pour une requête de 48 octets). Le KoD (désactivé sur la carte) fait aussi 48 octets.
- **Aucune réponse aux modes 6 et 7** (`ntpq`, `ntpdc`, `monlist`, le vecteur d'amplification de 2014, facteur jusqu'à ≈ 557 selon l'alerte US-CERT TA14-013A), ni aux modes 0, 1, 2, 4, 5, ni aux versions autres que 3 et 4, ni aux paquets de moins de 48 octets. Pas de boucle NTP↔NTP (un mode 4 n'est jamais répondu).
- **Parseurs robustes** : ≈ 2 000 trames aléatoires, tronquées, corrompues, surdimensionnées, malformées (options IPv4, fragments, chaînes d'en-têtes d'extension IPv6, 802.1Q, trames géantes, FCS faux, `rx_er`), en grande partie au débit ligne : aucun blocage, aucune trame émise qui ne soit une réponse légitime, volume émis ≤ volume reçu.
- **Réponses amont usurpées** : toute réponse qui n'a pas exactement l'Origin de 64 bits, l'adresse de l'amont, le port 123, le mode 4, VN 3/4, 48 octets et des sommes de contrôle bonnes est ignorée sans effet ; une réponse rejouée est rejetée.

Ce qui ne tient pas (constats principaux) :
- **F1 (haute)** — le limiteur de débit n'a que 64 cases de 8 réponses/s : capacité totale 512 réponses/s, et ≈ 576 requêtes/s usurpées (≈ 0,5 Mb/s) font taire le serveur pour **tous** les clients.
- **F3 (haute)** — l'Origin (T1) de nos requêtes est notre lecture d'horloge, prévisible à ≈ 7–8 bits près pour quiconque interroge notre serveur : un attaquant hors chemin peut forger des réponses amont acceptées.
- **F4 (haute)** — au démarrage, **une seule source** (la première à avoir 4 échantillons) décide d'un pas d'horloge **de taille quelconque**.
- **F5 (moyenne)** — empoisonnement ARP/NDP de la passerelle par n'importe quelle machine du lien (réponses non sollicitées apprises, pas de vieillissement).

## Périmètre et méthode

- Lu : `docs/architecture.md`, `docs/status.md`, `docs/formal.md`, `docs/references.md`, les en-têtes (et le code au besoin) de `ntp_server`, `ntp_client`, `clock_filter`, `selection`, `servo`, `sync_state`, `mii_rx`, `rx_path`, `fcs_check`, `eth_rx`, `ipv4_rx`, `ipv6_rx`, `udp4_rx`, `udp6_rx`, `ntp_demux`, `arp`, `icmp_echo`, `icmpv6`, `nbr_cache`, `tx_arb`, `tx_path`, `config`, `top.rs`, `top_client.rs`.
- Références protocole : RFC 5905 (§7.4 KoD, §8–9 contrôles de paquets, §11 sélection et discipline), RFC 8633 (BCP NTP : limitation de débit, KoD, amplification, modes 6/7, usurpation), RFC 9109 (ports source aléatoires côté client), *draft-ietf-ntp-data-minimization* (horodatage d'émission client aléatoire), RFC 826 (ARP), RFC 4861 §7.2.5 et §11 (NDP, absence de SEND/RFC 3971), RFC 1122 §3.2.1.3 et RFC 4291 (adresses source invalides), RFC 8915 (NTS, hors périmètre).
- Hors périmètre : preuves formelles (aucun `check!`/`assume!` ajouté), attaques physiques (JTAG, alimentation, horloges), canaux auxiliaires, timing Vivado.

## Modèle de menace

### Attaquants

| Qui | Capacités | Exemples |
|---|---|---|
| **Hors chemin** (Internet) | Envoie des paquets IP avec n'importe quelle adresse source ; **ne voit pas** notre trafic ; peut interroger notre serveur comme tout client | réflexion, déni par usurpation, réponses amont forgées à l'aveugle |
| **Sur le chemin** vers un amont ou un client | Lit, retarde, supprime, modifie, injecte | fausse heure, attaque par délai, KoD forgé |
| **Sur le lien** (même segment Ethernet) | Tout ce qui précède pour le lien, plus ARP/NDP, adresses MAC arbitraires, débit ligne | empoisonnement de cache, inondation de NS |
| Client légitime abusif | Requêtes valides en grand nombre | épuisement du limiteur |

### Actifs

| Actif | Ce qu'on protège |
|---|---|
| **A1** disponibilité du service | répondre à tous les clients légitimes |
| **A2** exactitude de notre horloge | elle est propagée à nos clients (stratum 2) |
| **A3** le reste d'Internet | ne pas servir de réflecteur ni d'amplificateur |
| A4 l'équipement | pas de code exécuté, pas d'état persistant, pas d'interface d'administration : un reset efface tout ; la configuration est fixée à la synthèse |

### Surface atteignable depuis le réseau

| Trafic accepté (`top.rs`, `top_client.rs`) | Réponse possible |
|---|---|
| ARP, destination notre MAC ou diffusion | réponse ARP (A1-A5) ; apprentissage du cache (`arp.rs`) |
| IPv4 vers notre adresse : ICMP type 8 | écho de même taille (`icmp_echo.rs`) |
| IPv4/IPv6 UDP vers le port 123 | réponse NTP de 48 octets (`ntp_server.rs`) ; mode 4 vers le client (`ntp_demux.rs`, `ntp_client.rs`) |
| ICMPv6 type 128, 135 (NS), 136 (NA) | écho, NA ; apprentissage du cache de voisins (`icmpv6.rs`, `nbr_cache.rs`) |
| Tout le reste | rien (pas de TCP, pas d'autre port, pas de DHCP, pas de RA ni de Redirect, pas de gestion à distance) |

Positif par conception : configuration **statique** (pas de DHCP voyou, pas de RA voyou, pas de Redirect ICMP/ICMPv6 : `ipv6_rx` rejette ff02::1, `icmpv6` ignore les types 134 et 137) ; les réponses du serveur partent vers la MAC source de la requête (`Meta.eth_src`) sans consulter le cache ARP, donc l'empoisonnement ARP n'affecte pas le service serveur, seulement le client.

Échelle de gravité : **haute** = exploitable hors chemin avec des moyens modestes contre A1, A2 ou A3 ; **moyenne** = demande d'être sur le chemin ou sur le lien, ou impact borné ; **basse** = mineur, durcissement.

## (a) Réflexion et amplification

| Menace | Ce que fait le design | Risque résiduel | Gravité | Correction proposée |
|---|---|---|---|---|
| Amplification en taille (requête usurpée → réponse plus grosse vers la victime) | `ntp_server` : réponse toujours 48 octets (N10), requête ≥ 48 (N1) ; octets au-delà du 48ᵉ (champs d'extension) lus et ignorés | aucun : facteur ≤ 1 (test `reply_never_larger_than_request` : 18 tailles de 48 à 1 472 octets, IPv4 et IPv6, facteur global 0,155) | — | — |
| Amplification par modes 6/7 (`monlist`, `readvar`) | seul le mode 3 avec VN 3/4 est servi | aucun (test `only_client_mode_is_answered` : 64 combinaisons mode×VN, mode 6 READVAR, mode 7 MON_GETLIST_1 de 8, 48, 192 octets) | — | — |
| Réflexion simple (facteur 1) vers un tiers | limiteur par source : 8 réponses/s par case (`RATE_LIMIT_PER_S`), silence au-delà (`kod_enable` = 0) | réflexion 1:1 possible jusqu'à 512 réponses/s ; inhérent à tout serveur UDP public | basse | voir F1 (table plus grande, clé secrète) |
| KoD comme vecteur | 48 octets, un par requête limitée, Receive = Transmit = Origin (aucune heure divulguée) ; désactivé sur la carte | pas d'amplification, mais pas de réduction du volume réfléchi quand activé (test `kod_is_never_larger_than_the_request`) | basse | garder `kod_enable` = 0 ; si activé, limiter aussi les KoD (1 par fenêtre et par case, comme ntpd) |
| **Capacité du limiteur et déni global** | table de 64 mots `{sec, cnt}` indexée par un pli XOR de 6 bits de l'adresse | **F1** : au plus 64 × 8 = 512 réponses normales/s pour tout le serveur (0,5 % de l'objectif de 109 000/s) ; 9 requêtes usurpées par case et par seconde (576 paquets/s, ≈ 0,5 Mb/s avec en-têtes) réduisent au silence **tous** les clients (test `limiter_capacity_and_global_denial`) ; le pli XOR est public : l'attaquant choisit ses cases | **haute** | F1 |
| **Déni ciblé par usurpation de la victime** | limiteur indexé sur l'adresse source annoncée | **F2** : 8 requêtes forgées « de » V consomment la part de V pour la seconde ; la vraie requête de V n'a pas de réponse ; une adresse W de la même case aussi (test `rate_limiter_and_spoofed_victim`) ; en IPv6 la clé est le /64 : tout un /64 partage un compteur (test `ipv6_limiter_keyed_on_slash64`) | moyenne | F2 (fuite aléatoire « leak ») |
| Collisions de hachage | une source n'est jamais servie plus que `lim` fois par fenêtre (par construction) ; une source peut être limitée plus tôt par une autre de sa case | voir F1/F2 | — | — |
| Réflexion vers un service UDP (écho, chargen…) et boucles | réponse au port source quel qu'il soit, y compris 0, 7, 13, 17, 19, 37 | **F8** : l'écho (7) renvoie notre mode 4, ignoré ; mais une ligne chargen (19) de ≥ 48 caractères commençant par `c` (0x63) ou `[` (0x5b) est un mode 3 valide : boucle serveur ↔ chargen, ralentie seulement par le limiteur (test `service_ports_and_text_loops_today`) | basse | F8 |
| Adresses source impossibles (127/8, notre adresse, diffusion du sous-réseau, classe E ; `::`, `::1`, notre adresse, `::ffff:0:0/96`) | `ipv4_rx` I6 ne rejette que 0.0.0.0, 224/4, 255.255.255.255 ; `ipv6_rx` G4 ne rejette que ff00::/8 et accepte `::` (pour le DAD) | **F8** : toutes reçoivent une réponse (test `martian_sources_today`) ; pour la diffusion du sous-réseau, la réponse porte 192.0.2.255 en destination IP dans une trame unicast vers l'émetteur ou le routeur, qui peut la diffuser sur le lien | basse | F8 |
| ICMP écho, NS/NA, ARP non limités | écho de même taille, NA de 86 octets pour un NS d'au moins 78, réponse ARP de 60 pour 60 | **F11** : réflexion 1:1 sans limite par l'écho ICMP (y compris 1 472 octets) ; NS→NA facteur ≤ 1,1 mais seulement sur le lien (hop limit 255) | basse | F11 |

## (b) Paquets malformés, surdimensionnés, blocages

| Menace | Ce que fait le design | Risque résiduel | Gravité |
|---|---|---|---|
| Trame courte (< 14 octets, < 5 octets) | `fcs_check` : < 5 octets bruts → rien, ni trame ni horodatage ; `eth_rx` : ≤ 14 octets → rien, mais l'horodatage est consommé | aucun (fuzz `fuzz_short_frames_line_rate` : 968 trames ≤ 64 octets au débit ligne) | — |
| Trame > 1 518 octets (géante) | `mii_rx` coupe après l'octet 1 518 avec `err` | aucun (fuzz, cas « jumbo ») | — |
| FCS faux, `rx_er` | `err` sur le dernier octet, propagé et jeté par chaque couche ; une réponse en coupe-à-travers (écho) se termine par `err` et `tx_path` la jette | aucun (fuzz ; `bad_fcs_is_dropped` de `top.rs`) | — |
| 802.1Q, EtherTypes inconnus | `eth_rx` n'accepte que 0x0800, 0x0806, 0x86DD | aucun ; conséquence fonctionnelle : le port doit être non étiqueté | — |
| IPv4 : options, fragments, longueurs | IHL ≠ 5 rejeté (I2), MF ou offset ≠ 0 rejeté (I3), TTL 0 (I4), longueur totale > trame → `err` en fin, < trame → bourrage ignoré | aucun (fuzz `fuzz_malformed_headers`) | — |
| IPv6 : en-têtes d'extension | tout `next header` ≠ 17/58 rejeté, y compris hop-by-hop, routage, fragment, ESP | aucun (fuzz, chaînes de 1 à 4 en-têtes) | — |
| UDP : longueur incohérente, somme nulle en IPv6 | longueur UDP ≠ longueur IP rejetée ; somme 0 rejetée en IPv6 ; somme fausse → `err` | aucun | — |
| NS/NA : options malformées | longueur 0, débordement, doublons rejetés | aucun (fuzz, `nd_client.rs`) | — |
| Trames dos à dos au débit ligne, IFG trop court | `rx_path` : FIFO 2 048 octets, 16 crédits d'horodatage ; une trame qui ne trouve pas de place est jetée entière | perte de trames sous surcharge (voulu) ; jamais de désalignement trame/horodatage | — |
| **Blocage du pipeline** (une unité qui attend pour toujours) | voir l'analyse ci-dessous | aucun blocage trouvé par lecture ni par le fuzz : après chaque lot, une requête témoin reçoit sa réponse | — |

**Analyse des attentes non bornées.** Chaque attente sans délai documentée a été vérifiée contre la garantie de son producteur :

| Attente | Garantie du producteur | Atteignable depuis le réseau ? |
|---|---|---|
| `fcs_check`, `eth_rx` attendent un horodatage par trame | `rx_path` : l'horodatage d'une trame est délivré au plus tard avec son premier octet (invariant d'appariement COMMIT/SFD, propriété formelle de `rx_path`) ; `fcs_check` ne transmet l'horodatage que si la trame émet quelque chose | non (trames de 1 à 4 octets testées) |
| `tx_path` attend qu'on consomme ses horodatages | `Drain` toujours prêt dans les deux tops | non |
| `tx_arb` attend la charge utile d'un `hdr` accordé | `arp` n'émet `hdr` qu'avec ses 28 octets en main ; `udp4_tx`/`udp6_tx` stockent avant d'émettre ; `icmp_echo`/`icmpv6` en coupe-à-travers terminent toujours (troncature → 4 octets + `err`) car la trame reçue se termine toujours (`rx_dv` ou limite 1 518) | non (échos tronqués et de 1 472 octets testés) |
| `ntp_server` attend la charge utile après un `Meta` | `udp4_rx`/`udp6_rx` n'émettent le `Meta` que si au moins un octet suit (longueur > 8) | non |
| `ntp_client` (état `LWT`) attend la réponse d'une consultation ARP/ND | `arp` et `nbr_cache` répondent à chaque consultation dès que `lookup_rsp` a de la place, indépendamment de leur automate de trames (`arp.rs`, lignes ≈ 331-342) | non |
| `ntp_demux` : blocage en tête de file du serveur par le client | chaque événement du client est borné (≤ 1 + n + 4 cycles pour une réponse, 52 pour une requête ; filtre ≤ 640, sélection ≤ 541, servo ≤ 59 cycles ; `sync_state` ne bloque jamais) | borné ; une inondation de mode 4 ne bloque pas (`reply_flood_does_not_starve` de `ntp_client.rs`) |
| Résolution ARP en attente | délai de 2^26 cycles (T4.2c) | non |

Non couvert : la version avec client (`ArtyCoreClient`) n'a pas été soumise au fuzz complet (le banc système avec client est trop lent en débogage) ; ses unités supplémentaires (`ntp_demux`, `MetaArb`, `UdpArb`) sont couvertes par l'analyse ci-dessus et par leurs tests unitaires.

## (c) Attaques sur le temps

| Menace | Ce que fait le design | Risque résiduel | Gravité | Correction |
|---|---|---|---|---|
| Réponse amont forgée hors chemin | `ntp_client` (CHK) exige : requête en cours (`out`), adresse source = celle de l'emplacement (128 bits en IPv6), port source 123, mode 4, VN 3/4, ≥ 48 octets, pas d'`err`, **Origin = T1 sur 64 bits** ; une réponse non authentique ne ferme pas la requête | toute falsification sans le bon Origin est sans effet (test `forged_replies_never_make_a_sample` : 64 bits d'Origin un par un, adresse, port, famille, modes, versions, troncatures, `err`) ; **mais l'Origin est prévisible : F3** | **haute** | F3 |
| Prévisibilité de T1 | T1 = lecture d'horloge + latence constante, prise un nombre fixe de cycles après le franchissement du « tick » de 2^-6 s qui déclenche le sondage ; port source fixe 123 ; nos propres réponses serveur publient notre horloge (T2/T3 à 10 ns près) et notre Reference Timestamp date chaque mise à jour (F10), donc l'instant de chaque sondage ; la période est fixe (2^poll s depuis le sondage précédent) | **F3** : mesuré (test `t1_is_predictable_from_the_poll_tick`, horloge au rythme réel, phase sous-cycle aléatoire, 40 sondages) : T1 − tick − latence ∈ [451 ; 488] ns, 161 valeurs de 2^-32 s, **≈ 7,3 bits** d'incertitude ; un attaquant qui envoie ≈ 160 réponses forgées (≈ 15 ko) entre notre requête et la vraie réponse (fenêtre d'un RTT : il suffit d'être plus proche de nous que l'amont visé) gagne le sondage, et la vraie réponse est ensuite rejetée. ntpd brouille les bits sous la précision, chrony envoie un horodatage d'émission aléatoire | **haute** | F3 |
| **Pas de démarrage** décidé par une seule source | après reset, l'emplacement 0 atteint 4 échantillons (son iburst, 6 s) avant les autres (décalés de 0,25 s) ; la sélection accepte un seul pair éligible (m = 1) ; le servo en NSET fait un pas **de taille quelconque**, sans PANICT | **F4** : contrôler les 4 réponses iburst de l'emplacement 0 (sur le chemin, ou hors chemin grâce à F3) met notre horloge à n'importe quelle date ; il n'y a ensuite plus jamais de pas : tant que la vraie source n'est pas éligible, le serveur annonce cette fausse heure en stratum 2 ; dès qu'elle l'est, le désaccord retire la synchronisation (LI 3, stratum 16) et le servo finit en PANIC : déni de service durable **jusqu'au reset** (test Python `test_single_source_decides_the_startup_step` : +1 an appliqué, puis aucune majorité) | **haute** | F4 |
| Sur le chemin d'**une** source, après démarrage | sélection (Marzullo) : avec 2 amonts (configuration `Board`), un menteur rend le pair inéligible (gigue) puis l'intersection vide : maintien (`valid` = 0), l'horloge garde sa fréquence ; avec 4 amonts, le menteur est un faux tic (test `test_one_forged_peer_of_two_gives_holdover`, `test_one_falseticker_of_four_is_ignored`) | déni de synchronisation (maintien) avec 2 amonts ; aucun effet sur l'heure | moyenne | recommandation R1 : 4 amonts d'opérateurs et de chemins distincts |
| Sur le chemin de **toutes** les sources, après démarrage | au-delà de STEPT (128 ms) : ignoré (SPIK) pendant STEPOUT (900 s), puis rattrapage **sans pas** à au plus 500 ppm ; au-delà de PANICT (1 000 s) : jamais suivi (PANIC, `synced` = 0) | **F7** : un mensonge persistant est suivi à 43 s/jour au plus (test `test_all_upstreams_forged_slew_at_most_500_ppm` : 10 s rattrapées en moins d'un jour) ; attaque par **délai** asymétrique : erreur ≤ délai/2, inhérente à NTP (NTS n'y change rien) | moyenne | F7 (authentification), R1 |
| Réponse rejouée | une seule requête en cours par emplacement, Origin = T1 courant ; `out` remis à 0 à la première réponse authentique | aucun (test `replayed_reply_is_rejected`) | — | — |
| KoD forgé | un KoD authentique (bon Origin) compte un sondage manqué, rien d'autre : pas d'arrêt, pas de recul | déni d'un sondage, au même prix qu'une fausse réponse (F3) ; en contrepartie un vrai KoD RATE/DENY de l'amont **n'est pas respecté** (F9) (test `kod_with_right_origin_costs_one_poll`) | basse | F9 |
| Boucle de synchronisation (un amont synchronisé sur nous) | pas de test de Reference ID (`selection.rs`, écart documenté) | improbable en configuration statique | basse | ajouter le test `ref_id ≠ notre adresse` dans `fit` |

## (d) Couche liaison

| Menace | Ce que fait le design | Risque résiduel | Gravité | Correction |
|---|---|---|---|---|
| **Empoisonnement ARP** de la passerelle (redirige nos requêtes NTP, donc F3/F4 deviennent triviaux pour l'attaquant du lien) | `arp.rs` apprend de tout paquet ARP valide **destiné à notre adresse**, requête ou réponse, sollicité ou non, et met à jour une entrée existante en place ; pas de vieillissement | **F5** : une réponse non sollicitée, ou une simple requête ARP « de » la passerelle, remplace sa MAC ; l'ARP gratuit (tpa = spa) et les requêtes pour un autre hôte ne sont pas appris (test `arp_poisoning_of_the_gateway`) | moyenne | F5 |
| **Usurpation NDP** (pas de SEND) | `icmpv6` apprend de tout NA bien formé (hop limit 255) quelle que soit sa cible et ses drapeaux S/O, et de tout NS qui nous vise avec option SLLA | **F5** : NA non sollicité (S = 0), NA avec O = 0 (RFC 4861 §7.2.5 : ne doit pas écraser), NS « de » la passerelle : la passerelle fe80::1 est réassociée ; un NA à hop limit < 255 ou vers ff02::1 est ignoré (test `nd_spoofing_of_the_gateway`) | moyenne | F5 |
| **Éviction** des caches (4 entrées, tourniquet) | toute adresse nouvelle apprise remplace l'entrée suivante | **F6** : quatre clients ordinaires du lien qui nous envoient une requête ARP évincent la passerelle (test `arp_cache_eviction_by_ordinary_requests`) ; quatre NA non sollicités font de même en IPv6 (test `nd_cache_eviction_by_unsolicited_na`) ; le client doit re-résoudre après chaque éviction, et perd le sondage si la résolution échoue 8 fois | basse | F6 |
| Inondation de NS / DAD | un NS valide pour nous reçoit un NA ; un DAD (source `::`) pour notre adresse reçoit un NA vers ff02::1 (on défend l'adresse) ; nous ne faisons pas de DAD (adresses statiques) : pas de déni par DAD contre nous | inondation de NS : NA de 86 octets pour un NS de 78 ; la réception IPv6 est suspendue pendant l'émission du NA : au débit ligne, des trames d'autres flux peuvent être jetées par `rx_path` (F11) | basse | F11 |
| Inondation de MAC (CAM du commutateur) | sans objet pour l'équipement (filtrage sur notre MAC et les groupes requis) | dépend du commutateur | — | sécurité de port côté commutateur |
| 802.1Q | trames étiquetées jetées | pas de saut de VLAN possible via nous | — | — |

## (e) Épuisement de ressources

| Ressource | Comportement | Risque | Gravité |
|---|---|---|---|
| Table du limiteur (64 mots) | partagée par toutes les sources | **F1** (déni global), **F2** (déni ciblé) | haute / moyenne |
| FIFO de réception (2 048 octets, 16 horodatages) | une trame sans place est jetée entière | perte sous surcharge, sans blocage ; toutes les classes de trafic partagent la file : une inondation ICMP/NS sur le lien fait perdre des requêtes NTP (F11) | basse |
| Chemin d'émission (`tx_arb` tourniquet, `udp*_tx` 128 octets, `tx_path` 4 096) | les réponses n'excèdent jamais les requêtes (sauf NA, +10 %) ; le tourniquet garantit à chaque source au plus 3 trames d'attente | partage équitable, pas de famine | — |
| `ntp_server` | 98 cycles par requête de 48 octets, n + 50 pour n octets, contre ≈ 917 disponibles au débit ligne | jamais le goulot | — |
| Caches ARP/voisins (4 entrées) | tourniquet, sans vieillissement | **F6** | basse |
| Client : files de réponses | une réponse prise par cycle `SCAN` en alternance avec les actions du séquenceur | pas de famine (`reply_flood_does_not_starve`) | — |

## (f) Fuites d'information

| Donnée publiée | Contenu | Risque | Gravité |
|---|---|---|---|
| Reference ID | IPv4 de l'amont système, ou 4 premiers octets du MD5 de son IPv6 ; conservé quand on n'est plus synchronisé | normal en NTP (RFC 5905) ; révèle notre amont | — |
| Reference Timestamp | instant (notre horloge) de la dernière `Selected` utilisable, mis à jour **à chaque réponse amont acceptée** | **F10** : révèle l'instant de chacun de nos sondages, donc la phase de l'ordonnanceur ; c'est ce qui rend F3 exploitable | basse (haute tant que F3 n'est pas corrigé) |
| Receive / Transmit | notre horloge à ≈ 10 ns | inhérent au service | — |
| Stratum, précision, root delay/dispersion, poll | valeurs normales ; le poll renvoie celui du client | — | — |
| Adresse link-local | identifiant EUI-64 de la MAC | traçabilité de la carte ; sans importance pour un serveur | — |

Aucune donnée privée, aucun secret (il n'y en a pas), aucune mémoire non initialisée n'est émise : chaque octet émis est construit à partir de champs définis (vérifié par l'oracle du fuzz pour les classes de réponses).

## (g) Chaîne d'approvisionnement et construction

- TxHDL épinglé à un commit (`tools/setup.sh`, `PIN=cacb2413…`, empreinte git SHA-1) ; dépendances Rust figées par `Cargo.lock` (sommes de contrôle crates.io). Aucun accès réseau à l'exécution : le bitstream est autonome.
- Points faibles : `setup.sh` clone depuis GitHub au premier lancement (faire confiance à l'hébergeur pour le contenu du commit épinglé, atténué par l'empreinte) ; les outils Python de preuve (`yowasp-yosys`, `z3-solver`) et Vivado ne sont pas épinglés par version ; le Verilog généré n'est pas comparé à une référence signée. Recommandation R2 : épingler les versions pip (`requirements.txt` avec empreintes), archiver l'empreinte des `out/*.v` et du bitstream livrés.

## Ce que l'absence de NTS (RFC 8915) laisse ouvert

NTS est hors périmètre v1. Sans lui (ni clés symétriques RFC 5905/RFC 8573) :
- côté client, l'unique défense contre une réponse forgée est l'Origin (F3) et l'adresse source : un attaquant sur le chemin d'un amont contrôle entièrement ses réponses (F4 au démarrage, F7 ensuite) ; NTS authentifierait chaque réponse et rendrait l'Origin imprévisible (nonce chiffré dans le cookie) ;
- côté serveur, nos clients ne peuvent pas nous authentifier : un attaquant sur leur chemin peut se faire passer pour nous ;
- NTS ne corrige ni les attaques par délai, ni la réflexion 1:1 (RFC 8915 impose d'ailleurs une réponse pas plus grande que la requête, ce que notre serveur respecte déjà), ni F1/F2.
- Étape intermédiaire réaliste sur FPGA : AES-CMAC (RFC 8573) avec clés pré-partagées vers les amonts (un cœur AES-128 tient dans quelques centaines de LUT) ; NTS complet demande en plus TLS pour l'échange de clés (NTS-KE), hors de portée d'une logique sans processeur.

## Tests de démonstration

Fichiers ajoutés (aucun fichier de `rtl/src/` modifié) :

| Fichier | Banc | Tests (ignorés = constat, échouent aujourd'hui) | Durée (débogage, 4 cœurs partagés) |
|---|---|---|---|
| `rtl/tests/security_common/mod.rs` | top serveur complet sur ses broches MII (`PinBench`, copie de `top.rs`), générateurs de trames avec sommes de contrôle, analyse des trames émises | — | — |
| `rtl/tests/security_amplification.rs` | top complet ; `NtpServer` seul | `reply_never_larger_than_request`, `only_client_mode_is_answered`, `rate_limiter_and_spoofed_victim`, `limiter_capacity_and_global_denial`, `ipv6_limiter_keyed_on_slash64`, `kod_is_never_larger_than_the_request`, `martian_sources_today`, `service_ports_and_text_loops_today` ; ignorés : `no_reply_to_martian_sources` (F8), `no_reply_to_service_ports` (F8) | ≈ 15 s |
| `rtl/tests/security_robustness.rs` | top complet, oracle des trames émises, requête témoin après chaque lot | `fuzz_random_and_corrupted_frames`, `fuzz_malformed_headers`, `fuzz_everything_mixed`, `fuzz_short_frames_line_rate` (≈ 2 000 trames en tout, graines fixes) | 30 à 50 s par test seul, ≈ 80 s pour le fichier (tests en parallèle) |
| `rtl/tests/security_client.rs` | `NtpClient` seul ; `Arp` seul ; `eth_rx → ipv6_rx → icmpv6 → nbr_cache` | `forged_replies_never_make_a_sample`, `replayed_reply_is_rejected`, `kod_with_right_origin_costs_one_poll`, `t1_is_predictable_from_the_poll_tick`, `arp_poisoning_of_the_gateway`, `arp_cache_eviction_by_ordinary_requests`, `nd_spoofing_of_the_gateway`, `nd_cache_eviction_by_unsolicited_na` ; ignorés : `t1_is_unpredictable` (F3), `arp_gateway_entry_is_not_rebound_by_unsolicited_packets` (F5) | < 1 s |
| `tools/golden/ntpclient/tests/test_security.py` | chaîne filtre → sélection → servo des modèles bit-exacts | `test_single_source_decides_the_startup_step`, `test_one_forged_peer_of_two_gives_holdover`, `test_one_falseticker_of_four_is_ignored`, `test_all_upstreams_forged_slew_at_most_500_ppm` ; sauté : `test_startup_step_needs_a_majority` (F4) | < 1 s |

Lancement : `cargo test -p ntp_fpga --test security_amplification --test security_robustness --test security_client` (ajouter `-- --include-ignored` pour voir les constats échouer) ; `python3 -m unittest discover tools/golden/ntpclient`.

Oracle du fuzz (`security_robustness.rs`, `explains`) : une trame émise est acceptée seulement si elle est (1) une réponse ARP de notre part à une requête ARP pour notre adresse, adressée à son émetteur ; (2) un écho ICMP/ICMPv6 portant exactement l'identifiant, la séquence et les données d'une requête reçue ; (3) un NA pour une de nos adresses répondant à un NS reçu pour cette cible ; (4) une réponse NTP de 48 octets, mode 4, depuis le port 123, vers l'adresse et le port d'une requête mode 3 reçue dont le Transmit est son Origin. FCS, en-tête IPv4 et somme UDP de chaque trame émise sont vérifiés, et le volume émis doit rester ≤ au volume reçu.

## Synthèse des constats

| N° | Constat | Gravité | Statut | Démonstration | Correction proposée (unité, fichier, changement) |
|---|---|---|---|---|---|
| F1 | Limiteur : 64 cases × 8/s = 512 réponses/s au total ; 576 requêtes/s usurpées font taire tous les clients ; cases choisies par l'attaquant (pli XOR public) | haute | correction nécessaire | `limiter_capacity_and_global_denial` | `ntp_server.rs` : `RL_ENTRIES` 64 → 4 096 en block RAM (`#[ram_style("block")]` au lieu de `#[distributed]`) ; `src_hash` remplacé par un hachage **non linéaire à clé secrète** (multiplication-décalage, `(a · clé mod 2^64) >> 52` avec `a` impair secret, calculée en série pendant RECV) ; `a` tiré au reset de la source d'entropie de F3 ; garder 8/s par case. Voir détail. |
| F2 | Déni ciblé : 8 requêtes usurpées de V (ou d'une adresse de sa case, ou de son /64) privent V de réponse pour la seconde | moyenne | correction nécessaire (atténuation) | `rate_limiter_and_spoofed_victim`, `ipv6_limiter_keyed_on_slash64` | `ntp_server.rs` : « fuite » à la chrony (`ratelimit leak`) : une requête limitée reçoit quand même une réponse normale avec probabilité 2^-L (L = 2 par défaut, bit aléatoire de la source d'entropie), nouvelle entrée `rl_leak: In<U<3>>` ; inhérent aux limiteurs par adresse (ntpd, chrony) : documenter. |
| F3 | Origin (T1) prévisible à ≈ 7–8 bits : réponse amont forgeable hors chemin | haute | correction nécessaire | `t1_is_predictable_from_the_poll_tick` ; `t1_is_unpredictable` (ignoré) | `ntp_client.rs` : envoyer dans les octets 40..47 un **nonce aléatoire de 64 bits** (nouvelle RAM 4 × 64 `m_org`, écrite en S avec le nonce, comparée à l'Origin dans CHK à la place de `m_t1`) et garder le vrai T1 dans `m_t1` pour l'arithmétique ; source d'entropie : LFSR/xorshift de 64 bits avancé à chaque cycle et mélangé avec les bits bas de chaque horodatage RX (`Meta.rx_ts`, phase asynchrone ClkRx/clk_sys). Modèle `client.py` (`build_request`, vecteurs) à mettre à jour. Variante minimale si la RAM manque : brouiller les 8 bits de fraction sous la précision (2^-24 s, erreur ≤ 60 ns sur T1) : ≈ 15 bits au total, insuffisant seul. Défense en profondeur : port source aléatoire (RFC 9109), demande d'accepter un port ≠ 123 dans `udp4_rx`/`udp6_rx` pour les réponses mode 4. |
| F4 | Au démarrage, une seule source décide d'un pas de taille quelconque | haute | correction nécessaire | `test_single_source_decides_the_startup_step` ; `test_startup_step_needs_a_majority` (sauté) | `selection.rs` : nouvelle entrée `n_en: In<U<3>>` (nombre d'emplacements activés, constante de `config`) ; tant qu'aucun pair système n'a jamais été choisi depuis le reset (nouveau registre `ever`), `valid` exige `survivants ≥ ⌊n_en/2⌋ + 1` (RFC 5905 NSANE/MINCLOCK) ; modèle `selection.py` et vecteurs à mettre à jour. Avec la configuration `Board` (2 amonts) : les deux doivent être d'accord avant le pas. |
| F5 | Empoisonnement ARP et NDP de la passerelle depuis le lien | moyenne | corrigé (RTL et modèle) : apprentissage sollicité uniquement ; reste la course contre l'attaquant du lien | `arp_poisoning_of_the_gateway`, `arp_invalid_replies_while_resolving_and_without_resolution`, `arp_gateway_entry_is_not_rebound_by_unsolicited_packets`, `arp_residual_race_attacker_answers_first` ; `nd_spoofing_of_the_gateway`, `nd_residual_race_attacker_answers_first`, `nd_na_after_the_timeout_is_not_learned` (`security_client.rs`) ; `arp.rs`, `nd_client.rs`, `test_nd_client.py` | `arp.rs` : n'insérer ou modifier une entrée que sur une **réponse** (op 2) dont `spa` est l'adresse de la résolution en cours (`wait`), ou si la MAC est inchangée (rafraîchissement) ; ne plus apprendre des requêtes. `icmpv6.rs` : registre `pend_t` (cible du dernier NS émis), n'apprendre d'un NA que si `target == pend_t` et S = 1, ignorer O = 0 pour une entrée existante de MAC différente ; ne plus apprendre des NS. Option `config` : MAC statique de la passerelle (`GATEWAY_MAC4/6`). Résiduel : course avec l'attaquant du lien sur notre propre requête (sans SEND/DAI) : recommander l'inspection ARP/ND et le RA-guard du commutateur. |
| F6 | Éviction de la passerelle des caches de 4 entrées par du trafic ordinaire | basse | corrigé par F5 : seule la réponse à notre propre requête crée une entrée ; une inondation de requêtes, de réponses ou de NA non sollicités ne peut plus évincer la passerelle (pas d'épinglage) | `arp_cache_eviction_by_ordinary_requests`, `nd_cache_eviction_by_unsolicited_na`, `arp_flooding_cannot_evict_the_gateway` (changés : la passerelle reste) | Couvert par F5 (on n'insère plus que les prochains sauts qu'on résout) ; sinon `arp.rs`/`nbr_cache.rs` : ne jamais évincer l'entrée de la passerelle (bit `pin` posé quand `ip == gateway`). |
| F7 | Sur le chemin de toutes les sources : heure tirée de 43 s/jour au plus après démarrage ; attaques par délai ; pas d'authentification | moyenne | limité par conception, résiduel accepté v1 | `test_all_upstreams_forged_slew_at_most_500_ppm` | Authentification des amonts (AES-CMAC RFC 8573, puis NTS) ; R1 : 4 amonts d'opérateurs et de chemins différents ; option : abaisser la pente maximale de rattrapage en SPIK (`servo.rs`, `MAXTRIM` séparé pour le cas `slw`, par exemple 50 ppm = 4,3 s/jour). |
| F8 | Réponses aux sources impossibles (127/8, nous-mêmes, diffusion du sous-réseau, classe E, `::`, `::1`, `::ffff:0:0/96`) et aux ports de services UDP (0, 7, 13, 17, 19, 37) ; boucle possible avec chargen | basse | adresses source : corrigé (`ipv4_rx` I6, `ipv6_rx` G9) sauf la diffusion du sous-réseau (pas de masque dans `ipv4_rx`, réponse encore émise) ; ports source : reste ouvert | `no_reply_to_martian_sources`, `subnet_broadcast_source_is_still_answered`, `rx_table` (`ipv4.rs`, `ipv6.rs`) ; `service_ports_and_text_loops_today` ; `no_reply_to_service_ports` (ignoré) | `ipv4_rx.rs` (I6) : rejeter aussi 127/8, 240/4, `local_ip`, `local_ip \| !netmask` (nouvelle entrée `netmask`) ; `udp6_rx.rs` (ou `ipv6_rx.rs` pour le seul UDP) : rejeter `::`, `::1`, `local_ll`, `local_gua`, `::ffff:0:0/96` ; `ntp_server.rs` (règle N1 étendue) : requête invalide si port source ∈ {0, 7, 13, 17, 19, 37} (ou < 1024 sauf 123). |
| F9 | Le client ne respecte pas un KoD RATE/DENY/RSTR (compte un manqué et continue) | basse | correction souhaitable | `kod_with_right_origin_costs_one_poll` | `ntp_client.rs` : sur KoD authentique, RATE → doubler l'intervalle de l'emplacement (registre de poll par emplacement, plafonné à 2^17 s) ; DENY/RSTR → désactiver l'emplacement jusqu'au reset (bit dans `m_fl`). |
| F10 | Le Reference Timestamp révèle l'instant de chaque sondage (rend F3 exploitable) | basse | correction souhaitable | lecture (`sync_state.rs`, `ref_ts = upd`) | `sync_state.rs` : publier `ref_ts` arrondi à la seconde inférieure (`upd & !0xFFFF_FFFF`) ; sans objet une fois F3 corrigé. |
| F11 | Échos ICMP/ICMPv6, NA et réponses ARP sans limite ; NS→NA facteur ≤ 1,1 sur le lien ; une inondation peut faire perdre des requêtes NTP dans la file commune | basse | correction souhaitable | lecture ; tailles vérifiées par l'oracle du fuzz | `icmp_echo.rs`, `icmpv6.rs`, `arp.rs` : seau à jetons commun (par ex. 1 000 réponses/s, rafale 16), au-delà la requête est avalée sans réponse ; option `config` pour désactiver l'écho ICMP. |

Recommandations sans changement de code : **R1** configurer 4 amonts (opérateurs et chemins distincts ; les adresses de `Board` sont des adresses de documentation à remplacer) ; **R2** épingler les outils de construction (section g) ; **R3** côté réseau : filtrage d'entrée BCP 38 en amont, inspection ARP/ND et RA-guard sur le port du commutateur, port non étiqueté dédié ; **R4** laisser `kod_enable` à 0 (silence) tant que les KoD ne sont pas eux-mêmes limités.

## Détail des corrections principales

### F1 et F2 — limiteur

État actuel (`ntp_server.rs`, « Rate limiter ») : `rl: Mem<U<32>, 64>` distribuée, index `src_hash(key)` = pli XOR de 6 bits, mis à jour au cycle L pour les requêtes valides.

Changement proposé :
1. `pub const RL_ENTRIES: usize = 4096;` et `#[ram_style("block")] pub rl: Mem<U<32>, 4096, DefaultClock>` (4 096 × 32 = 128 Kib, 4 RAMB36 sur 135 ; ou 4 096 × 18 avec `sec` sur 8 bits et `cnt` sur 10 bits, 2 RAMB36).
2. Index = `(a · key mod 2^64) >> 52` (hachage multiplication-décalage de Dietzfelbinger, universel : sans connaître `a`, l'attaquant ne sait pas quelles adresses partagent une case), `a: Reg<U<64>>` impair, chargé au reset depuis la source d'entropie de F3 (et renouvelable, par ex. toutes les 2^16 s). Pas de CRC ni de pli XOR, même salés : ils sont linéaires, deux clés y entrent en collision indépendamment du sel. Le produit se calcule en série (2 bits par cycle, 32 cycles, un additionneur de 64 bits) pendant les au moins 48 cycles de RECV ; le mot est lu dès que l'index est prêt, gardé dans un registre, avec contournement si la requête précédente vient d'écrire le même index.
3. Fuite (F2) : `allowed = unl | (cnt < lim) | (leak_bit & leak_en)` où `leak_bit` est vrai avec probabilité 2^-L.

Effet attendu : la capacité totale passe à 4 096 × 8 = 32 768 réponses/s (≈ 30 % du débit ligne) ; un déni global demande ≈ 37 000 requêtes/s et ne peut plus cibler les cases ; le déni ciblé d'une victime ne fait plus que réduire son service à 1/2^L.

### F3 — Origin imprévisible

1. `ntp_client.rs` : `pub m_org: Mem<U<64>, 4, DefaultClock>` (distribuée). En PAY, octets 40..47 = `nonce` (registre de 64 bits figé au début de PAY) au lieu de `t1p` ; en S, `m_t1[cs] := t1p` (inchangé) et `m_org[cs] := nonce`. En CHK : `auth = out & mode4 & vn_ok & (origin == m_org[cs])`. L'arithmétique (`a = T2 − T1`, `d1 = T4 − T1`) garde `m_t1`.
2. Source d'entropie (`rtl/src/entropy.rs`, nouvelle unité) : xorshift128+ avancé à chaque cycle, état ⊕= `rx_ts[7:0]` de chaque trame reçue (bits de phase entre l'oscillateur du PHY et celui de la carte) ; sortie `rnd: Out<U<64>>`. Sert aussi au multiplicateur secret `a` de F1 et à la fuite de F2. Un générateur rapide non cryptographique suffit : l'attaquant hors chemin ne voit jamais le nonce, et l'attaquant sur le chemin voit déjà la requête ; comme l'effet de la fuite F2 est observable de l'extérieur, prendre un générateur à sortie non linéaire (l'addition finale de xorshift128+) et des bits distincts pour chaque usage.
3. Coût : ≈ 64 + 128 bascules, une RAM 4 × 64 en LUT, un comparateur de 64 bits déjà présent. Le serveur, lui, ne change pas (il recopie l'Origin du client).

### F4 — pas de démarrage

`selection.rs` : nouvelle entrée `n_en` et registre `ever` (posé à la première `Selected` valide). Règle : si `!ever`, `valid := valid & (nsurv ≥ n_en/2 + 1)`. Effet : avec 2 amonts, les deux doivent être éligibles et d'accord à 3 s près (MAXDIST) avant le pas ; avec 4, trois. Un amont menteur au démarrage ne produit plus de pas, seulement un retard de synchronisation. Option complémentaire dans `servo.rs` : en NSET, exiger deux `Selected` successives cohérentes (écart < STEPT) avant de sauter.

### F5/F6 — caches

`arp.rs`, règle d'apprentissage (en-tête, « Cache ») : remplacer « op 1 *ou* op 2 avec tpa = `local_ip` » par « op 2, tpa = `local_ip`, et (`wait` et `spa == pending_ip`) ou (`spa` déjà en cache avec la même MAC) ». `icmpv6.rs` (N4) : n'émettre `learn` pour un NA que si `target == pend_t` (registre chargé à l'envoi d'un NS en RS1) et S = 1 ; supprimer l'apprentissage par NS (N5) ou le limiter au rafraîchissement d'une entrée existante. Résiduel : un attaquant du lien qui répond plus vite que la passerelle à notre propre requête gagne encore (limite d'ARP/NDP sans SEND).
