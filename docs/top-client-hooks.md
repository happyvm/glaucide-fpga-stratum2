# Câblage du client : état final (T1.14)

Le client NTP est câblé dans le top : `ArtyBoardWithClient` (`cargo run -p ntp_fpga --bin top_gen --
--with-client`) est la carte de strate 2 complète. `ArtyBoard` (serveur seul, `Standalone`) est inchangée :
sa netlist est identique octet pour octet à celle d'avant T1.14.

## Sélection de la variante

Le top est générique en son cœur : `ArtyTopOf<P, K>` (`rtl/src/top.rs`), `K: TopCore`.

- `ArtyTop<C, S, P>` = `ArtyTopOf<P, ArtyCore<ClkRx, ClkTx, C, S>>` : le chemin serveur seul (tests de
  `rtl/tests/top.rs`, `ArtyBoard`). `S` reste la source de synchronisation (`Standalone`, `FixedSync`,
  `FixedUnsync`, `ClientSync` sans client).
- `ArtyBoardWithClient` = `ArtyTopOf<Board, ArtyCoreClient<ClkRx, ClkTx, Board, DisciplinedClock>>`
  (`rtl/src/top_client.rs`) : serveur + client, synchronisation `ClientSync<C>`.

Un cœur distinct plutôt qu'un marqueur sur `S` : le client coupe des canaux du chemin serveur
(démultiplexeurs, arbitres), ce qui changerait la netlist et la latence calibrée du serveur seul.

## Ce qui est câblé

| De | Vers | Type |
|---|---|---|
| `udp4_rx`, `udp6_rx` (paire NTP) | `demux4`, `demux6` (`NtpDemux`) | `Meta`, `FrameByte` |
| sorties `srv` des démux | `server.meta4/req4`, `.meta6/req6` | `Meta`, `FrameByte` |
| sorties `cli` des démux | `cmerge` (`MetaArb`, sources 0 = v4, 1 = v6) → `client.meta`, `.rep` | `Meta`, `FrameByte` |
| `route` (réponses serveur) | `uarb4` / `uarb6` source 0 | `UdpTxReq` / `Udp6TxReq`, `FrameByte` |
| `client.req4/pay4`, `.req6/pay6` | `uarb4` / `uarb6` source 1 | idem |
| `uarb4` / `uarb6` (`UdpArb4`/`UdpArb6`, `rtl/src/udp_arb.rs`) | `udp4_tx` / `udp6_tx` | idem |
| `client.arp_lookup`, `.arp_resolve` | `arp.lookup`, `.resolve` | `U<32>` |
| `arp.lookup_rsp` | `client.arp_rsp` | `ArpEntry` |
| `client.nd_lookup` / `nbr.rsp` | `nbr.lookup` / `client.nd_rsp` (`NbrCache`, `flush` = 0) | `U<128>` / `ArpEntry` |
| `client.nd_resolve` | `icmp6.resolve` (émission des NS) | `U<128>` |
| `icmp6.learn` | `nbr.learn` (appris des seuls NA sollicités, F5) | `NdLearn` |
| `client.samples`, `.miss` | `sync.samples`, `.miss` (`ClientSync`) | `Sample`, `U<2>` |
| configuration | tous les fils du client (`up0..3`, adresses, masque, passerelles, `poll_log2`, `precision`) | fils |
| `clock.time` | client (`time`) | fil |
| constantes | `client.tx_latency4/6` = `TX_LATENCY_V4/V6` de `ntp_reply_route.rs` | fils |
| `route.tx_latency` + 10 ns (`lat`, `LatAdj`) | `server.tx_latency` | fil |

Chaque liaison entre deux unités est un `chan()` registré ; `tx_arb` source 3 et `tx_path.stamps` restent
sur des unités `Idle` / `Drain`. Synchronisation (inchangée) : `ClientSync` donne `synced`, `stratum`,
`ref_id` = `ref_id6`, `ref_ts`, `root_delay`, `root_disp` du serveur et `trim`/`step` de l'horloge ;
`peer_ref0..3` = `top::peer_ref` (`refid_ipv4` ou MD5 de l'adresse IPv6 de l'emplacement). Le serveur met
`ref_id` dans les réponses IPv4 et `ref_id6` dans les IPv6 : les deux valent l'identifiant du pair système,
quelle que soit la famille de la requête (RFC 5905), vérifié en simulation pour un pair IPv4.

## Fonctionnement des canaux

- **Une réponse par recherche** : `arp` et `nbr` répondent à chaque `lookup` dans le cycle où ils le
  prennent (ils ne le prennent que si leur canal de réponse a de la place) ; un `chan` ne perd rien ; le
  client n'a qu'une recherche en cours (état `LWT`) et lit la réponse de la famille demandée.
- **Requêtes et réponses sur le fil** : ARP, une réponse à une requête reçue passe avant une requête de
  résolution en attente (décision quand la sortie est libre) ; NS, `icmp6` ne prend un `resolve` qu'entre
  deux messages et un `resolve` en attente retient le message reçu suivant ; UDP, `uarb4`/`uarb6`
  alternent des datagrammes entiers (tourniquet, jamais d'entrelacement) ; `tx_arb` est un tourniquet.
- **Latences d'émission** : l'arbitre ajoute un étage de canal au chemin du serveur (`route → uarb →
  udp*_tx`), compensé par `SERVER_ARB_ADJ` (+10 ns) ; le chemin du client (`client → uarb → udp*_tx`) a
  autant d'étages que le chemin calibré (`server → route → udp*_tx`), il reprend les constantes K telles
  quelles. Mesuré à 25 MHz sur les huit phases de `clk_tx` (pas de 5 ns) : T1 du client et T3 du
  serveur dans **-25..+10 ns**, la même fenêtre que le top serveur seul sur ces huit phases (les cinq
  phases de `rtl/tests/top.rs` n'en voient que -10..+10 ns).

## Vérification (`rtl/tests/top_client.rs`)

Au niveau système (broches MII, cycle exact) : au démarrage, requête ARP et NS (vers le groupe
solicited-node) octet pour octet ; après la réponse ARP / la NA, requêtes NTP IPv4 et IPv6 octet pour
octet (modèle `client.py::build_request`, ports 123 → 123, sommes de contrôle, MAC apprise, source
lien-local pour l'amont `fe80::99`) ; T1 converti en temps réel contre le SFD ; une réponse falsifiée
(Origin faux) n'atteint pas le filtre, la bonne y arrive (offset et délai exacts à partir de T1..T4) ;
une réponse du serveur qui rencontre la requête du client dans `uarb4` attend la fin du paquet du client
(paquets entiers), les deux trames sont exactes ; synchronisation de bout en bout : les quatre
échantillons de l'iburst rendent le pair éligible, le servo **fait un pas** de l'horloge réelle (+100 s,
exactement le pas commandé), puis le serveur répond en IPv4 et IPv6 avec LI 0, strate 3 et
l'identifiant de 192.0.2.99.

Ces tests utilisent `WarpClock<A>` (`top_client.rs`, **simulation seulement**, absent des cartes) : une
seconde NTP dure 1e8 / A cycles (A = 1 000 ou 10 000) pour atteindre les délais du client (15,6 ms de
réessai, iburst à 2 s, échéance 1..2 s). Le comportement au cycle près est inchangé ; les valeurs
absolues des horodatages ne valent plus rien (les biais fixes en ns ne sont pas mis à l'échelle), d'où la
conversion de T1 en temps réel par une sonde sur les registres de l'horloge.

Au niveau des unités seulement : règles fines du client (échéances, abandon après 8 recherches,
désactivation d'un emplacement, KoD, strate 16, arithmétique sur tous les vecteurs), du filtre, de la
sélection, du servo (FREQ/SYNC, pointes, PANIC) et de l'état de synchronisation, du cache de voisins ;
le fonctionnement sur des intervalles de poll réels (64 s) et la convergence du servo.

## Ressources

`top_gen -- --with-client` : 52 modules (45 pour `ArtyBoard`), une netlist + l'enveloppe, `files.f`
inchangé. Yosys (`hierarchy -check`, `proc`, `check -assert`, `synth_xilinx` jusqu'à `map_luts`) :
**29 255 bascules** (14 950 pour `ArtyBoard`), 332 `RAM64M`, 167 `RAM32M` (11). Environ 5 600 bascules
pour `ClientSync`, 1 600 pour le client et le cache ; le reste est surtout dans les canaux registrés
(deux mots par canal, `Meta` fait 401 bits, `Selected`/`PeerStat` ~500 bits).

## Limites restantes

1. **T1 suppose un chemin d'émission libre** (comme T3) : une requête qui attend derrière une autre trame
   part plus tard que son T1 annonce (une trame de 90 octets : ~9 µs à 100 Mb/s). C'est le cas au
   démarrage (les deux emplacements font leur iburst en même temps : la seconde requête suit la première)
   et à chaque poll tant que les emplacements gardent le même calendrier ; cela biaise l'offset de
   l'amont servi en second de la moitié de l'attente. Remèdes possibles : horodatage pris sur
   `tx_path.stamps`, ou espacement des requêtes (modification du client).
2. Même chose pour T3 du serveur derrière une requête du client (rare : une requête par amont et par
   poll).
3. **ARP : une seule résolution en cours** ; si un saut IPv4 ne répond jamais, `arp` l'attend jusqu'à un
   `flush` (relié à 0) et la résolution d'une autre adresse IPv4 reste bloquée. Sans effet avec un seul
   amont IPv4 ou une seule passerelle, gênant avec deux sauts IPv4 dont un muet.
4. Pas de vieillissement dans les caches ARP et de voisins (`flush` à 0).
5. `STRATUM_OFFSET` n'existe plus : le `+ 1` du stratum est fixé par le protocole dans `SyncState`.
6. `poll_log2` du client et du servo vient de la configuration (pas d'adaptation).
7. Le démultiplexeur NTP bloque en tête de file (une réponse cliente non prise retient les requêtes
   serveur derrière elle ; le client prend ses réponses en quelques cycles).
