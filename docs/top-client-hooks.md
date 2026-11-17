# Top : points de raccordement du côté client (phase B de T1.11)

Ce document liste ce qu'il reste à câbler dans `rtl/src/top.rs` quand les unités `ntp_client` (T4.2),
`ntp_demux` (T4.2a) et le cache de voisins IPv6 seront fusionnées. Les noms de canaux sont ceux de
`ArtyCore::run`.

## Déjà câblé

- **`ClientSync<C>`** (source de synchronisation, paramètre `S` du top) : `samples`/`miss`
  → `ClockFilter` → `Selection` (`now` = `time >> 32`, registré) → `ForkSelected` →
  { `Servo` → `trim`, `step`, `step_en` de `DisciplinedClock` ; `SyncState` → `synced`, `stratum`,
  `ref_id` (= `ref_id6`), `ref_ts`, `root_delay`, `root_disp` de `ntp_server` }.
  `servo.synced` alimente `sync_state.servo_synced`. `peer_ref0..3` = `refid_ipv4`/`refid_ipv6` des
  emplacements `ConfigSet::UPSTREAM`, calculés à la synthèse (`top::peer_ref`).
- Construction de carte correspondante : `ArtyBoardWithClient` (`cargo run -p ntp_fpga --bin top_gen --
  --with-client`). Sans client, elle se comporte comme `ArtyBoard` (jamais synchronisée, trim nul).

## À raccorder

### 1. `ntp_client` (T4.2)

| Port du client (attendu) | Raccordement dans `ArtyCore` |
|---|---|
| sortie `Sample` | remplace `idle_samples` : canal `smp_tx` → `sync.samples` |
| sortie « pas de réponse » (`U<2>`, pair) | remplace `idle_miss` : canal `miss_tx` → `sync.miss` |
| `time` (`In<U<96>>`) | un clone de `time` (`DisciplinedClock.time`) |
| configuration | signaux `_up0_addr`, `_up0_v6`, `_up0_en` … `_up3_*`, `poll_log2`, `_gateway`, `_netmask`, `_gateway6` (déjà sortis de `Config<C>`, aujourd'hui inutilisés) |
| requêtes IPv4 (`UdpTxReq` + charge utile 48 octets) | vers `udp4_tx`, **via un arbitre 2 entrées** (voir 4) |
| requêtes IPv6 (`Udp6TxReq` + charge utile) | vers `udp6_tx`, via un arbitre 2 entrées |
| réponses reçues (`Meta` + charge utile) | sorties « client » de `ntp_demux` (voir 2) ; `Meta.rx_ts` donne T4 |
| T1 (horodatage d'émission) | soit prédit comme le T3 du serveur (constante K, `ntp_reply_route.rs`), soit pris sur `tx_path.stamps` (canal `st_rx`, aujourd'hui vidé par `drain_stamps`) : **attention**, `tx_path` produit un horodatage par trame émise, toutes sources confondues ; un client qui l'utilise doit savoir quelle trame est la sienne (par ex. une étiquette qui suit la trame, ou n'émettre que quand le chemin est vide). `tx_path` bloque l'émission si ce canal n'est pas vidé. |

Port source du client : **123**. Les réponses des serveurs amont (mode 4) arrivent alors sur le port
123, que `udp4_rx`/`udp6_rx` acceptent déjà ; c'est `ntp_demux` qui les sépare des requêtes clientes.
Un autre port source demanderait de modifier le filtre de port de `udp4_rx`/`udp6_rx`.

### 2. `ntp_demux` (T4.2a), devant `ntp_server`

Couper les canaux entre les UDP RX et le serveur :

- IPv4 : `n4m_tx`/`n4p_tx` (sorties `ntp_meta`/`ntp` de `udp4_rx`) → **demux** → `server.meta4`/`req4`
  (mode 3) et entrée réponses IPv4 du client (mode 4) ;
- IPv6 : `n6m_tx`/`n6p_tx` (`udp6_rx`) → demux → `server.meta6`/`req6` et réponses IPv6 du client.

Le serveur prend `meta4`/`meta6` indépendamment de la charge utile et lit la charge utile sans jamais la
bloquer : le demux doit respecter le même contrat (un `Meta` avant ou avec le premier octet, la charge
utile complète ensuite). Chaque liaison par `chan()` (registrée).

### 3. Résolution d'adresses

- **IPv4 (ARP, existant)** : `arp.resolve` (`Rx<U<32>>`, aujourd'hui `idle_resolve` → canal `res_tx`) et
  `arp.lookup` (`Rx<U<32>>`, `idle_lookup` → `lk_tx`) viennent du client ; `arp.lookup_rsp`
  (`ArpEntry`, canal `lr_rx`, aujourd'hui `drain_lookup`) va au client. Amont hors sous-réseau
  (`netmask`) : résoudre `gateway`. Les requêtes ARP partent déjà par la source 0 de `tx_arb`.
- **IPv6 (cache de voisins)** : le client demande l'adresse MAC d'un amont (ou de `gateway6` hors lien).
  Le cache doit :
  - émettre des NS : par un constructeur IPv6. `IpArb6` a ses deux entrées prises (`icmp6`, `udp6_tx`) :
    soit l'étendre à 3 entrées, soit donner au cache son propre `ipv6_tx` sur la **source 3 de
    `tx_arb`** (aujourd'hui `idle_h3`/`idle_p3`, canaux `h3_tx`/`p3_tx`) ;
  - recevoir les NA : `icmpv6` les avale (il ne répond qu'aux échos et aux NS). Il faut un démultiplexeur
    sur la sortie ICMPv6 de `ipv6_rx` (canaux `i6m_tx`/`i6p_tx`) : type 136 vers le cache, le reste vers
    `icmp6`.

### 4. Arbitres d'émission UDP

`udp4_tx` et `udp6_tx` n'ont qu'une paire d'entrée, occupée par `route` (réponses du serveur, canaux
`q4`/`y4`, `q6`/`y6`). Ajouter devant chacun un arbitre 2 entrées (copie de `IpArb4`/`IpArb6` de
`rtl/src/ip_arb.rs` pour `UdpTxReq`/`Udp6TxReq`) : source 0 = `route`, source 1 = client. La latence K
du T3 (`ntp_reply_route.rs`) suppose un chemin libre ; une requête cliente devant une réponse la décale
d'une trame (rare : une requête par amont et par intervalle de poll).

### 5. Sources de `tx_arb`

| Source | Phase A | Proposition phase B |
|---|---|---|
| 0 | ARP (réponses et requêtes) | inchangé |
| 1 | `ipv4_tx` (`IpArb4` : ICMP, UDP) | inchangé (le client passe par `udp4_tx`) |
| 2 | `ipv6_tx` (`IpArb6` : ICMPv6, UDP) | inchangé |
| 3 | inactive | NS du cache de voisins (son `ipv6_tx`) |

### 6. `ConfigSet`

Les emplacements amont (`UPSTREAM[0..4]`) et `POLL_LOG2` sont déjà des sorties de `Config<C>`.
`STRATUM_OFFSET` n'est lu par personne : `SyncState` annonce `stratum + 1` en dur. À trancher.
