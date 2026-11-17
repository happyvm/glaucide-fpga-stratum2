# Architecture

## Objectif
Serveur NTPv4 (RFC 5905) stratum 2 : synchronisé sur des serveurs stratum 1, répond en UDP/123 en IPv4 et IPv6.

## Chaîne de données (tout matériel)
```
DP83848 --MII--> mii_rx --> [timestamp RX @SFD] --> eth_parse --> {arp | ndp/icmpv6 | icmp echo | ipv4 | ipv6} --> udp --> ntp_server
                                                                                                               |
DP83848 <-MII--- mii_tx <-- [timestamp TX @SFD] <-- eth_build <-- {…}  <-----------------------------------------+
                                                                                                               
ntp_client (FSM) --> upstream stratum 1 (v4/v6) --> clock_servo (PI) --> disciplined_clock (64-bit, 32.32 s + frac)
```

## Horloge
- `clk_100` (E3) → MMCM → `clk_eth` 25 MHz, `clk_sys` 100 MHz (ou 200 MHz pour résolution 5 ns).
- `disciplined_clock` : accumulateur de phase 64 bits (secondes NTP 32 bits + fraction 32 bits), incrément réglable en ppb par le servo ; saut/slew selon l'écart.
- Timestamps RX/TX pris au SFD, côté domaine `eth_rx_clk`/`eth_tx_clk` (25 MHz), traversée CDC par gray/handshake. Résolution ~40 ns native, corrigée par la latence PHY fixe.

## Stratum 2
- `ntp_client` interroge N serveurs stratum 1 (configurables, v4/v6), filtre d'horloge RFC 5905 (8 échantillons, min delay), sélection/clustering simplifiés, puis `clock_servo` discipline l'horloge.
- Serveur répond stratum 2, Reference ID = adresse IPv4 de la source (ou 4 premiers octets MD5 de l'IPv6), Leap Indicator et Root Delay/Dispersion calculés. Si non synchronisé : LI=3, stratum 16.

## Pile réseau
| Couche | Remarques |
|---|---|
| Ethernet | MAC locale configurable, filtrage unicast/multicast (33:33:ff:xx:xx:xx pour NDP) |
| ARP | cache 1 entrée par pair actif, requête + réponse |
| IPv4 | pas de fragmentation, checksum header |
| IPv6 | link-local EUI-64 + adresse statique/SLAAC (RA), NDP NS/NA, ICMPv6 echo, checksum pseudo-header |
| UDP | port 123, checksum obligatoire en IPv6 |
| NTP | mode 3 → mode 4, 48 octets, pas d'extension ni d'auth (v1) |

## Découpage des phases
1. Squelette, MII RX/TX, timestamping SFD, loopback ARP/ICMP echo (ping).
2. IPv4/UDP/NTP serveur avec horloge libre (stratum 16).
3. IPv6 : NDP, ICMPv6, UDP/NTP v6.
4. Client NTP + servo → stratum 2.
5. Robustesse : rate limit, KoD, MMCM/CDC, mesures d'offset/jitter.

## Configuration réseau : statique uniquement (décision du propriétaire)

Le serveur **ne fonctionne pas en DHCP/BOOTP** : pas de client DHCPv4, BOOTP ou DHCPv6. Adresse MAC, adresse IPv4, masque, passerelle, adresse IPv6 globale statique, serveurs amont et paramètres NTP sont fixés à la synthèse (`rtl/src/config.rs`, `docs/roadmap.md` T4.1).

Conséquences :
- Les trames DHCP/BOOTP reçues (UDP 67/68, broadcast) sont déjà abandonnées par le filtre de ports ; rien à ajouter.
- IPv4 : adresse, masque et passerelle statiques ; ARP sert à résoudre la passerelle et les amonts (`arp.rs`), pas à obtenir une adresse.
- IPv6 : adresse link-local EUI-64 + adresse globale statique. Pas de DHCPv6. La configuration automatique SLAAC par Router Advertisement (T3.5) devient **optionnelle, hors périmètre v1** ; en revanche une **passerelle IPv6 statique** est nécessaire pour joindre des amonts IPv6 hors lien (T4.1b : ajouter `GATEWAY_IP6` à `config`).
- Aucune détection d'adresse en double IPv4 (ARP probe) : l'adresse est choisie par l'administrateur.
- Pour changer d'adresse il faut resynthétiser (ou, plus tard, ajouter un registre de configuration par UART).

## Outils
- TxHDL (Rust → Verilog), simulation native en Rust (`Running::cycle`), traces FST/VCD. Épinglé via `tools/setup.sh`.
- Synthèse/P&R : Vivado sur la machine de l'utilisateur, à partir de `out/*.v` et `constraints/arty_a7_100.xdc`.
- Horloge : accumulateur 32.64 bits, incrément 2^64/100e6 (module `rtl/src/clock.rs`, vérifié en simulation).
