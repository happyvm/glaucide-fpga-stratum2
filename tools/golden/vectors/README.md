# Vecteurs de test (format)

Fichiers `*.hex` générés par `python3 -m golden.vectors` (depuis `tools/`), ne pas éditer à la main.
Régénération identique octet par octet (graine fixe 20261117), vérifiée par les tests.

## Format

Une ligne par trame : `<direction> <octets en hexadécimal>` (minuscules, sans espaces ni `0x`).

- `rx` : trame reçue par l'équipement ;
- `tx` : trame que l'équipement doit émettre en réponse à la dernière trame `rx` qui précède.

Les lignes vides et les lignes commençant par `#` sont ignorées. Aucune ligne `tx` après un `rx` :
l'équipement ne doit rien émettre. Les trames se jouent dans l'ordre du fichier.

Convention de trame (`rtl/src/stream.rs`) : trames Ethernet **sans FCS**, de l'adresse MAC
destination à la fin de la charge utile.

- `rx` : complétées par des zéros jusqu'à 60 octets (ce que le MAC livre après retrait du FCS),
  sauf le scénario `neg_runt` (13 octets) ;
- `tx` : sortie du constructeur Ethernet, **sans bourrage** (`eth_tx` complète à 60 et ajoute le FCS).

## Configuration de l'équipement pour tous les scénarios (sauf mention)

| Champ | Valeur |
|---|---|
| MAC locale | `02:00:00:00:00:01` |
| IPv4 | `192.0.2.1` |
| IPv6 link-local | `fe80::ff:fe00:1` |
| IPv6 globale | `2001:db8::1` |
| Stratum / synchronisé | 2 / oui (`ntp_unsynchronised` : non, donc LI=3 et stratum 16) |
| Reference ID IPv4 | `c0000263` (192.0.2.99) |
| Reference ID IPv6 | `b70a6091` (MD5 de 2001:db8::99, 4 premiers octets) |
| Precision | -24 |
| Root delay / dispersion (16.16) | `00000333` / `00000666` |
| Reference timestamp | `ecc58cc000000000` |
| Receive timestamp (T2) | `ecc58cc040000000` |
| Transmit timestamp (T3) | `ecc58cc040014f8b` |

Client : MAC `02:00:00:00:00:02`, `192.0.2.2`, `2001:db8::2`, `fe80::ff:fe00:2`, port source UDP 40000.

## Scénarios

| Fichier | Contenu |
|---|---|
| `arp_request_reply.hex` | ARP who-has 192.0.2.1 -> reply (RFC 826); a request for another address gets nothing |
| `ping4.hex` | ICMP echo request to 192.0.2.1 -> echo reply (RFC 792), two sizes |
| `ping6.hex` | ICMPv6 echo request to 2001:db8::1 -> echo reply (RFC 4443) |
| `ndp_ns_na.hex` | NS for fe80::ff:fe00:1 via solicited-node multicast, and unicast NS for the global address -> NA S=1 O=1 (RFC 4861) |
| `ntp4_basic.hex` | NTPv4 client request over IPv4 -> mode 4 reply, synchronised stratum 2 |
| `ntp6_basic.hex` | NTPv4 client request over IPv6 (global address) -> mode 4 reply |
| `ntp_unsynchronised.hex` | Not synchronised: LI=3, stratum 16 |
| `ntp_version3.hex` | NTPv3 client request -> reply with version 3 |
| `neg_ipv4_checksum.hex` | IPv4 header checksum corrupted (first frame valid and answered, second frame dropped) |
| `neg_udp6_checksum.hex` | UDP checksum corrupted over IPv6 (first frame valid and answered, second frame dropped) |
| `neg_udp_port.hex` | UDP destination port 9999 instead of 123 (first frame valid and answered, second frame dropped) |
| `neg_ipv4_fragment.hex` | IPv4 first fragment (MF=1) (first frame valid and answered, second frame dropped) |
| `neg_ipv4_ttl0.hex` | IPv4 TTL 0 (first frame valid and answered, second frame dropped) |
| `neg_ndp_hoplimit.hex` | Neighbor Solicitation with hop limit 64 instead of 255 (first frame valid and answered, second frame dropped) |
| `neg_ntp_mode4.hex` | NTP packet in mode 4 (server) sent to us (first frame valid and answered, second frame dropped) |
| `neg_foreign_mac.hex` | Unicast frame addressed to another MAC (first frame valid and answered, second frame dropped) |
| `neg_runt.hex` | Runt frame of 13 bytes (shorter than an Ethernet header) (first frame valid and answered, second frame dropped) |

Les scénarios `neg_*` commencent par la trame valide correspondante (avec sa réponse `tx`) puis la
trame altérée, qui ne doit provoquer aucune émission.
