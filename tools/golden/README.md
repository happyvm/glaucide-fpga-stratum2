# Modèle de référence Python (tâche T0.3)

Modèle exécutable des protocoles et du **comportement attendu du serveur** : les tests Rust et les
preuves de `docs/formal.md` se comparent à lui. Bibliothèque standard Python 3 uniquement
(`scapy` est utilisé seulement par un test facultatif de recoupement, ignoré s'il est absent).

## Contenu

| Module | Rôle |
|---|---|
| `csum.py` | Checksum Internet RFC 1071, checksum d'en-tête IPv4, pseudo-en-têtes IPv4/IPv6, CRC-32 / FCS Ethernet |
| `eth.py`, `arp.py`, `ipv4.py`, `ipv6.py`, `icmp.py`, `icmpv6.py`, `udp.py` | Construction et analyse de chaque couche (echo, NDP NS/NA, analyse des RA) |
| `ntp.py` | Paquet NTPv4 de 48 octets (RFC 5905 §7.3), conversions secondes <-> 32.32 / 16.16, Reference ID |
| `responder.py` | `respond(trame, config) -> [trames]` : le serveur de référence |
| `vectors.py` | Générateur déterministe de `vectors/*.hex` (format dans `vectors/README.md`) |
| `tests/` | Tests `unittest` |

Toute analyse (`parse`) lève `golden.errors.ParseError` sur entrée malformée (mauvais checksum,
longueurs incohérentes, version incorrecte...).

## Utilisation

```sh
python3 -m unittest discover tools/golden          # tous les tests (depuis la racine du dépôt)
cd tools && python3 -m golden.vectors              # régénère tools/golden/vectors/
```

```python
from golden import vectors, responder
frames_out = responder.respond(trame_rx, vectors.CONFIG)   # trame sans FCS
```

Conventions : trames Ethernet **sans FCS** (comme `rtl/src/stream.rs` au-dessus du MAC). Les trames
reçues peuvent contenir du bourrage Ethernet (jusqu'à 60 octets) : les couches supérieures se
limitent à leur propre champ de longueur. Les trames émises ne sont **pas** complétées (c'est le
rôle de `eth_tx`). Les adresses utilisées sont celles de la documentation (`192.0.2.0/24`,
`2001:db8::/32`, `fe80::/64`, MAC `02:00:00:00:00:xx`).

## Règles du serveur de référence

Chaque règle est repérée dans `responder.py` par son identifiant (commentaire avec référence RFC).
Tout ce qui n'est pas explicitement accepté ne produit aucune trame.

- **Ethernet** : E1 adresse source de groupe rejetée ; E2 EtherType autre qu'ARP/IPv4/IPv6 ignoré ; trame < 14 octets ignorée.
- **ARP (RFC 826)** : A1 destination broadcast ou notre MAC ; A2 Ethernet/IPv4 (`htype=1`, `ptype=0x0800`, `hlen=6`, `plen=4`) ; A3 seulement les requêtes pour notre IPv4 ; A4 émetteur unicast ; A5 réponse avec adresses permutées.
- **IPv4 (RFC 791)** : I0 MAC destination = la nôtre (pas de réponse au broadcast) ; I1 version 4, longueur totale cohérente, checksum d'en-tête ; I2 options (IHL != 5) rejetées ; I3 fragments rejetés ; I4 TTL 0 rejeté ; I5 destination = notre IPv4 ; I6 source nulle/multicast/broadcast rejetée. Réponses : TTL 64, DF, identifiant 0.
- **ICMP (RFC 792)** : J0 checksum ; J1 seulement echo request code 0 ; J2 réponse type 0, mêmes identifiant, numéro de séquence et données.
- **IPv6 (RFC 8200)** : G1 version 6 et longueur de charge utile cohérente ; G2 en-têtes d'extension et protocoles autres que UDP/ICMPv6 rejetés ; G3 hop limit 0 rejeté ; G4 source multicast rejetée ; G5 unicast vers notre MAC ; G6 multicast sollicité-nœud avec la MAC `33:33:ff:xx:xx:xx` correspondante ; G7 destination non à nous rejetée ; G8 NTP uniquement en unicast.
- **ICMPv6 (RFC 4443, 4861)** : K1 echo seulement vers une de nos adresses unicast ; K2 réponse type 129 (mêmes id/seq/données) ; M1 NS : hop limit exactement 255 ; M2 cible = une de nos adresses ; M3 destination = cible ou son multicast sollicité-nœud ; M4 DAD (source `::`) : destination sollicité-nœud, sans option SLLA ; M5 réponse DAD vers `ff02::1`, S=0 ; M6 NA sollicité : R=0, S=1, O=1, option adresse cible = notre MAC, hop limit 255 ; M7 RA/NA/autres types : rien émis (les RA sont seulement analysées par `icmpv6.py`).
- **UDP (RFC 768, RFC 8200 §8.1)** : U1 longueur cohérente, checksum correct (0 accepté en IPv4 seulement, interdit en IPv6) ; U2 port destination 123 ; U3 ports permutés, checksum recalculé.
- **NTP (RFC 5905)** : N1 au moins 48 octets (extensions tolérées en entrée) ; N2 mode 3 seulement ; N3 version 3 ou 4 ; N4 LI=3 si non synchronisé, sinon 0 ; N5 version de la réponse = min(version requête, 4) ; N6 stratum de la config, 16 si non synchronisé ; N7 poll copié de la requête ; N8 Reference ID selon la famille (`ref_id` en IPv4, `ref_id_v6` en IPv6) ; N9 Origin = Transmit de la requête ; N10 réponse de 48 octets exactement ; Receive/Transmit fournis par la config (`recv_ts`, `xmit_ts`).

## Choix à confirmer

- Pas de table ARP : la MAC de destination d'une réponse est la MAC source de la requête.
- Reference ID : fourni par la config, sans calcul (`ntp.refid_ipv4` / `ntp.refid_ipv6` fournissent les valeurs de RFC 5905 pour un amont de stratum >= 2). Non synchronisé : la config conserve son Reference ID.
- Réponse DAD : envoyée vers `ff02::1` (RFC 4861 §7.2.4) ; le NA solicité inclut toujours l'option adresse cible.
- Les paquets NTP de plus de 48 octets (extensions) sont acceptés et reçoivent une réponse de 48 octets sans extension.
