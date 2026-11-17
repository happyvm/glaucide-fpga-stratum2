# Projets de référence

**Règle** : on s'en inspire pour l'architecture, les valeurs et les méthodes de test. On ne copie pas de code : la licence de Fakernet n'est pas indiquée sur la page consultée, et celle de Netnod (BSD d'après une présentation RIPE 84) est à vérifier dans le dépôt avant toute réutilisation. Les agents lisent les pages, pas le code, sauf autorisation explicite.

## Fakernet — NTP sur Arty A7 (Håkan T. Johansson, Chalmers)
Pages : <https://fy.chalmers.se/~f96hajo/fakernet/board_Arty_A7-35-GPS-NTP.html>

Serveur NTP sans OS, VHDL, sur Arty A7-35, testé à 100 Mb/s (Arty) et 1 Gb/s. **Stratum 1** (GPS + PPS), IPv4 seulement.

À retenir :
- **Horloge** : compteur NTP avancé par un incrément fractionnaire à chaque cycle, comme notre `NtpClock`. À chaque PPS, l'écart sert à corriger l'incrément (moitié appliquée durablement, moitié jusqu'au PPS suivant). Chez nous, le servo (T4.5) joue ce rôle à partir des mesures NTP.
- **Précision annoncée** : champ *precision* = log2 d'une moyenne décroissante des écarts de timestamp, typiquement −27 (~7,5 ns). Non synchronisé après 2 s sans PPS valide : le serveur annonce précision 127 et se déclare non synchronisé.
- **Résultats mesurés** : précision d'offset carte à carte < 20 ns ; contre chrony sur PC, écart-type 151 ns. Méthode de test réutilisable (T5.4).
- **Limites qu'ils notent** : sensibilité de l'oscillateur à la température, mise à jour du servo à 1 Hz (lente), débit de 50 000 paquets/s « à vérifier », et **risque de réflexion** (source usurpée pour viser un tiers).
- Absents : IPv6, et la page ne précise pas le point de timestamp ni le PHY.

## Netnod / Assured — FPGA_NTP_SERVER (NTP + NTS matériel)
Dépôt : <https://github.com/Netnod/FPGA_NTP_SERVER> (voir aussi <https://github.com/Assured/FPGA_NTP_SERVER>)
Présentation : <https://ripe84.ripe.net/presentations/93-2022-05-20-NTS-FPGA-Presentation-RIPE-84.pdf>

Pile réseau complète en matériel, en production pour le Swedish Distributed Time Service. Cibles : VC709, VCU118, Arista 7130. Passe à l'échelle par des **moteurs réseau parallèles indépendants**. NTS (RFC 8915) en matériel. IPv6 non mentionné.

À retenir :
- Preuve que l'approche « tout matériel » tient la production ; l'architecture à moteurs parallèles est une piste pour le débit, hors v1.
- NTS : hors périmètre v1, mais l'en-tête NTP doit tolérer des champs d'extension plus tard (ne pas figer la longueur à 48 octets dans le parseur).
- La tâche T0.3 peut s'appuyer sur leur documentation de format de paquet, pas sur leur code.

## Ce que notre projet change par rapport à eux
| | Fakernet / Netnod | Ce projet |
|---|---|---|
| Stratum | 1 (GPS/PPS) | **2** (client NTP amont) |
| IPv6 | non | **oui** |
| Langage | VHDL / Verilog | TxHDL (Rust) |
| Preuves | tests réels | **preuves formelles** (`docs/formal.md`) |
| Carte | Arty A7-35 (Fakernet) | Arty A7-100 |

## Décisions tirées de ces références
1. **Débit cible = ligne 100 Mb/s pleine**, soit environ 109 000 paquets/s en IPv4 (114 octets sur le fil par échange) et 93 000 en IPv6 (134 octets). À vérifier en simulation (T2.3) ; c'est un test d'acceptation, pas une promesse.
2. **Champ *precision*** : ne pas annoncer plus que ce qu'on mesure. Le timestamp est pris sur le MII à 25 MHz (granularité 40 ns), donc valeur initiale **−24** (≈ 60 ns), à affiner par mesure sur carte (T5.4).
3. **Non synchronisé** : après un délai sans réponse valide d'un amont, annoncer LI=3 / stratum 16 (cf. politique T2.4 et état T4.6).
4. **Réflexion** : le serveur répond à la source indiquée ; on ne peut pas l'empêcher mais on borne l'effet (réponse ≤ taille de la requête, limiteur de débit par source T2.5, étudié en T5.5).
5. **Servo plus rapide que 1 Hz** : la mise à jour au rythme des réponses d'amont (poll 16–64 s, mais filtre sur 8 échantillons) reste lente ; documenter la dérive tolérée entre deux mises à jour et la compenser par la mesure de fréquence (T4.5).
