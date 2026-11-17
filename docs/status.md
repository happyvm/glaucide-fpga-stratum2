# État du projet

Serveur NTP **stratum 2, IPv4 + IPv6**, entièrement en logique, sur Arty A7-100T, écrit en TxHDL (Rust → Verilog).

## Ce qui existe

**Une carte serveur complète** (`ArtyBoard`, 45 modules) et **une carte stratum 2 complète** (`ArtyBoardWithClient`, 52 modules), générées par `cargo run -p ntp_fpga --bin top_gen [-- --with-client]` dans `out/top/` (un fichier de netlist + wrapper de carte + liste de fichiers). Les deux sont simulées au cycle près, sur des broches MII simulées à 25 MHz réels, avec les trames des vecteurs de référence.

| Domaine | Modules |
|---|---|
| MAC / PHY | `mii_rx`, `mii_tx`, `rx_path` (passage d'horloge + horodatage SFD), `tx_path` (stockage de trame + horodatage), `fcs_check`, `crc32`, `phy_init` (reset + MDIO) |
| Ethernet / ARP / voisins | `eth_rx`, `eth_tx`, `tx_arb`, `arp` (avec expiration), `nbr_cache`, NS/NA dans `icmpv6` |
| IPv4 | `ipv4_rx`, `ipv4_tx`, `icmp_echo`, `udp4_rx`, `udp4_tx` |
| IPv6 | `ipv6_rx`, `ipv6_tx`, `icmpv6`, `udp6_rx`, `udp6_tx` |
| NTP serveur | `ntp_server` (réponses, limiteur de débit, KoD), `ntp_demux`, `ntp_reply_route` |
| NTP client / synchro | `ntp_client`, `clock_filter`, `selection`, `servo`, `sync_state`, `refid` |
| Horloge, config, top | `clock` (`DisciplinedClock`), `config` (statique, sans DHCP), `top`, `top_client`, `ip_arb`, `udp_arb` |

**Vérification**
- ~160 000 lignes de test et de code Rust/Python : 38 suites de tests Rust, 59 + 81 tests Python (modèles de référence : `tools/golden/` pour le protocole, `tools/golden/ntpclient/` pour filtre/sélection/servo/client).
- Preuves formelles (SymbiYosys) : voir `docs/formal-report.md`.
- Estimation de ressources : `docs/resources.md` (≈ 15 % des LUT pour le serveur ; la carte avec client compte environ 29 000 bascules sur 126 800).

## Mesuré en simulation
- Horodatage d'émission (T3) contre l'instant SFD réel sur les broches : **+5 à +10 ns** avec les vraies unités MII à 25 MHz (8 phases) ; réception (T2) : 0 à +5 ns. Hors latence propre du PHY, à mesurer sur carte.
- Servo (modèle fermé, poll 64 s, bruit 100 µs) : décalage de 50 ms absorbé sans saut, de 500 ms par un saut, dérive de 50 ppm apprise à 0,3 ppm, résiduel < 1 ms.
- Synchronisation de bout en bout : après 4 réponses d'amont, le servo saute l'horloge réelle du montant commandé, et le serveur répond en stratum amont + 1, LI 0.
- Débit : une requête de 48 octets traitée en 98 cycles, contre ≈ 917 disponibles au débit ligne.

## Hypothèses et limites connues
- **Configuration statique uniquement** (pas de DHCP/BOOTP/DHCPv6), adresse IPv6 globale statique, pas de SLAAC.
- Pas d'authentification NTP / NTS, pas de mode symétrique ni broadcast. Pas de propagation de seconde intercalaire (LI = 0 si synchronisé).
- Les caches ARP et voisins ne vieillissent pas (`flush` possible).
- Le client suppose un chemin d'émission libre pour prédire T1 (corrigé par l'étalement des créneaux).
- **Rien n'a tourné sur la carte ni dans Vivado** : le script `tools/vivado/build.tcl`, le timing à 100 MHz, les croisements d'horloge (BUFG/BUFGCE sur les horloges du PHY), le front de capture TX du PHY et la latence du PHY ne sont pas vérifiés.

## Reste à faire
1. **Sur machine avec Vivado** : `tools/vivado/README.md` ; revue de timing (T5.3). Points chauds : graine de checksum IPv6 (chaîne d'additions), multiplieurs de `ntp_server`/`selection`/`servo`.
2. **Mise en service** (T5.4) : mesures contre chrony/ntpd, carte contre carte, latence PHY à ajouter aux biais.
3. **Revue de sécurité** (T5.5) : réflexion/amplification (réponse ≤ requête, limiteur par source), usurpation.
4. **Preuves formelles restantes** : `docs/formal-report.md`.
5. Documentation utilisateur finale (T5.6).
