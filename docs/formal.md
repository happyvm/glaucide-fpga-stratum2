# Preuves formelles

Objectif : démontrer, pour **toutes** les entrées possibles et pas seulement pour les vecteurs de test, que le design respecte les comportements et protocoles visés.

## Ce que ça prouve, et ce que ça ne prouve pas

- Prouvé : le Verilog généré satisfait des propriétés écrites (`check!`) sous des hypothèses explicites (`assume!`).
- **Non prouvé** : que les propriétés traduisent fidèlement les RFC. Cette traduction est manuelle et relue ; chaque propriété cite sa source (RFC, section) dans son message.
- **Non prouvé** : la qualité de la synchronisation (précision, stabilité du servo). Elle se vérifie par simulation et mesures sur carte. Seules des bornes de sûreté (pas de saut non voulu, pas de dépassement) sont prouvables.
- Une preuve avec une hypothèse trop forte est vide de sens : voir « Règles anti-vacuité ».

## Outils (vérifiés dans ce dépôt)

TxHDL fournit trois macros dans une unité `#[lower]` : `check!(cond, "msg")`, `assume!(cond, "msg")`, `cover!(cond, "msg")`. Elles sont aussi vérifiées pendant la simulation Rust, et émises dans le Verilog en `assert`/`assume`/`cover` sous `` `ifdef FORMAL `` (invisibles pour Vivado).

```sh
pip install yowasp-yosys z3-solver
tools/formal/prove.sh formal_smoke prove      # k-induction non bornée, z3
tools/formal/prove.sh <unité> bmc 40          # recherche bornée, contre-exemple en VCD
tools/formal/prove.sh <unité> cover 40        # chemin jusqu'à chaque cover!
```

Validé : `formal_smoke` donne `successful proof by k-induction (PASS)`, et une propriété volontairement fausse donne `FAIL` avec un contre-exemple au pas 6. Un résultat UNKNOWN n'est pas une preuve.

Pour qu'une unité soit prouvable, il faut la déclarer dans `rtl/src/bin/formal_gen.rs`.

## Décision : où écrire les propriétés

**Les propriétés s'écrivent en Rust (`check!`/`assume!`/`cover!` dans l'unité) et se prouvent sur le Verilog généré.** Jamais à la main dans le Verilog ou le VHDL : ces fichiers sont regénérés et les modifications seraient perdues.

- En simulation Rust, `check!`/`assume!` sont contrôlés à chaque cycle sur les entrées simulées : détection, pas preuve.
- La preuve (SymbiYosys + z3) porte sur le Verilog émis avec `` `ifdef FORMAL ``. Le VHDL émis par TxHDL ne sert qu'à la simulation (nvc) dans ce flux.
- **Point de confiance 1** : la preuve porte sur le netlist, pas sur le Rust ; on fait confiance à la génération de TxHDL. Atténuation : comparer simulation Rust et simulation du netlist sur les mêmes vecteurs (tâche à ajouter dans F0.4).
- **Point de confiance 2** : les modèles de référence d'équivalence (CRC, checksum) sont des unités TxHDL simples et lentes, écrites en Rust ; on prouve que la version rapide leur est égale (F1.1, F2.1). Leur exactitude se valide contre les vecteurs normatifs (`123456789` → `0xCBF43926`, RFC 1071).
- Non retenu : preuve directe du Rust (type Kani), car la sémantique prouvée serait celle du Rust et non du circuit ; fichiers SystemVerilog externes liés par `bind`, remplacés par les moniteurs Rust (F0.5).

## Méthode : garantie/hypothèse

1. **Chaque module prouve ses garanties** (`check!`) sous des hypothèses sur ses entrées (`assume!`).
2. **Les hypothèses d'un module sont les garanties du module en amont.** Pour chaque `assume!` d'un module, il existe un `check!` de même texte dans le producteur, ou un moniteur de flux qui le prouve. `tools/formal/` contiendra un script qui vérifie cette correspondance par le texte des messages (tâche F0.3).
3. **Moniteurs** : pour les propriétés de protocole sur une interface, une unité d'observation (`rtl/src/monitors/`) porte les `check!` et se branche sur n'importe quelle interface `stream`. Un même moniteur sert comme `assume` côté consommateur et `assert` côté producteur.
4. **Intégration** : le top formel compose les modules réels, avec uniquement des hypothèses sur l'environnement externe (PHY, trames d'entrée), et prouve les propriétés bout en bout.

## Règles anti-vacuité

- Toute propriété importante a un `cover!` du cas qu'elle protège (ex. : « une requête NTP valide est acceptée », « un paquet est jeté pour mauvais checksum »). `prove.sh … cover` doit atteindre tous les `cover!`.
- Toute unité a un test de mutation : un fichier de mutations connues (par exemple inverser un bit de champ) doit faire **échouer** au moins une propriété. Procédure type : `bad.v` dans la validation de `formal_smoke`.
- **Registre des hypothèses de configuration** (garant = module de configuration T4.1, qui doit les prouver par `check!` de même texte, vérifié par F0.3) :
  - `local_mac` est une adresse unicast (bit de groupe à 0) — utilisé par `eth_rx` (T1.5).
- Les `assume!` sont limités à : (a) la forme des flux d'entrée définis dans `stream.rs`, (b) les garanties d'un module amont prouvées, (c) l'environnement physique documenté. Tout autre `assume!` est refusé en revue.

## Catalogue des propriétés par couche

| Couche | Source | Propriétés à prouver |
|---|---|---|
| Flux d'octets | contrat `stream.rs` | `last` précédé d'au moins un octet valide ; pas de `valid` après `last` avant nouveau début ; `err` ne reste pas sans `last` ; longueur ≤ 1518 |
| MII RX | IEEE 802.3 cl. 22 | une trame n'est émise qu'après 7 octets de préambule `0x55` + SFD `0xD5` ; les octets émis sont exactement ceux reçus ; `rx_er` ⇒ trame marquée en erreur |
| MII TX | IEEE 802.3 | préambule/SFD corrects ; IFG ≥ 96 bits (24 cycles MII) entre trames ; pas de glitch de `tx_en` en cours de trame |
| CRC / FCS | IEEE 802.3 | la valeur calculée égale celle d'un modèle bit à bit de référence (équivalence) ; trame + FCS valide ⇒ résidu constant `0xC704DD7B` ; TX ajoute le bon FCS |
| Ethernet | IEEE 802.3 | trame < 60 octets rembourrée à 60 ; seules les destinations acceptées (unicast, broadcast, `33:33:ff:xx:xx:xx` de l'adresse locale) sont transmises plus haut ; FCS invalide ⇒ jetée |
| ARP | RFC 826 | réponse uniquement à une requête pour notre IPv4 ; champs de la réponse (`htype=1`, `ptype=0x0800`, `hlen=6`, `plen=4`, `oper=2`, adresses source/cible permutées) ; aucune émission non sollicitée hors requête de résolution |
| IPv4 | RFC 791 | checksum d'en-tête correct à l'émission ; versions ≠ 4, IHL ≠ 5, fragments, TTL=0 jetés ; longueur totale cohérente avec la trame |
| ICMP echo | RFC 792 | réponse = type 0, mêmes identifiant/numéro de séquence/données, checksum correct ; pas de réponse à un broadcast |
| UDP | RFC 768, RFC 8200 §8.1 | checksum correct en v4 (ou 0 accepté) et obligatoire en v6 ; longueur UDP cohérente ; ports source/destination permutés dans la réponse |
| IPv6 | RFC 8200 | version 6 ; hop limit ; pas de traitement des en-têtes d'extension non gérés (jetés, aucun comportement indéfini) ; adresses source/destination permutées dans la réponse |
| ICMPv6 / NDP | RFC 4443, RFC 4861 | NA seulement en réponse à un NS pour notre adresse ; hop limit = 255 exigé pour NDP ; flags S/O corrects ; checksum pseudo-en-tête correct ; DAD : pas d'adresse utilisée si conflit |
| NTP serveur | RFC 5905 §7.3, §8 | réponse (mode 4) uniquement à une requête mode 3 (ou 1 en symétrique non géré : ignorée) ; version de la réponse = min(version requête, 4) ; `Origin = Transmit` de la requête ; `Receive ≤ Transmit` ; stratum = 2 si synchronisé, 16 si non, jamais 0 en réponse serveur ; `LI=3` ⇔ non synchronisé ; réponse de 48 octets exactement ; Reference ID conforme à la famille d'adresse |
| Politique / KoD | RFC 5905 §7.4, RFC 8633 | au-delà de la limite de débit, soit silence, soit KoD `RATE` ; jamais d'amplification : taille de réponse ≤ taille de requête |
| Horloge | RFC 5905 §6 | `acc` ne revient jamais en arrière sauf par un saut explicite commandé ; l'incrément appliqué reste dans `[nominal − 500 ppm, nominal + 500 ppm]` ; absence de dépassement sur la durée de vie du design (invariant) |
| Timestamp | conception | un timestamp TX/RX est associé à une et une seule trame ; jamais réutilisé ; CDC : valeur lue stable (Gray/handshake) |
| Client NTP | RFC 5905 §8, §9 | une réponse n'est acceptée que si `Origin` = `Transmit` envoyé (anti-spoofing) ; une même réponse n'est pas comptée deux fois ; mode 4 et source attendue |
| Filtre/sélection | RFC 5905 §10–11 | filtre d'horloge : le meilleur échantillon retenu a le delay minimal ; sélection : intersection non vide requise, sinon pas de synchro ; un amont `falseticker` n'influence pas l'offset |
| Servo | conception | sorties bornées (trim ± 500 ppm) ; pas de saut si écart < seuil ; saut au plus une fois par évènement ; anti-windup : l'intégrateur reste dans ses bornes |
| État de synchro | RFC 5905 §7.3 | stratum publié = stratum de la source sélectionnée + 1 ; ≤ 15 ou 16 |
| Bout en bout | tout | sous hypothèses sur le PHY et un client conforme : toute requête valide reçue reçoit une réponse valide en ≤ N cycles (borne en BMC), et aucune réponse n'est émise pour un trafic non NTP |

## Hors périmètre de la preuve formelle
- Précision d'horloge, stabilité du PI, jitter : simulation et mesure.
- Timing physique (setup/hold, latence du PHY) : analyse Vivado, mesure sur carte.
- Respect de l'intégralité de RFC 5905 (non cible : authentification, symétrique, broadcast, multicast).
