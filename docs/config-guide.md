# Guide de configuration

Le serveur n'a ni CPU ni bus de registres : toute la configuration est **fixée à la synthèse** par un type
Rust qui implémente le trait `ConfigSet` (`rtl/src/config.rs`). Changer une adresse demande de
régénérer le Verilog et de refaire le bitstream. Il n'y a pas de DHCP, BOOTP ni DHCPv6, et pas de SLAAC
(décision du propriétaire, [architecture.md](architecture.md)).

Les exemples de ce guide ont été compilés : un jeu `LabNet` (section 6) a été ajouté dans une copie du dépôt,
sélectionné dans `top.rs` comme décrit en section 3, et `top_gen` (avec et sans `--with-client`) a produit le
Verilog attendu (adresses du jeu dans les fils `local_mac`, `local_ip4`, `local_gua`). Une violation du
contrôle des 24 bits (section 4) a produit l'erreur de compilation citée. Rien n'a été essayé sur une carte.

## 1. Les constantes de `ConfigSet`

| Constante | Type | Sens | Contraintes |
|---|---|---|---|
| `LOCAL_MAC` | `u64` | adresse MAC de la carte, octet 0 (le premier sur le fil) dans l'octet de poids fort, p. ex. `0x02_4E_54_50_00_01` | 48 bits, **unicast** (bit 0 de l'octet 0 à zéro) : vérifié à la compilation. Choisissez une adresse administrée localement (bit 1 de l'octet 0 à un, p. ex. `02:...`) |
| `LOCAL_IP4` | `u32` | adresse IPv4, premier octet en poids fort (`0xC000_020A` = 192.0.2.10) | non vérifiée |
| `GATEWAY_IP4` | `u32` | passerelle IPv4 (résolue par ARP) | non vérifiée ; sert aux destinations hors du masque |
| `NETMASK4` | `u32` | masque IPv4 (`0xFFFF_FF00` = /24) | non vérifié (continuité des bits à 1 non contrôlée) |
| `GATEWAY_IP6` | `u128` | passerelle IPv6 : routeur, en pratique son adresse lien-local (`fe80::1`) ou globale | ni multicast (`ff00::/8`) ni `::` : vérifié à la compilation |
| `LOCAL_GUA` | `u128` | adresse IPv6 globale **statique** | ses **24 bits de poids faible = ceux de l'adresse lien-local** : vérifié à la compilation (voir 2) |
| `GUA_ENABLED` | `bool` | l'adresse globale est utilisée | si faux, la carte n'a que son adresse lien-local |
| `UPSTREAM` | `[(bool, u128, bool); 4]` | quatre emplacements d'amont : (est IPv6, adresse, activé). Une adresse IPv4 occupe les 32 bits de poids faible | pas de nom DNS ; un emplacement désactivé (`(false, 0, false)`) est ignoré |
| `POLL_LOG2` | `u8` | intervalle d'interrogation en puissance de deux secondes (6 = 64 s) | 4 à 10 : vérifié à la compilation |
| `PRECISION_LOG2` | `i8` | champ *precision* des réponses (-24 = environ 60 ns) | non vérifié |
| `RATE_LIMIT_PER_S` | `u16` | réponses autorisées par seconde et par source | 0 = illimité |
| `RATE_SECRET` | `u64` | clé secrète du hachage du limiteur de débit (qui décide de la case de chaque source) | **choisir une valeur aléatoire par déploiement** (`python3 -c 'import secrets; print(hex(secrets.randbits(64)))'`), ne pas la publier ; 0 déconseillé ; les valeurs de `Board` et `Test` sont publiques |

### Ce que `config.rs` en déduit

- **Adresse lien-local IPv6** : non configurable, dérivée de `LOCAL_MAC` par l'EUI-64 modifié (`ff:fe`
  inséré au milieu, bit universel/local inversé) sous `fe80::/64`. Pour `02:4e:54:50:00:01` : `fe80::4e:54ff:fe50:1`.
- **Groupe solicited-node** : `ff02::1:ff` suivi des 24 bits de poids faible de l'adresse lien-local
  (`ff02::1:ff50:1` pour la carte par défaut). La carte répond aux NS pour son adresse lien-local et, si
  `GUA_ENABLED`, pour son adresse globale.
- **Reference ID annoncé quand la carte est synchronisée** : l'adresse IPv4 de l'amont élu, ou les quatre
  premiers octets du MD5 de son adresse IPv6, calculés à la synthèse (`top::peer_ref`).

### Contrainte sur les 24 bits de l'adresse globale

`ipv6_rx` n'a qu'un seul `sn_low` pour les deux groupes solicited-node. Les deux adresses doivent donc
avoir les mêmes 24 bits de poids faible, soit **les trois derniers octets de la MAC** (ceux de l'EUI-64).
Seuls ces 24 bits sont contraints : le reste de l'adresse globale est libre, et n'a pas besoin d'être
l'EUI-64 complet. Exemple : MAC `02:4e:54:50:00:01` impose une adresse globale se terminant par `50:1`
(`...:50:1` = `0x500001`), comme `2001:db8:0:1::50:1` du jeu `Board`. Avec une MAC se terminant par `:00:02`, c'est
`...:50:2`.

### Règles de routage des amonts (`ntp_client.rs`)

- **IPv4** : l'amont est joint directement si `(adresse & NETMASK4) == (LOCAL_IP4 & NETMASK4)`, sinon via
  `GATEWAY_IP4`. La carte résout ce saut par ARP (en l'attendant jusqu'à 8 essais environ).
- **IPv6** : l'amont est joint directement s'il est dans `fe80::/10`, ou si `GUA_ENABLED` et que ses 64 bits de
  poids fort sont ceux de `LOCAL_GUA` (le préfixe est supposé /64) ; sinon via `GATEWAY_IP6`.
  L'adresse source est la lien-local pour un amont `fe80::/10`, sinon l'adresse globale si activée.
  **Piège avec `Board`** : l'amont `2001:db8::1` n'a pas le même /64 que `2001:db8:0:1::50:1` ; la requête part
  donc vers `fe80::1` (la passerelle), pas directement.
- **Réponses** : un amont n'est reconnu que si l'adresse source de la réponse est exactement son adresse
  configurée et le port source 123.

## 2. Valeurs par défaut

Deux jeux existent dans `config.rs` :

| Constante | `Board` (cartes `ArtyBoard` et `ArtyBoardWithClient`) | `Test` (tests, modèle Python) |
|---|---|---|
| `LOCAL_MAC` | `02:4e:54:50:00:01` | `02:00:00:00:00:01` |
| `LOCAL_IP4` | `192.0.2.10` | `192.0.2.1` |
| `GATEWAY_IP4`, `NETMASK4` | `192.0.2.254`, `255.255.255.0` | idem |
| `GATEWAY_IP6` | `fe80::1` | `fe80::1` |
| `LOCAL_GUA`, `GUA_ENABLED` | `2001:db8:0:1::50:1`, oui | `2001:db8::1`, oui |
| `UPSTREAM` | `192.0.2.1` (IPv4), `2001:db8::1` (IPv6), deux emplacements inactifs | `192.0.2.99`, `2001:db8::99` |
| `POLL_LOG2` | 6 | 6 |
| `PRECISION_LOG2` | -24 | -24 |
| `RATE_LIMIT_PER_S` | 8 | 8 |

Les amonts de `Board` sont des **adresses de documentation fictives** : ils doivent être remplacés avant tout
usage réel. Ne modifiez pas `Test` : les tests et les vecteurs de référence en dépendent.

## 3. Choisir le jeu utilisé par la carte

Il n'y a pas d'option de ligne de commande : le jeu est un paramètre de type des deux alias de carte de
`rtl/src/top.rs`, et `top_gen` (`rtl/src/bin/top_gen.rs`) construit ces alias.

1. Ajoutez votre jeu à la suite de `rtl/src/config.rs` (copie de `Board` : section 5) et son contrôle :
   `const _: () = Checks::<MonReseau>::OK;`.
2. Dans `rtl/src/top.rs`, remplacez `crate::config::Board` par votre type à **deux endroits** :

```rust
// carte stratum 2 (aux environs de la ligne 634)
pub type ArtyBoardWithClient = ArtyTopOf<
    crate::phy_init::Board,                 // temporisations du PHY : NE PAS toucher
    crate::top_client::ArtyCoreClient<
        ClkRx,
        ClkTx,
        crate::config::MonReseau,           // était crate::config::Board
        crate::clock::DisciplinedClock,
    >,
>;

// carte serveur seul (aux environs de la ligne 1137)
pub type ArtyBoard = ArtyTop<crate::config::MonReseau, Standalone, crate::phy_init::Board>;
```

3. Régénérez : `cargo run -p ntp_fpga --bin top_gen` (et `-- --with-client`), puis refaites le bitstream.
   `top_gen.rs` n'est pas modifié. Vérifiez que `out/top/arty_top.v` contient vos adresses (les fils
   `local_mac`, `local_ip4`, `local_ll`, `local_gua` du module de configuration sont des constantes binaires).

**Ne confondez pas** `crate::phy_init::Board` (temporisations du PHY : reset, attente, MDC) avec
`crate::config::Board` (adresses). Seul le second se remplace.

N'éditez pas `Board` en place : `rtl/tests/config.rs` (`board_wires_carry_the_board_constants`, etc.) compare
ses constantes à des valeurs attendues et échouerait. Un jeu séparé laisse la suite de tests intacte. Les
tests du top (`top.rs`, `top_client.rs`) utilisent leurs propres jeux (`Test`, `CliCfg`) et ne sont pas
touchés par ce changement.

## 4. Contrôles à la compilation

Ils s'exécutent quand le jeu est utilisé (`Checks::<C>::OK`, forcé par `Config::default()`) et, pour
`Board`, `Test` (et votre jeu si vous ajoutez la ligne `const _`), à la compilation du crate :

| Condition | Message de l'erreur |
|---|---|
| `GATEWAY_IP6` multicast ou `::` | `GATEWAY_IP6 must not be multicast (ff00::/8) nor unspecified (::)` |
| `LOCAL_MAC` non unicast, ou qui dépasse 48 bits | `LOCAL_MAC must be a 48-bit unicast address` |
| 24 bits de poids faible de `LOCAL_GUA` différents de ceux de l'adresse lien-local | `LOCAL_GUA and the link-local address must share their low 24 bits (one sn_low)` |
| `POLL_LOG2` hors de 4..=10 | `POLL_LOG2 must be in 4..=10` |

Le compilateur les affiche sous la forme `error[E0080]: evaluation panicked: <message>` (le troisième a
été essayé, avec une adresse globale se terminant par `50:3` pour une MAC se terminant par `50:00:01`).

Le contrôle de `LOCAL_MAC` est aussi une garantie formelle (`check!` « local_mac is a unicast address »)
dont dépend la preuve de `eth_rx` ([formal.md](formal.md)).

**Ce qui n'est pas contrôlé** : cohérence de `LOCAL_IP4` avec `GATEWAY_IP4` et `NETMASK4`, continuité du
masque, adresses multicast ou nulles en IPv4, adresse d'amont égale à la nôtre (une carte ne peut pas être
son propre amont), doublons dans `UPSTREAM`, valeur de `PRECISION_LOG2`, adresse d'amont IPv6 multicast.

## 5. Écrire son propre jeu

Copiez `Board` (toutes les constantes sont obligatoires, le trait n'a pas de valeur par défaut) :

```rust
/// Mon réseau.
#[derive(Default)]
pub struct MonReseau;
impl ConfigSet for MonReseau {
    const LOCAL_MAC: u64 = 0x02_4E_54_50_00_02;
    const LOCAL_IP4: u32 = 0xC000_0214; // 192.0.2.20
    const GATEWAY_IP4: u32 = 0xC000_0201; // 192.0.2.1
    const NETMASK4: u32 = 0xFFFF_FF00; // /24
    const GATEWAY_IP6: u128 = 0xFE80_0000_0000_0000_0000_0000_0000_0001; // fe80::1
    const LOCAL_GUA: u128 = 0x2001_0DB8_0000_0000_0000_0000_0050_0002; // 2001:db8::50:2
    const GUA_ENABLED: bool = true;
    const UPSTREAM: [(bool, u128, bool); 4] = [
        (false, 0xC000_027B, true), // 192.0.2.123
        (true, 0x2001_0DB8_0000_0000_0000_0000_0000_0123, true), // 2001:db8::123
        (false, 0, false),
        (false, 0, false),
    ];
    const POLL_LOG2: u8 = 6;
    const PRECISION_LOG2: i8 = -24;
    const RATE_LIMIT_PER_S: u16 = 8;
    const RATE_SECRET: u64 = 0x0123_4567_89AB_CDEF; // à remplacer par une valeur aléatoire
}
const _: () = Checks::<MonReseau>::OK;
```

Conversions : une adresse IPv4 `a.b.c.d` s'écrit `0xAABB_CCDD` (`192.0.2.20` = `0xC000_0214`) ; une adresse
IPv6 s'écrit en huit groupes de 16 bits (`2001:db8::50:2` = `0x2001_0DB8_0000_0000_0000_0000_0050_0002`).
`Checks` est public dans `config.rs`, donc la ligne `const _` marche depuis ce fichier.

## 6. Exemple complet : réseau de laboratoire 192.0.2.0/24 et 2001:db8::/64, deux amonts

C'est le jeu `MonReseau` ci-dessus (nom d'essai : `LabNet`). Réseau :

| Élément | Valeur |
|---|---|
| Carte | MAC `02:4e:54:50:00:02`, IPv4 `192.0.2.20/24`, IPv6 globale `2001:db8::50:2/64`, lien-local `fe80::4e:54ff:fe50:2` |
| Routeur | `192.0.2.1`, lien-local `fe80::1` |
| Amont 0 | `192.0.2.123` (IPv4, même /24 : joint directement) |
| Amont 1 | `2001:db8::123` (IPv6, même /64 que la carte : joint directement) |

Les deux amonts doivent être des serveurs de **stratum 1** (le client rejette les stratums 0 et 16, et
l'état de synchronisation n'accepte que 1 à 14 ; un amont de stratum 2 est accepté mais la carte annoncerait
alors 3). Pour la mise en service, voir [bringup.md](bringup.md).

## 7. Ce qui ne se configure pas

| Sujet | Situation |
|---|---|
| DHCP, BOOTP, DHCPv6, SLAAC | absents par décision (trames DHCP abandonnées) ; tout est statique |
| Décalage de stratum | fixe : annoncé = stratum de l'amont + 1 (`STRATUM_OFFSET` a été supprimé) ; pas de stratum forcé |
| KoD (`RATE`) | non configurable par `ConfigSet` : l'entrée `kod_enable` du serveur est reliée à 0 (`tie(Bit::Zero)` dans `top.rs` et `top_client.rs`, après `rate_limit`). Une requête limitée reste **sans réponse** |
| Nom DNS d'amont | non (adresses seulement) |
| Nombre d'amonts | 4 emplacements, fixe |
| Adresse MDIO du PHY | fixée à 1 (strap de l'Arty) dans le top |
| Temporisations du PHY | `phy_init::Board` (reset 10 ms, attente 200 ms, sondage 100 ms), pas dans `ConfigSet` |
| Longueur du préfixe IPv6 | /64 supposé pour décider « sur le lien » |
| Authentification, NTS, mode symétrique, broadcast, mode 6/7 | non implémentés |
| Adaptation du poll | non : `POLL_LOG2` est appliqué tel quel (le servo borne 4..10) |

## 8. Pièges

1. **Amonts fictifs.** `Board` pointe sur `192.0.2.1` et `2001:db8::1`, qui n'existent pas sur votre réseau :
   la carte ne se synchronisera jamais. Remplacez-les.
2. **Vous ne pouvez pas être votre propre amont** : `Test` utilise `.99` pour cette raison. Rien ne le vérifie.
3. **Adresse globale et MAC liées** : changer la MAC change l'adresse lien-local et impose de changer les 24
   bits de poids faible de `LOCAL_GUA` (erreur de compilation sinon).
4. **Deux cartes sur le même réseau** : deux MAC distinctes, deux adresses distinctes. Aucune détection
   d'adresse en double (pas de sonde ARP).
5. **Limite de débit** : `RATE_LIMIT_PER_S = 8` par source. Fenêtres d'une seconde calées sur les secondes NTP :
   un client en `minpoll 0` (1 requête par seconde) passe, un test de charge depuis une seule adresse non.
   La limite est par adresse IPv4, par /64 en IPv6, avec une table de 4 096 mots indexée par un hachage à clé
   secrète (`RATE_SECRET`) : des sources différentes peuvent encore partager un compteur, mais pas par choix d'un
   attaquant. Au-delà de la limite, une requête sur 16 reçoit quand même une réponse (fuite, `RL_LEAK_LOG2`). 0 désactive la limite (à ne pas faire sur un réseau exposé).
6. **Résolution ARP/NS en panne** : un amont IPv4 hors du masque passe par la passerelle ; si la passerelle ne
   répond pas à l'ARP, la résolution est abandonnée après environ 0,2 s et la carte réessaie au poll suivant.
   Une seule résolution ARP à la fois : un voisin muet retarde les autres adresses (délai 0,67 s).
7. **Mauvaise précision annoncée** : `PRECISION_LOG2 = -24` suppose l'erreur d'horodatage à 40 ns de
   granularité du MII (à mesurer sur la carte, [bringup.md](bringup.md)) ; ne l'abaissez pas sans mesure.
8. **Resynthèse nécessaire** : toute modification de configuration oblige à régénérer le Verilog et à
   refaire le bitstream. Il n'y a pas de registre à écrire.
9. **Horloge à zéro au démarrage** : la carte ne conserve pas l'heure (pas de mémoire non volatile). Après chaque
   reset l'horloge repart de 0 et le client doit la recaler depuis les amonts.
