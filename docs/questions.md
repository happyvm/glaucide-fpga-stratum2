# Questions et demandes ouvertes

## Questions résolues

- **Variable locale `rst` d'`icmp_echo`/`icmpv6`** : renommée `end_all` à la source ; `top::fix_rst_clash` n'est plus qu'un garde-fou qui doit ne rien trouver. À signaler en amont à TxHDL (un nom local égal à `rst`/`clk` devrait être échappé).
- **`mii_rx`/`mii_tx` figés sur `ClkRx`/`ClkTx`** : rendus génériques (T1.15), netlists identiques, tests sur broches à 25 MHz réels.

## T4.2c — points restants

3. **T4.2c : étalement du client, délai d'expiration d'ARP, `STRATUM_OFFSET` retiré.** (a) `NtpClient`
   étale les créneaux : le créneau `i` démarre son iburst à `t0 + i/4 s` (la planification est
   désormais en ticks de 2^-6 s, et non en secondes entières) ; deux requêtes ne sont plus jamais dues
   ensemble, donc T1 (prédit pour un chemin d'émission libre) reste exact. **À confirmer :** le pas
   de 1/4 s par créneau et le départ relatif à l'armement du créneau (pas de `t0` global). (b) `Arp`
   abandonne une résolution sans réponse après `ARP_TIMEOUT_CYCLES` = 2^26 cycles (≈ 0,67 s) ; un
   voisin muet retarde donc les autres adresses d'au plus ce délai. `Arp` reste le nom du type de la
   carte (`ArpWith<Board>`) ; `Sim`/`Formal` pour les tests et la preuve. **À faire côté
   orchestrateur :** les `check!` de `arp.rs` valent toujours, mais la preuve formelle doit couvrir
   le compteur `tmo` et l'expiration (`ArpWith::<Formal>`, `formal_gen.rs` utilise encore `Arp`).
   (c) `ConfigSet::STRATUM_OFFSET` et le fil `stratum_offset` de `Config` sont supprimés (le `+ 1` est
   fixé par le protocole, voir l'en-tête de `sync_state.rs`) ; `docs/top-client-hooks.md` ligne 110
   (point 5) est à retirer.

## T1.15 — suite de la demande 2 (MII génériques)

`MiiRxG<C>` / `MiiTxG<C>` sont génériques en l'horloge (alias `MiiRx`, `MiiTx` inchangés, netlists identiques) et
`ArtyTopG<R, T, P, K>` / `ArtyTopClk` portent les horloges MII ; les tests `real_ratio_*` de `rtl/tests/top.rs`
simulent le top complet à 25 MHz réels. Mesuré avec les vraies unités sur huit phases : erreur T2 0..+5 ns, T3
+5..+10 ns (modèle : T2 0..+5 ns, T3 -10..+10 ns). Point à noter : `mii_tx` écrit `txd.set(o_txd)`, donc en
simulation la broche suit le registre avec un cycle `clk_tx` de retard (en netlist, `assign txd = o_txd`, pas de
retard) ; le banc en tient compte pour l'instant SFD.
