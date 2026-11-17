# Questions et demandes ouvertes

## T1.11 (top) — deux demandes de modification d'unités terminées

1. **`icmp_echo` et `icmpv6` : valeur locale `rst`.** Les deux unités écrivent `let rst = end_in | end_out;`.
   TxHDL la traduit en `wire rst;` dans un module dont l'entrée de reset s'appelle aussi `rst` :
   redéclaration refusée par Vivado (yosys signale deux pilotes sur le même fil). Le Verilog autonome
   de ces unités (`out/icmp_echo.v`, `out/icmpv6.v`) a le même défaut. Contournement en place :
   `ntp_fpga::top::fix_rst_clash` (appelée par `top_gen`) renomme le fil local en `rst_l`, de façon
   vérifiée (panique sur tout autre usage). **Demande :** renommer la variable (par ex. `restart`) dans
   les deux unités, puis retirer `fix_rst_clash` (elle panique dès qu'il n'y a plus exactement deux
   modules à corriger). À signaler aussi en amont à TxHDL (un nom local égal à `rst`/`clk` devrait être
   échappé).
2. **`mii_rx` / `mii_tx` figés sur `ClkRx` / `ClkTx`.** En simulation, ces types d'horloge ont la période
   par défaut (2 pas), celle de `clk_sys` : le top complet ne peut donc pas être simulé avec des horloges
   MII à 25 MHz (4 périodes de `clk_sys`). Contournement : le top est découpé en `ArtyTop` (unités MII,
   `phy_init`) et `ArtyCore<R, T, ..>` (générique en horloges) ; les tests à 25 MHz utilisent des modèles
   exacts de `mii_rx`/`mii_tx` (`rtl/tests/top.rs`), les tests sur broches tournent avec le MII au rythme
   de `clk_sys`. **Demande :** rendre `MiiRx<C: Clock>` et `MiiTx<C: Clock>` génériques (comme
   `RxPath<R, S>`, avec `pub type MiiRxPhy = MiiRx<ClkRx>`), ce qui permettrait des tests sur broches à
   25 MHz.
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
