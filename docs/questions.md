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
