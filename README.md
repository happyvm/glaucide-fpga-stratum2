# glaucide-fpga-stratum2

Serveur NTP **stratum 2**, **IPv4 + IPv6**, entièrement en logique FPGA, sur **Digilent Arty A7-100T** (xc7a100tcsg324-1, PHY TI DP83848J en MII 100 Mb/s).

- Langage : [TxHDL](https://github.com/filmil/hdl-txhdl) (HDL transactionnel embarqué en Rust, sortie Verilog), épinglé au commit de `tools/setup.sh`.
- Le FPGA est client NTP de serveurs stratum 1 (amont) et sert ses clients avec des timestamps matériels au niveau MAC.

Voir [docs/architecture.md](docs/architecture.md), la [roadmap par tâches/agents](docs/roadmap.md) les [preuves formelles](docs/formal.md) et les [projets de référence](docs/references.md).

## Démarrage

```sh
tools/setup.sh                              # clone TxHDL au commit épinglé
cargo run -p ntp_fpga --bin clock_sim       # simule l'horloge NTP (1 ms)
tools/gen_verilog.sh                        # génère out/*.v pour Vivado
tools/formal/prove.sh formal_smoke prove    # preuve formelle (pip install yowasp-yosys z3-solver)
```

Cargo sert à simuler et générer le Verilog rapidement. TxHDL amont utilise Bazel (9.2.0) ; la chaîne Bazel complète n'a pas pu être testée ici (dépendance `filmil/bazel-rules-bid` bloquée par le proxy du conteneur).

## CI

Deux workflows GitHub Actions dans `.github/workflows/` :

- `ci.yml` (sur push et pull request) : `cargo build --workspace`, `cargo test -p ntp_fpga`, tests Python du modèle de référence (`python3 -m unittest discover tools/golden`), génération Verilog (artefact `verilog`). `rustfmt` et `clippy` sur `ntp_fpga` tournent dans des jobs séparés, non bloquants.
- `formal.yml` (push sur `main`, déclenchement manuel, chaque nuit) : preuve non bornée (`prove`) de chaque unité enregistrée dans `rtl/src/bin/formal_gen.rs`. Tout résultat autre que PASS (FAIL, UNKNOWN, délai dépassé) fait échouer le job. Les `cover` bornés à 40 pas sont non bloquants. Le tableau PASS/FAIL/UNKNOWN par unité est dans le résumé du run (`tools/formal/summary.sh`).

## Arborescence

| Dossier | Contenu |
|---|---|
| `rtl/` | crate Rust `ntp_fpga` : modules TxHDL et binaires de simulation/génération |
| `constraints/` | XDC Arty A7-100 |
| `tools/` | setup TxHDL, manifestes Cargo, génération Verilog |
| `docs/` | architecture, plan |

Note : l'exemple Ethernet amont (`eth/`, `txhdl_parts::eth`) vise une carte Alinx en RGMII 1 Gb/s ; l'Arty est en MII 100 Mb/s, donc la couche PHY est à écrire ici.
