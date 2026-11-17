# glaucide-fpga-stratum2

Serveur NTP **stratum 2**, **IPv4 + IPv6**, entièrement en logique FPGA, sur **Digilent Arty A7-100T** (xc7a100tcsg324-1, PHY TI DP83848J en MII 100 Mb/s).

- Langage : [TxHDL](https://github.com/filmil/hdl-txhdl) (HDL transactionnel embarqué en Rust, sortie Verilog), épinglé au commit de `tools/setup.sh`.
- Le FPGA est client NTP de serveurs stratum 1 (amont) et sert ses clients avec des timestamps matériels au niveau MAC.

Voir [docs/architecture.md](docs/architecture.md).

## Démarrage

```sh
tools/setup.sh                              # clone TxHDL au commit épinglé
cargo run -p ntp_fpga --bin clock_sim       # simule l'horloge NTP (1 ms)
tools/gen_verilog.sh                        # génère out/*.v pour Vivado
```

Cargo sert à simuler et générer le Verilog rapidement. TxHDL amont utilise Bazel (9.2.0) ; la chaîne Bazel complète n'a pas pu être testée ici (dépendance `filmil/bazel-rules-bid` bloquée par le proxy du conteneur).

## Arborescence

| Dossier | Contenu |
|---|---|
| `rtl/` | crate Rust `ntp_fpga` : modules TxHDL et binaires de simulation/génération |
| `constraints/` | XDC Arty A7-100 |
| `tools/` | setup TxHDL, manifestes Cargo, génération Verilog |
| `docs/` | architecture, plan |

Note : l'exemple Ethernet amont (`eth/`, `txhdl_parts::eth`) vise une carte Alinx en RGMII 1 Gb/s ; l'Arty est en MII 100 Mb/s, donc la couche PHY est à écrire ici.
