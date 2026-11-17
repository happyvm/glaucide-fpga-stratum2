# glaucide-fpga-stratum2

Serveur NTP **stratum 2**, **IPv4 + IPv6**, entièrement en logique FPGA, sur **Digilent Arty A7-100T** (xc7a100tcsg324-1, PHY TI DP83848J en MII 100 Mb/s).

- Langage : TL-Verilog (TL-X) compilé en SystemVerilog via SandPiper, glue SV là où c'est utile.
- Le FPGA est client NTP de serveurs stratum 1 (amont) et sert ses propres clients avec des timestamps matériels au niveau MAC.

Voir [docs/architecture.md](docs/architecture.md).

## Arborescence

| Dossier | Contenu |
|---|---|
| `rtl/tlv/` | sources TL-Verilog (`.tlv`) |
| `rtl/sv/` | SystemVerilog (top, wrappers MMCM/IO, code généré) |
| `constraints/` | XDC Arty A7-100 |
| `sim/` | bancs de test (cocotb/Verilator) |
| `tools/` | scripts (compilation SandPiper, golden model Python) |
| `docs/` | architecture, plan |
