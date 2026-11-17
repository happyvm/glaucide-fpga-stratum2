#!/usr/bin/env bash
# Generate Verilog for every module into out/. Add new modules to rtl/src/bin/gen_all.rs.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p out
for m in arp selection servo udp6_rx udp6_tx clock_filter sync_state ntp_server icmpv6 udp4_rx udp4_tx ipv6_rx ipv6_tx config_board ipv4_rx ipv4_tx icmp_echo ntp_clock disciplined_clock frame_counter frame_crc csum mii_rx mii_tx eth_rx eth_tx fcs_check tx_arb tx_path rx_path phy_init; do
  cargo run -q -p ntp_fpga --bin gen_all -- "$m" > "out/$m.v"
done
ls out/*.v
