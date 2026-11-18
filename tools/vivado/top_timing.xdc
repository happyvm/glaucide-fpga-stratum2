# Timing constraints of the board top (task T1.11), in addition to
# constraints/arty_a7_100.xdc (pins, MII I/O delays, primary clocks).
# Instance names are those of rtl/hdl/arty_top_board.v.
# Not checked with Vivado (not available where this was written).

# eth_ref_clk: 25 MHz from MMCM CLKOUT1, forwarded by the ODDR u_ref_oddr.
# It only feeds the PHY's reference input; no data is timed against it.
create_generated_clock -name eth_ref_clk_fwd -source [get_pins u_ref_oddr/C] \
    -divide_by 1 [get_ports eth_ref_clk]

# clk_sys (MMCM CLKOUT0) and clk_25 (CLKOUT1) are derived by Vivado from
# clk_100. The PHY clocks eth_rx_clk / eth_tx_clk (through BUFG and BUFGCE)
# are unrelated to them and to each other. Every crossing is a Gray-code or
# handshake synchroniser inside rx_path / tx_path (ChanCdc, req/ack), or a
# two-flop ASYNC_REG synchroniser in the wrapper (MII clock enables, MMCM
# locked); the reset reaches the MII domains only while their clocks are
# stopped (wrapper header). The XDC's own set_clock_groups names clk_100
# alone; this one includes the generated clocks.
set_clock_groups -asynchronous \
    -group [get_clocks -include_generated_clocks clk_100] \
    -group [get_clocks -include_generated_clocks eth_rx_clk] \
    -group [get_clocks -include_generated_clocks eth_tx_clk]

# PHY management: MDC at 1 MHz and MDIO are driven and sampled by registers
# of phy_init many clk_sys cycles apart (half an MDC period = 50 cycles);
# the PHY reset is a slow level. Not timed at the pins.
set_false_path -to [get_ports {eth_mdc eth_mdio eth_rstn}]
set_false_path -from [get_ports eth_mdio]
