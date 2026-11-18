// SPDX-License-Identifier: Apache-2.0
//
// Board top of the NTP server on the Digilent Arty A7-100T (task T1.11):
// clocks, reset sequencing and the PHY pins around the generated module
// `arty_top` (rtl/src/top.rs, written to out/top/arty_top.v by
// `cargo run -p ntp_fpga --bin top_gen`). Hand-written: only Xilinx
// 7-series primitives (MMCME2_BASE, BUFG, BUFGCE, ODDR, IOBUF, PULLUP).
// Port names are those of constraints/arty_a7_100.xdc.
//
// Clocks
//   clk_100 (E3, 100 MHz) -> MMCME2_BASE u_mmcm, VCO 1000 MHz:
//     CLKOUT0 /10 = 100 MHz -> BUFG -> clk_sys  (`clk` of arty_top)
//     CLKOUT1 /40 =  25 MHz -> BUFG -> clk_25   (PHY reference)
//   eth_ref_clk = clk_25 forwarded through an ODDR (D1 = 1, D2 = 0), so the
//   pin is driven by an IOB register and not by a clock net through fabric.
//   eth_rx_clk (F15) and eth_tx_clk (H16), 25 MHz from the PHY, each go
//   through one IBUF to two global buffers: a BUFG (free-running copy, only
//   for the two enable synchroniser flops) and a BUFGCE (the clock of the
//   MII domain, `clk_rx` / `clk_tx` of arty_top). Seven global buffers.
//
// Reset (one `rst` net reaches every module of arty_top, synchronous to the
// clock of each module; TxHDL netlists have one reset)
//   * `locked` of the MMCM is synchronised into clk_sys (two flops, cleared
//     asynchronously while the MMCM is unlocked).
//   * A sequencer in clk_sys then walks through four phases, counted by
//     `seq` (one clk_sys cycle per count):
//       HOLD    (seq   0..127): rst = 1, MII clocks enabled. Every MII-domain
//                                register sees the reset on >= 30 edges of
//                                its clock (when the PHY runs its clocks).
//       GATE    (seq 128..159): rst = 1, MII clock enables dropped. Within
//                                3 MII cycles (2 synchroniser flops + the
//                                BUFGCE) the MII clocks stop.
//       RELEASE (seq 160..191): rst = 0 in clk_sys (synchronous to clk_sys:
//                                the sequencer's own flop). The MII domains
//                                have no clock edge, so their registers
//                                cannot see the release near an edge.
//       RUN     (seq 192):       MII clock enables raised (synchronised in
//                                each MII clock by two ASYNC_REG flops
//                                clocked by the free-running BUFG copy; the
//                                BUFGCE switches glitch-free).
//   So the reset is released synchronously in clk_sys, and in each MII
//   domain while its clock is stopped: no recovery/removal hazard in any
//   domain. While the MMCM is unlocked the sequencer is held in HOLD.
//   At power-up every register already holds its initial value
//   (bitstream INIT = the unit's reset value, `reg x = v` in the netlist),
//   so the MII domains are consistent even if the PHY clocks are absent
//   during HOLD (the DP83848J is held in reset by `phy_init`, which only
//   starts after `rst` is released).
//
// PHY management
//   eth_rstn <= phy_rstn, eth_mdc <= mdc (registers of phy_init),
//   eth_mdio through an IOBUF: driven by mdio_o while mdio_oe, read back as
//   mdio_i. A weak PULLUP on the pad keeps the line high when released
//   (the board also has a pull-up resistor).
//
// MII data
//   eth_rxd / eth_rx_dv / eth_rx_er go straight into mii_rx (clk_rx);
//   eth_txd / eth_tx_en come straight from mii_tx's registers (clk_tx), so
//   Vivado can pack them in the IOB (IOB TRUE in the XDC). eth_col and
//   eth_crs are declared (the XDC places them) and unused.
`timescale 1ns / 1ps
module arty_top_board (
  input  wire       clk_100,
  // DP83848J, MII
  output wire       eth_ref_clk,
  output wire       eth_rstn,
  output wire       eth_mdc,
  inout  wire       eth_mdio,
  input  wire       eth_rx_clk,
  input  wire       eth_rx_dv,
  input  wire       eth_rx_er,
  input  wire [3:0] eth_rxd,
  input  wire       eth_tx_clk,
  output wire       eth_tx_en,
  output wire [3:0] eth_txd,
  input  wire       eth_col,
  input  wire       eth_crs
);
  // ------------------------------------------------------------------
  // clk_100 -> MMCM -> clk_sys (100 MHz), clk_25 (25 MHz)
  // ------------------------------------------------------------------
  wire clk_100_ibuf, mmcm_fb, mmcm_fb_buf, clkout0_sys, clkout1_25;
  wire clk_sys, clk_25, locked;

  IBUF u_clk_100_ibuf (.I(clk_100), .O(clk_100_ibuf));

  MMCME2_BASE #(
    .BANDWIDTH("OPTIMIZED"),
    .CLKIN1_PERIOD(10.000),
    .DIVCLK_DIVIDE(1),
    .CLKFBOUT_MULT_F(10.000),   // VCO 1000 MHz (600..1200 MHz, speed -1)
    .CLKFBOUT_PHASE(0.000),
    .CLKOUT0_DIVIDE_F(10.000),  // 100 MHz
    .CLKOUT0_PHASE(0.000),
    .CLKOUT0_DUTY_CYCLE(0.500),
    .CLKOUT1_DIVIDE(40),        // 25 MHz
    .CLKOUT1_PHASE(0.000),
    .CLKOUT1_DUTY_CYCLE(0.500),
    .REF_JITTER1(0.010),
    .STARTUP_WAIT("FALSE")
  ) u_mmcm (
    .CLKIN1(clk_100_ibuf),
    .CLKFBIN(mmcm_fb_buf),
    .CLKFBOUT(mmcm_fb),
    .CLKFBOUTB(),
    .CLKOUT0(clkout0_sys),
    .CLKOUT0B(),
    .CLKOUT1(clkout1_25),
    .CLKOUT1B(),
    .CLKOUT2(),
    .CLKOUT2B(),
    .CLKOUT3(),
    .CLKOUT3B(),
    .CLKOUT4(),
    .CLKOUT5(),
    .CLKOUT6(),
    .LOCKED(locked),
    .PWRDWN(1'b0),
    .RST(1'b0)
  );

  BUFG u_fb_bufg  (.I(mmcm_fb),     .O(mmcm_fb_buf));
  BUFG u_sys_bufg (.I(clkout0_sys), .O(clk_sys));
  BUFG u_25_bufg  (.I(clkout1_25),  .O(clk_25));

  // eth_ref_clk: 25 MHz forwarded by an output DDR register.
  wire eth_ref_clk_q;
  ODDR #(
    .DDR_CLK_EDGE("SAME_EDGE"),
    .INIT(1'b0),
    .SRTYPE("SYNC")
  ) u_ref_oddr (
    .Q(eth_ref_clk_q), .C(clk_25), .CE(1'b1),
    .D1(1'b1), .D2(1'b0), .R(1'b0), .S(1'b0)
  );
  OBUF u_ref_obuf (.I(eth_ref_clk_q), .O(eth_ref_clk));

  // ------------------------------------------------------------------
  // Reset sequencer (clk_sys)
  // ------------------------------------------------------------------
  (* ASYNC_REG = "TRUE" *) reg [1:0] locked_s = 2'b00;
  always @(posedge clk_sys or negedge locked) begin
    if (!locked) locked_s <= 2'b00;
    else         locked_s <= {locked_s[0], 1'b1};
  end

  localparam [7:0] SEQ_GATE    = 8'd128;
  localparam [7:0] SEQ_RELEASE = 8'd160;
  localparam [7:0] SEQ_RUN     = 8'd192;

  reg [7:0] seq    = 8'd0;
  reg       rst    = 1'b1;  // to every module of arty_top
  reg       mii_ce = 1'b1;  // MII clock enable request
  always @(posedge clk_sys) begin
    if (!locked_s[1]) begin
      seq    <= 8'd0;
      rst    <= 1'b1;
      mii_ce <= 1'b1;
    end else begin
      if (seq != SEQ_RUN) seq <= seq + 8'd1;
      rst    <= (seq < SEQ_RELEASE);
      mii_ce <= (seq < SEQ_GATE) || (seq == SEQ_RUN);
    end
  end

  // ------------------------------------------------------------------
  // MII clocks: free-running copy for the enable synchroniser, gated
  // copy for the MII domain.
  // ------------------------------------------------------------------
  wire rx_clk_ibuf, rx_clk_free, clk_rx;
  wire tx_clk_ibuf, tx_clk_free, clk_tx;
  IBUF u_rx_clk_ibuf (.I(eth_rx_clk), .O(rx_clk_ibuf));
  IBUF u_tx_clk_ibuf (.I(eth_tx_clk), .O(tx_clk_ibuf));
  BUFG u_rx_free_bufg (.I(rx_clk_ibuf), .O(rx_clk_free));
  BUFG u_tx_free_bufg (.I(tx_clk_ibuf), .O(tx_clk_free));

  (* ASYNC_REG = "TRUE" *) reg [1:0] rx_ce_s = 2'b00;
  (* ASYNC_REG = "TRUE" *) reg [1:0] tx_ce_s = 2'b00;
  always @(posedge rx_clk_free) rx_ce_s <= {rx_ce_s[0], mii_ce};
  always @(posedge tx_clk_free) tx_ce_s <= {tx_ce_s[0], mii_ce};

  BUFGCE u_rx_bufgce (.I(rx_clk_ibuf), .CE(rx_ce_s[1]), .O(clk_rx));
  BUFGCE u_tx_bufgce (.I(tx_clk_ibuf), .CE(tx_ce_s[1]), .O(clk_tx));

  // ------------------------------------------------------------------
  // MDIO
  // ------------------------------------------------------------------
  wire mdio_o, mdio_oe, mdio_i;
  IOBUF u_mdio_iobuf (.IO(eth_mdio), .I(mdio_o), .T(~mdio_oe), .O(mdio_i));
  PULLUP u_mdio_pullup (.O(eth_mdio));

  // ------------------------------------------------------------------
  // The design
  // ------------------------------------------------------------------
  wire phy_rstn, mdc, link_up, id_ok, phy_ready;
  arty_top u_top (
    .clk_rx(clk_rx),
    .clk_tx(clk_tx),
    .clk(clk_sys),
    .rst(rst),
    .rxd(eth_rxd),
    .rx_dv(eth_rx_dv),
    .rx_er(eth_rx_er),
    .mdio_i(mdio_i),
    .txd(eth_txd),
    .tx_en(eth_tx_en),
    .phy_rstn(phy_rstn),
    .mdc(mdc),
    .mdio_o(mdio_o),
    .mdio_oe(mdio_oe),
    .link_up(link_up),
    .id_ok(id_ok),
    .phy_ready(phy_ready)
  );
  assign eth_rstn = phy_rstn;
  assign eth_mdc  = mdc;

  // eth_col, eth_crs: carrier sense and collision are not used (full
  // duplex, auto-negotiated); the ports exist for the XDC.
endmodule
