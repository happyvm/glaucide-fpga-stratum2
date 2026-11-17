# Non-interactive Vivado build of the NTP server for the Arty A7-100T
# (task T0.5): synthesis, place and route, bitstream, reports.
#
#   cargo run -p ntp_fpga --bin top_gen          # writes out/top/
#   vivado -mode batch -source tools/vivado/build.tcl [-tclargs <build dir>]
#
# Non-project flow. Inputs: out/top/files.f (generated sources, in order),
# constraints/arty_a7_100.xdc, tools/vivado/top_timing.xdc. Outputs, in the
# build directory (default out/vivado/): arty_ntp.bit, checkpoints and
# reports. Exits with an error if timing is not met.
# Not run where it was written (no Vivado there): see tools/vivado/README.md.

set part      xc7a100tcsg324-1
set top       arty_top_board
set script_dir [file dirname [file normalize [info script]]]
set root      [file normalize [file join $script_dir .. ..]]
set src_dir   [file join $root out top]
set build_dir [file join $root out vivado]
if {$argc > 0} {
    set build_dir [file normalize [lindex $argv 0]]
}
file mkdir $build_dir
set rpt [file join $build_dir reports]
file mkdir $rpt

# ---- Sources -------------------------------------------------------------
set flist [file join $src_dir files.f]
if {![file exists $flist]} {
    error "$flist not found: run `cargo run -p ntp_fpga --bin top_gen` first"
}
set fh [open $flist r]
set sources {}
foreach line [split [read $fh] "\n"] {
    set l [string trim $line]
    if {$l eq "" || [string index $l 0] eq "#"} { continue }
    lappend sources [file join $src_dir $l]
}
close $fh
puts "Sources: $sources"
read_verilog $sources
read_xdc [file join $root constraints arty_a7_100.xdc]
read_xdc [file join $script_dir top_timing.xdc]

# ---- Synthesis -------------------------------------------------------------
synth_design -top $top -part $part
write_checkpoint -force [file join $build_dir post_synth.dcp]
report_utilization -file [file join $rpt utilization_synth.rpt]
report_timing_summary -file [file join $rpt timing_synth.rpt]

# ---- Implementation --------------------------------------------------------
opt_design
place_design
phys_opt_design
route_design
write_checkpoint -force [file join $build_dir post_route.dcp]

# ---- Reports ---------------------------------------------------------------
report_timing_summary -max_paths 20 -report_unconstrained \
    -file [file join $rpt timing_summary.rpt]
report_utilization -file [file join $rpt utilization.rpt]
report_utilization -hierarchical -file [file join $rpt utilization_hier.rpt]
report_clock_utilization -file [file join $rpt clock_utilization.rpt]
report_clock_interaction -file [file join $rpt clock_interaction.rpt]
report_cdc -details -file [file join $rpt cdc.rpt]
report_methodology -file [file join $rpt methodology.rpt]
report_drc -file [file join $rpt drc.rpt]
report_io -file [file join $rpt io.rpt]

# ---- Bitstream -------------------------------------------------------------
write_bitstream -force [file join $build_dir arty_ntp.bit]

# ---- Timing verdict ----------------------------------------------------------
set wns [get_property SLACK [get_timing_paths -delay_type max -max_paths 1]]
set whs [get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]
puts "WNS = $wns ns, WHS = $whs ns"
if {$wns eq "" || $whs eq ""} {
    error "no timed path found: check the constraints"
}
if {$wns < 0 || $whs < 0} {
    error "timing not met (WNS $wns ns, WHS $whs ns): see $rpt/timing_summary.rpt"
}
puts "Bitstream: [file join $build_dir arty_ntp.bit]"
