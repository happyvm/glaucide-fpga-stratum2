# Out-of-context synthesis (and optionally place and route) of ONE unit of the generated
# netlist, with its own clock. Gives per-unit area and an estimate of its maximum frequency
# without the rest of the design. Run through ooc.sh, which loops over every unit.
#   vivado -mode batch -source tools/vivado/ooc.tcl -tclargs src=out/top_server module=arty_top_core_server period=10 impl=1
# Unit I/O are not timed: the unit is judged on its register-to-register paths.
set script_dir [file dirname [file normalize [info script]]]
set root       [file normalize [file join $script_dir .. ..]]
source [file join $script_dir lib.tcl]
parse_args {src out/top module arty_top_core_server period 10.0 impl 0 out out/vivado_ooc
            part xc7a100tcsg324-1 jobs 2}
set src_dir [file normalize [file join $root $opt(src)]]
if {[file pathtype $opt(src)] eq "absolute"} { set src_dir $opt(src) }
set outdir [file normalize [file join $root $opt(out) $opt(module)]]
if {[file pathtype $opt(out)] eq "absolute"} { set outdir [file join $opt(out) $opt(module)] }
file mkdir [file join $outdir reports]
set_param general.maxThreads $opt(jobs)
read_sources $src_dir
synth_design -mode out_of_context -top $opt(module) -part $opt(part)
set clks [get_ports -quiet -filter {DIRECTION == IN && NAME =~ clk*}]
foreach c $clks {
    create_clock -name [get_property NAME $c] -period $opt(period) $c
}
if {$opt(impl)} {
    opt_design
    place_design
    phys_opt_design
    route_design
}
set r [file join $outdir reports]
rpt unit_reports {
    report_utilization -file [file join $r utilization.rpt]
    report_timing -delay_type max -max_paths 10 -nworst 1 -unique_pins -input_pins -file [file join $r timing.rpt]
    report_design_analysis -logic_level_distribution -min_level 3 -max_level 40 -file [file join $r logic_levels.rpt]
}
# One CSV row for the whole-project table
set lut   [llength [get_cells -quiet -hierarchical -filter {IS_PRIMITIVE && REF_NAME =~ LUT*}]]
set ff    [llength [get_cells -quiet -hierarchical -filter {IS_PRIMITIVE && REF_NAME =~ FD*}]]
set dsp   [llength [get_cells -quiet -hierarchical -filter {IS_PRIMITIVE && REF_NAME =~ DSP48*}]]
set bram  [llength [get_cells -quiet -hierarchical -filter {IS_PRIMITIVE && REF_NAME =~ RAMB*}]]
set lram  [llength [get_cells -quiet -hierarchical -filter {IS_PRIMITIVE && (REF_NAME =~ RAM*M* || REF_NAME =~ RAM*X1*)}]]
set wns ""; set lev ""; set dly ""; set from ""; set to ""
set p [get_timing_paths -quiet -delay_type max -max_paths 1]
if {[llength $p] > 0} {
    set p [lindex $p 0]
    set wns [get_property SLACK $p]
    set lev [get_property LOGIC_LEVELS $p]
    set dly [get_property DATAPATH_DELAY $p]
    set from [get_property STARTPOINT_PIN $p]
    set to [get_property ENDPOINT_PIN $p]
}
set fh [open [file join $outdir result.csv] w]
puts $fh "module,impl,period_ns,lut,ff,dsp,bram,lutram,wns_ns,logic_levels,datapath_ns,startpoint,endpoint"
puts $fh "$opt(module),$opt(impl),$opt(period),$lut,$ff,$dsp,$bram,$lram,$wns,$lev,$dly,$from,$to"
close $fh
log "unit $opt(module): LUT $lut FF $ff DSP $dsp WNS $wns levels $lev"
