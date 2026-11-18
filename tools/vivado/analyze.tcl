# Full Vivado analysis of the NTP board: synthesis, implementation, and every report
# that helps to judge timing, area, clocking, CDC and routability. Produces machine-readable
# files (CSV / JSON) next to the usual .rpt reports, so the results can be read back
# without Vivado.
#
#   cargo run -p ntp_fpga --bin top_gen out/top_server                 # server only
#   cargo run -p ntp_fpga --bin top_gen -- --with-client out/top_client # stratum 2
#   vivado -mode batch -source tools/vivado/analyze.tcl -tclargs src=out/top_server out=out/vivado_server
#
# Options (key=value, all optional):
#   src=<dir>           generated sources (top_gen output)                    [out/top]
#   out=<dir>           results                                               [out/vivado]
#   jobs=<n>            threads                                               [4]
#   synth_directive=    Default | PerformanceOptimized | AreaOptimized_high |
#                       RuntimeOptimized | AlternateRoutability               [Default]
#   flatten=            rebuilt | none | full                                 [rebuilt]
#   retiming=0|1        synth_design -retiming                                [0]
#   opt_directive=      Default | Explore | ExploreArea | ExploreSequentialArea [Default]
#   place_directive=    Default | Explore | ExtraNetDelay_high | ExtraPostPlacementOpt |
#                       AltSpreadLogic_high | AltSpreadLogic_medium | EarlyBlockPlacement |
#                       SSI_SpreadLogic_high | WLDrivenBlockPlacement          [Default]
#   physopt_directive=  Default | Explore | AggressiveExplore | AlternateReplication |
#                       AggressiveFanoutOpt | AlternateFlowWithRetiming        [Explore]
#   route_directive=    Default | Explore | NoTimingRelaxation | MoreGlobalIterations |
#                       HigherDelayCost | AggressiveExplore | AlternateCLBRouting [Default]
#   post_route_physopt=0|1  extra phys_opt_design after route                  [0]
#   clk_sys_mhz=<f>     what-if: model a faster system clock (>= 100) as extra setup
#                       uncertainty, to read the margin at that frequency      [""]
#   bitstream=0|1       write the bitstream (only if timing is met)           [0]
#   simnetlist=0|1      write post-route Verilog + SDF for timing simulation  [0]
#   top=                top module                                            [arty_top_board]
#   part=               device                                                [xc7a100tcsg324-1]
#
# Unlike tools/vivado/build.tcl this script never stops on a timing failure: the point is to
# collect the reports of a design that does not close.

set script_dir [file dirname [file normalize [info script]]]
set root       [file normalize [file join $script_dir .. ..]]
source [file join $script_dir lib.tcl]
parse_args {src out/top out out/vivado jobs 4 synth_directive Default flatten rebuilt retiming 0
            opt_directive Default place_directive Default physopt_directive Explore
            route_directive Default post_route_physopt 0 clk_sys_mhz "" bitstream 0 simnetlist 0
            top arty_top_board part xc7a100tcsg324-1}

set src_dir [file normalize [file join $root $opt(src)]]
if {[file pathtype $opt(src)] eq "absolute"} { set src_dir $opt(src) }
set outdir  [file normalize [file join $root $opt(out)]]
if {[file pathtype $opt(out)] eq "absolute"} { set outdir $opt(out) }
set rptdir  [file join $outdir reports]
file mkdir $rptdir
set_param general.maxThreads $opt(jobs)

set fh [open [file join $outdir run_options.txt] w]
foreach k [lsort [array names opt]] { puts $fh "$k = $opt($k)" }
puts $fh "vivado = [version -short]"
puts $fh "date = [clock format [clock seconds]]"
close $fh
log "Vivado [version -short], sources $src_dir, results $outdir"

# ---------------------------------------------------------------- sources
read_sources $src_dir
read_constraints $root $script_dir
# ---------------------------------------------------------------- synthesis
set synth_args [list -top $opt(top) -part $opt(part) -directive $opt(synth_directive) \
                     -flatten_hierarchy $opt(flatten)]
if {$opt(retiming)} { lappend synth_args -retiming }
log "synth_design $synth_args"
synth_design {*}$synth_args
write_checkpoint -force [file join $outdir post_synth.dcp]
rpt synth_reports {
    report_utilization -file [file join $rptdir utilization_synth.rpt]
    report_utilization -hierarchical -hierarchical_depth 6 -file [file join $rptdir utilization_hier_synth.rpt]
    report_timing_summary -delay_type max -max_paths 20 -file [file join $rptdir timing_synth.rpt]
    report_clocks -file [file join $rptdir clocks.rpt]
    check_timing -verbose -file [file join $rptdir check_timing_synth.rpt]
    report_high_fanout_nets -max_nets 100 -file [file join $rptdir fanout_synth.rpt]
}
if {$opt(clk_sys_mhz) ne ""} {
    rpt retarget_clock {
        # What-if on the system clock: the MMCM makes 100 MHz, so a faster target is modelled
        # as extra setup uncertainty on that clock (10 ns - target period). Slack then reads as
        # the margin at the faster clock, and the implementation works to it.
        set period [expr {1000.0 / $opt(clk_sys_mhz)}]
        set extra  [expr {10.0 - $period}]
        if {$extra < 0} { error "clk_sys_mhz must be >= 100" }
        set clk [get_clocks -of_objects [get_pins u_sys_bufg/O]]
        set_clock_uncertainty -setup $extra $clk
        log "system clock modelled at $opt(clk_sys_mhz) MHz: +$extra ns setup uncertainty on $clk"
    }
}

# ---------------------------------------------------------------- implementation
opt_design -directive $opt(opt_directive)
write_checkpoint -force [file join $outdir post_opt.dcp]
place_design -directive $opt(place_directive)
write_checkpoint -force [file join $outdir post_place.dcp]
rpt place_reports {
    report_timing_summary -delay_type max -max_paths 10 -file [file join $rptdir timing_place.rpt]
    report_utilization -file [file join $rptdir utilization_place.rpt]
    report_control_sets -verbose -file [file join $rptdir control_sets.rpt]
}
phys_opt_design -directive $opt(physopt_directive)
write_checkpoint -force [file join $outdir post_physopt.dcp]
route_design -directive $opt(route_directive)
if {$opt(post_route_physopt)} { phys_opt_design -directive $opt(physopt_directive) }
write_checkpoint -force [file join $outdir post_route.dcp]

# ---------------------------------------------------------------- reports
source [file join $script_dir reports.tcl]

if {$opt(simnetlist)} {
    rpt sim_netlist {
        write_verilog -force -mode timesim -sdf_anno true -sdf_file post_route.sdf \
            [file join $outdir post_route_timesim.v]
        write_sdf -force [file join $outdir post_route.sdf]
    }
}

# ---------------------------------------------------------------- verdict and bitstream
set wns [worst_slack max]
set whs [worst_slack min]
log "WNS = $wns ns, WHS = $whs ns"
if {$opt(bitstream)} {
    if {$wns ne "" && $whs ne "" && $wns >= 0 && $whs >= 0} {
        write_bitstream -force [file join $outdir arty_ntp.bit]
        log "bitstream written"
    } else {
        log "bitstream NOT written: timing not met"
    }
}
log "done: results in $outdir (summary.json, reports/, post_route.dcp)"
