# Quick iteration: synthesis only (a few minutes), with estimated utilization and timing.
# Timing after synthesis has no routing delay: it is optimistic; use it to find logic depth
# and area, and the full flow (analyze.tcl) for slack.
#   vivado -mode batch -source tools/vivado/synth_only.tcl -tclargs src=out/top_server out=out/vivado_synth
set script_dir [file dirname [file normalize [info script]]]
set root       [file normalize [file join $script_dir .. ..]]
source [file join $script_dir lib.tcl]
parse_args {src out/top out out/vivado_synth top arty_top_board part xc7a100tcsg324-1
            synth_directive Default flatten rebuilt retiming 0 jobs 4}
set src_dir [file normalize [file join $root $opt(src)]]
if {[file pathtype $opt(src)] eq "absolute"} { set src_dir $opt(src) }
set outdir [file normalize [file join $root $opt(out)]]
if {[file pathtype $opt(out)] eq "absolute"} { set outdir $opt(out) }
set rptdir [file join $outdir reports]
file mkdir $rptdir
set_param general.maxThreads $opt(jobs)
read_sources $src_dir
read_constraints $root $script_dir
set args [list -top $opt(top) -part $opt(part) -directive $opt(synth_directive) -flatten_hierarchy $opt(flatten)]
if {$opt(retiming)} { lappend args -retiming }
synth_design {*}$args
write_checkpoint -force [file join $outdir post_synth.dcp]
rpt synth_reports {
    report_utilization -file [file join $rptdir utilization.rpt]
    report_utilization -hierarchical -hierarchical_depth 6 -file [file join $rptdir utilization_hier.rpt]
    report_utilization -hierarchical -hierarchical_depth 6 -format csv -file [file join $rptdir utilization_hier.csv]
    report_timing_summary -delay_type max -max_paths 100 -file [file join $rptdir timing.rpt]
    report_design_analysis -logic_level_distribution -min_level 4 -max_level 40 -file [file join $rptdir logic_levels.rpt]
    report_design_analysis -complexity -hierarchical_depth 6 -file [file join $rptdir design_complexity.rpt]
    report_high_fanout_nets -max_nets 200 -file [file join $rptdir fanout.rpt]
    report_ram_utilization -detail -file [file join $rptdir ram_utilization.rpt]
    report_drc -file [file join $rptdir drc.rpt]
    report_methodology -file [file join $rptdir methodology.rpt]
}
log "done: $outdir"
