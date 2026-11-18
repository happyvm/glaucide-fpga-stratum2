# One implementation run from a synthesis checkpoint with chosen directives: the elementary
# step of sweep.sh. Writes <out>/<tag>/result.csv.
#   vivado -mode batch -source tools/vivado/sweep.tcl -tclargs dcp=out/vivado/post_synth.dcp \
#          place=Explore physopt=AggressiveExplore route=Explore out=out/vivado_sweep
set script_dir [file dirname [file normalize [info script]]]
set root       [file normalize [file join $script_dir .. ..]]
source [file join $script_dir lib.tcl]
parse_args {dcp out/vivado/post_synth.dcp out out/vivado_sweep opt Default place Default
            physopt Explore route Default keep_dcp 0 jobs 2}
set dcp [file normalize [file join $root $opt(dcp)]]
if {[file pathtype $opt(dcp)] eq "absolute"} { set dcp $opt(dcp) }
set tag "opt-$opt(opt)_place-$opt(place)_physopt-$opt(physopt)_route-$opt(route)"
set outdir [file normalize [file join $root $opt(out) $tag]]
if {[file pathtype $opt(out)] eq "absolute"} { set outdir [file join $opt(out) $tag] }
file mkdir [file join $outdir reports]
set_param general.maxThreads $opt(jobs)
open_checkpoint $dcp
set t0 [clock seconds]
opt_design -directive $opt(opt)
place_design -directive $opt(place)
if {$opt(physopt) ne "none"} { phys_opt_design -directive $opt(physopt) }
route_design -directive $opt(route)
set secs [expr {[clock seconds] - $t0}]
set wns [worst_slack max]
set whs [worst_slack min]
set tns [total_slack max]
set ths [total_slack min]
rpt sweep_reports {
    report_timing_summary -max_paths 10 -file [file join $outdir reports timing_summary.rpt]
    report_utilization -file [file join $outdir reports utilization.rpt]
}
if {$opt(keep_dcp)} { write_checkpoint -force [file join $outdir post_route.dcp] }
set fh [open [file join $outdir result.csv] w]
puts $fh "opt,place,physopt,route,wns_ns,tns_ns,whs_ns,ths_ns,seconds"
puts $fh "$opt(opt),$opt(place),$opt(physopt),$opt(route),$wns,$tns,$whs,$ths,$secs"
close $fh
log "$tag: WNS $wns TNS $tns WHS $whs ($secs s)"
