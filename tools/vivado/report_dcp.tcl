# Re-run every report on an existing checkpoint (no implementation).
#   vivado -mode batch -source tools/vivado/report_dcp.tcl -tclargs dcp=out/vivado/post_route.dcp out=out/vivado_reports
# Useful to get a report you forgot, or to try other report options, without a new run.
set script_dir [file dirname [file normalize [info script]]]
set root       [file normalize [file join $script_dir .. ..]]
source [file join $script_dir lib.tcl]
parse_args {dcp out/vivado/post_route.dcp out out/vivado_reports}
set outdir [file normalize [file join $root $opt(out)]]
if {[file pathtype $opt(out)] eq "absolute"} { set outdir $opt(out) }
set rptdir [file join $outdir reports]
file mkdir $rptdir
set dcp [file normalize [file join $root $opt(dcp)]]
if {[file pathtype $opt(dcp)] eq "absolute"} { set dcp $opt(dcp) }
open_checkpoint $dcp
source [file join $script_dir reports.tcl]
