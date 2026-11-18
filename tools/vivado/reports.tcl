# Reports on the routed design in memory. Sourced by analyze.tcl and by report_dcp.tcl;
# expects $outdir and $rptdir to be set and a routed (or at least placed) design open.

set r $rptdir

# ---- timing, the usual files ------------------------------------------------------------
rpt timing_summary {
    report_timing_summary -delay_type min_max -report_unconstrained -max_paths 50 \
        -input_pins -routable_nets -file [file join $r timing_summary.rpt]
}
rpt timing_setup_detail {
    report_timing -delay_type max -sort_by group -max_paths 100 -nworst 3 -unique_pins \
        -path_type full_clock_expanded -input_pins -routable_nets \
        -file [file join $r timing_setup_paths.rpt]
}
rpt timing_hold_detail {
    report_timing -delay_type min -sort_by group -max_paths 30 -nworst 1 -unique_pins \
        -path_type full_clock_expanded -input_pins -file [file join $r timing_hold_paths.rpt]
}
rpt timing_by_clock {
    # Worst 20 paths ending in each clock domain (the MII clocks are real 25 MHz, the
    # system clock 100 MHz).
    foreach clk [get_clocks] {
        set n [get_property NAME $clk]
        set regs [all_registers -clock $clk -data_pins]
        if {[llength $regs] == 0} { continue }
        report_timing -to $regs -delay_type max -max_paths 20 -nworst 1 -unique_pins -input_pins \
            -file [file join $r timing_to_clock_$n.rpt]
    }
}
rpt timing_io {
    report_timing -from [all_inputs] -delay_type max -max_paths 20 -file [file join $r timing_inputs.rpt]
    report_timing -to [all_outputs] -delay_type max -max_paths 20 -file [file join $r timing_outputs.rpt]
}
rpt exceptions {
    report_exceptions -ignored -file [file join $r exceptions.rpt]
    check_timing -verbose -file [file join $r check_timing.rpt]
}
rpt pulse_width {
    report_pulse_width -all_violators -file [file join $r pulse_width.rpt]
}
rpt bus_skew { report_bus_skew -file [file join $r bus_skew.rpt] }
rpt datasheet { report_datasheet -file [file join $r datasheet.rpt] }
rpt clock_interaction {
    report_clock_interaction -delay_type min_max -significant_digits 3 -file [file join $r clock_interaction.rpt]
}
rpt clock_networks {
    report_clock_networks -file [file join $r clock_networks.rpt]
    report_clock_utilization -file [file join $r clock_utilization.rpt]
}

# ---- the worst 500 paths as CSV, plus one row per endpoint cell hierarchy ----------------------
rpt paths_csv {
    set fh [open [file join $r paths_setup.csv] w]
    puts $fh $::path_header
    foreach p [get_timing_paths -delay_type max -max_paths 500 -nworst 1 -unique_pins] {
        puts $fh [path_row $p]
    }
    close $fh
    set fh [open [file join $r paths_hold.csv] w]
    puts $fh $::path_header
    foreach p [get_timing_paths -delay_type min -max_paths 200 -nworst 1 -unique_pins] {
        puts $fh [path_row $p]
    }
    close $fh
}
rpt paths_by_unit {
    # Worst setup path ending in the sequential cells of each unit of the core
    # (u_top/core/<unit>): which unit limits the clock, its logic depth, logic vs routing.
    set fh [open [file join $r unit_timing.csv] w]
    puts $fh "unit,$::path_header"
    set units {}
    foreach c [get_cells -quiet -hierarchical -filter {IS_PRIMITIVE == 0}] {
        set n [get_property NAME $c]
        if {[regexp {^u_top/core/[^/]+$} $n]} { lappend units $n }
    }
    if {[llength $units] == 0} {
        # flatten=full: no hierarchy left, fall back on name prefixes of the registers
        foreach c [get_cells -quiet -hierarchical -filter {IS_SEQUENTIAL}] {
            if {[regexp {^(u_top/core/[^/_]+)} [get_property NAME $c] -> u]} { dict set seen $u 1 }
        }
        if {[info exists seen]} { set units [dict keys $seen] }
    }
    foreach u [lsort $units] {
        set cells [get_cells -quiet -hierarchical -filter "IS_SEQUENTIAL && NAME =~ ${u}/*"]
        if {[llength $cells] == 0} { continue }
        set p [get_timing_paths -quiet -delay_type max -max_paths 1 -to $cells]
        if {[llength $p] == 0} { continue }
        puts $fh "[lindex [split $u /] end],[path_row [lindex $p 0]]"
    }
    close $fh
}

# ---- area ----------------------------------------------------------------------------------
rpt utilization {
    report_utilization -file [file join $r utilization.rpt]
    report_utilization -hierarchical -hierarchical_depth 6 -hierarchical_percentages \
        -file [file join $r utilization_hier.rpt]
    report_utilization -hierarchical -hierarchical_depth 6 -format csv -file [file join $r utilization_hier.csv]
}
rpt ram_utilization { report_ram_utilization -detail -file [file join $r ram_utilization.rpt] }
rpt dsp_carry {
    report_carry_chains -file [file join $r carry_chains.rpt]
}
rpt control_sets { report_control_sets -verbose -file [file join $r control_sets.rpt] }

# ---- routability, fanout, complexity ------------------------------------------------------------
rpt fanout {
    report_high_fanout_nets -timing -load_types -max_nets 200 -file [file join $r fanout.rpt]
}
rpt route_status { report_route_status -file [file join $r route_status.rpt] }
rpt design_analysis_complexity {
    report_design_analysis -complexity -hierarchical_depth 6 -file [file join $r design_complexity.rpt]
}
rpt design_analysis_timing {
    report_design_analysis -timing -max_paths 100 -show_all -file [file join $r design_timing.rpt]
}
rpt design_analysis_levels {
    report_design_analysis -logic_level_distribution -min_level 4 -max_level 40 \
        -file [file join $r logic_levels.rpt]
}
rpt design_analysis_congestion {
    report_design_analysis -congestion -file [file join $r congestion.rpt]
}
rpt qor {
    report_qor_summary -file [file join $r qor_summary.rpt]
    report_qor_suggestions -file [file join $r qor_suggestions.rpt]
}

# ---- correctness checks --------------------------------------------------------------------------
rpt cdc { report_cdc -details -file [file join $r cdc.rpt] }
rpt cdc_summary { report_cdc -summary -file [file join $r cdc_summary.rpt] }
rpt methodology { report_methodology -file [file join $r methodology.rpt] }
rpt drc { report_drc -file [file join $r drc.rpt] }
rpt io {
    report_io -file [file join $r io.rpt]
    report_clock_utilization -file [file join $r clock_utilization.rpt]
}
rpt power {
    report_power -file [file join $r power.rpt]
    report_power -hier all -file [file join $r power_hier.rpt]
}
rpt config_timing { report_config_timing -file [file join $r config_timing.rpt] }

# ---- one JSON file with the headline numbers ----------------------------------------------------------
rpt summary_json {
    proc jnum {v} { if {$v eq ""} { return null } else { return $v } }
    set wns [worst_slack max]
    set whs [worst_slack min]
    set tns [total_slack max]
    set ths [total_slack min]
    set clocks {}
    foreach clk [get_clocks] {
        set n [get_property NAME $clk]
        set per [get_property PERIOD $clk]
        set regs [all_registers -clock $clk -data_pins]
        set ws ""
        if {[llength $regs] > 0} {
            set p [get_timing_paths -quiet -delay_type max -max_paths 1 -to $regs]
            if {[llength $p] > 0} { set ws [get_property SLACK [lindex $p 0]] }
        }
        lappend clocks "\"$n\": {\"period_ns\": [jnum $per], \"wns_ns\": [jnum $ws]}"
    }
    set fh [open [file join $outdir summary.json] w]
    puts $fh "{"
    puts $fh "  \"vivado\": \"[version -short]\","
    puts $fh "  \"part\": \"[get_property PART [current_design]]\","
    puts $fh "  \"top\": \"[get_property TOP [current_design]]\","
    puts $fh "  \"wns_ns\": [jnum $wns], \"tns_ns\": $tns, \"whs_ns\": [jnum $whs], \"ths_ns\": $ths,"
    puts $fh "  \"clocks\": {[join $clocks {, }]},"
    puts $fh "  \"cells\": {"
    set cnt {}
    foreach {key filt} {lut {REF_NAME =~ LUT*} ff {REF_NAME =~ FD*} carry4 {REF_NAME == CARRY4}
                        dsp {REF_NAME =~ DSP48*} bram {REF_NAME =~ RAMB*} lutram {REF_NAME =~ RAM*M* || REF_NAME =~ RAM*X1*}
                        bufg {REF_NAME =~ BUFG*} mmcm {REF_NAME =~ MMCM*}} {
        lappend cnt "    \"$key\": [llength [get_cells -quiet -hierarchical -filter "IS_PRIMITIVE && ($filt)"]]"
    }
    puts $fh [join $cnt ",\n"]
    puts $fh "  }"
    puts $fh "}"
    close $fh
}
log "reports written in $r"
