# Shared helpers for the analysis scripts (sourced, never run alone).
#
# Every report goes through `rpt`, which catches errors: a report that does not
# exist in the installed Vivado version, or that needs another licence, is logged
# in $::outdir/reports/_skipped.log and the run goes on. Nothing here changes the
# design.

proc log {msg} {
    puts "\[[clock format [clock seconds] -format %H:%M:%S]\] $msg"
}

# Parse `key=value` arguments from -tclargs into the array `opt`, over defaults.
proc parse_args {defaults} {
    global argv opt
    array set opt $defaults
    foreach a $argv {
        if {[regexp {^([A-Za-z_0-9]+)=(.*)$} $a -> k v]} {
            if {![info exists opt($k)]} {
                error "unknown option '$k' (known: [lsort [array names opt]])"
            }
            set opt($k) $v
        } else {
            error "argument '$a' must be key=value"
        }
    }
}

# Run a body, log the time it took and keep going if it fails.
proc rpt {name body} {
    global outdir
    set t0 [clock seconds]
    if {[catch {uplevel 1 $body} err]} {
        log "SKIPPED $name: $err"
        set fh [open [file join $outdir reports _skipped.log] a]
        puts $fh "$name: $err"
        close $fh
    } else {
        log "$name done ([expr {[clock seconds] - $t0}] s)"
    }
}

# Read the generated sources listed in <dir>/files.f.
proc read_sources {src_dir} {
    set flist [file join $src_dir files.f]
    if {![file exists $flist]} {
        error "$flist not found: run `cargo run -p ntp_fpga --bin top_gen \[-- --with-client\] <dir>` first"
    }
    set fh [open $flist r]
    set sources {}
    foreach line [split [read $fh] "\n"] {
        set l [string trim $line]
        if {$l eq "" || [string index $l 0] eq "#"} { continue }
        lappend sources [file join $src_dir $l]
    }
    close $fh
    read_verilog $sources
    return $sources
}

proc read_constraints {root script_dir} {
    read_xdc [file join $root constraints arty_a7_100.xdc]
    read_xdc [file join $script_dir top_timing.xdc]
}

# Worst slack of a kind ("max" setup / "min" hold) over the whole design, "" if no path.
proc worst_slack {type} {
    set p [get_timing_paths -delay_type $type -max_paths 1 -quiet]
    if {[llength $p] == 0} { return "" }
    return [get_property SLACK [lindex $p 0]]
}

# Total negative / positive-hold slack of the design (sum of the worst path of every endpoint).
proc total_slack {type} {
    set tot 0.0
    foreach p [get_timing_paths -delay_type $type -max_paths 20000 -nworst 1 -slack_lesser_than 0 -quiet] {
        set tot [expr {$tot + [get_property SLACK $p]}]
    }
    return $tot
}

# One CSV row of a timing path: slack, levels, data path delay and its split, start, end, clocks.
proc path_row {p} {
    set f {}
    foreach prop {SLACK LOGIC_LEVELS DATAPATH_DELAY LOGIC_DELAY NET_DELAY REQUIREMENT SKEW STARTPOINT_CLOCK ENDPOINT_CLOCK STARTPOINT_PIN ENDPOINT_PIN} {
        if {[catch {set v [get_property $prop $p]}]} { set v "" }
        lappend f [string map {, ;} $v]
    }
    return [join $f ,]
}
set ::path_header "slack_ns,logic_levels,datapath_ns,logic_ns,net_ns,requirement_ns,skew_ns,start_clock,end_clock,startpoint,endpoint"
