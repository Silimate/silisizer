# -least must upsize the smallest delay contributor first, where the default
# policy upsizes the largest one first.

# Run silisize with the given extra flags and return the ordered list of
# instance names recorded in resized_cells.tsv.
proc resize_order {tag flags} {
    set workdir [file normalize [file join [pwd] work_$tag]]
    file delete -force $workdir
    file mkdir [file join $workdir data]

    read_liberty least_policy.lib
    read_verilog least_policy.v
    link_design least_policy

    create_clock -name test_clk -period 1.0
    set_input_delay 0.0 -clock test_clk [get_ports b]
    set_output_delay 0.0 -clock test_clk [get_ports y]

    if {[catch {sta::silisize {*}$flags $workdir} result]} {
        file delete -force $workdir
        error "silisize $flags failed: $result"
    }
    if {$result != 0} {
        file delete -force $workdir
        error "silisize $flags returned $result"
    }

    set stream [open [file join $workdir data resized_cells.tsv] r]
    set lines [split [string trim [read $stream]] "\n"]
    close $stream
    file delete -force $workdir

    # Drop the "Scope\tInstance" header and keep the instance column in order.
    set order {}
    foreach line [lrange $lines 1 end] {
        lappend order [lindex [split $line "\t"] 1]
    }
    return $order
}

set failures {}

if {[catch {resize_order most {}} order]} {
    lappend failures $order
} elseif {[lindex $order 0] ne "big_cell"} {
    lappend failures "default policy resized [lindex $order 0] first, expected big_cell (order: $order)"
}

# Reset the design so the second run starts from all-sp0 cells again.
sta::clear_sta

if {[catch {resize_order least {-least}} order]} {
    lappend failures $order
} elseif {[lindex $order 0] ne "small_cell"} {
    lappend failures "-least resized [lindex $order 0] first, expected small_cell (order: $order)"
}

if {[llength $failures]} {
    puts "LEAST_POLICY_TEST: FAIL"
    foreach failure $failures {
        puts "  - $failure"
    }
    exit 1
}

puts "LEAST_POLICY_TEST: PASS"
