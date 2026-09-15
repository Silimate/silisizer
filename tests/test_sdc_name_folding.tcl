# read_sdc must resolve constraints written against a synthesis netlist's
# flattened names (gen_0_child, wrapper_child, reg, CP) on an elaborated
# hierarchical netlist (gen[0].child, wrapper/child, reg[0], CLK) through
# OpenSTA's sta_sdc_name_folding fallback, and leave ambiguous names alone.
# The design, liberty and SDC are OpenSTA's sdc_name_folding test inputs.
set failures {}
set test_dir [file normalize [file join \
        [file dirname [info script]] ../third_party/OpenSTA/test]]

proc names { collection } {
    set result {}
    foreach_in_collection obj $collection {
        lappend result [get_full_name $obj]
    }
    return [lsort $result]
}

proc clock_names {} {
    set result {}
    foreach_in_collection clk [all_clocks] {
        lappend result [get_name $clk]
    }
    return [lsort $result]
}

proc expect { label actual expected } {
    upvar 1 failures failures
    if { $actual ne $expected } {
        lappend failures "$label: got {$actual}, expected {$expected}"
    }
}

proc load_design { test_dir } {
    read_verilog [file join $test_dir sdc_name_folding.v]
    link_design sdc_name_folding
}

read_liberty [file join $test_dir sdc_name_folding.lib]
set sta_continue_on_error 1

if { ![info exists sta_sdc_name_folding] || $sta_sdc_name_folding != 1 } {
    lappend failures "sta_sdc_name_folding should default to 1"
}

# Without folding the flattened names miss and the generated clocks are lost.
set sta_sdc_name_folding 0
load_design $test_dir
read_sdc [file join $test_dir sdc_name_folding.sdc]
expect "clocks without folding" [clock_names] {CLK}

# With folding every constraint lands.
set sta_sdc_name_folding 1
load_design $test_dir
read_sdc [file join $test_dir sdc_name_folding.sdc]
expect "clocks with folding" [clock_names] {CLK GCLK_CH0 SYNC_CLK_CH0}
expect "generated clock source pin" \
    [names [get_property [get_clocks GCLK_CH0] sources]] \
    {{clkgen_u0/genblk1[0].gen_ch[0].cg_wrap_u0/icg_u0/Q}}
expect "register bit index" \
    [names [get_pins ctrl_u0/gen_lane_0_sync_u/ready_meta_0_reg/Q]] \
    {{ctrl_u0/gen_lane[0].sync_u/ready_meta_0_reg[0]/Q}}
expect "clock pin alias" \
    [names [get_pins csr_u0/*mode_sel_reg*/CP]] \
    {{csr_u0/mode_sel_reg[0]/CLK} {csr_u0/mode_sel_reg[1]/CLK}}
expect "bus bit glob" [names [get_ports status*1*]] {{status[1]}}

# Ambiguous and nonexistent names stay unresolved.
expect "ambiguous folded name" [names [get_pins -quiet amb.0.x/Z]] {}
expect "ambiguous register bit index" \
    [names [get_pins -quiet ctrl_u0/gen_lane_0_sync_u/two_bit_reg/Q]] {}
expect "no clock pin on a buffer" [names [get_pins -quiet csr_u0/buf_u0/CP]] {}
expect "two clock pins" [names [get_pins -quiet csr_u0/dual_reg/CP]] {}
expect "missing instance" [names [get_pins -quiet no_such_inst/Q]] {}

if {[llength $failures]} {
    puts "SDC_NAME_FOLDING_TEST: FAIL"
    foreach failure $failures {
        puts "  - $failure"
    }
    exit 1
}

puts "SDC_NAME_FOLDING_TEST: PASS"
