# Offenders with equal scores must be upsized in instance path name order,
# independent of the order paths or the score map yield them.

set workdir [file normalize [file join [pwd] work_tie_order]]
file delete -force $workdir
file mkdir [file join $workdir data]

read_liberty ../wns_policy/wns_policy.lib
read_verilog tie_order.v
link_design tie_order

# Every BUF_sp0_X1 path misses this period and meets it once upsized.
create_clock -name test_clk -period 0.3
set_input_delay 0.0 -clock test_clk [get_ports a]
set_output_delay 0.0 -clock test_clk [get_ports y*]

if {[catch {sta::silisize $workdir} result] || $result != 0} {
    file delete -force $workdir
    puts "TIE_ORDER_TEST: FAIL (silisize: $result)"
    exit 1
}

set stream [open [file join $workdir data resized_cells.tsv] r]
set lines [split [string trim [read $stream]] "\n"]
close $stream
file delete -force $workdir

set order {}
foreach line [lrange $lines 1 end] {
    lappend order [lindex [split $line "\t"] 1]
}
set expected {u0 u1 u2 u3 u4 u5}
if {$order ne $expected} {
    puts "TIE_ORDER_TEST: FAIL (order: $order, expected $expected)"
    exit 1
}

puts "TIE_ORDER_TEST: PASS"
