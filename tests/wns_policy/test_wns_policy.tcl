set workdir [file normalize [file join [pwd] work]]
file delete -force $workdir
file mkdir [file join $workdir data]

read_liberty wns_policy.lib
read_verilog wns_policy.v
link_design wns_policy

create_clock -name test_clk -period 1.0
set_input_delay 0.0 -clock test_clk [get_ports {a b}]
set_output_delay 0.0 -clock test_clk [get_ports {fixed_y opt_y}]

if {[sta::silisize -wns $workdir] != 0} {
    puts "WNS_POLICY_TEST: FAIL (silisize returned an error)"
    file delete -force $workdir
    exit 1
}

set transforms [open [file join $workdir data resized_cells.tsv] r]
set lines [split [string trim [read $transforms]] "\n"]
close $transforms
file delete -force $workdir

# Adaptive batches resize 1, 2, then 4 cells. The third subsequent timing pass
# still has the same unfixable WNS, so the policy stops before resizing the
# final three eligible cells.
if {[llength $lines] != 8} {
    puts "WNS_POLICY_TEST: FAIL (expected 7 resizes, got [expr {[llength $lines] - 1}])"
    exit 1
}

puts "WNS_POLICY_TEST: PASS"
