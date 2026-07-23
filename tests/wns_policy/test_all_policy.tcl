# -all (sizing_fast) must upsize every eligible offender in one timing pass
set workdir [file normalize [file join [pwd] work_all]]
file delete -force $workdir
file mkdir [file join $workdir data]

read_liberty wns_policy.lib
read_verilog wns_policy.v
link_design wns_policy

create_clock -name test_clk -period 1.0
set_input_delay 0.0 -clock test_clk [get_ports {a b}]
set_output_delay 0.0 -clock test_clk [get_ports {fixed_y opt_y}]

if {[catch {sta::silisize -all $workdir} result]} {
    puts "ALL_POLICY_TEST: FAIL (silisize error: $result)"
    file delete -force $workdir
    exit 1
}
if {$result != 0} {
    puts "ALL_POLICY_TEST: FAIL (silisize returned $result)"
    file delete -force $workdir
    exit 1
}

set transforms [open [file join $workdir data resized_cells.tsv] r]
set lines [split [string trim [read $transforms]] "\n"]
close $transforms
file delete -force $workdir

# Ten BUF_sp0 cells sit on the optimizable path; -all resizes all of them
# in the first pass (adaptive mode would only resize 1, then 2, then 4).
if {[llength $lines] != 11} {
    puts "ALL_POLICY_TEST: FAIL (expected 10 resizes, got [expr {[llength $lines] - 1}])"
    exit 1
}

puts "ALL_POLICY_TEST: PASS"
