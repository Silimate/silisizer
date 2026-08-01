# Mirrors what preqorsor ista loads: ops.lib, the silisized netlist and the SDC.
set wd /Users/stanlee/preqorsor/example/preqorsor

set sta_report_default_digits 8
set sta_strip_escaped_bus 1

set t0 [clock milliseconds]
read_liberty $wd/data/ops.lib
set t1 [clock milliseconds]
puts "LIBERTY_MS [expr {$t1 - $t0}]"

read_verilog $wd/data/ops_final.netlist
link_design picorv32
set t2 [clock milliseconds]
puts "LINK_MS [expr {$t2 - $t1}]"

read_sdc $wd/data/preqorsor.sdc
unset_propagated_clock [all_clocks]
unset_clock_latency [all_clocks]
unset_clock_transition [all_clocks]
unset_clock_uncertainty [all_clocks]

report_wns
report_tns
puts "TOTAL_LOAD_MS [expr {[clock milliseconds] - $t0}]"

report_checks -digits 4 -path_delay max -group_count 3

set t3 [clock milliseconds]
sta::save_sta_image /tmp/preqorsor.staimg
puts "SAVE_MS [expr {[clock milliseconds] - $t3}]"
