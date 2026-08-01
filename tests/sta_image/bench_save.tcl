# Heavier liberty load, so restore time can be compared against a load that is
# dominated by liberty parsing rather than by process startup.
set t0 [clock milliseconds]
set examples ../../third_party/OpenSTA/examples
set libs [list \
  ../common/sky130_fd_sc_hd__tt_025C_1v80.lib.gz \
  $examples/nangate45_typ.lib.gz \
  $examples/nangate45_fast.lib.gz \
  $examples/nangate45_slow.lib.gz \
  $examples/sky130hd_tt.lib.gz \
  $examples/asap7_small_ss.lib.gz \
  $examples/asap7_small_ff.lib.gz]
foreach lib $libs { read_liberty $lib }
set t1 [clock milliseconds]
puts "LIBERTY_MS [expr {$t1 - $t0}]"

read_verilog ../picorv32/picorv32.nl.v.gz
link_design picorv32
create_clock [get_ports clk] -name clk -period 8.004
report_wns
set t2 [clock milliseconds]
puts "TOTAL_LOAD_MS [expr {$t2 - $t0}]"

sta::save_sta_image /tmp/bench.staimg
puts "SAVE_MS [expr {[clock milliseconds] - $t2}]"
