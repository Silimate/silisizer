# Cold load of a real netlist, then snapshot the session.
read_liberty ../common/sky130_fd_sc_hd__tt_025C_1v80.lib.gz
read_verilog ../picorv32/picorv32.nl.v.gz
link_design picorv32
create_clock [get_ports clk] -name clk -period 8.004

puts "=== cold load reference ==="
report_wns
report_tns
report_checks -digits 4 -path_delay max

if { [sta::save_sta_image /tmp/picorv32.staimg] != 0 } {
  puts "SAVE FAILED"
  exit 1
}
puts "=== saved ==="
