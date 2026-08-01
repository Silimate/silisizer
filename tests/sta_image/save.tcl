# Load an asap7 design, report timing, then snapshot the session.
set examples ../../third_party/OpenSTA/examples

read_liberty $examples/asap7_small_ss.lib.gz
read_verilog $examples/reg1_asap7.v
link_design top

create_clock -name clk -period 500 {clk1 clk2 clk3}
set_input_delay -clock clk 0 {in1 in2}
set_output_delay -clock clk 0 out

puts "=== report_checks before save ==="
report_checks -digits 4
puts "=== report_wns before save ==="
report_wns
report_tns

if { [sta::save_sta_image /tmp/reg1_asap7.staimg] != 0 } {
  puts "SAVE FAILED"
  exit 1
}
puts "=== saved ==="
