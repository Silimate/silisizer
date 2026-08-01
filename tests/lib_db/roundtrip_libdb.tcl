# Same analysis as roundtrip_liberty.tcl, but the library comes from the
# compiled database. Output must match byte for byte.
set examples ../../third_party/OpenSTA/examples

if { [sta::read_lib_db /tmp/asap7_small_ss.libdb] != 0 } {
  puts "READ FAILED"
  exit 1
}
read_verilog $examples/reg1_asap7.v
link_design top

create_clock -name clk -period 500 {clk1 clk2 clk3}
set_input_delay -clock clk 0 {in1 in2}
set_output_delay -clock clk 0 out

report_checks -digits 6 -path_delay min_max
report_wns -digits 6
report_tns -digits 6
