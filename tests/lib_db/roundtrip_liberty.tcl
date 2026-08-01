# Reference run: parse the .lib, then run the full analysis.
set examples ../../third_party/OpenSTA/examples

read_liberty $examples/asap7_small_ss.lib.gz
read_verilog $examples/reg1_asap7.v
link_design top

create_clock -name clk -period 500 {clk1 clk2 clk3}
set_input_delay -clock clk 0 {in1 in2}
set_output_delay -clock clk 0 out

report_checks -digits 6 -path_delay min_max
report_wns -digits 6
report_tns -digits 6
