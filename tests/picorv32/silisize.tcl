read_liberty ../common/sky130_fd_sc_hd__tt_025C_1v80.lib.gz
read_verilog picorv32.nl.v.gz
link_design picorv32
create_clock [get_ports clk] -name clk -period 8.004
sta::silisize .
