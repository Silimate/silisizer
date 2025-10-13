read_liberty ../common/sky130_fd_sc_hd__tt_025C_1v80.lib.gz
read_verilog picorv32.nl.v.gz
link_design picorv32
create_clock [get_ports clk] -name clk -period 8.004
# report_checks  -no_line_splits -path_delay max -endpoint_count 1 -group_count 10 -unique_paths_to_endpoint
#             workdir   max_iters   min_pths/grp    max_pths/grp    min_swaps/it    max_swaps/it    delay_weight_exp    slack_weight_exp
sta::silisize "."       50          100             10000           1               200             2.0                 1.0
