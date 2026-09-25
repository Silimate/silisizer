read_liberty ../common/sky130_fd_sc_hd__tt_025C_1v80.lib.gz
read_verilog picorv32.nl.v.gz
link_design picorv32
create_clock [get_ports clk] -name clk -period 8.004
sta::report_retime_candidates

# Ground truth style path groups on the two worst endpoints, requested out of slack order
set paths [find_timing_paths -path_delay max -group_path_count 2 -endpoint_path_count 1 \
             -unique_paths_to_endpoint -sort_by_slack]
set worst_end [get_full_name [get_property [lindex $paths 0] endpoint]]
set second_end [get_full_name [get_property [lindex $paths 1] endpoint]]
group_path -name gt_w1 -to [get_pins $second_end]
group_path -name gt_w2 -to [get_pins $worst_end]
sta::report_retime_candidates -path_groups {gt_w1 no_such_group gt_w2}
sta::report_retime_candidates -nworst 1 -path_groups {gt_w1 gt_w2}
