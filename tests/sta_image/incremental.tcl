# A restored session must still be able to re-run analysis, not just echo the
# results computed before the save.
puts "=== retime with a tighter clock ==="
create_clock -name clk -period 120 {clk1 clk2 clk3}
report_checks -digits 4 -path_delay max
report_wns

puts "=== resize a cell in the restored netlist ==="
replace_cell [get_cells u1] BUFx4_ASAP7_75t_R
report_checks -digits 4 -path_delay max
report_wns
