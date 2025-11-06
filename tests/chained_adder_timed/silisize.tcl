puts "Starting static timing analysis (STA)..."
set sta_report_default_digits 8
set sta_dynamic_loop_breaking 0
set sta_gated_clock_checks_enabled 0
set sta_boolean_props_as_int 0
set sta_direction_props_short 1
set sta_liberty_line_debug 0
puts "Reading Liberty files..."
read_liberty preqorsor/data/ops.lib
puts "Reporting units..."
::sta::write_units_json preqorsor/results/units.json
puts "Reading Verilog files..."
read_verilog preqorsor/data/ops.netlist
puts "Linking design chained_adder..."
link_design chained_adder
puts "Reading constraints..."
read_sdc ../../tmpl/default.sdc
read_sdc constraints.sdc
puts "Reporting combo loops..."
check_setup -verbose -loops > preqorsor/reports/loops.rpt
puts "Reporting Intial timing..."
report_checks  -no_line_splits -path_delay max -endpoint_count 1 -group_count 10000 -unique_paths_to_endpoint  > preqorsor/reports/sta_initial.rpt
puts "Silisizer..."
sta::silisize .
puts "Reporting Final timing..."
report_checks  -no_line_splits -path_delay max -endpoint_count 1 -group_count 10000 -unique_paths_to_endpoint  > preqorsor/reports/sta_final.rpt
puts "STA finished!"
