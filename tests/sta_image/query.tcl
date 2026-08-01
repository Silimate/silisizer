# Run against a restored session image; output must match save.tcl exactly.
puts "=== report_checks after restore ==="
report_checks -digits 4
puts "=== report_wns after restore ==="
report_wns
report_tns

puts "=== network still queryable ==="
report_object_names [get_cells *]
report_object_names [get_ports *]
puts "=== liberty still queryable ==="
report_object_names [get_lib_cells *ASAP7*]
