# Tcl variables live in the interpreter, not the engine, so a restored session
# re-applies them the same way a fresh session does.
set sta_report_default_digits 8
set sta_strip_escaped_bus 1

report_wns
report_tns
report_checks -digits 4 -path_delay max -group_count 3
