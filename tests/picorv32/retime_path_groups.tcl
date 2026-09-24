read_liberty ../common/sky130_fd_sc_hd__tt_025C_1v80.lib.gz
read_verilog picorv32.nl.v.gz
link_design picorv32
create_clock [get_ports clk] -name clk -period 8.004

set failures {}

# Top-level "to" pins of a report_retime_candidates dump, in report order. The command
# nests with_output_to_variable itself, so capture through a file like preqorsor does.
proc candidate_ends { args } {
  set dump [file normalize retime_path_groups_tmp.json]
  redirect $dump [list sta::report_retime_candidates {*}$args]
  set stream [open $dump r]
  set report [read $stream]
  close $stream
  file delete $dump
  return [lmap {_ pin} [regexp -all -inline {\n    "to": "([^"]*)"} $report] {set pin}]
}

# Two distinct worst endpoints to build ground truth path groups from
set paths [find_timing_paths -path_delay max -group_path_count 2 -endpoint_path_count 1 \
             -unique_paths_to_endpoint -sort_by_slack]
set worst_end [get_full_name [get_property [lindex $paths 0] endpoint]]
set second_end [get_full_name [get_property [lindex $paths 1] endpoint]]
group_path -name gt_w1 -to [get_pins $second_end]
group_path -name gt_w2 -to [get_pins $worst_end]

# Groups are reported in the given order, not by slack; unknown groups drop out
set ends [candidate_ends -path_groups {gt_w1 no_such_group gt_w2}]
if { $ends != [list $second_end $worst_end] } {
  lappend failures "-path_groups order: expected {$second_end $worst_end}, got {$ends}"
}

# -nworst caps the number of groups reported
set ends [candidate_ends -nworst 1 -path_groups {gt_w1 gt_w2}]
if { $ends != [list $second_end] } {
  lappend failures "-nworst 1 -path_groups: expected {$second_end}, got {$ends}"
}

# Without -path_groups every group is searched, worst slack first
set ends [candidate_ends -nworst 2]
if { [lindex $ends 0] != $worst_end } {
  lappend failures "default: expected $worst_end first, got {$ends}"
}

if { [llength $failures] } {
  puts "RETIME_PATH_GROUPS_TEST: FAIL"
  foreach failure $failures {
    puts "  - $failure"
  }
  exit 1
}

puts "RETIME_PATH_GROUPS_TEST: PASS"
