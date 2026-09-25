read_liberty ../common/sky130_fd_sc_hd__tt_025C_1v80.lib.gz
read_verilog picorv32.nl.v.gz
link_design picorv32
create_clock [get_ports clk] -name clk -period 8.004

set failures {}

# {to slack_ns} of each candidate in a report_retime_candidates dump, in report order. The
# command nests with_output_to_variable itself, so capture through a file like preqorsor does.
proc candidates { args } {
  set dump [file normalize retime_path_groups_tmp.json]
  redirect $dump [list sta::report_retime_candidates {*}$args]
  set stream [open $dump r]
  set report [read $stream]
  close $stream
  file delete $dump
  set ends [lmap {_ pin} [regexp -all -inline {\n    "to": "([^"]*)"} $report] {set pin}]
  # Slack is the last field of the worst_slack path, right before worst_path_before
  set slacks [lmap {_ slack} [regexp -all -inline \
    {"slack": ([^\s,]+)\s*\},\s*"worst_path_before"} $report] {expr {$slack * 1e9}}]
  return [lmap pin $ends slack $slacks {list $pin $slack}]
}

# Same endpoint and the worst slack to it (the reported path is the group's worst path).
# The JSON report keeps 4 significant digits, hence the relative tolerance.
proc matches { got want } {
  if { [llength $got] != [llength $want] } {
    return 0
  }
  foreach g $got w $want {
    lassign $g got_pin got_slack
    lassign $w want_pin want_slack
    if { $got_pin != $want_pin
         || abs($got_slack - $want_slack) > 1e-3 * abs($want_slack) + 1e-6 } {
      return 0
    }
  }
  return 1
}

# Two distinct worst endpoints and their worst slacks (ns), taken before later queries
# invalidate the PathEnds
set paths [find_timing_paths -path_delay max -group_path_count 2 -endpoint_path_count 1 \
             -unique_paths_to_endpoint -sort_by_slack]
set worst [list [get_full_name [get_property [lindex $paths 0] endpoint]] \
             [get_property [lindex $paths 0] slack]]
set second [list [get_full_name [get_property [lindex $paths 1] endpoint]] \
              [get_property [lindex $paths 1] slack]]
group_path -name gt_w1 -to [get_pins [lindex $second 0]]
group_path -name gt_w2 -to [get_pins [lindex $worst 0]]

# Groups are reported in the given order, not by slack; unknown groups drop out
set got [candidates -path_groups {gt_w1 no_such_group gt_w2}]
if { ![matches $got [list $second $worst]] } {
  lappend failures "-path_groups order: expected {$second} {$worst}, got {$got}"
}

# -nworst caps the number of groups reported
set got [candidates -nworst 1 -path_groups {gt_w1 gt_w2}]
if { ![matches $got [list $second]] } {
  lappend failures "-nworst 1 -path_groups: expected {$second}, got {$got}"
}

# Without -path_groups every group is searched, worst slack first
set got [candidates -nworst 2]
if { ![matches [lrange $got 0 0] [list $worst]] } {
  lappend failures "default: expected {$worst} first, got {$got}"
}

if { [llength $failures] } {
  puts "RETIME_PATH_GROUPS_TEST: FAIL"
  foreach failure $failures {
    puts "  - $failure"
  }
  exit 1
}

puts "RETIME_PATH_GROUPS_TEST: PASS"
