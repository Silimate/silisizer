# Silisizer: resize operator-level cells to resolve timing violations
# Copyright (c) 2024, Silimate Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

namespace eval sta {

define_cmd_args "retime_tcl" {[-nworst path_count]} \
  -help {Find nworst setup paths; report JSON from/to, path slack, neighbor slacks.} \
  -arg_help {
    -nworst {`path_count`: Number of worst paths to report. The default is 20.}
  }

proc retime_seq_instance { pin } {
  if { [$pin is_top_level_port] } {
    return "NULL"
  }
  set inst [$pin instance]
  if { $inst == "NULL" } {
    return "NULL"
  }
  set cell [$inst liberty_cell]
  if { $cell == "NULL" || ![$cell is_sequential] } {
    return "NULL"
  }
  return $inst
}

# Worst slack_max on the named pins of pin's flop. "null" if not a flop.
proc get_flop_slack { pin pin_names } {
  set inst [retime_seq_instance $pin]
  if { $inst == "NULL" } {
    return "null"
  }
  set worst ""
  foreach name $pin_names {
    set p [$inst find_pin $name]
    if { $p == "NULL" } {
      continue
    }
    set slack [get_property $p slack_max]
    if { $slack != "" && ($worst == "" || $slack < $worst) } {
      set worst $slack
    }
  }
  if { $worst == "" } {
    return "null"
  }
  return [format %.4f $worst]
}

proc retime_tcl { args } {
  parse_key_args "retime_tcl" args keys {-nworst} flags {}
  check_argc_eq0 "retime_tcl" $args

  set nworst 20
  if { [info exists keys(-nworst)] } {
    set nworst $keys(-nworst)
    check_positive_integer "-nworst" $nworst
  }

  set paths [find_timing_paths \
               -path_delay max \
               -group_path_count $nworst \
               -endpoint_path_count 1 \
               -unique_paths_to_endpoint \
               -unique_edges_to_endpoint \
               -sort_by_slack]

  # Snapshot before any later STA queries. They invalidate PathEnd objects.
  set_report_path_format json
  set rows {}
  foreach path $paths {
    set source [get_property $path startpoint]
    set sink [get_property $path endpoint]
    with_output_to_variable report [list report_path_end $path]
    lappend rows [list $source $sink [string trim $report]]
  }
  set_report_path_format full

  set objects {}
  foreach row $rows {
    lassign $row source sink report
    set from [get_full_name $source]
    set to [get_full_name $sink]
    set report [string map {"\n" "\n    "} $report]
    set before [get_flop_slack $source {D}]
    set after [get_flop_slack $sink {Q QN}]
    lappend objects "  {
    \"from\": \"$from\",
    \"to\": \"$to\",
    \"worst_slack\": $report,
    \"worst_slack_before\": $before,
    \"worst_slack_after\": $after
  }"
  }

  if { $objects == {} } {
    report_line {[]}
  } else {
    report_line "\["
    report_line [join $objects ",\n"]
    report_line "\]"
  }
}

}
