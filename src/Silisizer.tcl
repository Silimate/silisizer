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

define_cmd_args "report_retime_candidates" {[-nworst path_count] [-path_groups group_names]} \
  -help {Find nworst setup paths; report JSON from/to, the path, and worst neighbor paths.} \
  -arg_help {
    -nworst {`path_count`: Number of worst paths to report. The default is 20.}
    -path_groups {`group_names`: Report only the worst path of each group, in the given\
      order (e.g. ground truth paths, worst first), instead of sorting all paths by slack.}
  }

proc retime_seq_instance { pin } {
  if { [$pin is_top_level_port] } {
    return "NULL"
  }
  set inst [$pin instance]
  if { $inst == "NULL" } {
    return "NULL"
  }
  if { [$inst liberty_cell] == "NULL" } {
    return "NULL"
  }
  return $inst
}

proc json_escape { str } {
  string map {\\ \\\\ \" \\\" \n \\n \r \\r \t \\t} $str
}

# Worst setup path JSON to (-to) or from (-from) a flop instance, or "null".
proc report_neighbor_path { key inst } {
  if { $inst == "NULL" } {
    return "null"
  }
  set paths [find_timing_paths \
               $key $inst \
               -path_delay max \
               -group_path_count 1 \
               -endpoint_path_count 1 \
               -sort_by_slack]
  if { $paths == {} } {
    return "null"
  }
  set path [lindex $paths 0]
  with_output_to_variable report [list report_path_end $path]
  set report [string trim $report]
  return [string map {"\n" "\n    "} $report]
}

# Startpoint, endpoint and JSON report of each path. Call right after find_timing_paths:
# later STA queries invalidate PathEnd objects.
proc snapshot_paths { paths } {
  set rows {}
  foreach path $paths {
    set source [get_property $path startpoint]
    set sink [get_property $path endpoint]
    with_output_to_variable report [list report_path_end $path]
    lappend rows [list $source $sink [string trim $report]]
  }
  return $rows
}

proc report_retime_candidates { args } {
  parse_key_args "report_retime_candidates" args keys {-nworst -path_groups} flags {}
  check_argc_eq0 "report_retime_candidates" $args

  set nworst 20
  if { [info exists keys(-nworst)] } {
    set nworst $keys(-nworst)
    check_positive_integer "-nworst" $nworst
  }

  set_report_path_format json
  if { [info exists keys(-path_groups)] } {
    # Worst path of each group in the caller's order; unknown groups warn and drop out
    set rows {}
    foreach group [parse_path_group_arg $keys(-path_groups)] {
      if { [llength $rows] >= $nworst } {
        break
      }
      set paths [find_timing_paths \
                   -path_group $group \
                   -path_delay max \
                   -group_path_count 1 \
                   -endpoint_path_count 1 \
                   -sort_by_slack]
      lappend rows {*}[snapshot_paths [lrange $paths 0 0]]
    }
  } else {
    set rows [snapshot_paths [find_timing_paths \
                                -path_delay max \
                                -group_path_count $nworst \
                                -endpoint_path_count 1 \
                                -unique_paths_to_endpoint \
                                -unique_edges_to_endpoint \
                                -sort_by_slack]]
  }

  set objects {}
  foreach row $rows {
    lassign $row source sink report
    set from [json_escape [get_full_name $source]]
    set to [json_escape [get_full_name $sink]]
    set report [string map {"\n" "\n    "} $report]
    set before [report_neighbor_path -to [retime_seq_instance $source]]
    set after [report_neighbor_path -from [retime_seq_instance $sink]]
    lappend objects "  {
    \"from\": \"$from\",
    \"to\": \"$to\",
    \"worst_slack\": $report,
    \"worst_path_before\": $before,
    \"worst_path_after\": $after
  }"
  }

  set_report_path_format full

  if { $objects == {} } {
    report_line {[]}
  } else {
    report_line "\["
    report_line [join $objects ",\n"]
    report_line "\]"
  }
}

}
