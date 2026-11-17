# CMake copies this script into a build directory with spaces in its name and
# runs it from there, so reaching this line means silisizer sourced the
# command file path as one word. silisize then writes into a work directory
# whose path also has spaces.
set failures {}
set script_dir [file dirname [file normalize [info script]]]
if {[string first " " $script_dir] < 0} {
    lappend failures "script path has no space: $script_dir"
}

set workdir [file join $script_dir "work dir"]
file delete -force $workdir
file mkdir [file join $workdir data]

set netlist [file join $workdir "empty top.v"]
set stream [open $netlist w]
puts $stream "module path_spaces_top(); endmodule"
close $stream
# The test runs from the source tests/ directory.
read_liberty [file normalize \
        ../third_party/OpenSTA/test/read_saif_null_instance.lib]
read_verilog $netlist
link_design path_spaces_top

if {[catch {sta::silisize $workdir} result]} {
    lappend failures "sta::silisize failed: $result"
} elseif {$result != 0} {
    lappend failures "sta::silisize returned $result instead of 0"
} elseif {![file exists [file join $workdir data resized_cells.tsv]]} {
    lappend failures "missing [file join $workdir data resized_cells.tsv]"
}

file delete -force $workdir

if {[llength $failures]} {
    puts "PATH_SPACES_TEST: FAIL"
    foreach failure $failures {
        puts "  - $failure"
    }
    exit 1
}

puts "PATH_SPACES_TEST: PASS"
