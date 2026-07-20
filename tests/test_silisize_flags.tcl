set failures {}
set workdir [file normalize [file join [pwd] silisize_flags_tmp]]
file mkdir [file join $workdir data]

set netlist [file join $workdir empty.v]
set stream [open $netlist w]
puts $stream "module silisize_flags_top(); endmodule"
close $stream
set liberty [file normalize [file join \
        [file dirname [info script]] \
        ../third_party/OpenSTA/test/read_saif_null_instance.lib]]
read_liberty $liberty
read_verilog $netlist
link_design silisize_flags_top

foreach command [list \
        [list sta::silisize $workdir] \
        [list sta::silisize -all $workdir] \
        [list sta::silisize -wns $workdir] \
        [list sta::silisize -all -wns $workdir]] {
    if {[catch $command result]} {
        lappend failures "$command failed: $result"
    } elseif {$result != 0} {
        lappend failures "$command returned $result instead of 0"
    }
}

if {![catch {sta::silisize -unknown $workdir} result]} {
    lappend failures "sta::silisize accepted an unknown flag"
} elseif {$result ne {unknown option "-unknown": must be -all or -wns}} {
    lappend failures "sta::silisize returned a misleading error: $result"
}

file delete -force $workdir

if {[llength $failures]} {
    puts "SILISIZE_FLAGS_TEST: FAIL"
    foreach failure $failures {
        puts "  - $failure"
    }
    exit 1
}

puts "SILISIZE_FLAGS_TEST: PASS"
