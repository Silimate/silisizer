# silisize must always create data/resized_cells.tsv (even when data/ is missing)
set workdir [file normalize [file join [pwd] resized_cells_tsv_tmp]]
file delete -force $workdir
file mkdir $workdir
# deliberately omit data/ — silisize must create it

set netlist [file join $workdir empty.v]
set stream [open $netlist w]
puts $stream "module resized_cells_top(); endmodule"
close $stream
set liberty [file normalize [file join \
        [file dirname [info script]] \
        ../third_party/OpenSTA/test/read_saif_null_instance.lib]]
read_liberty $liberty
read_verilog $netlist
link_design resized_cells_top

if {[catch {sta::silisize $workdir} result]} {
    puts "RESIZED_CELLS_TSV_TEST: FAIL (silisize error: $result)"
    file delete -force $workdir
    exit 1
}
if {$result != 0} {
    puts "RESIZED_CELLS_TSV_TEST: FAIL (silisize returned $result)"
    file delete -force $workdir
    exit 1
}

set tsv [file join $workdir data resized_cells.tsv]
if {![file exists $tsv]} {
    puts "RESIZED_CELLS_TSV_TEST: FAIL (missing $tsv)"
    file delete -force $workdir
    exit 1
}

set transforms [open $tsv r]
set lines [split [string trim [read $transforms]] "\n"]
close $transforms
file delete -force $workdir

if {[llength $lines] < 1 || [lindex $lines 0] ne "Scope\tInstance"} {
    puts "RESIZED_CELLS_TSV_TEST: FAIL (bad header: [lindex $lines 0])"
    exit 1
}

puts "RESIZED_CELLS_TSV_TEST: PASS"
