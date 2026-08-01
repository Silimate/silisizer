# Compare cold liberty parse against compiled database load.
# Usage: silisizer bench.tcl  (edit lib below to change the subject)
set lib ../../third_party/OpenSTA/examples/sky130hd_tt.lib.gz
set db /tmp/bench.libdb

set t0 [clock milliseconds]
read_liberty $lib
set t1 [clock milliseconds]
puts "read_liberty:  [expr {$t1 - $t0}] ms"

sta::write_lib_db $db
set t2 [clock milliseconds]
puts "write_lib_db:  [expr {$t2 - $t1}] ms"
puts "db size:       [file size $db] bytes"
puts "lib size:      [file size $lib] bytes (gzipped)"
