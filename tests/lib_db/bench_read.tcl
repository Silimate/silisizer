set t0 [clock milliseconds]
sta::read_lib_db /tmp/bench.libdb
set t1 [clock milliseconds]
puts "read_lib_db:   [expr {$t1 - $t0}] ms"
