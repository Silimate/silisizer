# Compile the .lib to a .libdb for the reload test to consume.
set examples ../../third_party/OpenSTA/examples

read_liberty $examples/asap7_small_ss.lib.gz
if { [sta::write_lib_db /tmp/asap7_small_ss.libdb] != 0 } {
  puts "WRITE FAILED"
  exit 1
}
puts "=== compiled ==="
