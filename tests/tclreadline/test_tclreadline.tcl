# Verify that the tclreadline integration is compiled into the binary AND fully
# initialized at runtime. This is the feature that gives silisizer an
# interactive prompt with line editing, history and tab completion.
#
# main.cpp initializes tclreadline (C library + Tcl scripts) *before* it sources
# the command file passed via -exit, so by the time this script runs everything
# that the interactive ::tclreadline::Loop needs must already be in place. That
# makes this a deterministic, non-interactive check suitable for CI and for
# smoke-testing the packaged tarballs.

set failures {}

# 1. C extension command registered by Tclreadline_Init(). Its presence proves
#    that libtclreadline (and therefore GNU readline) is linked into the binary
#    and that TCL_READLINE was enabled at build time.
if {[info commands ::tclreadline::readline] eq {}} {
    lappend failures "::tclreadline::readline command is missing (tclreadline C library not linked or TCL_READLINE disabled)"
}

# 2. The interactive loop defined in tclreadlineSetup.tcl. Its presence proves
#    that the tclreadline Tcl scripts were found and sourced at runtime -- the
#    part that typically breaks in a relocated/packaged install.
if {[info procs ::tclreadline::Loop] eq {}} {
    lappend failures "::tclreadline::Loop is missing (tclreadline Tcl scripts not found at runtime)"
}

# 3. The package must be registered with a version so `package require
#    tclreadline` works for downstream scripts.
if {[catch {package present tclreadline} version]} {
    if {[catch {package require tclreadline} version]} {
        lappend failures "tclreadline package is not available: $version"
    }
}

if {[llength $failures]} {
    puts "TCLREADLINE_TEST: FAIL"
    foreach f $failures {
        puts "  - $f"
    }
    exit 1
}

puts "TCLREADLINE_TEST: PASS (tclreadline $version)"
