# FILE: tclreadlineEmbedInit.tcl
# ---
# Silisizer-specific bootstrap for the embedded tclreadline scripts.
#
# The stock tclreadlineInit.tcl loads libtclreadline, sources
# tclreadlineSetup.tcl and registers a file-based autoload for
# tclreadlineCompleter.tcl. None of that is needed here: libtclreadline is
# statically linked and initialized from C (Tclreadline_Init), and the Setup +
# Completer scripts are compiled directly into the silisizer executable and
# evaluated just before this file. All that remains is to point readline at the
# completer that is now already defined, so there is no external file to locate
# at run time.
if {[info commands ::tclreadline::readline] ne ""} {
    catch {::tclreadline::readline customcompleter ::tclreadline::ScriptCompleter}
}
