// Silisizer: resize operator-level cells to resolve timing violations
// Copyright (c) 2024, Silimate Inc.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

#include <iostream>

#include "Silisizer.h"
#include "sta/StaMain.hh"
#include "StaConfig.hh"  // TCL_READLINE
#include "StringUtil.hh"
#include "backward.hpp"
#include <csignal>
#include <string>

#include <tcl.h>
#if TCL_READLINE
  #include <tclreadline.h>
#endif

using namespace silisizer;
static int silisizerTclAppInit(Tcl_Interp *interp);

char *
findCmdLineKey(int &argc,
	       char *argv[],
	       const char *key)
{
  for (int i = 1; i < argc; i++) {
    char *arg = argv[i];
    if (sta::stringEq(arg, key) && i + 1 < argc) {
      char *value = argv[i + 1];
      // Remove key and value from argv.
      for (int j = i + 2; j < argc; j++, i++)
	argv[i] = argv[j];
      argc -= 2;
      argv[argc] = nullptr;
      return value;
    }
  }
  return nullptr;
}

int
parseThreadsArg(int &argc,
		char *argv[])
{
  char *thread_arg = findCmdLineKey(argc, argv, "-threads");
  if (thread_arg) {
    if (sta::stringEqual(thread_arg, "max"))
      return sta::processorCount();
    else if (sta::isDigits(thread_arg))
      return atoi(thread_arg);
    else
      fprintf(stderr,"Warning: -threads must be max or a positive integer.\n");
  }
  return 1;
}

static void showUsage(char *prog) {
  printf("Usage: %s [-help] [-version] [-threads count|max] cmd_file\n",
         prog);
  printf("  -help              show help and exit\n");
  printf("  -version           show version and exit\n");
  printf("  -threads count|max use count threads\n");
  printf("  cmd_file           source cmd_file and exit\n");
}

// Swig uses C linkage for init functions.
extern "C" {
extern int Silisizer_Init(Tcl_Interp *interp);
extern int Sta_Init(Tcl_Interp *interp);
}

namespace sta {
// extern const char *silisizer_tcl_inits[];
extern const char *tcl_inits[];
}  // namespace sta

// "Arguments" passed to staTclAppInit.
static int silisizer_argc;
static char **silisizer_argv;
static Silisizer *sizer = nullptr;

int silisize(const char *workdir) {
  return sizer->silisize(workdir);
}

void dump_icg_json(const char *path) {
  silisizer::dumpIcgJson(path);
}

void segv_call_fn() {
  int a;
  a = 6;
  a += 4;
  a -= 2;
  a /= 2;
  raise(SIGSEGV);
  printf("segv a: %d", a);
}

void test_abrt() { raise(SIGABRT); }
void test_segv() {
  printf("About to raise segfault\n");
  segv_call_fn();
}

// Flush stdout/stderr 
void signalHandler(int signo) {
  fflush(stderr);
  fflush(stdout);
  backward::SignalHandling sh;
  raise(SIGABRT);
}

int main(int argc, char *argv[]) {
  signal(SIGSEGV, signalHandler);
  signal(SIGFPE, signalHandler);
  signal(SIGINT, signalHandler);
  signal(SIGABRT, signalHandler);
  sizer = new Silisizer();
  sta::initSta();
  sta::Sta::setSta(sizer);
  sizer->makeComponents();

  silisizer_argc = argc;
  silisizer_argv = argv;
  // Set argc to 1 so Tcl_Main doesn't source any files.
  // Tcl_Main never returns.
  Tcl_Main(1, argv, silisizerTclAppInit);
  return 0;
}

// Tcl init executed inside Tcl_Main.
static int silisizerTclAppInit(Tcl_Interp *interp) {
  std::cout << "Silimate Silisizer executable" << std::endl;
  int argc = silisizer_argc;
  char **argv = silisizer_argv;

  // source init.tcl
  if (Tcl_Init(interp) == TCL_ERROR) {
    printf("Error: Tcl_Init failed: %s\n", Tcl_GetStringResult(interp));
    printf("Set TCL_LIBRARY to the directory containing init.tcl\n");
    return TCL_ERROR;
  }

#if TCL_READLINE
  // Initialize the C side of tclreadline. The library is linked into this
  // binary, so we also register it as a static package and (below) tell the Tcl
  // init script to skip its own dlopen of libtclreadline. This is what lets
  // tclreadline keep working in a relocated/packaged install where the shared
  // library is only reachable via the executable's RPATH.
  if (Tclreadline_Init(interp) == TCL_ERROR)
    return TCL_ERROR;
  Tcl_StaticPackage(interp, "tclreadline", Tclreadline_Init, Tclreadline_SafeInit);

  // Locate and source tclreadlineInit.tcl (which sources tclreadlineSetup.tcl
  // and thereby defines ::tclreadline::Loop). The compile-time TCLRL_LIBRARY
  // path does not exist once the build is packaged and extracted elsewhere, so
  // search next to the executable first (the bundled copy), then fall back to
  // the build-time path, $TCL_LIBRARY and a few standard locations.
  static const char *tclreadline_loader =
    "namespace eval ::tclreadline {}\n"
    // Pretend the C library is already loaded so ::tclreadline::Init does not
    // try to dlopen libtclreadline (it is statically referenced by this binary).
    "set ::tclreadline::library {builtin}\n"
    "proc ::tclreadline::_locate_init {} {\n"
    "  set bases [list [file join [file dirname [info nameofexecutable]] .. lib] {" TCLRL_LIBRARY "} /usr/local/lib /usr/lib]\n"
    "  if {[info exists ::env(TCL_LIBRARY)]} { lappend bases $::env(TCL_LIBRARY) [file join $::env(TCL_LIBRARY) ..] }\n"
    "  foreach base $bases {\n"
    "    set cands [list [file join $base tclreadlineInit.tcl]]\n"
    "    foreach g [lsort -decreasing [glob -nocomplain [file join $base tclreadline* tclreadlineInit.tcl]]] { lappend cands $g }\n"
    "    foreach cand $cands { if {[file exists $cand]} { return $cand } }\n"
    "  }\n"
    "  return {}\n"
    "}\n"
    "set ::tclreadline::_init_script [::tclreadline::_locate_init]\n"
    "if {$::tclreadline::_init_script ne {}} {\n"
    "  if {[catch {source $::tclreadline::_init_script} ::tclreadline::_err]} { puts stderr \"tclreadline: failed to load $::tclreadline::_init_script: $::tclreadline::_err\" }\n"
    "} else {\n"
    "  puts stderr {tclreadline: tclreadlineInit.tcl not found; interactive line editing disabled}\n"
    "}\n";
  Tcl_Eval(interp, tclreadline_loader);
#endif

  // Define swig commands.
  Silisizer_Init(interp);
  Sta_Init(interp);

  sta::Sta *sta = sta::Sta::sta();
  sta->setTclInterp(interp);
  int thread_count = parseThreadsArg(argc, argv);
  sta->setThreadCount(thread_count);

  // Eval encoded sta TCL sources.
  sta::evalTclInit(interp, sta::tcl_inits);
  // sta::evalTclInit(interp, sta::silisizer_tcl_inits);

  // Import exported commands from sta namespace to global namespace.
  Tcl_Eval(interp, "sta::define_sta_cmds");
  Tcl_Eval(interp, "namespace import sta::*");

  bool exit_after_cmd_file = sta::findCmdLineFlag(argc, argv, "-exit");

  if (argc > 2 || (argc > 1 && argv[1][0] == '-'))
    showUsage(argv[0]);
  else {
    if (argc == 2) {
      char *cmd_file = argv[1];
      if (cmd_file) {
        sta::sourceTclFile(cmd_file, false, false, interp);
        if (exit_after_cmd_file) exit(EXIT_SUCCESS);
      }
    }
  }
#if TCL_READLINE
  // Only enter the tclreadline loop if it actually loaded above. Otherwise fall
  // back to Tcl_Main's built-in interactive REPL so the binary stays usable.
  if (Tcl_Eval(interp, "info procs ::tclreadline::Loop") == TCL_OK
      && Tcl_GetStringResult(interp)[0] != '\0') {
    Tcl_Eval(interp, "proc ::tclreadline::prompt1 {} { return {% } }");
    Tcl_Eval(interp, "proc ::tclreadline::prompt2 {} { return {> } }");
    return Tcl_Eval(interp, "::tclreadline::Loop");
  }
  return TCL_OK;
#else
  return TCL_OK;
#endif
}
