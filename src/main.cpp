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

#include <tcl.h>

#if TCL_READLINE
  #include <tclreadline.h>
#endif

#include <iostream>

#include "Silisizer.h"
#include "sta/StaMain.hh"
#include "backward.hpp"
#include <csignal>

using namespace SILISIZER;
static int silisizerTclAppInit(Tcl_Interp *interp);

static void showUsage(char *prog) {
  printf("Usage: %s [-help] [-version] [-no_init] [-no_splash] cmd_file\n",
         prog);
  printf("  -help              show help and exit\n");
  printf("  -version           show version and exit\n");
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

int silisize(const char *workdir,
             int max_iter = 200,
             int min_paths_per_group = 100,
             int max_paths_per_group = 2000,
             int min_swaps_per_iter = 3,
             int max_swaps_per_iter = 20,
             double delay_weight_exp = 1.0,
             double slack_weight_exp = 1.0) {
  return sizer->silisize(
      workdir,
      max_iter, min_paths_per_group, max_paths_per_group,
      min_swaps_per_iter, max_swaps_per_iter,
      delay_weight_exp, slack_weight_exp);
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
  Tcl_Init(interp);

#if TCL_READLINE
  if (Tclreadline_Init(interp) == TCL_ERROR)
    return TCL_ERROR;
  Tcl_StaticPackage(interp, "tclreadline", Tclreadline_Init, Tclreadline_SafeInit);
  if (Tcl_EvalFile(interp, TCLRL_LIBRARY "/tclreadlineInit.tcl") != TCL_OK)
    printf("Failed to load tclreadline.tcl\n");
#endif

  // Define swig commands.
  Silisizer_Init(interp);
  Sta_Init(interp);

  sta::Sta *sta = sta::Sta::sta();
  sta->setTclInterp(interp);

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
  return Tcl_Eval(interp, "::tclreadline::Loop");
#else
  return TCL_OK;
#endif
}
