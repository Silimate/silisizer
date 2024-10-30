// Silisizer, Operator sizer
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
#include "sta/StaMain.hh"
#include "Silisizer.h"

using namespace SILISIZER;
static int
silisizerTclAppInit(Tcl_Interp *interp);

static void
showUsage(char *prog)
{
  printf("Usage: %s [-help] [-version] [-no_init] [-no_splash] cmd_file\n", prog);
  printf("  -help              show help and exit\n");
  printf("  -version           show version and exit\n");
  printf("  -no_init           do not read .sta init file\n");
  printf("  cmd_file           source cmd_file and exit\n");
}

// Swig uses C linkage for init functions.
extern "C"
{
  // extern int Silisizer_Init(Tcl_Interp *interp);
  extern int Sta_Init(Tcl_Interp *interp);
}

namespace sta
{
  //extern const char *silisizer_tcl_inits[];
  extern const char *tcl_inits[];
}

// "Arguments" passed to staTclAppInit.
static int silisizer_argc;
static char **silisizer_argv;

int main(int argc, char *argv[])
{
  Silisizer *sizer = new Silisizer();
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
static int
silisizerTclAppInit(Tcl_Interp *interp)
{
  int argc = silisizer_argc;
  char **argv = silisizer_argv;

  // source init.tcl
  Tcl_Init(interp);

  // Define swig commands.
  // Silisizer_Init(interp);
  Sta_Init(interp);

  sta::Sta *sta = sta::Sta::sta();
  sta->setTclInterp(interp);

  // Eval encoded sta TCL sources.
  sta::evalTclInit(interp, sta::tcl_inits);
  //sta::evalTclInit(interp, sta::silisizer_tcl_inits);

  // Import exported commands from sta namespace to global namespace.
  Tcl_Eval(interp, "sta::define_sta_cmds");
  Tcl_Eval(interp, "namespace import sta::*");

  if (!sta::findCmdLineFlag(argc, argv, "-no_init"))
  {
    const char *init_filename = "[file join $env(HOME) .silisizer]";
    sta::sourceTclFile(init_filename, true, false, interp);
  }

  bool exit_after_cmd_file = sta::findCmdLineFlag(argc, argv, "-exit");

  if (argc > 2 ||
      (argc > 1 && argv[1][0] == '-'))
    showUsage(argv[0]);
  else
  {
    if (argc == 2)
    {
      char *cmd_file = argv[1];
      if (cmd_file)
      {
        sta::sourceTclFile(cmd_file, false, false, interp);
        if (exit_after_cmd_file)
          exit(EXIT_SUCCESS);
      }
    }
  }
  return TCL_OK;
}
