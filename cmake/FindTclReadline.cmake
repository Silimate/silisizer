set(TCL_READLINE 0)
set(TCL_READLINE_POSSIBLE_NAMES
  tclreadline-2.1.0
  tclreadline-2.3.2
  tclreadline-2.3.6
  tclreadline-2.3.7
  tclreadline-2.3.8
  tclreadline-2.4.1
  )
find_library(TCL_READLINE_LIBRARY
  NAMES tclreadline ${TCL_READLINE_POSSIBLE_NAMES}
  PATHS ${TCL_LIB_PATHS}
  )
if (TCL_READLINE_LIBRARY)
  message(STATUS "TCL readline library: ${TCL_READLINE_LIBRARY}")
else()
  message(STATUS "TCL readline library: NOT FOUND")
endif()

find_path(TCL_READLINE_INCLUDE tclreadline.h)
if (TCL_READLINE_INCLUDE)
  message(STATUS "TCL readline header: ${TCL_READLINE_INCLUDE}/tclreadline.h")
else()
  message(STATUS "TCL readline header: NOT FOUND")
endif()

if (TCL_READLINE_LIBRARY AND TCL_READLINE_INCLUDE)
  set(TCL_READLINE 1)
endif()
