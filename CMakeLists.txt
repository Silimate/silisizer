# -*- mode:cmake -*-
cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

project(SILISIZER VERSION 1.84)

# Detect build type, fallback to release and throw a warning if use didn't
# specify any
if(NOT CMAKE_BUILD_TYPE)
  message(WARNING "Build type not set, falling back to Release mode.
 To specify build type use:
 -DCMAKE_BUILD_TYPE=<mode> where <mode> is Debug or Release.")
  set(CMAKE_BUILD_TYPE
      "Release"
      CACHE STRING "Choose the type of build, options are: Debug Release."
            FORCE)
endif(NOT CMAKE_BUILD_TYPE)


set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules" ${CMAKE_MODULE_PATH})

option(
  WITH_LIBCXX
  "If buildling with clang++ and libc++(in Linux). To enable with: -DWITH_LIBCXX=On"
  OFF)

cmake_policy(SET CMP0091 NEW)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(WITH_LIBCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

set(WITH_STATIC_CRT OFF CACHE BOOL "Use Static CRT")

set(CMAKE_CXX_FLAGS "-Wno-unused-function -Werror ${CMAKE_CXX_FLAGS} ${MY_CXX_WARNING_FLAGS}")

if(MSVC)
  add_compile_definitions(_CRT_NONSTDC_NO_WARNINGS)

  set(CMAKE_CXX_FLAGS_DEBUG
      "${CMAKE_CXX_FLAGS_DEBUG} /Zc:__cplusplus /W4 /bigobj ${MY_CXX_WARNING_FLAGS}"
  )
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO
      "${CMAKE_CXX_FLAGS_RELEASE} /Zc:__cplusplus /W4 /bigobj ${MY_CXX_WARNING_FLAGS}"
  )
  set(CMAKE_CXX_FLAGS_RELEASE
      "${CMAKE_CXX_FLAGS_RELEASE} /Zc:__cplusplus /W4 /bigobj ${MY_CXX_WARNING_FLAGS}"
  )
  set(CMAKE_EXE_LINKER_FLAGS /STACK:8388608)  # 8MB stack size
elseif(WIN32 AND (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  # The stack size unnecessarily high here. Investigate and bring it back to something more reasonable.
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Xclang --stack-size=33554432")  # 32MB stack size
else()
  if(DEFINED ENV{MSYSTEM})
    # Under MSYS some files are too large to build without additional flags
    set(MSYS_COMPILE_OPTIONS "-m64 -Wa,-mbig-obj")
  endif()
  #set(MEM_SANITIZER_FLAGS
  #    " -fsanitize=address -fno-omit-frame-pointer"
  #)
  set(CMAKE_CXX_FLAGS_DEBUG
      "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O0 -g ${MSYS_COMPILE_OPTIONS} ${MY_CXX_WARNING_FLAGS} ${MEM_SANITIZER_FLAGS}"
  )
  set(CMAKE_CXX_FLAGS_RELEASE
      "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O3 ${MSYS_COMPILE_OPTIONS} -DNDEBUG ${MY_CXX_WARNING_FLAGS} ${MEM_SANITIZER_FLAGS}"
  )
endif()

################################################################
#
# Locate TCL library.
#
# Note that the cmake findTcl module is hopeless for OSX
# because there doesn't appear to be a way to override
# searching OSX system directories before unix directories.

set(TCL_POSSIBLE_NAMES tcl87 tcl8.7
  tcl86 tcl8.6
  tcl85 tcl8.5
  tcl84 tcl8.4
  tcl83 tcl8.3
  tcl82 tcl8.2
  )

# tcl lib path guesses.
if (NOT TCL_LIB_PATHS)
  if (CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(TCL_LIB_PATHS /usr/local/lib)
    set(TCL_NO_DEFAULT_PATH TRUE)
  endif()
elseif (CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(TCL_LIB_PATHS /usr/lib
    /usr/local/lib
    )
  set(TCL_NO_DEFAULT_PATH FALSE)
endif()

if (NOT TCL_LIB)
  # bagbiter cmake doesn't have a way to pass NO_DEFAULT_PATH as a parameter.
  if (TCL_NO_DEFAULT_PATH)
    find_library(TCL_LIB
      NAMES tcl ${TCL_POSSIBLE_NAMES}
      PATHS ${TCL_LIB_PATHS}
      NO_DEFAULT_PATH
      )
  else()
    find_library(TCL_LIB
      NAMES tcl ${TCL_POSSIBLE_NAMES}
      PATHS ${TCL_LIB_PATHS}
      )
  endif()
endif()
message(STATUS "TCL lib: ${TCL_LIB}")

get_filename_component(TCL_LIB_DIR "${TCL_LIB}" PATH)
get_filename_component(TCL_LIB_PARENT1 "${TCL_LIB_DIR}" PATH)
get_filename_component(TCL_LIB_PARENT2 "${TCL_LIB_PARENT1}" PATH)

# Locate tcl.h
if (NOT TCL_HEADER)
  find_file(TCL_HEADER tcl.h
    PATHS ${TCL_LIB_PARENT1} ${TCL_LIB_PARENT2}
    PATH_SUFFIXES include include/tcl
    NO_DEFAULT_PATH
    )
endif()
message(STATUS "TCL header: ${TCL_HEADER}")
get_filename_component(TCL_HEADER_DIR "${TCL_HEADER}" PATH)
message(STATUS "TCL header path: ${TCL_HEADER_DIR}")
################################################################

add_subdirectory(OpenSTA)
set(OPENSTA_HOME ${PROJECT_SOURCE_DIR}/OpenSTA)

set(SILISIZER_TCL_FILES
  src/Silisizer.tcl
  )

set(SILISIZER_SWIG_FILES
  src/Silisizer.i
  )

include(FindSWIG)
add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/Silisizer_wrap.cc
  COMMAND ${SWIG_EXECUTABLE} -tcl8 -c++ -namespace -prefix sta -I${OPENSTA_HOME}/tcl -I${OPENSTA_HOME}/sdf -I${OPENSTA_HOME}/dcalc -I${OPENSTA_HOME}/parasitics -I${OPENSTA_HOME}/verilog -o ${PROJECT_SOURCE_DIR}/Silisizer_wrap.cc ${PROJECT_SOURCE_DIR}/src/Silisizer.i
  WORKING_DIRECTORY ${OPENSTA_HOME}
  DEPENDS ${SILISIZER_SWIG_FILES}
  )

if (WIN32)
  add_compile_definitions(WIN32_LEAN_AND_MEAN)
endif()

set(silisizer_SRC
  ${PROJECT_SOURCE_DIR}/src/Silisizer.cpp
  ${PROJECT_SOURCE_DIR}/Silisizer_wrap.cc)


add_library(silisizer ${silisizer_SRC})

if(BUILD_SHARED_LIBS)
  set_property(TARGET silisizer PROPERTY POSITION_INDEPENDENT_CODE 1)
endif()

target_include_directories(silisizer PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(silisizer PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/OpenSTA/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(silisizer PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/OpenSTA>
  $<INSTALL_INTERFACE:include>)
target_include_directories(silisizer PUBLIC
  ${TCL_HEADER_DIR}
  $<INSTALL_INTERFACE:include>)

  add_executable(silisizer-bin ${PROJECT_SOURCE_DIR}/src/main.cpp)
set_target_properties(silisizer-bin PROPERTIES OUTPUT_NAME silisizer)

if (MSVC OR WIN32)
  set_target_properties(silisizer-bin PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
endif()

target_link_libraries(silisizer-bin PUBLIC silisizer)
target_link_libraries(silisizer PUBLIC OpenSTA sta_swig ${TCL_LIB})

if (UNIX)
  target_link_libraries(silisizer PRIVATE dl)
  target_link_libraries(silisizer PRIVATE util)
  target_link_libraries(silisizer PRIVATE m)
  target_link_libraries(silisizer PRIVATE pthread)
endif()


if (CMAKE_SYSTEM_NAME MATCHES "Linux")
  target_link_libraries(silisizer PRIVATE rt)
endif()

install(
  TARGETS silisizer-bin
  EXPORT Silisizer
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(
  TARGETS silisizer
  EXPORT Silisizer
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/Silisizer)

