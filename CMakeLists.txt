
cmake_minimum_required (VERSION 2.8)
project (wbus)
# The version number.
set (wbus_VERSION_MAJOR 0)
set (wbus_VERSION_MINOR 1)
 
# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/wbusConfig.h.in"
  "${PROJECT_BINARY_DIR}/wbusConfig.h"
  )
 
# add the binary tree to the search path for include files
# so that we will find TutorialConfig.h
include_directories("${PROJECT_BINARY_DIR}")
 
# add the executable
add_executable(wbusd 
               src/wbusd.cpp src/CRC.cpp src/WayneBus.cpp)

 