cmake_minimum_required(VERSION 2.8.3)

# run shell script to build the python module
add_custom_command (OUTPUT pymodules 
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/build.sh 
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/)

add_custom_target( PYMODULES ALL
  DEPENDS pymodules
)