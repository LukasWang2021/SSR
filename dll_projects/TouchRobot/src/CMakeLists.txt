cmake_minimum_required(VERSION 2.8.3)
project(RtmProject)

message(STATUS ${PLATFORM})

set(PROJECT_ROOT_PATH ${RtmProject_SOURCE_DIR}/../)
set(EXECUTABLE_OUTPUT_PATH ${RtmProject_SOURCE_DIR}/../install/bin)
set(LIBRARY_OUTPUT_PATH ${RtmProject_SOURCE_DIR}/../install/lib)
set(CONFIG_FILE_OUTPUT_PATH ${RtmProject_SOURCE_DIR}/../install/runtime)
SET(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -lm -D PB_FIELD_32BIT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -O0 -Wno-unused-but-set-variable -Wno-unused-local-typedefs -fpermissive -lrt -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")

add_subdirectory(timer_help)
add_subdirectory(buff_help)
add_subdirectory(rpc_help)
add_subdirectory(log_help)
add_subdirectory(ssr_touch_test)
add_subdirectory(touch_help)
