cmake_minimum_required(VERSION 2.8.3)
project(dll_projects)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -O0 -Wno-unused-but-set-variable -Wno-unused-local-typedefs -fpermissive -lrt")

set(PROJECT_ROOT_PATH ${dll_projects_SOURCE_DIR})
set (LIBRARY_OUTPUT_PATH ${dll_projects_SOURCE_DIR}/out/linux/)

add_subdirectory(controller)
add_subdirectory(kinematics)

