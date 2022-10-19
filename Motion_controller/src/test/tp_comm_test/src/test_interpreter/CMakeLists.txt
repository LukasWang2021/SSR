cmake_minimum_required(VERSION 2.8.3)
project(test_interpreter)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D PB_FIELD_32BIT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D PB_FIELD_32BIT")
SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/interpreter)


###################################################################################################event
######### test_start #########
add_executable(test_start test_start.cpp)
target_link_libraries(test_start ${rtm_LIBRARIES} pb tp_comm_test protos)