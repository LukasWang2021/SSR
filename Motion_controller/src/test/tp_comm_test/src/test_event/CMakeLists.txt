cmake_minimum_required(VERSION 2.8.3)
project(tp_comm_test)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D PB_FIELD_32BIT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D PB_FIELD_32BIT")
SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/event)


###################################################################################################event
######### test_recv_event #########
add_executable(test_recv_event test_recv_event.cpp)
target_link_libraries(test_recv_event ${rtm_LIBRARIES} pb tp_comm_test protos)
