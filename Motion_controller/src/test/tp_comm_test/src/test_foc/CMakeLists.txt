cmake_minimum_required(VERSION 2.8.3)
project(tp_comm_test)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D PB_FIELD_32BIT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D PB_FIELD_32BIT")
SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/fio_device)

##################################################################################################IO
######### test_read_di #########
add_executable(test_send_fioCmd test_send_fioCmd.cpp)
target_link_libraries(test_send_fioCmd ${rtm_LIBRARIES} pb tp_comm_test protos)
