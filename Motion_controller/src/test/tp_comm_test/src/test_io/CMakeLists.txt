cmake_minimum_required(VERSION 2.8.3)
project(tp_comm_test)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D PB_FIELD_32BIT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D PB_FIELD_32BIT")
SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/io)

##################################################################################################IO
######### test_read_di #########
add_executable(test_read_di test_read_di.cpp)
target_link_libraries(test_read_di ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_read_do #########
add_executable(test_read_do test_read_do.cpp)
target_link_libraries(test_read_do ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_write_do #########
add_executable(test_write_do test_write_do.cpp)
target_link_libraries(test_write_do ${rtm_LIBRARIES} pb tp_comm_test protos)
