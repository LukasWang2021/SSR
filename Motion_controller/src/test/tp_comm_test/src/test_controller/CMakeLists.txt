cmake_minimum_required(VERSION 2.8.3)
project(tp_comm_test)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D PB_FIELD_32BIT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D PB_FIELD_32BIT")
SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/controller)

##################################################################################################controller
######### test_get_version #########
add_executable(test_get_version test_get_version.cpp)
target_link_libraries(test_get_version ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_controller_get_system_time #########
add_executable(test_controller_get_system_time test_controller_get_system_time.cpp)
target_link_libraries(test_controller_get_system_time ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_controller_set_system_time #########
add_executable(test_controller_set_system_time test_controller_set_system_time.cpp)
target_link_libraries(test_controller_set_system_time ${rtm_LIBRARIES} pb tp_comm_test protos)
