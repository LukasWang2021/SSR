cmake_minimum_required(VERSION 2.8.3)
project(tp_comm_test)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D PB_FIELD_32BIT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D PB_FIELD_32BIT")
SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/group)

##################################################################################################controller
######### test_group_mcGroupReset #########
add_executable(test_group_mcGroupReset test_group_mcGroupReset.cpp)
target_link_libraries(test_group_mcGroupReset ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_group_mcGroupEnable #########
add_executable(test_group_mcGroupEnable test_group_mcGroupEnable.cpp)
target_link_libraries(test_group_mcGroupEnable ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_group_mcGroupDisable #########
add_executable(test_group_mcGroupDisable test_group_mcGroupDisable.cpp)
target_link_libraries(test_group_mcGroupDisable ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_group_mcGroupReadError #########
add_executable(test_group_mcGroupReadError test_group_mcGroupReadError.cpp)
target_link_libraries(test_group_mcGroupReadError ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_group_mcGroupReadStatus #########
add_executable(test_group_mcGroupReadStatus test_group_mcGroupReadStatus.cpp)
target_link_libraries(test_group_mcGroupReadStatus ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_group_resetAllEncoder #########
add_executable(test_group_resetAllEncoder test_group_resetAllEncoder.cpp)
target_link_libraries(test_group_resetAllEncoder ${rtm_LIBRARIES} pb tp_comm_test protos)