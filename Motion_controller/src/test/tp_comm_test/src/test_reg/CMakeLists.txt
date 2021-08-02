cmake_minimum_required(VERSION 2.8.3)
project(tp_comm_test)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D PB_FIELD_32BIT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D PB_FIELD_32BIT")
SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/reg)

##################################################################################################controller
######### test_reg_move #########
add_executable(test_reg_move test_reg_move.cpp)
target_link_libraries(test_reg_move ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_reg_delete #########
add_executable(test_reg_delete test_reg_delete.cpp)
target_link_libraries(test_reg_delete ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_reg_get_changed_list #########
add_executable(test_reg_get_changed_list test_reg_get_changed_list.cpp)
target_link_libraries(test_reg_get_changed_list ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_reg_get_valid_list #########
add_executable(test_reg_get_valid_list test_reg_get_valid_list.cpp)
target_link_libraries(test_reg_get_valid_list ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_reg_pr_add #########
add_executable(test_reg_pr_add test_reg_pr_add.cpp)
target_link_libraries(test_reg_pr_add ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_reg_pr_get #########
add_executable(test_reg_pr_get test_reg_pr_get.cpp)
target_link_libraries(test_reg_pr_get ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_reg_pr_update #########
add_executable(test_reg_pr_update test_reg_pr_update.cpp)
target_link_libraries(test_reg_pr_update ${rtm_LIBRARIES} pb tp_comm_test protos)

