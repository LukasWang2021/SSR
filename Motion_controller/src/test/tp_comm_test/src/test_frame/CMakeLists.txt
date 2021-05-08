cmake_minimum_required(VERSION 2.8.3)
project(tp_comm_test)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D PB_FIELD_32BIT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D PB_FIELD_32BIT")
SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/frame)

##################################################################################################controller
######### test_tool_add #########
add_executable(test_tool_add test_tool_add.cpp)
target_link_libraries(test_tool_add ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_tool_delete #########
add_executable(test_tool_delete test_tool_delete.cpp)
target_link_libraries(test_tool_delete ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_tool_get_all_valid_summary_info #########
add_executable(test_tool_get_all_valid_summary_info test_tool_get_all_valid_summary_info.cpp)
target_link_libraries(test_tool_get_all_valid_summary_info ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_tool_get_info_by_id #########
add_executable(test_tool_get_info_by_id test_tool_get_info_by_id.cpp)
target_link_libraries(test_tool_get_info_by_id ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_tool_move #########
add_executable(test_tool_move test_tool_move.cpp)
target_link_libraries(test_tool_move ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_tool_update #########
add_executable(test_tool_update test_tool_update.cpp)
target_link_libraries(test_tool_update ${rtm_LIBRARIES} pb tp_comm_test protos)



######### test_user_add #########
add_executable(test_user_add test_user_add.cpp)
target_link_libraries(test_user_add ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_user_delete #########
add_executable(test_user_delete test_user_delete.cpp)
target_link_libraries(test_user_delete ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_user_get_all_valid_summary_info #########
add_executable(test_user_get_all_valid_summary_info test_user_get_all_valid_summary_info.cpp)
target_link_libraries(test_user_get_all_valid_summary_info ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_user_get_info_by_id #########
add_executable(test_user_get_info_by_id test_user_get_info_by_id.cpp)
target_link_libraries(test_user_get_info_by_id ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_user_move #########
add_executable(test_user_move test_user_move.cpp)
target_link_libraries(test_user_move ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_user_update #########
add_executable(test_user_update test_user_update.cpp)
target_link_libraries(test_user_update ${rtm_LIBRARIES} pb tp_comm_test protos)