cmake_minimum_required(VERSION 2.8.3)
project(tp_comm_test)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D PB_FIELD_32BIT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D PB_FIELD_32BIT")
SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/common)

###########################################################################################common
######### test_get_publish_table #########
add_executable(test_get_publish_table  test_get_publish_table.cpp)
target_link_libraries(test_get_publish_table ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_add_topic #########
add_executable(test_add_topic  test_add_topic.cpp)
target_link_libraries(test_add_topic ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_subscribe #########
add_executable(test_subscribe  test_subscribe.cpp)
target_link_libraries(test_subscribe ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_get_rpc_table #########
add_executable(test_get_rpc_table test_get_rpc_table.cpp)
target_link_libraries(test_get_rpc_table ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_delete_topic #########
add_executable(test_delete_topic test_delete_topic.cpp)
target_link_libraries(test_delete_topic ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_nonexistent_rpc #########
add_executable(test_nonexistent_rpc test_nonexistent_rpc.cpp)
target_link_libraries(test_nonexistent_rpc ${rtm_LIBRARIES} pb tp_comm_test protos)

