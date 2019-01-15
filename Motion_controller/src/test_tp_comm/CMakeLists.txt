cmake_minimum_required(VERSION 2.8.3)
project(test_tp_comm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  log_manager
  protos
  tp_comm
  thread_help
)

if(catkin_CROSS)
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS})
    set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS})
	set(CMAKE_CXX_FLAGS "-DCROSS_PLATFORM")
	set(CMAKE_C_FLAGS "-DCROSS_PLATFORM")
    message(STATUS "ARM-Cross")
else()
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_x86})
    set(catkin_LIBRARIES ${catkin_LIBRARYS_x86})
    message(STATUS "x86-64")
endif()

set(catkin_INCLUDE_DIRS
    ${catkin_INCLUDE_DIRS}
    ${depend_PATH_INC}/log_manager/include
    ${depend_PATH_INC}/protos/include
    ${depend_PATH_INC}/tp_comm/include
    ${depend_PATH_INC}/parameter_manager/include
    ${depend_PATH_INC}/thread_help/include
    ${depend_PATH_INC}/common/
)

catkin_package(
    LIBRARIES test_tp_comm
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -D PB_FIELD_32BIT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")

  
#--enable-64it --disable-32bit --enable-shared CXXFLAGS=-m64 -DNDEBUG LDFLAGS=-m64

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${NANOPB_DIR}
)

add_library(test_tp_comm
  ${NANOPB_DIR}/pb_decode.c
  ${NANOPB_DIR}/pb_encode.c
  ${NANOPB_DIR}/pb_common.c
  src/tp_comm_test.cpp
)

target_link_libraries(test_tp_comm
  ${catkin_LIBRARIES}
  logger
  protos
  tp_comm
  thread_help
  parameter_manager
)

install(TARGETS test_tp_comm
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

######### for test client #########
add_executable(test_get_publish_table  ${SRC_LIST}  src/test_common/test_get_publish_table.cpp)
target_link_libraries(test_get_publish_table ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_get_publish_table RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

######### for test client #########
add_executable(test_add_topic  ${SRC_LIST}  src/test_common/test_add_topic.cpp)
target_link_libraries(test_add_topic ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_add_topic RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

######### for test client #########
add_executable(test_add_reg_topic  ${SRC_LIST}  src/test_common/test_add_reg_topic.cpp)
target_link_libraries(test_add_reg_topic ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_add_reg_topic RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

######### for test client #########
add_executable(test_add_io_topic  ${SRC_LIST}  src/test_common/test_add_io_topic.cpp)
target_link_libraries(test_add_io_topic ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_add_io_topic RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


######### for test client #########
add_executable(test_subscribe  ${SRC_LIST}  src/test_common/test_subscribe.cpp)
target_link_libraries(test_subscribe ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_subscribe RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

######### for test client #########
add_executable(test_subscribe_reg  ${SRC_LIST}  src/test_common/test_subscribe_reg.cpp)
target_link_libraries(test_subscribe_reg ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_subscribe_reg RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

######### for test client #########
add_executable(test_get_rpc_table  ${SRC_LIST}  src/test_common/test_get_rpc_table.cpp)
target_link_libraries(test_get_rpc_table ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_get_rpc_table RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

######### for test client #########
add_executable(test_delete_reg_topic  ${SRC_LIST}  src/test_common/test_delete_reg_topic.cpp)
target_link_libraries(test_delete_reg_topic ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_delete_reg_topic RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

######### for test client #########
add_executable(test_delete_io_topic  ${SRC_LIST}  src/test_common/test_delete_io_topic.cpp)
target_link_libraries(test_delete_io_topic ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_delete_io_topic RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

######### for test client #########
add_executable(test_delete_topic  ${SRC_LIST}  src/test_common/test_delete_topic.cpp)
target_link_libraries(test_delete_topic ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_delete_topic RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

######### for test client #########
add_executable(test_nonexistent_rpc  ${SRC_LIST}  src/test_common/test_nonexistent_rpc.cpp)
target_link_libraries(test_nonexistent_rpc ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_nonexistent_rpc RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

######### for test client #########
add_executable(test_subscribe_io  ${SRC_LIST}  src/test_common/test_subscribe_io.cpp)
target_link_libraries(test_subscribe_io ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_subscribe_io RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

######### for test client #########
add_executable(test_get_version  ${SRC_LIST}  src/test_controller/test_get_version.cpp)
target_link_libraries(test_get_version ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_get_version RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/controller)

######### for test client #########
add_executable(test_get_errorcode_list  ${SRC_LIST}  src/test_controller/test_get_errorcode_list.cpp)
target_link_libraries(test_get_errorcode_list ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_get_errorcode_list RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/controller)


######### for test client #########
add_executable(test_controller_get_system_time  ${SRC_LIST}  src/test_controller/test_controller_get_system_time.cpp)
target_link_libraries(test_controller_get_system_time ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_controller_get_system_time RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/controller)


######### for test client #########
add_executable(test_controller_set_system_time  ${SRC_LIST}  src/test_controller/test_controller_set_system_time.cpp)
target_link_libraries(test_controller_set_system_time ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_controller_set_system_time RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/controller)


######### for test client #########
add_executable(test_tool_add ${SRC_LIST}  src/test_frame/test_tool_add.cpp)
target_link_libraries(test_tool_add ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_tool_add RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/tool_manager)

######### for test client #########
add_executable(test_tool_update  ${SRC_LIST}  src/test_frame/test_tool_update.cpp)
target_link_libraries(test_tool_update ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_tool_update RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/tool_manager)


######### for test client #########
add_executable(test_user_add ${SRC_LIST}  src/test_frame/test_user_add.cpp)
target_link_libraries(test_user_add ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_user_add RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/coord_manager)

######### for test client #########
add_executable(test_user_update ${SRC_LIST}  src/test_frame/test_user_update.cpp)
target_link_libraries(test_user_update ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_user_update RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/coord_manager)


######### for test client #########
add_executable(test_tool_move  ${SRC_LIST}  src/test_frame/test_tool_move.cpp)
target_link_libraries(test_tool_move ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_tool_move RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/tool_manager)

######### for test client #########
add_executable(test_user_move  ${SRC_LIST}  src/test_frame/test_user_move.cpp)
target_link_libraries(test_user_move ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_user_move RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/coord_manager)

######### for test client #########
add_executable(test_tool_get_info_by_id  ${SRC_LIST}  src/test_frame/test_tool_get_info_by_id.cpp)
target_link_libraries(test_tool_get_info_by_id ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_tool_get_info_by_id RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/tool_manager)

######### for test client #########
add_executable(test_user_get_info_by_id  ${SRC_LIST}  src/test_frame/test_user_get_info_by_id.cpp)
target_link_libraries(test_user_get_info_by_id ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_user_get_info_by_id RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/coord_manager)

######### for test client #########
add_executable(test_tool_get_all_valid_summary_info  ${SRC_LIST}  src/test_frame/test_tool_get_all_valid_summary_info.cpp)
target_link_libraries(test_tool_get_all_valid_summary_info ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_tool_get_all_valid_summary_info RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/tool_manager)

######### for test client #########
add_executable(test_user_get_all_valid_summary_info  ${SRC_LIST}  src/test_frame/test_user_get_all_valid_summary_info.cpp)
target_link_libraries(test_user_get_all_valid_summary_info ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_user_get_all_valid_summary_info RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/coord_manager)

######### for test tool frame  #########
add_executable(test_tool_delete  ${SRC_LIST}  src/test_frame/test_tool_delete.cpp)
target_link_libraries(test_tool_delete ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_tool_delete RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/tool_manager)

######### for test tool frame  #########
add_executable(test_user_delete  ${SRC_LIST}  src/test_frame/test_user_delete.cpp)
target_link_libraries(test_user_delete ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_user_delete RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/coord_manager)

######### for test register #########
add_executable(test_reg_r_add  ${SRC_LIST}  src/test_register/test_reg_r_add.cpp)
target_link_libraries(test_reg_r_add ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_reg_r_add RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/reg_manager)

######### for test register #########
add_executable(test_reg_r_update  ${SRC_LIST}  src/test_register/test_reg_r_update.cpp)
target_link_libraries(test_reg_r_update ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_reg_r_update RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/reg_manager)

######### for test register #########
add_executable(test_reg_pr_add  ${SRC_LIST}  src/test_register/test_reg_pr_add.cpp)
target_link_libraries(test_reg_pr_add ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_reg_pr_add RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/reg_manager)

######### for test register #########
add_executable(test_reg_pr_update  ${SRC_LIST}  src/test_register/test_reg_pr_update.cpp)
target_link_libraries(test_reg_pr_update ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_reg_pr_update RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/reg_manager)

######### for test register #########
add_executable(test_reg_mr_add  ${SRC_LIST}  src/test_register/test_reg_mr_add.cpp)
target_link_libraries(test_reg_mr_add ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_reg_mr_add RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/reg_manager)

######### for test register #########
add_executable(test_reg_mr_update  ${SRC_LIST}  src/test_register/test_reg_mr_update.cpp)
target_link_libraries(test_reg_mr_update ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_reg_mr_update RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/reg_manager)


######### for test register #########
add_executable(test_reg_hr_add  ${SRC_LIST}  src/test_register/test_reg_hr_add.cpp)
target_link_libraries(test_reg_hr_add ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_reg_hr_add RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/reg_manager)

######### for test register #########
add_executable(test_reg_hr_update  ${SRC_LIST}  src/test_register/test_reg_hr_update.cpp)
target_link_libraries(test_reg_hr_update ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_reg_hr_update RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/reg_manager)

######### for test register #########
add_executable(test_reg_sr_add  ${SRC_LIST}  src/test_register/test_reg_sr_add.cpp)
target_link_libraries(test_reg_sr_add ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_reg_sr_add RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/reg_manager)

######### for test register #########
add_executable(test_reg_sr_update  ${SRC_LIST}  src/test_register/test_reg_sr_update.cpp)
target_link_libraries(test_reg_sr_update ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_reg_sr_update RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/reg_manager)

######### for test register #########
add_executable(test_reg_delete  ${SRC_LIST}  src/test_register/test_reg_delete.cpp)
target_link_libraries(test_reg_delete ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_reg_delete RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/reg_manager)

######### for test register #########
add_executable(test_reg_move  ${SRC_LIST}  src/test_register/test_reg_move.cpp)
target_link_libraries(test_reg_move ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_reg_move RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/reg_manager)

######### for test register #########
add_executable(test_reg_get_changed_list  ${SRC_LIST}  src/test_register/test_reg_get_changed_list.cpp)
target_link_libraries(test_reg_get_changed_list ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_reg_get_changed_list RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/reg_manager)

######### for test register #########
add_executable(test_reg_get_valid_list  ${SRC_LIST}  src/test_register/test_reg_get_valid_list.cpp)
target_link_libraries(test_reg_get_valid_list ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_reg_get_valid_list RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/reg_manager)

######### for test register #########
add_executable(test_reg_r_get  ${SRC_LIST}  src/test_register/test_reg_r_get.cpp)
target_link_libraries(test_reg_r_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_reg_r_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/reg_manager)

######### for test register #########
add_executable(test_reg_mr_get  ${SRC_LIST}  src/test_register/test_reg_mr_get.cpp)
target_link_libraries(test_reg_mr_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_reg_mr_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/reg_manager)

######### for test register #########
add_executable(test_reg_sr_get  ${SRC_LIST}  src/test_register/test_reg_sr_get.cpp)
target_link_libraries(test_reg_sr_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_reg_sr_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/reg_manager)

######### for test register #########
add_executable(test_reg_pr_get  ${SRC_LIST}  src/test_register/test_reg_pr_get.cpp)
target_link_libraries(test_reg_pr_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_reg_pr_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/reg_manager)

######### for test register #########
add_executable(test_reg_hr_get  ${SRC_LIST}  src/test_register/test_reg_hr_get.cpp)
target_link_libraries(test_reg_hr_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_reg_hr_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/reg_manager)

######### for test motion control #########
add_executable(joints_feed_back_get ${SRC_LIST} src/test_motion_control/axis_group/joints_feed_back_get.cpp)
target_link_libraries(joints_feed_back_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS joints_feed_back_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(manua_move_do_continus ${SRC_LIST} src/test_motion_control/axis_group/manua_move_do_continus.cpp)
target_link_libraries(manua_move_do_continus ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS manua_move_do_continus RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(manual_move_do_goto_cartesian_point ${SRC_LIST} src/test_motion_control/axis_group/manual_move_do_goto_cartesian_point.cpp)
target_link_libraries(manual_move_do_goto_cartesian_point ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS manual_move_do_goto_cartesian_point RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(manual_move_do_step ${SRC_LIST} src/test_motion_control/axis_group/manual_move_do_step.cpp)
target_link_libraries(manual_move_do_step ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS manual_move_do_step RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(manual_frame_set ${SRC_LIST} src/test_motion_control/axis_group/manual_frame_set.cpp)
target_link_libraries(manual_frame_set ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS manual_frame_set RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(motion_control_reset ${SRC_LIST} src/test_motion_control/motion_control_reset.cpp)
target_link_libraries(motion_control_reset ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS motion_control_reset RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control)

######### for test motion control #########
add_executable(motion_control_stop ${SRC_LIST} src/test_motion_control/motion_control_stop.cpp)
target_link_libraries(motion_control_stop ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS motion_control_stop RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control)

######### for test motion control #########
add_executable(manual_stop_do ${SRC_LIST} src/test_motion_control/axis_group/manual_stop_do.cpp)
target_link_libraries(manual_stop_do ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS manual_stop_do RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(global_vel_ratio_get ${SRC_LIST} src/test_motion_control/global_vel_ratio_get.cpp)
target_link_libraries(global_vel_ratio_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS global_vel_ratio_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control)

######### for test motion control #########
add_executable(global_vel_ratio_set ${SRC_LIST} src/test_motion_control/global_vel_ratio_set.cpp)
target_link_libraries(global_vel_ratio_set ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS global_vel_ratio_set RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control)

######### for test motion control #########
add_executable(global_acc_ratio_get ${SRC_LIST} src/test_motion_control/global_acc_ratio_get.cpp)
target_link_libraries(global_acc_ratio_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS global_acc_ratio_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control)

######### for test motion control #########
add_executable(global_acc_ratio_set ${SRC_LIST} src/test_motion_control/global_acc_ratio_set.cpp)
target_link_libraries(global_acc_ratio_set ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS global_acc_ratio_set RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control)

######### for test motion control #########
add_executable(hard_limit_set ${SRC_LIST} src/test_motion_control/axis_group/hard_limit_set.cpp)
target_link_libraries(hard_limit_set ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS hard_limit_set RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(hard_limit_get ${SRC_LIST} src/test_motion_control/axis_group/hard_limit_get.cpp)
target_link_libraries(hard_limit_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS hard_limit_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(coord_get ${SRC_LIST} src/test_motion_control/axis_group/coord_get.cpp)
target_link_libraries(coord_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS coord_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(coord_set ${SRC_LIST} src/test_motion_control/axis_group/coord_set.cpp)
target_link_libraries(coord_set ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS coord_set RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(tool_get ${SRC_LIST} src/test_motion_control/axis_group/tool_get.cpp)
target_link_libraries(tool_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS tool_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(tool_set ${SRC_LIST} src/test_motion_control/axis_group/tool_set.cpp)
target_link_libraries(tool_set ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS tool_set RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(convert_joint_to_cart ${SRC_LIST} src/test_motion_control/axis_group/convert_joint_to_cart.cpp)
target_link_libraries(convert_joint_to_cart ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS convert_joint_to_cart RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(convert_cart_to_joint ${SRC_LIST} src/test_motion_control/axis_group/convert_cart_to_joint.cpp)
target_link_libraries(convert_cart_to_joint ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS convert_cart_to_joint RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(manual_move_do_goto_joint_point ${SRC_LIST} src/test_motion_control/axis_group/manual_move_do_goto_joint_point.cpp)
target_link_libraries(manual_move_do_goto_joint_point ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS manual_move_do_goto_joint_point RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(soft_limit_get_manu ${SRC_LIST} src/test_motion_control/axis_group/soft_limit_get_manu.cpp)
target_link_libraries(soft_limit_get_manu ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS soft_limit_get_manu RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(soft_limit_set_manu ${SRC_LIST} src/test_motion_control/axis_group/soft_limit_set_manu.cpp)
target_link_libraries(soft_limit_set_manu ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS soft_limit_set_manu RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(soft_limit_set_user ${SRC_LIST} src/test_motion_control/axis_group/soft_limit_set_user.cpp)
target_link_libraries(soft_limit_set_user ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS soft_limit_set_user RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

##//////////////////////////////
######### for test motion control #########
add_executable(hard_limit_with_unit_get ${SRC_LIST} src/test_motion_control/axis_group/hard_limit_with_unit_get.cpp)
target_link_libraries(hard_limit_with_unit_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS hard_limit_with_unit_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(manual_step_cartesian_get ${SRC_LIST} src/test_motion_control/axis_group/manual_step_cartesian_get.cpp)
target_link_libraries(manual_step_cartesian_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS manual_step_cartesian_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(manual_step_cartesian_set ${SRC_LIST} src/test_motion_control/axis_group/manual_step_cartesian_set.cpp)
target_link_libraries(manual_step_cartesian_set ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS manual_step_cartesian_set RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(manual_step_prismatic_get ${SRC_LIST} src/test_motion_control/axis_group/manual_step_prismatic_get.cpp)
target_link_libraries(manual_step_prismatic_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS manual_step_prismatic_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(manual_step_prismatic_set ${SRC_LIST} src/test_motion_control/axis_group/manual_step_prismatic_set.cpp)
target_link_libraries(manual_step_prismatic_set ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS manual_step_prismatic_set RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(manual_step_robot_get ${SRC_LIST} src/test_motion_control/axis_group/manual_step_robot_get.cpp)
target_link_libraries(manual_step_robot_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS manual_step_robot_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(manual_step_robot_set ${SRC_LIST} src/test_motion_control/axis_group/manual_step_robot_set.cpp)
target_link_libraries(manual_step_robot_set ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS manual_step_robot_set RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(reference_point_delete ${SRC_LIST} src/test_motion_control/axis_group/reference_point_delete.cpp)
target_link_libraries(reference_point_delete ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS reference_point_delete RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(reference_point_is_exist ${SRC_LIST} src/test_motion_control/axis_group/reference_point_is_exist.cpp)
target_link_libraries(reference_point_is_exist ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS reference_point_is_exist RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(reference_point_save ${SRC_LIST} src/test_motion_control/axis_group/reference_point_save.cpp)
target_link_libraries(reference_point_save ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS reference_point_save RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(soft_limit_with_unit_manu_get ${SRC_LIST} src/test_motion_control/axis_group/soft_limit_with_unit_manu_get.cpp)
target_link_libraries(soft_limit_with_unit_manu_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS soft_limit_with_unit_manu_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(soft_limit_with_unit_user_get ${SRC_LIST} src/test_motion_control/axis_group/soft_limit_with_unit_user_get.cpp)
target_link_libraries(soft_limit_with_unit_user_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS soft_limit_with_unit_user_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(zero_error_get_all_mask_status ${SRC_LIST} src/test_motion_control/axis_group/zero_error_get_all_mask_status.cpp)
target_link_libraries(zero_error_get_all_mask_status ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS zero_error_get_all_mask_status RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(zero_error_lost_ignore ${SRC_LIST} src/test_motion_control/axis_group/zero_error_lost_ignore.cpp)
target_link_libraries(zero_error_lost_ignore ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS zero_error_lost_ignore RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(zero_point_calibrate_all_offset ${SRC_LIST} src/test_motion_control/axis_group/zero_point_calibrate_all_offset.cpp)
target_link_libraries(zero_point_calibrate_all_offset ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS zero_point_calibrate_all_offset RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(zero_point_calibrate_offsets ${SRC_LIST} src/test_motion_control/axis_group/zero_point_calibrate_offsets.cpp)
target_link_libraries(zero_point_calibrate_offsets ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS zero_point_calibrate_offsets RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(zero_point_calibrate_singal_offset ${SRC_LIST} src/test_motion_control/axis_group/zero_point_calibrate_singal_offset.cpp)
target_link_libraries(zero_point_calibrate_singal_offset ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS zero_point_calibrate_singal_offset RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(zero_point_fast_calibrate_all_offsets ${SRC_LIST} src/test_motion_control/axis_group/zero_point_fast_calibrate_all_offsets.cpp)
target_link_libraries(zero_point_fast_calibrate_all_offsets ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS zero_point_fast_calibrate_all_offsets RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(zero_point_fast_calibrate_offsets ${SRC_LIST} src/test_motion_control/axis_group/zero_point_fast_calibrate_offsets.cpp)
target_link_libraries(zero_point_fast_calibrate_offsets ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS zero_point_fast_calibrate_offsets RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(point_fast_calibrate_singal_offset ${SRC_LIST} src/test_motion_control/axis_group/point_fast_calibrate_singal_offset.cpp)
target_link_libraries(point_fast_calibrate_singal_offset ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS point_fast_calibrate_singal_offset RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(zero_point_get_all_offsets ${SRC_LIST} src/test_motion_control/axis_group/zero_point_get_all_offsets.cpp)
target_link_libraries(zero_point_get_all_offsets ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS zero_point_get_all_offsets RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(zero_point_save_all_offsets ${SRC_LIST} src/test_motion_control/axis_group/zero_point_save_all_offsets.cpp)
target_link_libraries(zero_point_save_all_offsets ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS zero_point_save_all_offsets RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(zero_point_set_single_status ${SRC_LIST} src/test_motion_control/axis_group/zero_point_set_single_status.cpp)
target_link_libraries(zero_point_set_single_status ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS zero_point_set_single_status RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)

######### for test motion control #########
add_executable(get_axis_group_info_list ${SRC_LIST} src/test_motion_control/get_axis_group_info_list.cpp)
target_link_libraries(get_axis_group_info_list ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_axis_group_info_list RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control)

######### for test motion control #########
add_executable(zero_point_single_offset_set ${SRC_LIST} src/test_motion_control/axis_group/zero_point_single_offset_set.cpp)
target_link_libraries(zero_point_single_offset_set ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS zero_point_single_offset_set RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/motion_control/axis_group)
#///////////////////////////////////////


######### for test io mapping  #########
add_executable(get_di ${SRC_LIST} src/test_io_mapping/get_di.cpp)
target_link_libraries(get_di ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_di RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/io_mapping)

######### for test io mapping  #########
add_executable(get_do ${SRC_LIST} src/test_io_mapping/get_do.cpp)
target_link_libraries(get_do ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_do RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/io_mapping)

######### for test io mapping  #########
add_executable(set_di ${SRC_LIST} src/test_io_mapping/set_di.cpp)
target_link_libraries(set_di ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS set_di RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/io_mapping)

######### for test io mapping  #########
add_executable(set_do ${SRC_LIST} src/test_io_mapping/set_do.cpp)
target_link_libraries(set_do ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS set_do RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/io_mapping)

######### for test io mapping  #########
add_executable(cycle_set_do ${SRC_LIST} src/test_io_mapping/cycle_set_do.cpp)
target_link_libraries(cycle_set_do ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS cycle_set_do RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/io_mapping)

######### for test io mapping  #########
add_executable(get_ri ${SRC_LIST} src/test_io_mapping/get_ri.cpp)
target_link_libraries(get_ri ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_ri RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/io_mapping)

######### for test io mapping  #########
add_executable(get_ro ${SRC_LIST} src/test_io_mapping/get_ro.cpp)
target_link_libraries(get_ro ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_ro RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/io_mapping)

######### for test io mapping  #########
add_executable(set_ri ${SRC_LIST} src/test_io_mapping/set_ri.cpp)
target_link_libraries(set_ri ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS set_ri RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/io_mapping)

######### for test io mapping  #########
add_executable(set_ro ${SRC_LIST} src/test_io_mapping/set_ro.cpp)
target_link_libraries(set_ro ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS set_ro RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/io_mapping)

######### for test io mapping  #########
add_executable(sync_file_io_mapping ${SRC_LIST} src/test_io_mapping/sync_file_io_mapping.cpp)
target_link_libraries(sync_file_io_mapping ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS sync_file_io_mapping RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/io_mapping)

######### for test io mapping  #########
add_executable(sync_file_io_status ${SRC_LIST} src/test_io_mapping/sync_file_io_status.cpp)
target_link_libraries(sync_file_io_status ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS sync_file_io_status RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/io_mapping)

######### for test controller #########
add_executable(test_controller_shutdown  ${SRC_LIST}  src/test_controller/test_controller_shutdown.cpp)
target_link_libraries(test_controller_shutdown ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_controller_shutdown RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/controller)

######### for test controller #########
add_executable(test_controller_call_estop  ${SRC_LIST}  src/test_controller/test_controller_call_estop.cpp)
target_link_libraries(test_controller_call_estop ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_controller_call_estop RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/controller)

######### for test controller #########
add_executable(test_controller_call_reset  ${SRC_LIST}  src/test_controller/test_controller_call_reset.cpp)
target_link_libraries(test_controller_call_reset ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_controller_call_reset RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/controller)

######### for test controller #########
add_executable(test_controller_ctrl_status  ${SRC_LIST}  src/test_controller/test_controller_ctrl_status.cpp)
target_link_libraries(test_controller_ctrl_status ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_controller_ctrl_status RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/controller)

######### for test controller #########
add_executable(test_controller_get_interpreter_status  ${SRC_LIST}  src/test_controller/test_controller_get_interpreter_status.cpp)
target_link_libraries(test_controller_get_interpreter_status ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_controller_get_interpreter_status RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/controller)

######### for test controller #########
add_executable(test_controller_get_robot_status  ${SRC_LIST}  src/test_controller/test_controller_get_robot_status.cpp)
target_link_libraries(test_controller_get_robot_status ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_controller_get_robot_status RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/controller)

######### for test controller #########
add_executable(test_controller_get_running_status  ${SRC_LIST}  src/test_controller/test_controller_get_running_status.cpp)
target_link_libraries(test_controller_get_running_status ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_controller_get_running_status RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/controller)

######### for test controller #########
add_executable(test_controller_get_safety_alarm  ${SRC_LIST}  src/test_controller/test_controller_get_safety_alarm.cpp)
target_link_libraries(test_controller_get_safety_alarm ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_controller_get_safety_alarm RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/controller)

######### for test controller #########
add_executable(test_controller_get_servo_status  ${SRC_LIST}  src/test_controller/test_controller_get_servo_status.cpp)
target_link_libraries(test_controller_get_servo_status ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_controller_get_servo_status RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/controller)

######### for test controller #########
add_executable(test_controller_get_user_opmode  ${SRC_LIST}  src/test_controller/test_controller_get_user_opmode.cpp)
target_link_libraries(test_controller_get_user_opmode ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_controller_get_user_opmode RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/controller)

######### for test client #########
add_executable(test_controller_set_user_mode  ${SRC_LIST}  src/test_controller/test_controller_set_user_mode.cpp)
target_link_libraries(test_controller_set_user_mode ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_controller_set_user_mode RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/controller)

######### for test interpreter #########
add_executable(test_interpreter_abort ${SRC_LIST} src/test_interpreter/test_interpreter_abort.cpp)
target_link_libraries(test_interpreter_abort ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_interpreter_abort RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/interpreter)

######### for test interpreter #########
add_executable(test_interpreter_backward ${SRC_LIST} src/test_interpreter/test_interpreter_backward.cpp)
target_link_libraries(test_interpreter_backward ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_interpreter_backward RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/interpreter)

######### for test interpreter #########
add_executable(test_interpreter_debug ${SRC_LIST} src/test_interpreter/test_interpreter_debug.cpp)
target_link_libraries(test_interpreter_debug ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_interpreter_debug RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/interpreter)

######### for test interpreter #########
add_executable(test_interpreter_foward ${SRC_LIST} src/test_interpreter/test_interpreter_foward.cpp)
target_link_libraries(test_interpreter_foward ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_interpreter_foward RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/interpreter)

######### for test interpreter #########
add_executable(test_interpreter_jump ${SRC_LIST} src/test_interpreter/test_interpreter_jump.cpp)
target_link_libraries(test_interpreter_jump ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_interpreter_jump RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/interpreter)

######### for test interpreter #########
add_executable(test_interpreter_pause ${SRC_LIST} src/test_interpreter/test_interpreter_pause.cpp)
target_link_libraries(test_interpreter_pause ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_interpreter_pause RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/interpreter)

######### for test interpreter #########
add_executable(test_interpreter_resume ${SRC_LIST} src/test_interpreter/test_interpreter_resume.cpp)
target_link_libraries(test_interpreter_resume ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_interpreter_resume RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/interpreter)

######### for test interpreter #########
add_executable(test_interpreter_start ${SRC_LIST} src/test_interpreter/test_interpreter_start.cpp)
target_link_libraries(test_interpreter_start ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_interpreter_start RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/interpreter)

######### for test interpreter #########
add_executable(test_interpreter_switch_step ${SRC_LIST} src/test_interpreter/test_interpreter_switch_step.cpp)
target_link_libraries(test_interpreter_switch_step ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS test_interpreter_switch_step RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/interpreter)


######### for test interpreter #########
add_executable(get_method ${SRC_LIST} src/test_program_launching/get_method.cpp)
target_link_libraries(get_method ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_method RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/program_launching)

######### for test interpreter #########
add_executable(set_method ${SRC_LIST} src/test_program_launching/set_method.cpp)
target_link_libraries(set_method ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS set_method RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/program_launching)

######### for test interpreter #########
add_executable(sync_file_macro_config ${SRC_LIST} src/test_program_launching/sync_file_macro_config.cpp)
target_link_libraries(sync_file_macro_config ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS sync_file_macro_config RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/program_launching)

######### for test device manager  #########
add_executable(get_device_list ${SRC_LIST} src/test_device_manager/get_device_list.cpp)
target_link_libraries(get_device_list ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_device_list RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/device_manager)

######### for test device manager  #########
add_executable(get_FRP8A_io_device_info ${SRC_LIST} src/test_device_manager/get_FRP8A_io_device_info.cpp)
target_link_libraries(get_FRP8A_io_device_info ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_FRP8A_io_device_info RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/device_manager)

######### for test device manager  #########
add_executable(get_modbus_io_device_info ${SRC_LIST} src/test_device_manager/get_modbus_io_device_info.cpp)
target_link_libraries(get_modbus_io_device_info ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_modbus_io_device_info RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/device_manager)

######### for test device manager  #########
add_executable(get_io_device_info_list ${SRC_LIST} src/test_device_manager/get_io_device_info_list.cpp)
target_link_libraries(get_io_device_info_list ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_io_device_info_list RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/device_manager)

####### for test modbus  #########
add_executable(server_close ${SRC_LIST} src/test_modbus/server/server_close.cpp)
target_link_libraries(server_close ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS server_close RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/server)

####### for test modbus  #########
add_executable(get_server_config_params ${SRC_LIST} src/test_modbus/server/get_server_config_params.cpp)
target_link_libraries(get_server_config_params ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_server_config_params RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/server)


####### for test modbus server #########
add_executable(function_info_get ${SRC_LIST} src/test_modbus/server/function_info_get.cpp)
target_link_libraries(function_info_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS function_info_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/server)

####### for test modbus server #########
add_executable(function_info_set ${SRC_LIST} src/test_modbus/server/function_info_set.cpp)
target_link_libraries(function_info_set ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS function_info_set RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/server)

####### for test modbus server #########
add_executable(get_running_status ${SRC_LIST} src/test_modbus/server/get_running_status.cpp)
target_link_libraries(get_running_status ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_running_status RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/server)

####### for test modbus  #########
add_executable(enable_status_get ${SRC_LIST} src/test_modbus/server/enable_status_get.cpp)
target_link_libraries(enable_status_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS enable_status_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/server)

####### for test modbus  #########
add_executable(enable_status_set ${SRC_LIST} src/test_modbus/server/enable_status_set.cpp)
target_link_libraries(enable_status_set ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS enable_status_set RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/server)


####### for test modbus  #########
add_executable(start_info_get ${SRC_LIST} src/test_modbus/server/start_info_get.cpp)
target_link_libraries(start_info_get ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS start_info_get RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/server)


####### for test modbus  #########
add_executable(start_info_set ${SRC_LIST} src/test_modbus/server/start_info_set.cpp)
target_link_libraries(start_info_set ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS start_info_set RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/server)

####### for test modbus  #########
add_executable(server_open ${SRC_LIST} src/test_modbus/server/server_open.cpp)
target_link_libraries(server_open ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS server_open RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/server)

####### for test modbus  #########
add_executable(set_start_mode ${SRC_LIST} src/test_modbus/set_start_mode.cpp)
target_link_libraries(set_start_mode ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS set_start_mode RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus)

####### for test modbus  #########
add_executable(get_start_mode ${SRC_LIST} src/test_modbus/get_start_mode.cpp)
target_link_libraries(get_start_mode ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_start_mode RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus)

####### for test modbus  #########
add_executable(read_coils ${SRC_LIST} src/test_modbus/read_coils.cpp)
target_link_libraries(read_coils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS read_coils RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus)

####### for test modbus  #########
add_executable(read_discrepte_input ${SRC_LIST} src/test_modbus/read_discrepte_input.cpp)
target_link_libraries(read_discrepte_input ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS read_discrepte_input RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus)

####### for test modbus  #########
add_executable(read_holding_reg ${SRC_LIST} src/test_modbus/read_holding_reg.cpp)
target_link_libraries(read_holding_reg ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS read_holding_reg RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus)

####### for test modbus  #########
add_executable(read_input_reg ${SRC_LIST} src/test_modbus/read_input_reg.cpp)
target_link_libraries(read_input_reg ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS read_input_reg RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus)

####### for test modbus  #########
add_executable(write_coils ${SRC_LIST} src/test_modbus/write_coils.cpp)
target_link_libraries(write_coils ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS write_coils RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus)

####### for test modbus  #########
add_executable(write_holding_reg ${SRC_LIST} src/test_modbus/write_holding_reg.cpp)
target_link_libraries(write_holding_reg ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS write_holding_reg RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus)

####### for test modbus  #########
add_executable(add_client ${SRC_LIST} src/test_modbus/client/add_client.cpp)
target_link_libraries(add_client ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS add_client RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/client)

####### for test modbus  #########
add_executable(close_client ${SRC_LIST} src/test_modbus/client/close_client.cpp)
target_link_libraries(close_client ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS close_client RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/client)

####### for test modbus  #########
add_executable(connec_client ${SRC_LIST} src/test_modbus/client/connec_client.cpp)
target_link_libraries(connec_client ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS connec_client RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/client)

####### for test modbus  #########
add_executable(delete_client ${SRC_LIST} src/test_modbus/client/delete_client.cpp)
target_link_libraries(delete_client ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS delete_client RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/client)

####### for test modbus  #########
add_executable(get_client_all_function_addr_info ${SRC_LIST} src/test_modbus/client/get_client_all_function_addr_info.cpp)
target_link_libraries(get_client_all_function_addr_info ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_client_all_function_addr_info RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/client)

####### for test modbus  #########
add_executable(get_client_config ${SRC_LIST} src/test_modbus/client/get_client_config.cpp)
target_link_libraries(get_client_config ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_client_config RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/client)

####### for test modbus  #########
add_executable(get_client_control_status ${SRC_LIST} src/test_modbus/client/get_client_control_status.cpp)
target_link_libraries(get_client_control_status ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_client_control_status RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/client)

####### for test modbus  #########
add_executable(get_client_enable_status ${SRC_LIST} src/test_modbus/client/get_client_enable_status.cpp)
target_link_libraries(get_client_enable_status ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_client_enable_status RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/client)

####### for test modbus  #########
add_executable(get_client_id_list ${SRC_LIST} src/test_modbus/client/get_client_id_list.cpp)
target_link_libraries(get_client_id_list ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_client_id_list RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/client)

####### for test modbus  #########
add_executable(get_client_start_info ${SRC_LIST} src/test_modbus/client/get_client_start_info.cpp)
target_link_libraries(get_client_start_info ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_client_start_info RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/client)

####### for test modbus  #########
add_executable(is_client_connected ${SRC_LIST} src/test_modbus/client/is_client_connected.cpp)
target_link_libraries(is_client_connected ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS is_client_connected RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/client)

####### for test modbus  #########
add_executable(set_client_all_function_addr_info ${SRC_LIST} src/test_modbus/client/set_client_all_function_addr_info.cpp)
target_link_libraries(set_client_all_function_addr_info ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS set_client_all_function_addr_info RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/client)

####### for test modbus  #########
add_executable(set_client_enable_status ${SRC_LIST} src/test_modbus/client/set_client_enable_status.cpp)
target_link_libraries(set_client_enable_status ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS set_client_enable_status RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/client)

####### for test modbus  #########
add_executable(update_client_start_info ${SRC_LIST} src/test_modbus/client/update_client_start_info.cpp)
target_link_libraries(update_client_start_info ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS update_client_start_info RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/client)

####### for test modbus  #########
add_executable(replace_client ${SRC_LIST} src/test_modbus/client/replace_client.cpp)
target_link_libraries(replace_client ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS replace_client RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/client)

####### for test modbus  #########
add_executable(get_connected_id_list ${SRC_LIST} src/test_modbus/client/get_connected_id_list.cpp)
target_link_libraries(get_connected_id_list ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} test_tp_comm protos)
install(TARGETS get_connected_id_list RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/modbus/client)
