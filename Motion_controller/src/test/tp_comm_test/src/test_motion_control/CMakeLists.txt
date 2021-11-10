cmake_minimum_required(VERSION 2.8.3)
project(tp_comm_test)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D PB_FIELD_32BIT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D PB_FIELD_32BIT")
SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/motion_control)

##################################################################################################controller
######### global_vel_ratio_set #########
add_executable(global_vel_ratio_set global_vel_ratio_set.cpp)
target_link_libraries(global_vel_ratio_set ${rtm_LIBRARIES} pb tp_comm_test protos)

######### global_vel_ratio_get #########
add_executable(global_vel_ratio_get global_vel_ratio_get.cpp)
target_link_libraries(global_vel_ratio_get ${rtm_LIBRARIES} pb tp_comm_test protos)

######### global_acc_ratio_set #########
add_executable(global_acc_ratio_set global_acc_ratio_set.cpp)
target_link_libraries(global_acc_ratio_set ${rtm_LIBRARIES} pb tp_comm_test protos)

######### global_acc_ratio_get #########
add_executable(global_acc_ratio_get global_acc_ratio_get.cpp)
target_link_libraries(global_acc_ratio_get ${rtm_LIBRARIES} pb tp_comm_test protos)

######### get_postrue_by_joint #########
add_executable(get_postrue_by_joint get_postrue_by_joint.cpp)
target_link_libraries(get_postrue_by_joint ${rtm_LIBRARIES} pb tp_comm_test protos)



######### convert_cart_to_joint #########
add_executable(convert_cart_to_joint ./axis_group/convert_cart_to_joint.cpp)
target_link_libraries(convert_cart_to_joint ${rtm_LIBRARIES} pb tp_comm_test protos)

######### convert_joint_to_cart #########
add_executable(convert_joint_to_cart ./axis_group/convert_joint_to_cart.cpp)
target_link_libraries(convert_joint_to_cart ${rtm_LIBRARIES} pb tp_comm_test protos)

######### coord_get #########
add_executable(coord_get ./axis_group/coord_get.cpp)
target_link_libraries(coord_get ${rtm_LIBRARIES} pb tp_comm_test protos)

######### coord_set #########
add_executable(coord_set ./axis_group/coord_set.cpp)
target_link_libraries(coord_set ${rtm_LIBRARIES} pb tp_comm_test protos)

######### fcp_pose_from_base #########
add_executable(fcp_pose_from_base ./axis_group/fcp_pose_from_base.cpp)
target_link_libraries(fcp_pose_from_base ${rtm_LIBRARIES} pb tp_comm_test protos)

######### hard_limit_get #########
add_executable(hard_limit_get ./axis_group/hard_limit_get.cpp)
target_link_libraries(hard_limit_get ${rtm_LIBRARIES} pb tp_comm_test protos)

######### hard_limit_set #########
add_executable(hard_limit_set ./axis_group/hard_limit_set.cpp)
target_link_libraries(hard_limit_set ${rtm_LIBRARIES} pb tp_comm_test protos)

######### joints_feed_back_get #########
add_executable(joints_feed_back_get ./axis_group/joints_feed_back_get.cpp)
target_link_libraries(joints_feed_back_get ${rtm_LIBRARIES} pb tp_comm_test protos)

######### manual_joint_step_get #########
add_executable(manual_joint_step_get ./axis_group/manual_joint_step_get.cpp)
target_link_libraries(manual_joint_step_get ${rtm_LIBRARIES} pb tp_comm_test protos)

######### manual_joint_step_set #########
add_executable(manual_joint_step_set ./axis_group/manual_joint_step_set.cpp)
target_link_libraries(manual_joint_step_set ${rtm_LIBRARIES} pb tp_comm_test protos)


######### manual_move_do_continus #########
add_executable(manual_move_do_continus ./axis_group/manual_move_do_continus.cpp)
target_link_libraries(manual_move_do_continus ${rtm_LIBRARIES} pb tp_comm_test protos)

######### manual_move_do_goto_cartesian_point #########
add_executable(manual_move_do_goto_cartesian_point ./axis_group/manual_move_do_goto_cartesian_point.cpp)
target_link_libraries(manual_move_do_goto_cartesian_point ${rtm_LIBRARIES} pb tp_comm_test protos)

######### manual_move_do_goto_joint_point #########
add_executable(manual_move_do_goto_joint_point ./axis_group/manual_move_do_goto_joint_point.cpp)
target_link_libraries(manual_move_do_goto_joint_point ${rtm_LIBRARIES} pb tp_comm_test protos)

######### manual_move_do_step #########
add_executable(manual_move_do_step ./axis_group/manual_move_do_step.cpp)
target_link_libraries(manual_move_do_step ${rtm_LIBRARIES} pb tp_comm_test protos)

######### manual_step_cartesian_get #########
add_executable(manual_step_cartesian_get ./axis_group/manual_step_cartesian_get.cpp)
target_link_libraries(manual_step_cartesian_get ${rtm_LIBRARIES} pb tp_comm_test protos)

######### manual_step_cartesian_set #########
add_executable(manual_step_cartesian_set ./axis_group/manual_step_cartesian_set.cpp)
target_link_libraries(manual_step_cartesian_set ${rtm_LIBRARIES} pb tp_comm_test protos)

######### manual_step_orientation_get #########
add_executable(manual_step_orientation_get ./axis_group/manual_step_orientation_get.cpp)
target_link_libraries(manual_step_orientation_get ${rtm_LIBRARIES} pb tp_comm_test protos)

######### manual_step_orientation_set #########
add_executable(manual_step_orientation_set ./axis_group/manual_step_orientation_set.cpp)
target_link_libraries(manual_step_orientation_set ${rtm_LIBRARIES} pb tp_comm_test protos)

######### manual_stop_do #########
add_executable(manual_stop_do ./axis_group/manual_stop_do.cpp)
target_link_libraries(manual_stop_do ${rtm_LIBRARIES} pb tp_comm_test protos)

######### soft_limit_get_manu #########
add_executable(soft_limit_get_manu ./axis_group/soft_limit_get_manu.cpp)
target_link_libraries(soft_limit_get_manu ${rtm_LIBRARIES} pb tp_comm_test protos)

######### soft_limit_get_user #########
add_executable(soft_limit_get_user ./axis_group/soft_limit_get_user.cpp)
target_link_libraries(soft_limit_get_user ${rtm_LIBRARIES} pb tp_comm_test protos)

######### soft_limit_set_manu #########
add_executable(soft_limit_set_manu ./axis_group/soft_limit_set_manu.cpp)
target_link_libraries(soft_limit_set_manu ${rtm_LIBRARIES} pb tp_comm_test protos)

######### soft_limit_set_user #########
add_executable(soft_limit_set_user ./axis_group/soft_limit_set_user.cpp)
target_link_libraries(soft_limit_set_user ${rtm_LIBRARIES} pb tp_comm_test protos)

######### tcp_current_pose_get #########
add_executable(tcp_current_pose_get ./axis_group/tcp_current_pose_get.cpp)
target_link_libraries(tcp_current_pose_get ${rtm_LIBRARIES} pb tp_comm_test protos)

######### tool_get #########
add_executable(tool_get ./axis_group/tool_get.cpp)
target_link_libraries(tool_get ${rtm_LIBRARIES} pb tp_comm_test protos)

######### tool_set #########
add_executable(tool_set ./axis_group/tool_set.cpp)
target_link_libraries(tool_set ${rtm_LIBRARIES} pb tp_comm_test protos)

######### trajectory_eulerFileConvert2JointFile #########
add_executable(trajectory_eulerFileConvert2JointFile ./axis_group/trajectory_eulerFileConvert2JointFile.cpp)
target_link_libraries(trajectory_eulerFileConvert2JointFile ${rtm_LIBRARIES} pb tp_comm_test protos)

######### trajectory_file_offline_set #########
add_executable(trajectory_file_offline_set ./axis_group/trajectory_file_offline_set.cpp)
target_link_libraries(trajectory_file_offline_set ${rtm_LIBRARIES} pb tp_comm_test protos)

######### trajectory_offline_move #########
add_executable(trajectory_offline_move ./axis_group/trajectory_offline_move.cpp)
target_link_libraries(trajectory_offline_move ${rtm_LIBRARIES} pb tp_comm_test protos)

######### trajectory_offline_prepare #########
add_executable(trajectory_offline_prepare ./axis_group/trajectory_offline_prepare.cpp)
target_link_libraries(trajectory_offline_prepare ${rtm_LIBRARIES} pb tp_comm_test protos)



######### user_coord_id_get #########
add_executable(user_coord_id_get ./axis_group/user_coord_id_get.cpp)
target_link_libraries(user_coord_id_get ${rtm_LIBRARIES} pb tp_comm_test protos)

######### user_coord_id_set #########
add_executable(user_coord_id_set ./axis_group/user_coord_id_set.cpp)
target_link_libraries(user_coord_id_set ${rtm_LIBRARIES} pb tp_comm_test protos)

######### valid_current_offsets #########
add_executable(valid_current_offsets ./axis_group/valid_current_offsets.cpp)
target_link_libraries(valid_current_offsets ${rtm_LIBRARIES} pb tp_comm_test protos)

######### zero_error_get_all_mask_status #########
add_executable(zero_error_get_all_mask_status ./axis_group/zero_error_get_all_mask_status.cpp)
target_link_libraries(zero_error_get_all_mask_status ${rtm_LIBRARIES} pb tp_comm_test protos)

######### zero_error_lost_ignore #########
add_executable(zero_error_lost_ignore ./axis_group/zero_error_lost_ignore.cpp)
target_link_libraries(zero_error_lost_ignore ${rtm_LIBRARIES} pb tp_comm_test protos)

######### zero_point_get_all_offsets #########
add_executable(zero_point_get_all_offsets ./axis_group/zero_point_get_all_offsets.cpp)
target_link_libraries(zero_point_get_all_offsets ${rtm_LIBRARIES} pb tp_comm_test protos)

######### zero_point_set_all_offsets #########
add_executable(zero_point_set_all_offsets ./axis_group/zero_point_set_all_offsets.cpp)
target_link_libraries(zero_point_set_all_offsets ${rtm_LIBRARIES} pb tp_comm_test protos)

######### zero_point_set_single_status #########
add_executable(zero_point_set_single_status ./axis_group/zero_point_set_single_status.cpp)
target_link_libraries(zero_point_set_single_status ${rtm_LIBRARIES} pb tp_comm_test protos)

######### zero_point_single_offset_set #########
add_executable(zero_point_single_offset_set ./axis_group/zero_point_single_offset_set.cpp)
target_link_libraries(zero_point_single_offset_set ${rtm_LIBRARIES} pb tp_comm_test protos)
