cmake_minimum_required(VERSION 2.8.3)
project(motion_control)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -lrt -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT -lboost_system")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -lrt -O0 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")

include_directories(
    include
    ../common
    ../log_manager/include
    ../yaml_help/include/
    ../base_device/include 
    ../coordinate_manager/include
    ../tool_manager/include 
    ../basic_alg/include
    ../dynamic_alg/include 
    ../xml_help/include
    ../error_queue/include
    ../thread_help/include
    ../kinematics_alg/include
    ../transformation/include
    ../nvram/include
    ../rtm_spi/include
    ../trajectory_planner/include
    ../joint_constraint/include
    ${rtm_INCLUDE_DIRS}
)

add_library(motion_control SHARED
    src/motion_control.cpp
    src/motion_control_param.cpp
    src/motion_control_arm_group.cpp
    src/motion_control_base_group.cpp
    src/motion_control_state_machine.cpp
    src/motion_control_offline_trajectory.cpp
    src/motion_control_manual_teach.cpp
    src/motion_control_core_interface.cpp
    src/motion_control_offset_calibrator.cpp
)

target_link_libraries(motion_control
    ${rtm_LIBRARIES}
    yaml_help
    log_manager_producer
    base_device
    coordinate_manager
    tool_manager
    basic_alg 
    dynamic_alg 
    xml_help 
    error_queue
    thread_help
    kinematics_alg
    transformation
    nvram_handler
    dynamic_alg
    trajectory_planner
    joint_constraint
)

install(FILES ./config/motion_control.yaml 
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/component/
)

install(FILES ./config/base_group.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/axis_group
)

install(FILES ./config/arm_group.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/axis_group
)

install(FILES ./config/scara_group.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/axis_group
)

install(FILES ./config/hard_constraint.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/axis_group
)

install(FILES ./config/firm_constraint.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/axis_group
)

install(FILES ./config/soft_constraint.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/axis_group
)

install(FILES ./config/group_offset.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/axis_group
)

install(FILES ./config/arm_group_offset_reference.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/axis_group
)

install(FILES ./config/group_gear_ratio.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/axis_group
)

install(FILES ./config/group_coupling_coeff.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/axis_group
)

install(FILES ./config/arm_manual_teach.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/axis_group
)

install(FILES ./config/scara_manual_teach.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/axis_group
)

install(FILES ./config/segment_alg.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/component
)

