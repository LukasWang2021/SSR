cmake_minimum_required(VERSION 2.8.3)
project(motion_control)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -lrt -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT -lboost_system")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -lrt -O0 -D PB_FIELD_32BIT")

include_directories(
    include
    ../common
    ../log_manager/include
    ../yaml_help/include/
    ../xml_help/include
    ../sem_help/include
    ../base_device/include 
    ../algorithm_base/include 
    ../core_comm/include 
	../servo_comm/include
    ../system_model_manager/include
    ../reg_manager/include
    ../coordinate_manager/include
    ../tool_manager/include 
    ../basic_alg/include
    ../error_queue/include
    ../thread_help/include  
    ../axis/include
    ../group/include
    ../rtm_spi/include
    ../nvram/include
    ../joint_constraint/include
    ../kinematics_alg/include
    ../dynamic_alg/include 
    ../transformation/include
    ../trajectory_planner/include
    ${rtm_INCLUDE_DIRS}
)

aux_source_directory(src SRC)

add_library(motion_control SHARED
    ${SRC}
)

link_directories(
	../../install/lib
)
target_link_libraries(motion_control
    ${rtm_LIBRARIES}
    yaml_help
    xml_help 
    sem_help 
    log_manager_producer
    base_device
    algorithm_base
    error_queue
    thread_help
    core_comm_system
    servo_comm
    system_model_manager
    axis
    group
    nvram_handler
    reg_manager
    coordinate_manager
    tool_manager
    basic_alg 
    joint_constraint
    dynamic_alg 
    kinematics_alg
    transformation
    trajectory_planner
)

install(FILES ./config/base_group.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/axis_group
)

install(FILES ./config/arm_group.yaml
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

install(FILES ./config/arm_manual_teach.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/axis_group
)

