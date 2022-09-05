cmake_minimum_required(VERSION 2.8.3)
project(sensor_process)

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
    ../motion_control/include
    ../controller/include
    ../pb/include
    ../protos/include
    ../tp_comm/include
    ../io_1000/include
    ../io_safety/include
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(src SRC)

add_library(sensor_process SHARED
    ${SRC}
)

link_directories(
	../../install/lib
)

target_link_libraries(sensor_process
    ${rtm_LIBRARIES}
    motion_control
    log_manager_producer
    error_queue
    system_model_manager
    basic_alg
)
