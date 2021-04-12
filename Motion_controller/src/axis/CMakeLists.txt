cmake_minimum_required(VERSION 2.8.3)
project(axis)

include_directories(
	include
	../common
	../yaml_help/include
	../core_comm/include 
	../servo_comm/include
	../log_manager/include
    ../algorithm_base/include 
	../system_model_manager/include
	../xml_help/include 
    ../error_queue/include
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(src SRC)

add_library(axis SHARED
    ${SRC}
)

link_directories(
	../../install/lib
)

target_link_libraries(axis
    ${rtm_LIBRARIES}
    yaml_help
    core_comm_system
    servo_comm
    log_manager_producer
    algorithm_base
    system_model_manager
    xml_help
    error_queue
)

