cmake_minimum_required(VERSION 2.8.3)
project(sensor_process)

include_directories(
	include
	../common
	../motion_control/include
	../log_manager/include
    ../error_queue/include
    ../system_model_manager/include
    ../core_comm/include/comm
    ../basic_alg/include
    ../controller/include
    ../
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(src SRC)

add_library(sensor_process SHARED
    ${SRC}
)

link_directories(
	../../install/lib
)

target_link_libraries(io_1000
    ${rtm_LIBRARIES}
    motion_control
    log_manager_producer
    error_queue
    system_model_manager
    basic_alg
    controller
)
