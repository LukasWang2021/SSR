cmake_minimum_required(VERSION 2.8.3)
project(io_safety)

include_directories(
	include
	../common
	../yaml_help/include
	../log_manager/include
    ../error_queue/include
    ../base_device/include
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(src SRC)

add_library(io_safety SHARED
    ${SRC}
)

link_directories(
	../../install/lib
)

target_link_libraries(io_safety
    ${rtm_LIBRARIES}
    yaml_help
    log_manager_producer
    error_queue
    base_device
)
