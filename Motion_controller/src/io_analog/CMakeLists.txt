cmake_minimum_required(VERSION 2.8.3)
project(io_analog)

include_directories(
	include
	../common
	../yaml_help/include
	../log_manager/include
    ../base_device/include
    ../error_queue/include
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(src SRC)

add_library(io_analog SHARED
    ${SRC}
)

link_directories(
	../../install/lib
)

target_link_libraries(io_analog
    ${rtm_LIBRARIES}
    yaml_help
    log_manager_producer
    error_queue
    base_device
)
