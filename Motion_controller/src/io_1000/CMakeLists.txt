cmake_minimum_required(VERSION 2.8.3)
project(io_1000)

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

add_library(io_1000 SHARED
    ${SRC}
)

link_directories(
	../../install/lib
)

target_link_libraries(io_1000
    ${rtm_LIBRARIES}
    yaml_help
    log_manager_producer
    error_queue
    base_device
)
