cmake_minimum_required(VERSION 2.8.3)
project(error_queue)

include_directories(
	include
	../common
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(src SRC)

add_library(error_queue SHARED
    ${SRC}
)

link_directories(
	../../install/lib
)

target_link_libraries(error_queue
    ${rtm_LIBRARIES}
)


