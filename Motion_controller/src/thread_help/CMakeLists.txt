cmake_minimum_required(VERSION 2.8.3)
project(thread_help)

include_directories(
	include
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(src SRC)
add_library(thread_help SHARED
	${SRC}
)

target_link_libraries(thread_help
    ${rtm_LIBRARIES}
)
