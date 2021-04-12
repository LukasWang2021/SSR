cmake_minimum_required(VERSION 2.8.3)
project(file_manager)

include_directories(
	include
	../common
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(src SRC)
add_library(file_manager SHARED
        ${SRC}
)

target_link_libraries(file_manager
        ${rtm_LIBRARIES}
)


