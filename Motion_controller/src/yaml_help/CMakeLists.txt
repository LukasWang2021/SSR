cmake_minimum_required(VERSION 2.8.3)
project(yaml_help)

include_directories(
	include
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(src SRC)
add_library(yaml_help SHARED
	${SRC}
)

link_directories(
	../../install/lib
)
target_link_libraries(yaml_help
	${rtm_LIBRARIES}
)


