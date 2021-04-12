cmake_minimum_required(VERSION 2.8.3)
project(xml_help)

include_directories(
	include
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(src SRC)
add_library(xml_help SHARED
	${SRC}
)

target_link_libraries(xml_help
	${rtm_LIBRARIES}
)


