cmake_minimum_required(VERSION 2.8.3)
project(pb)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D PB_FIELD_32BIT")

include_directories(
	include
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(src SRC)
add_library(pb SHARED
	${SRC}
)

target_link_libraries(pb
    ${rtm_LIBRARIES}
)


