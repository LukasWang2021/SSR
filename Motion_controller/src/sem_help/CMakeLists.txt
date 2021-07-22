cmake_minimum_required(VERSION 2.8.3)
project(sem_help)

include_directories(
	include
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(src SRC)
add_library(sem_help SHARED
	${SRC}
)

target_link_libraries(sem_help
    ${rtm_LIBRARIES}
)
