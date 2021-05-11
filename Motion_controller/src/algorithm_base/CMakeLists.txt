cmake_minimum_required(VERSION 2.8.3)
project(algorithm_base)

include_directories(
	include
    ../common/ 
    ../yaml_help/include 
    ../system_model_manager/include 
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(src SRC)
add_library(algorithm_base SHARED
	${SRC}
)

target_link_libraries(algorithm_base
    ${rtm_LIBRARIES}
)