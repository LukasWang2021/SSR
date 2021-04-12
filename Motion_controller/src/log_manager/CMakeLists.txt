cmake_minimum_required(VERSION 2.8.3)
project(log_manager)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D${COMPILE_OPTION}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D${COMPILE_OPTION}")

include_directories(
	include
	../common
	../yaml_help/include
	../thread_help/include 
    ../basic_alg/include
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(src BASE_SRCS)

add_library(log_manager_producer SHARED
    src/log_manager_producer.cpp
    src/log_manager_config.cpp
)

target_link_libraries(log_manager_producer
    ${rtm_LIBRARIES}
    yaml_help
    thread_help
)

install(FILES ./config/log_manager.yaml 
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/component/
)


add_executable(log_manager_comsumer 
	src/log_manager_comsumer.cpp
	src/log_manager_config.cpp
)

link_directories(
	../../install/lib
)
target_link_libraries(log_manager_comsumer
	${rtm_LIBRARIES}
	yaml_help
	thread_help
)


add_library(log_manager_producer_bare SHARED
    src/log_manager_producer_bare.c
)

