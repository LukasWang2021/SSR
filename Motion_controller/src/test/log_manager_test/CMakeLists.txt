cmake_minimum_required(VERSION 2.8.3)
project(log_manager_test)

SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/log_manager_test)

include_directories(
	../../common
    ../../log_manager/include 
    ../../yaml_help/include
	../../thread_help/include 
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(./ SRC)
add_executable(log_manager_producer_test
	log_manager_producer_test.cpp
)

link_directories(
	../../install/lib
)

target_link_libraries(log_manager_producer_test 
	${rtm_LIBRARIES}
	yaml_help
    thread_help
    log_manager_producer 
    log_manager_producer_bare
)


#install(TARGETS log_manager_producer_test DESTINATION ${EXECUTABLE_OUTPUT_PATH}/)

