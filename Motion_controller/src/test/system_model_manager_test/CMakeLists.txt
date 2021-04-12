cmake_minimum_required(VERSION 2.8.3)
project(system_model_manager_test)

SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/system_model_manager_test)

include_directories(
	../../common
	../../yaml_help/include
	../../xml_help/include
    ../../system_model_manager/include 
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(./ SRC)
add_executable(test_system_model_manager
	${SRC}
)

link_directories(
	../../install/lib
)

target_link_libraries(test_system_model_manager
	${rtm_LIBRARIES}
	yaml_help
	xml_help
    system_model_manager 
)

