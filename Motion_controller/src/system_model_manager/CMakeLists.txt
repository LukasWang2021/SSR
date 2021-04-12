cmake_minimum_required(VERSION 2.8.3)
project(system_model_manager)

include_directories(
	include
	../common
    ../yaml_help/include 
    ../xml_help/include 
    ../log_manager/include
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(src SRC)
add_library(system_model_manager SHARED
	${SRC}
)

target_link_libraries(system_model_manager
	${rtm_LIBRARIES}
    yaml_help
    xml_help 
    log_manager_producer
)

install(FILES ./config/system_model_manager.yaml 
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/component/)

install(DIRECTORY ./config/system_model 
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}
)

