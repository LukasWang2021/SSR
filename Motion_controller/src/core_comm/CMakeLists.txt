cmake_minimum_required(VERSION 2.8.3)
project(core_comm)

include_directories(
	include
	../common
	../yaml_help/include
	../xml_help/include
	../thread_help/include 
	${rtm_INCLUDE_DIRS}  
)

aux_source_directory(src/common COMMON_SRC)
aux_source_directory(src/system SYSTEM_SRC)
aux_source_directory(src/bare BARE_SRC)
add_library(core_comm_system SHARED
	${COMMON_SRC}
	${SYSTEM_SRC}
)
add_library(core_comm_bare SHARED
	${COMMON_SRC}
	${BARE_SRC}
)

link_directories(
	../../install/lib
)
target_link_libraries(core_comm_system
	${rtm_LIBRARIES}
	yaml_help 
	xml_help 
	thread_help 
)

install(FILES config/core_comm_config.xml
	DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/comm/
)
install(FILES config/core_comm.yaml 
	DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/component/
)
install(FILES config/core_comm.yaml.slave 
	DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/component/
)

