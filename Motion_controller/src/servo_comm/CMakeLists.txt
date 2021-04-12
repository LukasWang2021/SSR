cmake_minimum_required(VERSION 2.8.3)
project(servo_comm)

include_directories(
	include
	../common 
	../core_comm/include 
	../yaml_help/include
	../log_manager/include
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(src/common COMMON_SRC)
aux_source_directory(src/system SYSTEM_SRC)
add_library(servo_comm SHARED
	${COMMON_SRC}
	${SYSTEM_SRC}
)

link_directories(
	../../install/lib
)
target_link_libraries(servo_comm
	${rtm_LIBRARIES}
	core_comm_system
	yaml_help
	log_manager_producer
)

