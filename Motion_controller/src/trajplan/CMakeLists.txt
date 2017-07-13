cmake_minimum_required(VERSION 2.8.3)
project(trajplan)


if(DEFINED PLATFORM)
	set(src_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/${PLATFORM}/lib)
else()
	set(src_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64/lib)	
endif()
message(${src_LIB_PATH})
execute_process(COMMAND	rm ${CMAKE_HOME_DIRECTORY}/../devel/lib/libtrajplan.so
		COMMAND	rm ${CMAKE_HOME_DIRECTORY}/../install/lib/libtrajplan.so
		COMMAND	cp ${src_LIB_PATH}/libtrajplan.so ${CMAKE_HOME_DIRECTORY}/../devel/lib)
 
catkin_package(
 INCLUDE_DIRS include
  LIBRARIES trajplan
)

install(DIRECTORY ${src_LIB_PATH}
	DESTINATION ${CMAKE_HOME_DIRECTORY}/../install)


