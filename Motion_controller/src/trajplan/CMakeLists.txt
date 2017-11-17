cmake_minimum_required(VERSION 2.8.3)
project(trajplan)


if(DEFINED PLATFORM)
	set(LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/${PLATFORM}/lib)
else()
	set(LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64/lib)	
endif()
message(${LIB_PATH})
execute_process(COMMAND	cp -r ${LIB_PATH} ${CATKIN_DEVEL_PREFIX})
 
catkin_package(
 INCLUDE_DIRS include
  LIBRARIES trajplan
)

install(DIRECTORY ${LIB_PATH}
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/..)


