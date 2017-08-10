cmake_minimum_required(VERSION 2.8.3)
project(tp_interface)

#add_custom_command(OUTPUT vitual_target	
#	COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_HOME_DIRECTORY}/../devel/${CONFIG_DIR}
#	COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/${CONFIG_DIR} ${CMAKE_HOME_DIRECTORY}/../devel/${CONFIG_DIR}
#	) 
#add_custom_target(vitual_target ALL DEPENDS ${CPY_CONF})
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp rospy std_msgs motion_controller middleware_to_mem parameter_manager safety io_manager)
 
set(CONFIG_DIR config)
#execute_process(COMMAND rm -r ${CMAKE_HOME_DIRECTORY}/../devel/${CONFIG_DIR}
#		COMMAND cp -r ${CMAKE_CURRENT_SOURCE_DIR}/${CONFIG_DIR} ${CMAKE_HOME_D#IRECTORY}/../devel)
if(${catkin_CROSS})
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS} ${depend_PATH_INC}/motion_controller/include ${depend_PATH_INC}/middleware_to_mem/include ${depend_PATH_INC}/trajplan/include ${depend_PATH_INC}/parameter_manager/include ${depend_PATH_INC}/safety/include ${depend_PATH_INC}/log_manager/include ${depend_PATH_INC}/service_manager/include ${depend_PATH_INC}/io_manager/include ${depend_PATH_INC}/ioboard/include)
	set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS} ${depend_PATH_LIB}/libmotion_controller.so ${depend_PATH_LIB}/libcomm_interface.so ${depend_PATH_LIB}/libcore_interface.so ${depend_PATH_LIB}/libtrajplan.so ${depend_PATH_LIB}/libparameter_manager.so ${depend_PATH_LIB}/libsafety.so ${depend_PATH_LIB}/libio_manager.so)
	set(CMAKE_CXX_FLAGS "-DCROSS_PLATFORM")
	set(CMAKE_C_FLAGS "-DCROSS_PLATFORM")
else(${catkin_CROSS})
    set(NANOPB_DIR ~/my_work/nanopb)
	set(EXT_LIB nanomsg profiler)
    include_directories(/home/fst/fst_ros/src/service_manager/include /home/fst/fst_ros/src/trajplan/include/)
endif(${catkin_CROSS})



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fst_controller
  CATKIN_DEPENDS parameter_manager
  DEPENDS motion_controller 
  DEPENDS comm_interface
  DEPENDS core_interface
  DEPENDS safety
  DEPENDS io_manager
)
 
###########
## Build ##
###########
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -DPB_FIELD_32BIT -DSIMMULATION")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -DPB_FIELD_32BIT")

message(${CMAKE_CXX_FLAGS})

##############version manager##############
set(VERSION_MAJOR 2)
set(VERSION_MINOR 0)
set(VERSION_PATCH 7)
string(TIMESTAMP BUILD_TIME \"%Y-%m-%d|%H:%M:%S\")
configure_file(
  "${PROJECT_SOURCE_DIR}/include/version.h.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version.h" 
)
include_directories(${PROJECT_BINARY_DIR})
set(${PROJECT_NAME}_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})
file(WRITE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version "${PROJECT_NAME}:${${PROJECT_NAME}_VERSION}\n")
####################################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ./include  ./include/proto ./include/common ${NANOPB_DIR})

#set(CMAKE_LD_FLAGS "-std=c++11")
## Declare a C++ library
# add_library(fst_controller
#   src/${PROJECT_NAME}/fst_controller.cpp
# ) 
aux_source_directory(src BASE_SRCS)
aux_source_directory(src/proto PROTO_SRCS)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(fst_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
include_directories(include ${catkin_INCLUDE_DIRS} /usr/include/eigen3/ )
## Declare a C++ executable
#add_executable(controller src/controller.cpp src/nn_socket.cpp src/proto_parse.cpp src/robot_motion.cpp src/json_parse.cpp src/share_mem.cpp src/watchdog.cpp src/ip_address.cpp src/timer.cpp src/ros_basic.cpp ${NANOPB_DIR}/pb_decode.c  ${NANOPB_DIR}/pb_encode.c ${NANOPB_DIR}/pb_common.c  src/proto/motionSL.pb.cpp src/proto/base_types.pb.cpp)
add_executable(controller ${BASE_SRCS} ${PROTO_SRCS} ${NANOPB_DIR}/pb_decode.c  ${NANOPB_DIR}/pb_encode.c ${NANOPB_DIR}/pb_common.c)
#target_link_libraries(fst_controller )


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(fst_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(controller
  ${catkin_LIBRARIES}
  ${EXT_LIB}
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS controller 
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}    
 )

install(DIRECTORY ${CONFIG_DIR}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )
install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version 
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/version
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_fst_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

