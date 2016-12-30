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
find_package(catkin REQUIRED roscpp rospy std_msgs motion_controller middleware_to_mem parameter_manager safety)

set(CONFIG_DIR config)
#execute_process(COMMAND rm -r ${CMAKE_HOME_DIRECTORY}/../devel/${CONFIG_DIR}
#		COMMAND cp -r ${CMAKE_CURRENT_SOURCE_DIR}/${CONFIG_DIR} ${CMAKE_HOME_D#IRECTORY}/../devel)

if(${catkin_CROSS})
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS} ${depend_PATH_INC}/motion_controller/include ${depend_PATH_INC}/middleware_to_mem/include ${depend_PATH_INC}/trajplan/include ${depend_PATH_INC}/parameter_manager/include ${depend_PATH_INC}/safety/include ${depend_PATH_INC}/log_manager/include)
	set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS} ${depend_PATH_LIB}/libmotion_controller.so ${depend_PATH_LIB}/libcomm_interface.so ${depend_PATH_LIB}/libcore_interface.so ${depend_PATH_LIB}/libtrajplan.so ${depend_PATH_LIB}/libparameter_manager.so ${depend_PATH_LIB}/libsafety.so)
	set(CMAKE_CXX_FLAGS "-DCROSS_PLATFORM")
	set(CMAKE_C_FLAGS "-DCROSS_PLATFORM")
else(${catkin_CROSS})
    set(NANOPB_DIR ~/my_work/nanopb)
	set(EXT_LIB nanomsg profiler)
endif(${catkin_CROSS})


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
)

###########
## Build ##
###########
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -O0 -DPB_FIELD_32BIT -DSIMMULATION")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -DPB_FIELD_32BIT")

message(${CMAKE_CXX_FLAGS})

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

