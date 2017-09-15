cmake_minimum_required(VERSION 2.8.3)
project(motion_controller)

# setting for include file.
set(VERSION_MAJOR 4)
set(VERSION_MINOR 1)
set(VERSION_PATCH 0)
string(TIMESTAMP BUILD_TIME \"%Y-%m-%d|%H:%M:%S\")

# setting for package version file.
set(${PROJECT_NAME}_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

# configure a header file to pass some settings to the source code.
configure_file(
  "${PROJECT_SOURCE_DIR}/include/version.h.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version.h" 
)

# write the verson info to a file.
file(WRITE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version "${PROJECT_NAME}:${${PROJECT_NAME}_VERSION}\n")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp trajplan parameter_manager middleware_to_mem log_manager
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
if(${catkin_CROSS})
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS} ${depend_PATH_INC}/trajplan/include ${depend_PATH_INC}/parameter_manager/include ${depend_PATH_INC}/middleware_to_mem/include ${depend_PATH_INC}/log_manager/include)
	set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS} ${depend_PATH_LIB}/libtrajplan.so ${depend_PATH_LIB}/libparameter_manager.so ${depend_PATH_LIB}/libparam_value.so ${depend_PATH_LIB}/libparam_builder.so ${depend_PATH_LIB}/libmiddleware_to_mem.so ${depend_PATH_LIB}/libcomm_interface.so ${depend_PATH_LIB}/liblogger.so)
endif(${catkin_CROSS})


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
  INCLUDE_DIRS include
  LIBRARIES calibrator
  LIBRARIES motion_controller
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
    DEPENDS trajplan
    DEPENDS parameter_manager
    DEPENDS middleware_to_mem
    DEPENDS log_manager
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
#  ${Boost_INCLUDE_DIRS}
  ${PROJECT_BINARY_DIR}
)

#link_directories(${Boost_LIBRARY_DIRS})
#link_directories(${catkin_LIBRARY_DIRS})

## Declare a C++ library
add_library(calibrator  src/motion_controller_offset_calibrator.cpp)
target_link_libraries(calibrator ${catkin_LIBRARIES})

add_library(motion_controller   src/motion_controller_arm_group.cpp
                                src/motion_controller_arm_group_private.cpp
                                src/motion_controller_arm_group_planning.cpp
                                src/motion_controller_planning_interface.cpp)
target_link_libraries(motion_controller calibrator ${catkin_LIBRARIES})
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(fst_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(motion_demo src/demo.cpp)
target_link_libraries(motion_demo motion_controller ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(fst_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(fst_controller_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS motion_controller
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS calibrator
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
## Mark executables and/or libraries for installation
 install(TARGETS motion_demo 
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES config/dh.yaml
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/configuration/machine
)

install(FILES config/offset.yaml
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/configuration/machine
)

install(FILES config/jtac.yaml
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/configuration/machine
)

install(FILES config/soft_constraints.yaml
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/configuration/configurable
)

install(FILES config/hard_constraints.yaml
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/configuration/model
)

install(FILES config/motion_controller.yaml
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/configuration/configurable
)

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
