cmake_minimum_required(VERSION 2.8.3)
project(parameter_manager)

# setting for include file.
set(VERSION_MAJOR 3)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)
string(TIMESTAMP BUILD_TIME \"%Y-%m-%d|%H:%M:%S\")

# setting for package version file.
set(${PROJECT_NAME}_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

# configure a header file to pass some settings to the source code.
configure_file(
  "${PROJECT_SOURCE_DIR}/include/version.h.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version.h" 
)

# add the search path for include files
include_directories(${PROJECT_BINARY_DIR})

# write the verson info to a file.
file(WRITE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version "${PROJECT_NAME}:${${PROJECT_NAME}_VERSION}\n")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp 
)

if(${catkin_CROSS})
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS})
	set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS})
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
  INCLUDE_DIRS include
  LIBRARIES param_value
  LIBRARIES param_builder
  LIBRARIES parameter_manager
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
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
  ${PROJECT_BINARY_DIR}
)

## Declare a C++ library
add_library(param_value
  src/parameter_manager_param_value.cpp
)

target_link_libraries(param_value
  ${catkin_LIBRARIES}
)

add_library(param_builder
  src/parameter_manager_param_builder.cpp
)
target_link_libraries(param_builder
  ${catkin_LIBRARIES}
)

add_library(parameter_manager
  src/parameter_manager_param_group.cpp
)

target_link_libraries(parameter_manager
  ${catkin_LIBRARIES}
  param_value
  param_builder
)

## Declare a C++ executable
add_executable(manager_node
  src/manager_node.cpp
)

target_link_libraries(manager_node
  ${catkin_LIBRARIES}
  param_value
  param_builder
  parameter_manager
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

install(TARGETS param_value
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS param_builder
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS parameter_manager
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS manager_node 
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/version
)


