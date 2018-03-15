cmake_minimum_required(VERSION 2.8.3)
project(motion_plan)

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
  roscpp parameter_manager log_manager
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
if(${catkin_CROSS})
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS} ${depend_PATH_INC}/trajplan/include ${depend_PATH_INC}/parameter_manager/include ${depend_PATH_INC}/log_manager/include )
	set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS} ${depend_PATH_LIB}/libparameter_manager.so ${depend_PATH_LIB}/libparam_value.so ${depend_PATH_LIB}/libparam_builder.so  ${depend_PATH_LIB}/liblogger.so)
endif(${catkin_CROSS})



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES motion_plan
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
    DEPENDS parameter_manager
    DEPENDS log_manager
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

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
add_library(motion_plan  src/motion_plan_arm_group.cpp)
target_link_libraries(motion_plan ${catkin_LIBRARIES})


install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/version
)

install(TARGETS motion_plan
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

