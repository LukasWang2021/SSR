cmake_minimum_required(VERSION 2.8.3)
project(service_manager)

# setting for include file.
set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)
string(TIMESTAMP BUILD_TIME \"%Y-%m-%d|%H:%M:%S\")

# setting for package version file.
set(${PROJECT_NAME}_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

# configure a header file to pass some settings to the source code.
configure_file(
  "${PROJECT_SOURCE_DIR}/include/version.h.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version.h" 
#  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}_version.h"
)

# add the search path for include files.
include_directories(${PROJECT_BINARY_DIR})

# write the verson info to a file.
file(WRITE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version "${PROJECT_NAME}:${${PROJECT_NAME}_VERSION}\n")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  middleware_to_mem
)

if(${catkin_CROSS})
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS} ${depend_PATH_INC}/middleware_to_mem/include)
	set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS} ${depend_PATH_LIB}/libcomm_interface.so ${depend_PATH_LIB}/libmiddleware_to_mem.so)
endif(${catkin_CROSS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS roscpp middleware_to_mem comm_interface
  DEPENDS libcomm_interface.so libmiddleware_to_mem.so
)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -lrt -lpthread -O0 -DPB_FIELD_32BIT -DSIMMULATION")

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(service_manager src/service_manager.cpp src/response_actions.cpp)
target_link_libraries(service_manager ${catkin_LIBRARIES} middleware_to_mem comm_interface)

add_executable(mimic_mcs test/mimic_mcs.cpp)
target_link_libraries(mimic_mcs ${catkin_LIBRARIES} comm_interface)

install(TARGETS service_manager mimic_mcs
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version 
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/version
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
