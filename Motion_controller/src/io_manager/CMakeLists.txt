cmake_minimum_required(VERSION 2.8.3)
project(io_manager)

# setting for include file.
set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)
set(VERSION_PATCH 2)
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

# write the verson info to a file
file(WRITE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version "${PROJECT_NAME}:${${PROJECT_NAME}_VERSION}\n")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  parameter_manager
  ioboard
)

if(${catkin_CROSS})
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS} ${depend_PATH_INC}/parameter_manager/include ${depend_PATH_INC}/ioboard/include)
	set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS} ${depend_PATH_LIB}/libparameter_manager.so ${depend_PATH_LIB}/libioboard.so)
endif(${catkin_CROSS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES io_manager
  CATKIN_DEPENDS roscpp parameter_manager ioboard
  DEPENDS libparameter_manager.so libioboard.so
)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -lrt -lpthread -O0 -DPB_FIELD_32BIT -DSIMMULATION")

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(io_manager src/io_manager.cpp)
target_link_libraries(io_manager ${catkin_LIBRARIES} parameter_manager ioboard)

add_executable(io_manager_node test/io_manager_node.cpp)
target_link_libraries(io_manager_node ${catkin_LIBRARIES} io_manager)

install(TARGETS io_manager
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS io_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/configuration/configurable
)

install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version 
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/version
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

