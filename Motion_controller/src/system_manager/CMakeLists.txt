cmake_minimum_required(VERSION 2.8.3)
project(system_manager)

# setting for include file.
set(VERSION_MAJOR 1)
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

# add the search path for include files.
include_directories(${PROJECT_BINARY_DIR})

# write the verson info to a file.
file(WRITE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version "${PROJECT_NAME}:${${PROJECT_NAME}_VERSION}\n")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  middleware_to_mem
  parameter_manager
)

if(${catkin_CROSS})
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS} ${depend_PATH_INC}/middleware_to_mem/include ${depend_PATH_INC}/parameter_manager/include)
	set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS} ${depend_PATH_LIB}/libcomm_interface.so)
endif(${catkin_CROSS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES file_operations
  CATKIN_DEPENDS roscpp
#  DEPENDS 
)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -lrt -lpthread -O0 -DPB_FIELD_32BIT -DSIMMULATION")
#add_subdirectory(test)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(file_operations src/file_operations.cpp)
target_link_libraries(file_operations ${catkin_LIBRARIES} 
#libarchive.so
)
if(NOT catkin_CROSS)
target_link_libraries(file_operations libarchive.so)
endif(NOT catkin_CROSS)

add_executable(system_manager src/system_manager.cpp src/system_execute.cpp)
target_link_libraries(system_manager ${catkin_LIBRARIES} file_operations comm_interface parameter_manager)

add_executable(one_command test/one_command.cpp)
target_link_libraries(one_command ${catkin_LIBRARIES} comm_interface)

add_executable(loop_command test/loop_command.cpp)
target_link_libraries(loop_command ${catkin_LIBRARIES} comm_interface file_operations)

install(TARGETS file_operations
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS system_manager one_command loop_command
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version 
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/version
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


