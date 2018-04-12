cmake_minimum_required(VERSION 2.8.3)
project(io_manager)

set(${PROJECT_NAME}_VERSION_MAJOR 1)
set(${PROJECT_NAME}_VERSION_MINOR 0)
set(${PROJECT_NAME}_VERSION_PATCH 0)
string(TIMESTAMP BUILD_TIME \"%Y-%m-%d|%H:%M:%S\")

set(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH})

# configure a header file to pass some settings to the source code.
configure_file(
    "${PROJECT_SOURCE_DIR}/include/version.h.in"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version.h" 
)

# write the verson info to a file
file(WRITE ${PROJECT_NAME}_version "${PROJECT_NAME}:${${PROJECT_NAME}_VERSION}\n")


find_package(catkin REQUIRED COMPONENTS
    parameter_manager
    ioboard
)

if(catkin_CROSS)
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS})
    set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS})
    message(STATUS "ARM-Cross")
else()
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_x86})
    set(catkin_LIBRARIES ${catkin_LIBRARYS_x86})
    message(STATUS "x86-64")
endif()

set(catkin_INCLUDE_DIRS
    ${catkin_INCLUDE_DIRS}
    ${depend_PATH_INC}/parameter_manager/include
    ${depend_PATH_INC}/ioboard/include
)

catkin_package(
    LIBRARIES io_manager
)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -O0 -DPB_FIELD_32BIT -DSIMMULATION")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_BINARY_DIR}
)

#############
##  Build  ##
#############
add_library(io_manager
    src/io_manager.cpp
)

add_executable(io_manager_node
    src/io_manager_node.cpp
)

target_link_libraries(io_manager
    ${catkin_LIBRARIES}
    parameter_manager
    ioboard
)

target_link_libraries(io_manager_node
    ${catkin_LIBRARIES}
    io_manager
)

#############
## Install ##
#############
install(TARGETS io_manager
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS io_manager_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES ${PROJECT_NAME}_version 
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/version
)

