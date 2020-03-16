cmake_minimum_required(VERSION 2.8.3)
project(servo_diag)

find_package(catkin REQUIRED COMPONENTS
    middleware_to_mem
    parameter_manager
)

set(VERSION_MAJOR 1)
set(VERSION_MINOR 7)
set(VERSION_PATCH 0)
string(TIMESTAMP BUILD_TIME \"%Y-%m-%d|%H:%M:%S\")

set(${PROJECT_NAME}_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

configure_file(
    "${PROJECT_SOURCE_DIR}/include/version.h.in"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version.h" 
)

file(WRITE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version
    "${PROJECT_NAME}:${${PROJECT_NAME}_VERSION}\n"
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
    ${depend_PATH_INC}/middleware_to_mem/include
    ${depend_PATH_INC}/parameter_manager/include
    ${depend_PATH_INC}/common 
)

catkin_package(
#    CATKIN_DEPENDS middleware_to_mem parameter_manager
)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -O0 -DPB_FIELD_32BIT -DSIMMULATION")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_BINARY_DIR}
)

add_executable(servo_diag
    src/servo_diag.cpp
    src/servo_conf.cpp
    src/servo_service.cpp
    src/data_monitor.cpp
)

target_link_libraries(servo_diag
    ${catkin_LIBRARIES}
    middleware_to_mem
    comm_interface
    parameter_manager
)

install(TARGETS servo_diag
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config/
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/configuration/machine
)

install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version 
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/version
)

