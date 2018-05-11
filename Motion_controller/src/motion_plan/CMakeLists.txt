cmake_minimum_required(VERSION 2.8.3)
project(motion_plan)

set(VERSION_MAJOR 4)
set(VERSION_MINOR 1)
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

find_package(catkin REQUIRED COMPONENTS
    parameter_manager
    log_manager
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
    ${depend_PATH_INC}/log_manager/include
)

catkin_package(
    LIBRARIES motion_plan
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_BINARY_DIR}
)

###########
## Build ##
###########
add_library(motion_plan
    # include algerithm functions here:
    src/motion_plan_variable.cpp
    src/motion_plan_matrix.cpp
    src/motion_plan_kinematics.cpp
    src/motion_plan_motion_command.cpp
    src/motion_plan_manual_teach.cpp
    src/motion_plan_traj_plan.cpp

    src/motion_plan_arm_group.cpp
    src/motion_plan_frame_manager.cpp
    src/dynamics_interface.cpp
)

## Declare a C++ executable
add_executable(motion_plan_demo1
    src/motion_plan_demo1.cpp
)

add_executable(motion_plan_demo3
    src/motion_plan_demo3.cpp
)

add_executable(motion_plan_demo4
    src/motion_plan_demo4.cpp
)

add_executable(motion_plan_demo5
    src/motion_plan_demo5.cpp
)

add_executable(motion_plan_demo_manual
    src/motion_plan_demo_manual.cpp
)

add_executable(motion_plan_demo_constraint
    src/motion_plan_demo_constraint.cpp
)

target_link_libraries(motion_plan
    ${catkin_LIBRARIES}
    parameter_manager
    logger
)

target_link_libraries(motion_plan_demo1 
    ${catkin_LIBRARIES}
    motion_plan
)

target_link_libraries(motion_plan_demo3
    ${catkin_LIBRARIES}
    motion_plan
)

target_link_libraries(motion_plan_demo4
    ${catkin_LIBRARIES}
    motion_plan
)

target_link_libraries(motion_plan_demo5
    ${catkin_LIBRARIES}
    motion_plan
)

target_link_libraries(motion_plan_demo_manual
    ${catkin_LIBRARIES}
    motion_plan
)

target_link_libraries(motion_plan_demo_constraint
    ${catkin_LIBRARIES}
    motion_plan
)

#############
## Install ##
#############
install(TARGETS motion_plan
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS motion_plan_demo1
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS motion_plan_demo3
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS motion_plan_demo4
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS motion_plan_demo5
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS motion_plan_demo_manual
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS motion_plan_demo_constraint
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/version
)

install(FILES config/motion_plan.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/configuration/configurable
)

install(FILES config/hard_constraints.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/configuration/model
)

install(FILES config/soft_constraints.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/configuration/configurable
)

install(FILES config/dh.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/configuration/machine
)

install(FILES config/dynamics.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/configuration/machine
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

