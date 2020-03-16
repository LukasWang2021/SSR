cmake_minimum_required(VERSION 2.8.3)
project(trajectory_planner)

find_package(catkin REQUIRED COMPONENTS
    basic_alg
    log_manager
    parameter_manager
    kinematics_alg
    dynamic_alg
    joint_constraint
)

if(catkin_CROSS)
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS})
    set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS})
	set(CMAKE_CXX_FLAGS "-DCROSS_PLATFORM")
	set(CMAKE_C_FLAGS "-DCROSS_PLATFORM")
    message(STATUS "ARM-Cross")
else()
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_x86})
    set(catkin_LIBRARIES ${catkin_LIBRARYS_x86})
    message(STATUS "x86-64")
endif()

set(catkin_INCLUDE_DIRS
    ${catkin_INCLUDE_DIRS}
    ${depend_PATH_INC}/common
    ${depend_PATH_INC}/basic_alg/include
    ${depend_PATH_INC}/log_manager/include
    ${depend_PATH_INC}/parameter_manager/include 
    ${depend_PATH_INC}/kinematics_alg/include
    ${depend_PATH_INC}/dynamic_alg/include
    ${depend_PATH_INC}/motion_control/include
    ${depend_PATH_INC}/joint_constraint/include
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES trajectory_planner
    DEPENDS basic_alg
    DEPENDS log_manager
    DEPENDS parameter_manager
    DEPENDS kinematics_alg
    DEPENDS dynamic_alg
    DEPENDS joint_constraint
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(src BASE_SRCS)
aux_source_directory(src/ds_planner DS_SRCS)
aux_source_directory(src/spline_planner SPLINE_SRCS)
aux_source_directory(src/bezier_planner BEZIER_SRCS)

add_library(trajectory_planner
    ${BASE_SRCS}
    ${DS_SRCS}
    ${SPLINE_SRCS}
    ${BEZIER_SRCS}
)

target_link_libraries(trajectory_planner
    ${catkin_LIBRARIES}
    basic_alg
    logger
    parameter_manager
    kinematics_alg
    dynamic_alg
    joint_constraint
)


add_executable(test_trajectory_planner
    src/test_trajectory_planner.cpp
)

target_link_libraries(test_trajectory_planner
    ${catkin_LIBRARIES}
    trajectory_planner
)

install(FILES config/config.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/algorithm/
)

install(FILES config/constraint.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/algorithm/
)

install(TARGETS trajectory_planner
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS test_trajectory_planner
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

