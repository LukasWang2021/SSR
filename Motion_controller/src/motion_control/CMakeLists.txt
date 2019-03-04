cmake_minimum_required(VERSION 2.8.3)
project(motion_control)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    log_manager
    base_device
    fst_axis_device
    virtual_axis_device
    device_manager
    axis_group_manager
    base_axis_group 
    coordinate_manager
    tool_manager 
    basic_alg
    path_traj_alg
    dynamic_alg
    xml_help
    error_monitor
    thread_help
    kinematics_alg
    path_traj_alg
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
    ${depend_PATH_INC}/log_manager/include
    ${depend_PATH_INC}/middleware_to_mem/include
    ${depend_PATH_INC}/service_manager/include
    ${depend_PATH_INC}/parameter_manager/include 
    ${depend_PATH_INC}/base_device/include 
    ${depend_PATH_INC}/fst_axis_device/include 
    ${depend_PATH_INC}/virtual_axis_device/include
    ${depend_PATH_INC}/device_manager/include 
    ${depend_PATH_INC}/axis_group_manager/include 
    ${depend_PATH_INC}/base_axis_group/include
    ${depend_PATH_INC}/coordinate_manager/include
    ${depend_PATH_INC}/tool_manager/include 
    ${depend_PATH_INC}/basic_alg/include 
    ${depend_PATH_INC}/path_traj_alg/include 
    ${depend_PATH_INC}/dynamic_alg/include 
    ${depend_PATH_INC}/xml_help/include
    ${depend_PATH_INC}/error_monitor/include
    ${depend_PATH_INC}/thread_help/include
    ${depend_PATH_INC}/kinematics_alg/include
    ${depend_PATH_INC}/path_traj_alg/include
)

catkin_package(
    INCLUDE_DIRS include
    #LIBRARIES fst_axis_group
    LIBRARIES motion_control
    DEPENDS roscpp
    DEPENDS base_device
    DEPENDS fst_axis_device 
    DEPENDS virtual_axis_device 
    DEPENDS device_manager 
    DEPENDS axis_group_manager 
    DEPENDS base_axis_group
    DEPENDS coordinate_manager 
    DEPENDS tool_manager 
    DEPENDS basic_alg
    DEPENDS dynamic_alg 
    DEPENDS xml_help
    DEPENDS kinematics_alg
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(motion_control
    src/motion_control.cpp
    src/motion_control_param.cpp
    src/motion_control_arm_group.cpp
    src/motion_control_scara_group.cpp
    src/motion_control_base_group.cpp
    src/motion_control_manual_teach.cpp
    src/motion_control_core_interface.cpp
    src/motion_control_offset_calibrator.cpp
    src/motion_control_constraint.cpp
    src/motion_control_traj_fifo.cpp
    src/motion_control_ros_basic.cpp
)

add_executable(manual_test
    src/manual_test.cpp
)

target_link_libraries(motion_control
    ${catkin_LIBRARIES}
    base_device
    fst_axis_device 
    virtual_axis_device 
    device_manager 
    axis_group_manager 
    base_axis_group
    coordinate_manager
    tool_manager
    basic_alg 
    dynamic_alg 
    xml_help 
    comm_interface
    core_interface
    logger
    parameter_manager
    error_monitor
    thread_help
    kinematics_alg
    path_traj_alg
)

target_link_libraries(manual_test
    ${catkin_LIBRARIES}
    motion_control
)

install(TARGETS motion_control
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS manual_test
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES config/motion_control.yaml 
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/component_param/
)

install(FILES config/base_group.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/axis_group
)

install(FILES config/arm_group.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/axis_group
)

install(FILES config/scara_group.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/axis_group
)

install(FILES config/hard_constraint.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/axis_group
)

install(FILES config/firm_constraint.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/axis_group
)

install(FILES config/soft_constraint.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/axis_group
)

install(FILES config/arm_group_offset.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/axis_group
)

install(FILES config/arm_group_offset_reference.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/axis_group
)

install(FILES config/arm_group_gear_ratio.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/axis_group
)

install(FILES config/arm_group_coupling_coeff.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/axis_group
)

install(FILES config/arm_manual_teach.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/axis_group
)

install(FILES config/segment_alg.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/component_param
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
