cmake_minimum_required(VERSION 2.8.3)
project(motion_control)

find_package(catkin REQUIRED COMPONENTS
    base_device
    fst_axis_device
    virtual_axis_device
    device_manager
    axis_group_manager
    base_axis_group 
    coordinate_manager
    tool_manager 
    basic_alg
    trajectory_alg
    dynamic_alg
    xml_help
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
    ${depend_PATH_INC}/trajectory_alg/include 
    ${depend_PATH_INC}/dynamic_alg/include 
    ${depend_PATH_INC}/xml_help/include 
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES fst_axis_group
    DEPENDS base_device 
    DEPENDS fst_axis_device 
    DEPENDS virtual_axis_device 
    DEPENDS device_manager 
    DEPENDS axis_group_manager 
    DEPENDS base_axis_group
    DEPENDS coordinate_manager 
    DEPENDS tool_manager 
    DEPENDS basic_alg
    DEPENDS trajectory_alg
    DEPENDS dynamic_alg 
    DEPENDS xml_help 
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(src BASE_SRCS)

add_library(motion_control
    ${BASE_SRCS}
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
    trajectory_alg 
    dynamic_alg 
    xml_help 
)

install(TARGETS motion_control
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES config/motion_control.yaml 
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/component_param/
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

