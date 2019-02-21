cmake_minimum_required(VERSION 2.8.3)
project(fst_axis_device)

find_package(catkin REQUIRED COMPONENTS
    base_device
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
    ${depend_PATH_INC}/base_device/include
    ${depend_PATH_INC}/common
    ${depend_PATH_INC}/parameter_manager/include
    ${depend_PATH_INC}/basic_alg/include
    ${depend_PATH_INC}/log_manager/include
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES fst_axis_device
    DEPENDS base_device
    DEPENDS parameter_manager
    DEPENDS log_manager
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(src BASE_SRCS)

add_library(fst_axis_device
    ${BASE_SRCS}
)

target_link_libraries(fst_axis_device
    ${catkin_LIBRARIES}
    base_device
)

install(TARGETS fst_axis_device
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES config/fst_axis_device.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/component_param/
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


