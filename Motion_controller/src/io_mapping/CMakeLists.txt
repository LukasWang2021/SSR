cmake_minimum_required(VERSION 2.8.3)
project(io_mapping)

find_package(catkin REQUIRED COMPONENTS
    base_device
    fst_io_device
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
    ${depend_PATH_INC}/log_manager/include
    ${depend_PATH_INC}/fst_io_device/include
    ${depend_PATH_INC}/error_monitor/include
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES io_mapping
    DEPENDS base_device
    DEPENDS parameter_manager
    DEPENDS log_manager
    DEPENDS fst_io_device
    DEPENDS error_monitor
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(src BASE_SRCS)

add_library(io_mapping
    ${BASE_SRCS}
)

target_link_libraries(io_mapping
    ${catkin_LIBRARIES}
    parameter_manager
    logger
    base_device
    fst_io_device
    error_monitor
)

add_executable(mapping_test
    test/mapping_test.cpp
)

target_link_libraries(mapping_test
    ${catkin_LIBRARIES}
    io_mapping
    base_device
)

install(TARGETS io_mapping
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS	mapping_test
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES config/io_mapping.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/component_param/
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


