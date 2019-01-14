cmake_minimum_required(VERSION 2.8.3)
project(process_comm)

find_package(catkin REQUIRED COMPONENTS
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
    ${NANOPB_DIR}
    ${depend_PATH_INC}/common
    ${depend_PATH_INC}/parameter_manager/include
    ${depend_PATH_INC}/log_manager/include
    ${depend_PATH_INC}/reg_manager/include 
    ${depend_PATH_INC}/thread_help/include 
    ${depend_PATH_INC}/fst_io_device/include 
    ${depend_PATH_INC}/base_device/include
    ${depend_PATH_INC}/prg_interpreter/include
    ${depend_PATH_INC}/error_monitor/include 
    ${depend_PATH_INC}/modbus/include
    ${depend_PATH_INC}/error_monitor/include
    ${depend_PATH_INC}/motion_control/include
    ${depend_PATH_INC}/io_mapping/include 
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES process_comm 
    DEPENDS parameter_manager
    DEPENDS log_manager
    DEPENDS fst_io_device 
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(src BASE_SRCS)

add_library(process_comm 
    ${BASE_SRCS}
)

target_link_libraries(process_comm 
    ${catkin_LIBRARIES}
    thread_help
    logger
    error_monitor 
    fst_io_device 
    error_monitor
)

install(TARGETS process_comm 
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES config/process_comm.yaml 
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/component_param/
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


