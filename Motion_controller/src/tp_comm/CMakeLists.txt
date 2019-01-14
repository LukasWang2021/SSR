cmake_minimum_required(VERSION 2.8.3)
project(tp_comm)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    protos
    parameter_manager
    thread_help
    log_manager
    error_monitor
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
    ${depend_PATH_INC}/protos/include
    ${depend_PATH_INC}/common/
    ${depend_PATH_INC}/parameter_manager/include/
    ${depend_PATH_INC}/log_manager/include/
    ${depend_PATH_INC}/thread_help/include/
    ${depend_PATH_INC}/error_monitor/include/
    ${depend_PATH_INC}/log_service/include 
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES tp_comm
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -D PB_FIELD_32BIT")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/python2.7 
)

aux_source_directory(src BASE_SRCS)

add_library(tp_comm
    ${NANOPB_DIR}/pb_decode.c
    ${NANOPB_DIR}/pb_encode.c
    ${NANOPB_DIR}/pb_common.c
    ${BASE_SRCS}
)

target_link_libraries(tp_comm
    ${catkin_LIBRARIES}
    protos
    thread_help
    parameter_manager
    error_monitor
    log_service
)

install(TARGETS tp_comm
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES config/tp_comm.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/component_param/
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
