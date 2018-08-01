cmake_minimum_required(VERSION 2.8.3)
project(tp_comm)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    protos
    log_manager
    parameter_manager
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
    ${depend_PATH_INC}/log_manager/include/
    ${depend_PATH_INC}/parameter_manager/include
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES tp_comm
    DEPENDS parameter_manager
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")

include_directories(
  include
  include/common
  ${catkin_INCLUDE_DIRS}
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
    logger
    parameter_manager
)

install(TARGETS tp_comm
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES config/tp_comm.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/configuration/configurable
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


