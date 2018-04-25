cmake_minimum_required(VERSION 2.8.3)
project(tp_interface)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    motion_plan
    middleware_to_mem
    service_manager
    parameter_manager
    safety io_manager
    mem_share
)

set(VERSION_MAJOR 2)
set(VERSION_MINOR 1)
set(VERSION_PATCH 2)
string(TIMESTAMP BUILD_TIME \"%Y-%m-%d|%H:%M:%S\")
configure_file(
    "${PROJECT_SOURCE_DIR}/include/version.h.in"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version.h" 
)
set(${PROJECT_NAME}_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})
file(WRITE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version "${PROJECT_NAME}:${${PROJECT_NAME}_VERSION}\n")

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
    ${depend_PATH_INC}/motion_plan/include
    ${depend_PATH_INC}/middleware_to_mem/include
    ${depend_PATH_INC}/parameter_manager/include
    ${depend_PATH_INC}/safety/include
    ${depend_PATH_INC}/log_manager/include
    ${depend_PATH_INC}/service_manager/include
    ${depend_PATH_INC}/io_manager/include
    ${depend_PATH_INC}/ioboard/include
    ${depend_PATH_INC}/mem_share/include
)

catkin_package(
    LIBRARIES io_interface
    DEPENDS parameter_manager
    DEPENDS motion_plan 
    DEPENDS comm_interface
    DEPENDS core_interface
    DEPENDS safety
    DEPENDS io_manager
    DEPENDS log_manager
)
 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -DPB_FIELD_32BIT")

include_directories(
    include
    include/proto
    include/service
    include/common
    include/tp_interface
    ${NANOPB_DIR}
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_BINARY_DIR}
)

###########
## Build ##
###########
aux_source_directory(src BASE_SRCS)
aux_source_directory(src/service SERV_SRCS)
aux_source_directory(src/proto PROTO_SRCS)
aux_source_directory(src/tp_interface/ TPI_SRCS)

add_library(io_interface
    ${NANOPB_DIR}/pb_decode.c
    ${NANOPB_DIR}/pb_encode.c
    ${NANOPB_DIR}/pb_common.c
    ${PROTO_SRCS}
    src/io_interface.cpp
    src/error_monitor.cpp
)

add_executable(controller
    ${BASE_SRCS}
    ${PROTO_SRCS}
    ${SERV_SRCS}
    ${TPI_SRCS}
    #${NANOPB_DIR}/pb_decode.c
    #${NANOPB_DIR}/pb_encode.c
    #${NANOPB_DIR}/pb_common.c
)

target_link_libraries(io_interface
    ${catkin_LIBRARIES}
    io_manager
)

target_link_libraries(controller
    ${catkin_LIBRARIES}
    io_interface
    comm_interface
    core_interface
    safety
    io_manager
    motion_plan
)

#############
## Install ##
#############
install(TARGETS io_interface
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS controller 
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}    
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version 
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/version
)

#############
## Testing ##
#############
if(GTEST)
    message("GTest==>" ${GTEST_LIBRARIES})
    # Link utest with what we want to test and the GTest and pthread library.
    add_executable(ti_test test/unit/main.cpp)
    target_link_libraries(ti_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
    install(TARGETS ti_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
