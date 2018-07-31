cmake_minimum_required(VERSION 2.8.3)
project(controller)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    middleware_to_mem
    parameter_manager
    service_manager
    protos
    tp_comm
)

set(VERSION_MAJOR 2)
set(VERSION_MINOR 1)
set(VERSION_PATCH 21)
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
    ${depend_PATH_INC}/middleware_to_mem/include
    ${depend_PATH_INC}/parameter_manager/include
    ${depend_PATH_INC}/log_manager/include
    ${depend_PATH_INC}/service_manager/include
    ${depend_PATH_INC}/prg_interpreter/include
    ${depend_PATH_INC}/protos/include 
    ${depend_PATH_INC}/tp_comm/include 
    ${depend_PATH_INC}/common/ 
)

catkin_package(
    DEPENDS parameter_manager
    DEPENDS comm_interface
    DEPENDS core_interface
    DEPENDS log_manager
)
 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -DPB_FIELD_32BIT")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_BINARY_DIR}
)

###########
## Build ##
###########
aux_source_directory(src BASE_SRCS)

add_executable(controller
    ${BASE_SRCS}
)

target_link_libraries(controller
    ${catkin_LIBRARIES}
    comm_interface
    core_interface
    protos 
    tp_comm 
)

#############
## Install ##
#############
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
