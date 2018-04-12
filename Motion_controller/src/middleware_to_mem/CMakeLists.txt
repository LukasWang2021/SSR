cmake_minimum_required(VERSION 2.8.3)
project(middleware_to_mem)

set(VERSION_MAJOR 1)
set(VERSION_MINOR 1)
set(VERSION_PATCH 2)
string(TIMESTAMP BUILD_TIME \"%Y-%m-%d|%H:%M:%S\")

set(${PROJECT_NAME}_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

configure_file(
    "${PROJECT_SOURCE_DIR}/include/version.h.in"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version.h" 
)

file(WRITE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version
    "${PROJECT_NAME}:${${PROJECT_NAME}_VERSION}\n"
)

if(catkin_CROSS)
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS})
    set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS})
    message(STATUS "ARM-Cross")
else()
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_x86})
    set(catkin_LIBRARIES ${catkin_LIBRARYS_x86})
    message(STATUS "x86-64")
endif()


catkin_package(
    LIBRARIES middleware_to_mem comm_interface core_interface
)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -O0 -DPB_FIELD_32BIT -DSIMMULATION")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_BINARY_DIR}
)


#############
##  Build  ##
#############
add_library(middleware_to_mem
    src/middleware_to_sharedmem.c
    src/middleware_to_sharedmem_ptr.c
    src/logfifo.c
)

add_library(core_interface
    src/core_interface.cpp
)

add_library(comm_interface
    src/comm_interface.cpp
)

add_executable(test_app
    test/comm_tcp_server_test.cpp
)

add_executable(test_app2
    test/comm_tcp_client2_test.cpp
)

add_executable(test_app3
    test/comm_tcp_client_win7.cpp
)

target_link_libraries(middleware_to_mem
    ${catkin_LIBRARIES}
)

target_link_libraries(core_interface
    ${catkin_LIBRARIES}
    middleware_to_mem
)

target_link_libraries(comm_interface
    ${catkin_LIBRARIES}
)

target_link_libraries(test_app
    ${catkin_LIBRARIES}
    comm_interface
)

target_link_libraries(test_app2
    ${catkin_LIBRARIES}
    comm_interface
)

target_link_libraries(test_app3
    ${catkin_LIBRARIES}
    comm_interface
)

#############
## Install ##
#############
install(TARGETS middleware_to_mem core_interface comm_interface
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version 
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/version
)

install(TARGETS test_app test_app2 test_app3
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

