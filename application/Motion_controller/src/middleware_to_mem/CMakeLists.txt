cmake_minimum_required(VERSION 2.8.3)
project(middleware_to_mem)

# setting for include file.
set(VERSION_MAJOR 1)
set(VERSION_MINOR 1)
set(VERSION_PATCH 2)
string(TIMESTAMP BUILD_TIME \"%Y-%m-%d|%H:%M:%S\")

# setting for package version file.
set(${PROJECT_NAME}_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

# configure a header file to pass some settings to the source code.
configure_file(
  "${PROJECT_SOURCE_DIR}/include/version.h.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version.h" 
)

# add the search path for include files
include_directories(${PROJECT_BINARY_DIR})

# write the verson info to a file
file(WRITE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version "${PROJECT_NAME}:${${PROJECT_NAME}_VERSION}\n")

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

if(${catkin_CROSS})
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS} ${depend_PATH_INC})
	set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS})
else(${catkin_CROSS})
	set(EXT_LIB nanomsg)
endif(${catkin_CROSS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES middleware_to_mem comm_interface core_interface
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -lrt -lpthread -O0 -DPB_FIELD_32BIT -DSIMMULATION")

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(middleware_to_mem src/middleware_to_sharedmem.c src/middleware_to_sharedmem_ptr.c src/logfifo.c)
target_link_libraries(middleware_to_mem ${catkin_LIBRARIES})

add_library(core_interface src/core_interface.cpp)
target_link_libraries(core_interface ${catkin_LIBRARIES} middleware_to_mem)

add_library(comm_interface src/comm_interface.cpp)
target_link_libraries(comm_interface ${catkin_LIBRARIES} ${EXT_LIB})

add_executable(test_app test/comm_tcp_server_test.cpp)
target_link_libraries(test_app ${catkin_LIBRARIES} comm_interface)

add_executable(test_app2 test/comm_tcp_client2_test.cpp)
target_link_libraries(test_app2 ${catkin_LIBRARIES} comm_interface)

add_executable(test_app3 test/comm_tcp_client_win7.cpp)
target_link_libraries(test_app3 ${catkin_LIBRARIES} comm_interface)

install(TARGETS middleware_to_mem core_interface comm_interface
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version 
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/version
)

install(TARGETS test_app test_app2 test_app3
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

