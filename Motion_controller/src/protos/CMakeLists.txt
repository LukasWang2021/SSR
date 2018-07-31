cmake_minimum_required(VERSION 2.8.3)
project(protos)

find_package(catkin REQUIRED COMPONENTS
    roscpp
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


catkin_package(
    LIBRARIES protos
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -D PB_FIELD_32BIT")

include_directories(
  include
  include/message_type/
  include/protocal/ 
  include/request/
  include/response/
  ${NANOPB_DIR}
)

add_library(protos
  src/protocal/comm.pb.cpp 
  src/protocal/publish.pb.cpp 
  src/protocal/request_header.pb.cpp 
  src/protocal/response_header.pb.cpp 
  src/message_type/base.pb.cpp
  src/message_type/test.pb.cpp
  src/request/request_base.pb.cpp
  src/response/response_base.pb.cpp
)

target_link_libraries(protos
  ${catkin_LIBRARIES}
)

install(TARGETS protos
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
