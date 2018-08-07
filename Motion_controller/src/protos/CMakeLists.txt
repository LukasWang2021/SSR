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
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")


include_directories(
  include
  include/message_type/
  include/protocal/
  include/request/
  include/response/
  ${NANOPB_DIR}
)

add_library(protos
  src/protocal/comm.pb.c
  src/protocal/publish.pb.c
  src/protocal/request_header.pb.c
  src/protocal/response_header.pb.c

  src/message_type/base.pb.c
  src/message_type/publish_topic.pb.c
  src/message_type/publish_table.pb.c
  src/message_type/rpc_table.pb.c
  src/message_type/joint_list.pb.c
  src/message_type/pose_euler.pb.c
  src/message_type/register_mr.pb.c
  src/message_type/register_pr.pb.c
  src/message_type/register_r.pb.c
  src/message_type/register_sr.pb.c
  src/message_type/tool_frame.pb.c
  src/message_type/user_frame.pb.c

  src/request/request_base.pb.c
  src/request/request_publish_topic.pb.c
  src/request/request_register_pr.pb.c
  src/request/request_register_r.pb.c
  src/request/request_register_mr.pb.c
  src/request/request_register_sr.pb.c
  src/request/request_tool_frame.pb.c
  src/request/request_user_frame.pb.c

  src/response/response_base.pb.c
  src/response/response_publish_table.pb.c
  src/response/response_rpc_table.pb.c
  src/response/response_register_pr.pb.c
  src/response/response_register_r.pb.c
  src/response/response_register_mr.pb.c
  src/response/response_register_sr.pb.c
  src/response/response_tool_frame.pb.c
  src/response/response_user_frame.pb.c
)

target_link_libraries(protos
  ${catkin_LIBRARIES}
)

install(TARGETS protos
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
