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

aux_source_directory(src BASE_SRCS)
aux_source_directory(src/protocal/ PROTOCAL_SRCS)
aux_source_directory(src/message_type/ MSG_SRCS)
aux_source_directory(src/request/ REQ_SRCS)
aux_source_directory(src/response/ REP_SRCS)

add_library(protos
    ${BASE_SRCS}
    ${PROTOCAL_SRCS}
    ${MSG_SRCS}
    ${REQ_SRCS}
    ${REP_SRCS}
)

target_link_libraries(protos
  ${catkin_LIBRARIES}
)

install(TARGETS protos
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
