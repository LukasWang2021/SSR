cmake_minimum_required(VERSION 2.8.3)
project(joint_constraint)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -D PB_FIELD_32BIT ")

include_directories(
  include
  ../basic_alg/include
  ${rtm_INCLUDE_DIRS}
)

aux_source_directory(src BASE_SRCS)

add_library(joint_constraint SHARED
    ${BASE_SRCS}
)

link_directories(
	../../install/lib
)
target_link_libraries(joint_constraint
    ${rtm_LIBRARIES}
    basic_alg
)
