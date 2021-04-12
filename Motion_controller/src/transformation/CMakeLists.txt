cmake_minimum_required(VERSION 2.8.3)
project(transformation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")

include_directories(
  include
  ../basic_alg/include
  ../kinematics_alg/include
  ../yaml_help/include
  ${rtm_INCLUDE_DIRS}
)

aux_source_directory(src BASE_SRCS)

add_library(transformation SHARED
    ${BASE_SRCS}
)

target_link_libraries(transformation 
    ${rtm_LIBRARIES}
    kinematics_alg
)


