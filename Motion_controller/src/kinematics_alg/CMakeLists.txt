cmake_minimum_required(VERSION 2.8.3)
project(kinematics_alg)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")

include_directories(
  include
  ../basic_alg/include
  ../yaml_help/include
  ${rtm_INCLUDE_DIRS}
)

aux_source_directory(src BASE_SRCS)

add_library(kinematics_alg SHARED
    ${BASE_SRCS}
)

target_link_libraries(kinematics_alg
    ${rtm_LIBRARIES}
    basic_alg
    yaml_help
)


install(FILES ./config/arm_dh.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/axis_group
)

install(FILES ./config/toll_dh.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/axis_group
)
