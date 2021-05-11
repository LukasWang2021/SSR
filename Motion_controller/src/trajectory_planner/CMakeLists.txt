cmake_minimum_required(VERSION 2.8.3)
project(trajectory_planner)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -D PB_FIELD_32BIT")

include_directories(
  include
  ../common
  ../yaml_help/include
  ../log_manager/include
  ../basic_alg/include
  ../kinematics_alg/include
  ../dynamic_alg/include
  ../joint_constraint/include  
  ${rtm_INCLUDE_DIRS}
)

aux_source_directory(src BASE_SRCS)
aux_source_directory(src/ds_planner DS_SRCS)

link_directories(
	../../install/lib
)

add_library(trajectory_planner SHARED
    ${BASE_SRCS}
    ${DS_SRCS}
)

target_link_libraries(trajectory_planner
    ${rtm_LIBRARIES}
    yaml_help
    basic_alg
    kinematics_alg
    dynamic_alg
    joint_constraint
)

install(FILES ./config/config.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/algorithm/
)

install(FILES ./config/constraint.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/algorithm/
)
