cmake_minimum_required(VERSION 2.8.3)
project(trajectory_test)

SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/trajectory_test)

include_directories(
  include
  ../../common
  ../../yaml_help/include
  ../../basic_alg/include
  ../../kinematics_alg/include
  ../../dynamic_alg/include
  ../../joint_constraint/include  
  ../../trajectory_planner/include
  ${rtm_INCLUDE_DIRS}
)

add_executable(test_trajectory_planner
    test_trajectory_planner.cpp
)

add_executable(test_ds_curve
    test_ds_curve.cpp
)

target_link_libraries(test_trajectory_planner
    ${rtm_LIBRARIES}
    trajectory_planner
)

target_link_libraries(test_ds_curve
    ${rtm_LIBRARIES}
    trajectory_planner
)
