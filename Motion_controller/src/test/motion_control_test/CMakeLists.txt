cmake_minimum_required(VERSION 2.8.3)
project(kinematics_test)

SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/motino_control_test)

include_directories(
  include
  ../../common
  ../../log_manager/include
  ../../basic_alg/include
  ../../yaml_help/include
  ../../error_queue/include
  ../../thread_help/include
  ../../coordinate_manager/include
  ../../tool_manager/include 
  ../../transformation/include
  ../../kinematics_alg/include
  ../../dynamic_alg/include 
  ../../nvram/include
  ../../rtm_spi/include
  ../../joint_constraint/include
  ../../trajectory_planner/include
  ../../motion_control/include
  ${rtm_INCLUDE_DIRS}
)

add_executable(manual_test
    manual_test.cpp
)

add_executable(output_trajectory
    output_trajectory.cpp
)

add_executable(print_trajectory
    print_trajectory.cpp
)

target_link_libraries(manual_test
    ${rtm_LIBRARIES}
    motion_control
)

target_link_libraries(output_trajectory
    ${rtm_LIBRARIES}
    motion_control
)

target_link_libraries(print_trajectory
    ${rtm_LIBRARIES}
    motion_control
)
