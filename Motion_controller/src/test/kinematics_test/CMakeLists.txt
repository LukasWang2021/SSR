cmake_minimum_required(VERSION 2.8.3)
project(kinematics_test)

SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/kinematics_test)

include_directories(
  include
  ../../basic_alg/include
  ../../yaml_help/include
  ../../kinematics_alg/include
  ${rtm_INCLUDE_DIRS}
)

add_executable(rtm_test kinematics_rtm_test.cpp)
target_link_libraries(rtm_test ${rtm_LIBRARIES} kinematics_alg basic_alg)
