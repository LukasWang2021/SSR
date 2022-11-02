cmake_minimum_required(VERSION 2.8.3)
project(numerical_doik_test)

SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/numerical_doik_test)

include_directories(
  include
  ../../basic_alg/include
  ../../yaml_help/include
  ../../kinematics_alg/include
  ${rtm_INCLUDE_DIRS}
)

add_executable(numerical_doik_test numeriacl_doik_test.cpp)
target_link_libraries(numerical_doik_test ${rtm_LIBRARIES} kinematics_alg basic_alg)
