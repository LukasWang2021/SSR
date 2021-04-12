cmake_minimum_required(VERSION 2.8.3)
project(dynamics_test)

SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/dynamics_test)

include_directories(
  include
  ../../common
  ../../yaml_help/include
  ../../log_manager/include
  ../../basic_alg/include
  ../../kinematics_alg/include
  ../../dynamic_alg/include
  ${rtm_INCLUDE_DIRS}
)

add_executable(test_inverse
    test_inverse.cpp
)

target_link_libraries(test_inverse
    ${rtm_LIBRARIES}
    dynamic_alg
)

add_executable(test_inverse_time
    test_inverse_time.cpp
)

target_link_libraries(test_inverse_time
    ${rtm_LIBRARIES}
    dynamic_alg
)

add_executable(test_direct_time
    test_direct_time.cpp
)

target_link_libraries(test_direct_time
    ${rtm_LIBRARIES}
    dynamic_alg
)

add_executable(test_scale_compare
    test_scale_compare.cpp
)

target_link_libraries(test_scale_compare
    ${rtm_LIBRARIES}
    dynamic_alg
)
