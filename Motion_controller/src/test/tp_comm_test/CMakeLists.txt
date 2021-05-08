cmake_minimum_required(VERSION 2.8.3)
project(tp_comm_test)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D PB_FIELD_32BIT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D PB_FIELD_32BIT")
SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/tp_comm_test)

include_directories(
  include
  ../../pb/include
  ../../protos/include
  ../../common
  ../../thread_help/include/
  ${rtm_INCLUDE_DIRS}
)

add_library(tp_comm_test SHARED
  ../../pb/src/pb_decode.c
  ../../pb/src/pb_encode.c
  ../../pb/src/pb_common.c
  src/tp_comm_test.cpp
)

target_link_libraries(tp_comm_test
  ${rtm_LIBRARIES}
  protos
  thread_help
)

add_subdirectory(src/test_axis)
add_subdirectory(src/test_common)
add_subdirectory(src/test_controller)
add_subdirectory(src/test_event)
add_subdirectory(src/test_io)
add_subdirectory(src/test_servo_sampling)
add_subdirectory(src/test_servo1001)
add_subdirectory(src/test_group)
add_subdirectory(src/test_frame)