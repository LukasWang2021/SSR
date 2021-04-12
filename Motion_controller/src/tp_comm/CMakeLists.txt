cmake_minimum_required(VERSION 2.8.3)
project(tp_comm)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D PB_FIELD_32BIT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D PB_FIELD_32BIT")

include_directories(
  include
  ../pb/include
  ../protos/include
  ../common
  ../yaml_help/include
  ../thread_help/include/
  ../log_manager/include/
  ../error_queue/include/
  /usr/include/python2.7 
  ${rtm_INCLUDE_DIRS}
)

aux_source_directory(src BASE_SRCS)

add_library(tp_comm SHARED
    ../pb/src/pb_decode.c
    ../pb/src/pb_encode.c
    ../pb/src/pb_common.c
    ${BASE_SRCS}
)

target_link_libraries(tp_comm
    ${rtm_LIBRARIES}
    protos
    yaml_help
    thread_help
    log_manager_producer
    error_queue
)

install(FILES ./config/tp_comm.yaml 
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/component/
)

