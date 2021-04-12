cmake_minimum_required(VERSION 2.8.3)
project(protos)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D PB_FIELD_32BIT")

include_directories(
  include
  include/message_type/
  include/protocol/
  include/request/
  include/response/
  include/event/
  ../pb/include
  ../common
  ${rtm_INCLUDE_DIRS}
)

aux_source_directory(src BASE_SRCS)
aux_source_directory(src/protocol/ PROTOCAL_SRCS)
aux_source_directory(src/message_type/ MSG_SRCS)
aux_source_directory(src/request/ REQ_SRCS)
aux_source_directory(src/response/ REP_SRCS)
aux_source_directory(src/event/ EVENT_SRCS)

add_library(protos SHARED
    ${BASE_SRCS}
    ${PROTOCAL_SRCS}
    ${MSG_SRCS}
    ${REQ_SRCS}
    ${REP_SRCS}
    ${EVENT_SRCS}
)

target_link_libraries(protos
  ${rtm_LIBRARIES}
  pb
)

