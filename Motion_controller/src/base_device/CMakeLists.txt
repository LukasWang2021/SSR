cmake_minimum_required(VERSION 2.8.3)
project(base_device)

include_directories(
  include
  ../common/
  ${rtm_INCLUDE_DIRS}
)
aux_source_directory(src SRC)

add_library(base_device SHARED
    ${SRC}
)

target_link_libraries(base_device
	${rtm_LIBRARIES}
)
