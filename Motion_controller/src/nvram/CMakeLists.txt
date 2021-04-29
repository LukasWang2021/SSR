cmake_minimum_required(VERSION 2.8.3)
project(nvram)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

include_directories(
  include
  ../yaml_help/include
  ../log_manager/include
  ../thread_help/include 
  ../rtm_spi/include
  ${rtm_INCLUDE_DIRS}
)

aux_source_directory(src SRC)

#add_executable(nvram_server
#    src/nvram_server.cpp
#)

link_directories(
	../../install/lib
)
add_library(nvram_handler SHARED
    src/nvram_handler.cpp
)

#target_link_libraries(nvram_server
#    ${rtm_LIBRARIES}
#    yaml_help
#    log_manager_producer
#    thread_help
#    rtm_spi
#)

target_link_libraries(nvram_handler 
    ${rtm_LIBRARIES}
)

