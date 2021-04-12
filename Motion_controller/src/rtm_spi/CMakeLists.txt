cmake_minimum_required(VERSION 2.8.3)
project(rtm_spi)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -pthread")

include_directories(
  include
  ${rtm_INCLUDE_DIRS}
)

add_library(rtm_spi SHARED
    src/rtm_spi.cpp
)

link_directories(
	../../install/lib
)

target_link_libraries(rtm_spi 
    ${rtm_LIBRARIES}
 )


