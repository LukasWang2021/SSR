cmake_minimum_required(VERSION 2.8.3)
project(mem_share)

# add the search path for include files
include_directories(${PROJECT_BINARY_DIR})

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

if(${catkin_CROSS})
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS} )
endif(${catkin_CROSS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES io_manager
)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -lrt -lpthread -O0 ")

include_directories(include ${catkin_INCLUDE_DIRS})

aux_source_directory(src BASE_SRCS)

add_library(mem_share ${BASE_SRCS})

target_link_libraries(mem_share)


install(TARGETS mem_share
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

