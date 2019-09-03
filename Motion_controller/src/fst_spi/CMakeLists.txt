cmake_minimum_required(VERSION 2.8.3)
project(fst_spi)


find_package(catkin REQUIRED COMPONENTS
    roscpp)



if(catkin_CROSS)
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS})
    set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS})
	set(CMAKE_CXX_FLAGS "-DCROSS_PLATFORM")
	set(CMAKE_C_FLAGS "-DCROSS_PLATFORM")
    message(STATUS "ARM-Cross")
else()
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_x86})
    set(catkin_LIBRARIES ${catkin_LIBRARYS_x86})
    message(STATUS "x86-64")
endif()


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES fst_spi
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -pthread")
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(fst_spi
    src/fst_spi.cpp
)

target_link_libraries(fst_spi
    ${catkin_LIBRARIES}
 )

install(TARGETS fst_spi
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

