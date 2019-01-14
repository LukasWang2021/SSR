cmake_minimum_required(VERSION 2.8.3)
project(path_alg)

find_package(catkin REQUIRED COMPONENTS
  basic_alg
  log_manager
)

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

set(catkin_INCLUDE_DIRS
    ${catkin_INCLUDE_DIRS}
    ${depend_PATH_INC}/common
    ${depend_PATH_INC}/log_manager/include
    ${depend_PATH_INC}/basic_alg/include 
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES path_plan
    DEPENDS basic_alg
    DEPENDS logger
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(path_plan
    src/path_plan.cpp
)

target_link_libraries(path_plan
    ${catkin_LIBRARIES}
    basic_alg 
    logger
)

install(TARGETS path_plan
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
