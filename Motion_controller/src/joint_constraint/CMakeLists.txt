cmake_minimum_required(VERSION 2.8.3)
project(joint_constraint)

find_package(catkin REQUIRED COMPONENTS
    basic_alg
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
    ${depend_PATH_INC}/basic_alg/include 
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES joint_constraint
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(src BASE_SRCS)

add_library(joint_constraint
    ${BASE_SRCS}
)

target_link_libraries(joint_constraint
    ${catkin_LIBRARIES}
    basic_alg
)

install(TARGETS joint_constraint
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

