cmake_minimum_required(VERSION 2.8.3)
project(safety)

set(VERSION_MAJOR ?)
set(VERSION_MINOR ?)
set(VERSION_PATCH ?)
string(TIMESTAMP BUILD_TIME \"%Y-%m-%d|%H:%M:%S\")

set(${PROJECT_NAME}_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

configure_file(
    "${PROJECT_SOURCE_DIR}/include/version.h.in"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version.h" 
)

file(WRITE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version
    "${PROJECT_NAME}:${${PROJECT_NAME}_VERSION}\n"
)

if(catkin_CROSS)
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS})
    set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS})
    message(STATUS "ARM-Cross")
else()
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_x86})
    set(catkin_LIBRARIES ${catkin_LIBRARYS_x86})
    message(STATUS "x86-64")
endif()

catkin_package(
    LIBRARIES safety
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_BINARY_DIR}
)

###########
## Build ##
###########
add_library(safety
    src/safety.c
)

target_link_libraries(safety
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS safety # safety_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/version
)
