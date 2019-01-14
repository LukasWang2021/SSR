cmake_minimum_required(VERSION 2.8.3)
project(log_service)

find_package(catkin REQUIRED COMPONENTS
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
    /usr/include/python2.7 
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES log_service
    DEPENDS python2.7 
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(src BASE_SRCS)

add_library(log_service
    ${BASE_SRCS}
)

target_link_libraries(log_service
    ${catkin_LIBRARIES}
    python2.7
)

install(FILES config/send_one_alarm.py 
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/python/
)

install(TARGETS log_service
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


