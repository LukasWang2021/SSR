cmake_minimum_required(VERSION 2.8.3)
project(service_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  middleware_to_mem
)

if(${catkin_CROSS})
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS} ${depend_PATH_INC}/middleware_to_mem/include)
	set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS} ${depend_PATH_LIB}/libcomm_interface.so)
endif(${catkin_CROSS})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS roscpp middleware_to_mem comm_interface
#  DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -lrt -lpthread -O0 -DPB_FIELD_32BIT -DSIMMULATION")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(service_manager src/service_manager.cpp src/response_actions.cpp)
target_link_libraries(service_manager ${catkin_LIBRARIES} middleware_to_mem comm_interface)

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

install(TARGETS service_manager
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
