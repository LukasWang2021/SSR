cmake_minimum_required(VERSION 2.8.3)
project(servo_diag)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  middleware_to_mem
  parameter_manager
)

if(${catkin_CROSS})
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS} ${depend_PATH_INC}/middleware_to_mem/include
            ${depend_PATH_INC}/parameter_manager/include)
	set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS} ${depend_PATH_LIB}/libcomm_interface.so)
endif(${catkin_CROSS})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS roscpp middleware_to_mem parameter_manager
#  DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -lrt -lpthread -O0 -DPB_FIELD_32BIT -DSIMMULATION")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(servo_diag src/servo_diag.cpp src/servo_conf.cpp src/servo_service.cpp src/data_monitor.cpp)
target_link_libraries(servo_diag ${catkin_LIBRARIES} middleware_to_mem comm_interface parameter_manager)

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

install(TARGETS servo_diag
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY cfg
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
