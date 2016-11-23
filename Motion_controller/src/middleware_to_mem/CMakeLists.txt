cmake_minimum_required(VERSION 2.8.3)
project(middleware_to_mem)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES middleware_to_mem comm_interface
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(middleware_to_mem src/middleware_to_sharedmem.c src/middleware_to_sharedmem_ptr.c)
target_link_libraries(middleware_to_mem ${catkin_LIBRARIES} librt.so)

add_library(comm_interface src/comm_interface.cpp)
target_link_libraries(comm_interface ${catkin_LIBRARIES} middleware_to_mem nanomsg)

add_executable(comm_monitor src/comm_monitor.cpp src/response_actions.cpp)
target_link_libraries(comm_monitor ${catkin_LIBRARIES} middleware_to_mem comm_interface)

add_executable(send_trajectory_node src/trajectory_queue.cpp src/service_actions.c)
target_link_libraries(send_trajectory_node ${catkin_LIBRARIES} librt.so middleware_to_mem)

add_executable(send_trajectory_fake src/trajectory_queue_fake.cpp)
target_link_libraries(send_trajectory_fake ${catkin_LIBRARIES} librt.so middleware_to_mem)

add_executable(test_app test/test.cpp)
target_link_libraries(test_app ${catkin_LIBRARIES} middleware_to_mem comm_interface librt.so)

install(TARGETS middleware_to_mem comm_interface
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 )
install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

install(TARGETS send_trajectory_node send_trajectory_fake test_app comm_monitor
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
