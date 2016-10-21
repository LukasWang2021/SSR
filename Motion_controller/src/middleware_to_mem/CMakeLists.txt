cmake_minimum_required(VERSION 2.8.3)
project(middleware_to_mem)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES middleware_to_mem
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(middleware_to_mem src/middleware_to_sharedmem.c src/middleware_to_sharedmem_ptr.c)
target_link_libraries(middleware_to_mem ${catkin_LIBRARIES} librt.so)

add_executable(send_trajectory_node src/trajectory_queue.cpp src/service_actions.c)
target_link_libraries(send_trajectory_node ${catkin_LIBRARIES} librt.so middleware_to_mem)

add_executable(send_trajectory_fake src/trajectory_queue_fake.cpp)
target_link_libraries(send_trajectory_fake ${catkin_LIBRARIES} librt.so middleware_to_mem)

add_executable(service_test test/service_test.c)
target_link_libraries(service_test ${catkin_LIBRARIES} librt.so middleware_to_mem)

install(TARGETS middleware_to_mem
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 )
install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

install(TARGETS send_trajectory_node send_trajectory_fake service_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
