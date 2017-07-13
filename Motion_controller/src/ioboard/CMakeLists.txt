cmake_minimum_required(VERSION 2.8.3)
project(ioboard)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ioboard
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include)

add_library(${PROJECT_NAME}
   src/IOboard.c src/fakeio.c
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ioboard # ioboard_node 
#       TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


