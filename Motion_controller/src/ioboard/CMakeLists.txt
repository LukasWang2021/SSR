cmake_minimum_required(VERSION 2.8.3)
project(ioboard)

# setting for include file.
set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)
string(TIMESTAMP BUILD_TIME \"%Y-%m-%d|%H:%M:%S\")

# setting for package version file.
set(${PROJECT_NAME}_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

# configure a header file to pass some settings to the source code.
configure_file(
  "${PROJECT_SOURCE_DIR}/include/version.h.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version.h" 
)

# add the search path for include files.
include_directories(${PROJECT_BINARY_DIR})

# write the verson info to a file.
file(WRITE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version "${PROJECT_NAME}:${${PROJECT_NAME}_VERSION}\n")

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

install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version 
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/version
)
