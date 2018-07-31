cmake_minimum_required(VERSION 2.8.3)
project(parameter_manager)

set(VERSION_MAJOR 3)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)
string(TIMESTAMP BUILD_TIME \"%Y-%m-%d|%H:%M:%S\")

set(${PROJECT_NAME}_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

configure_file(
    "${PROJECT_SOURCE_DIR}/include/version.h.in"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version.h" 
)

file(WRITE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version
    "${PROJECT_NAME}:${${PROJECT_NAME}_VERSION}\n"
)

if(${catkin_CROSS})
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS})
	set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS})
    message(STATUS "ARM-Cross")
else()
	set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_x86})
    set(catkin_LIBRARIES ${catkin_LIBRARYS_x86})
    message(STATUS "x86-64")
endif()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES param_value param_builder parameter_manager
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
add_library(param_value
    src/parameter_manager_param_value.cpp
)

add_library(param_builder
    src/parameter_manager_param_builder.cpp
)

add_library(parameter_manager
    src/parameter_manager_param_group.cpp
)

add_executable(manager_node
    src/manager_node.cpp
)

target_link_libraries(param_value
    ${catkin_LIBRARIES}
)

target_link_libraries(param_builder
    ${catkin_LIBRARIES}
)

target_link_libraries(parameter_manager
    ${catkin_LIBRARIES}
    param_value
    param_builder
)

target_link_libraries(manager_node
    ${catkin_LIBRARIES}
    param_value
    param_builder
    parameter_manager
)

#############
## Install ##
#############
install(TARGETS param_value
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS param_builder
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS parameter_manager
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS manager_node 
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/version
)


