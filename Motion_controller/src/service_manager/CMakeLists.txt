cmake_minimum_required(VERSION 2.8.3)
project(service_manager)

find_package(catkin REQUIRED COMPONENTS
    middleware_to_mem
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

set(catkin_INCLUDE_DIRS
    ${catkin_INCLUDE_DIRS}
    ${depend_PATH_INC}/middleware_to_mem/include
    ${depend_PATH_INC}/common 
    ${depend_PATH_INC}/log_manager/include
)

catkin_package(
    DEPENDS middleware_to_mem
    DEPENDS log_manager
    DEPENDS parameter_manager
)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -O0 -DPB_FIELD_32BIT -DSIMMULATION")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_BINARY_DIR}
    ${depend_PATH_INC}/parameter_manager/include
)


#############
##  Build  ##
#############
aux_source_directory(src BASE_SRCS)

add_executable(service_manager
    ${BASE_SRCS}
)

target_link_libraries(service_manager
    ${catkin_LIBRARIES}
    middleware_to_mem
    comm_interface
    logger
    parameter_manager
)

#############
## Install ##
#############
install(TARGETS service_manager
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES config/service_manager.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/component_param/
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
