cmake_minimum_required(VERSION 2.8.3)
project(modbus_manager)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    log_manager
    parameter_manager
    error_monitor
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

catkin_package( 
    INCLUDE_DIRS include
    LIBRARIES modbus_manager
)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g -lrt -lpthread -O0 -DPB_FIELD_32BIT -DSIMMULATION")

set(catkin_INCLUDE_DIRS 
    ${catkin_INCLUDE_DIRS}
    ${depend_PATH_INC}/log_manager/include/
    ${depend_PATH_INC}/log_service/include/
    ${depend_PATH_INC}/error_monitor/include/
    ${depend_PATH_INC}/parameter_manager/include/
    ${depend_PATH_INC}/common/
)

include_directories(
    include/
    include/modbus/
    ${catkin_INCLUDE_DIRS}
    /usr/include/python2.7 
)

aux_source_directory(src/ BASE_SRCS)
aux_source_directory(src/modbus/ MODBUS_SRCS)

add_library(modbus_manager
    ${BASE_SRCS}
    ${MODBUS_SRCS}
)

target_link_libraries(modbus_manager
    ${catkin_LIBRARIES}
    parameter_manager
  )

install(TARGETS modbus_manager
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 )

install(FILES config/modbus_manager.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/component_param/
)

install(FILES config/modbus_device_manager.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/component_param/
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# add_executable(test_modbus_manager 
    # ${BASE_SRCS}
    # ${MODBUS_SRCS}
    # )
# 
# target_link_libraries(test_modbus_manager 
    # ${catkin_LIBRARIES}
    # parameter_manager
    # logger
    # )
# 
# install(TARGETS test_modbus_manager 
    # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/test)
