cmake_minimum_required(VERSION 2.8.3)
project(controller)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    middleware_to_mem
    parameter_manager
#    service_manager
#    protos
#    tp_comm
)

#set(controller_MAIN_VERSION 1) #uncommemt if release version
set(VERSION_MAJOR 3)
set(VERSION_MINOR 7)
set(VERSION_PATCH 1)
set(VERSION_COMMIT \"18dd901dc\") #from gitlab commit number.
string(TIMESTAMP BUILD_DATE \"%Y%m%d\")
string(TIMESTAMP BUILD_TIME \"%H:%M:%S\")

configure_file(
    "${PROJECT_SOURCE_DIR}/include/version.h.in"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version.h" 
)
set(${PROJECT_NAME}_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})
file(WRITE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version "${PROJECT_NAME}:${${PROJECT_NAME}_VERSION}\n")

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
    ${depend_PATH_INC}/middleware_to_mem/include
    ${depend_PATH_INC}/parameter_manager/include
    ${depend_PATH_INC}/log_manager/include
    ${depend_PATH_INC}/service_manager/include
    ${depend_PATH_INC}/prg_interpreter/include
    ${depend_PATH_INC}/protos/include 
    ${depend_PATH_INC}/tp_comm/include 
    ${depend_PATH_INC}/thread_help/include 
    ${depend_PATH_INC}/common/
    ${depend_PATH_INC}/tool_manager/include 
    ${depend_PATH_INC}/coordinate_manager/include 
    ${depend_PATH_INC}/reg_manager/include 
    ${depend_PATH_INC}/process_comm/include 
    ${depend_PATH_INC}/device_manager/include 
    ${depend_PATH_INC}/xml_help/include 
    ${depend_PATH_INC}/base_device/include
    ${depend_PATH_INC}/error_monitor/include 
    ${depend_PATH_INC}/motion_control/include 
    ${depend_PATH_INC}/path_traj_alg/include 
    ${depend_PATH_INC}/axis_group_manager/include
    ${depend_PATH_INC}/base_axis_group/include
    ${depend_PATH_INC}/log_service/include
    ${depend_PATH_INC}/basic_alg/include 
    ${depend_PATH_INC}/kinematics_alg/include 
    ${depend_PATH_INC}/transformation/include 
    ${depend_PATH_INC}/process_comm/include 
    ${depend_PATH_INC}/preformance_monitor/include 
    ${depend_PATH_INC}/modbus_manager/include
    ${depend_PATH_INC}/fst_safety_device/include 
    ${depend_PATH_INC}/io_mapping/include 
    ${depend_PATH_INC}/io_manager/include 
    ${depend_PATH_INC}/fst_io_device/include 
    ${depend_PATH_INC}/virtual_io_device/include 
    ${depend_PATH_INC}/program_launching/include 
    ${depend_PATH_INC}/file_manager/include 
    ${depend_PATH_INC}/system_manager/include 
    ${depend_PATH_INC}/param_manager/include 
    ${depend_PATH_INC}/nvram/include
    ${depend_PATH_INC}/fst_spi/include
)

catkin_package(
    DEPENDS parameter_manager
    DEPENDS comm_interface
    DEPENDS core_interface
    DEPENDS log_manager
    DEPENDS thread_help 
    DEPENDS error_monitor
    DEPENDS modbus_manager
    DEPENDS io_mapping
    DEPENDS io_manager
    DEPENDS program_launching
    DEPENDS file_manager
    DEPENDS system_manager
    DEPENDS param_manager
    DEPENDS transformation
    DEPENDS nvram 
    DEPENDS fst_spi 
)
 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -DPB_FIELD_32BIT")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_BINARY_DIR}
    /usr/include/python2.7 
)

###########
## Build ##
###########
aux_source_directory(src BASE_SRCS)

add_executable(controller
    ${BASE_SRCS}
    ${NANOPB_DIR}/pb_decode.c 
    ${NANOPB_DIR}/pb_encode.c 
    ${NANOPB_DIR}/pb_common.c
)

target_link_libraries(controller
    ${catkin_LIBRARIES}
    comm_interface
    core_interface
    protos 
    tp_comm
    modbus_manager
    thread_help
    tool_manager 
    coordinate_manager 
    reg_manager 
    process_comm 
    device_manager 
    error_monitor 
    motion_control
    log_service
    preformance_monitor 
    fst_safety_device
    io_mapping
    io_manager
    program_launching
    file_manager
    system_manager
    param_manager
    transformation
)

#############
## Install ##
#############
install(TARGETS controller 
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}    
)

install(FILES config/controller.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/component_param/
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version 
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/version
)

#############
## Testing ##
#############
if(GTEST)
    message("GTest==>" ${GTEST_LIBRARIES})
    # Link utest with what we want to test and the GTest and pthread library.
    add_executable(ti_test test/unit/main.cpp)
    target_link_libraries(ti_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
    install(TARGETS ti_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
