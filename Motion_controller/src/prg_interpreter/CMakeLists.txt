cmake_minimum_required(VERSION 2.8.3)
project(prg_interpreter)


find_package(catkin REQUIRED COMPONENTS 
    device_manager 
    controller 
    log_manager
#    io_manager
#    motion_plan
    parameter_manager
#    mem_share
    motion_control
    kinematics_alg
    xml_help
    middleware_to_mem
    dynamic_alg
)
set(VERSION_MAJOR ?)
set(VERSION_MINOR ?)
set(VERSION_PATCH ?)
string(TIMESTAMP BUILD_TIME \"%Y-%m-%d|%H:%M:%S\")

set(${PROJECT_NAME}_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

configure_file(
    "${PROJECT_SOURCE_DIR}/include/version.h.in"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version.h" 
)

file(WRITE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version
    "${PROJECT_NAME}:${${PROJECT_NAME}_VERSION}\n"
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
    /usr/include/python2.7 

    ${depend_PATH_INC}/controller/include 
    ${depend_PATH_INC}/comm_interface/include 
    ${depend_PATH_INC}/middleware_to_mem/include
    ${depend_PATH_INC}/parameter_manager/include
    ${depend_PATH_INC}/log_manager/include
    ${depend_PATH_INC}/service_manager/include
    ${depend_PATH_INC}/prg_interpreter/include
    ${depend_PATH_INC}/prg_interpreter/include/reg_manager
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
    ${depend_PATH_INC}/kinematics_alg/include
    ${depend_PATH_INC}/dynamic_alg/include
    ${depend_PATH_INC}/trajectory_planner/include
    ${depend_PATH_INC}/joint_constraint/include
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wwrite-strings -fpermissive -O0")

catkin_package(
#    DEPENDS motion_plan  
    DEPENDS device_manager 
    DEPENDS controller 
    DEPENDS fst_io_device 
    DEPENDS motion_control 
    DEPENDS basic_alg 
    DEPENDS nvram 
    DEPENDS fst_spi 
    DEPENDS dynamic_alg
    DEPENDS kinematics_alg
    DEPENDS trajectory_planner
    DEPENDS joint_constraint
    DEPENDS python2.7 
)
 

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_BINARY_DIR}
)

###########
## Build ##
###########
add_library(prg_interpreter
    src/reg_manager/reg_manager_interface_wrapper.cpp
    src/reg_manager/forsight_registers_manager.cpp
    src/forsight_innercmd.cpp
    src/forsight_xml_reader.cpp
    src/forsight_int_main.cpp
    src/forsight_basint.cpp
    src/forsight_innerfunc.cpp
    src/forsight_inter_control.cpp
    src/forsight_io_controller.cpp
    src/forsight_program_property.cpp
    src/forsight_cJSON.cpp
    src/launch_code_mgr.cpp
    src/forsight_home_pose.cpp
    src/forsight_external_resource.cpp
)

target_link_libraries(prg_interpreter
    ${catkin_LIBRARIES}
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
    file_manager
    system_manager
    param_manager
    transformation
    xml2
    process_comm 
    thread_help
    pthread
    modbus_manager
    logger
    basic_alg
    kinematics_alg
    dynamic_alg
    xml_help
    trajectory_planner
    joint_constraint
    python2.7
    boost_filesystem
)

#############
## Install ##
#############
install(TARGETS prg_interpreter
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/version
)

install(FILES config/programs_path.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/component_param
)

install(FILES config/user_defined_variable.xml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/interpreter
)

install(FILES config/prg_interpreter_config.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/component_param
)


