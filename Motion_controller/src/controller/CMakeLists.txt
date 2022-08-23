cmake_minimum_required(VERSION 2.8.3)
project(controller)

#set(controller_MAIN_VERSION 1) #uncommemt if release version
set(VERSION_MAJOR 0)
set(VERSION_MINOR 1)
set(VERSION_PATCH 0)
set(VERSION_COMMIT \"ffffffff\") #from gitlab commit number.
string(TIMESTAMP BUILD_DATE \"%Y%m%d\")
string(TIMESTAMP BUILD_TIME \"%H:%M:%S\")

configure_file(
	"${PROJECT_SOURCE_DIR}/include/version.h.in"
	"${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version.h" 
)
set(${PROJECT_NAME}_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}.${BUILD_DATE}.${BUILD_TIME})
file(WRITE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version "${PROJECT_NAME}:${${PROJECT_NAME}_VERSION}\n")


#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D PB_FIELD_32BIT")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D PB_FIELD_32BIT")

include_directories(
	include
	../common
	../yaml_help/include
	../xml_help/include
    ../sem_help/include
	../thread_help/include 
	../core_comm/include
	../servo_comm/include
    ../pb/include
    ../protos/include
    ../file_manager/include
    ../basic_alg/include
    ../algorithm_base/include 
    ../system_model_manager/include 
    ../tp_comm/include
    ../log_manager/include
    ../axis/include
    ../error_queue/include
    ../base_device/include
    ../axis1000/include
    ../axis1001/include
    ../io_1000/include
    ../io_safety/include
    ../group/include
    ../rtm_spi/include
    ../nvram/include 
    ../reg_manager/include
    ../fio_device/include
    ../joint_constraint/include
    ../kinematics_alg/include
    ../dynamic_alg/include
    ../transformation/include
    ../coordinate_manager/include
    ../tool_manager/include
    ../trajectory_planner/include
    ../motion_control/include
    ../embed_python/include
    ../sensor_process/include
    ${PROJECT_BINARY_DIR}
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(src SRC)
add_executable(controller 
	${SRC}
)

link_directories(
	../../install/lib
)
target_link_libraries(controller
	${rtm_LIBRARIES}
	yaml_help
	xml_help
    sem_help
	thread_help
	core_comm_system
	servo_comm
    pb
    protos
    file_manager
    algorithm_base
    system_model_manager 
    tp_comm
    log_manager_producer
    axis
    error_queue
    axis1000
    base_device
    io_1000
    io_safety
    group
    rtm_spi
    nvram_handler
    reg_manager
    fio_device
    joint_constraint
    kinematics_alg
    dynamic_alg
    transformation
    tool_manager
    coordinate_manager
    trajectory_planner
    motion_control
    embed_python
    sensor_process
)
install(FILES ./config/controller.yaml 
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/component/
)

install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_version 
  DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/..
)
