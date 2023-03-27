cmake_minimum_required(VERSION 2.8.3)
project(embed_python)
#This embed python module based on python 3.9.5

include_directories(
	include
	../common
	../yaml_help/include
	../xml_help/include
    ../sem_help/include
	../thread_help/include 
	../core_comm/include
	../servo_comm/include
    ../file_manager/include
    ../basic_alg/include
    ../algorithm_base/include 
    ../system_model_manager/include 
    ../log_manager/include
    ../reg_manager/include
    ../axis/include
    ../error_queue/include
    ../base_device/include
    ../axis1000/include
    ../axis1001/include
    ../group/include
    ../io_1000/include
    ../rtm_spi/include
    ../nvram/include
    ../joint_constraint/include
    ../kinematics_alg/include
    ../dynamic_alg/include 
    ../transformation/include
    ../trajectory_planner/include
    ../coordinate_manager/include
    ../tool_manager/include
    ../motion_control/include
    ../fio_device/include
    ../sensor_process/include
	${CROSS_ROOT}/usr/local/include/python3.9/internal
    ${CROSS_ROOT}/usr/local/include/python3.9/cpython
    ${CROSS_ROOT}/usr/local/include/python3.9
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(src SRC)

add_library(embed_python SHARED
	${SRC}
)
link_directories(
	${CROSS_ROOT}/usr/local/lib
    ${CROSS_ROOT}/usr/local/lib/python3.9/config-3.9-aarch64-linux-gnu
	../../install/lib
)
target_link_libraries(embed_python
	${CROSS_ROOT}/usr/local/lib/libpython3.9.so
    ${USR_LIB_DIR}/libcrypt.so
    ${USR_LIB_DIR}/libpthread.so
    ${USR_LIB_DIR}/libdl.so
    ${USR_LIB_DIR}/libutil.so
    ${USR_LIB_DIR}/libm.so
	${rtm_LIBRARIES}
	yaml_help
	thread_help
    sem_help
    log_manager_producer
    error_queue
    motion_control
)

install(FILES ./config/embed_python.yaml 
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/component/
)

add_subdirectory(./module)
