cmake_minimum_required(VERSION 2.8.3)
project(fio_device)

include_directories(
	include
	../common
	../base_device/include
	../log_manager/include
	../yaml_help/include
	${rtm_INCLUDE_DIRS}
)


aux_source_directory(src SRC)
add_library(fio_device SHARED
        ${SRC}
)

target_link_libraries(fio_device
        ${rtm_LIBRARIES}
)

install(FILES ./config/fio_config.yaml 
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/component/
)





