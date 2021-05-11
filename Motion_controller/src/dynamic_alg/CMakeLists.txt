cmake_minimum_required(VERSION 2.8.3)
project(dynamic_alg)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -lrt -O0 -D PB_FIELD_32BIT")

include_directories(
  include
  ../common
  ../yaml_help/include
  ../log_manager/include
  ../basic_alg/include
  ../reg_manager/include
  ../kinematics_alg/include
  ${rtm_INCLUDE_DIRS}
)

aux_source_directory(src BASE_SRCS)

link_directories(
	../../install/lib
)

add_library(dynamic_alg SHARED
    ${BASE_SRCS}
)

target_link_libraries(dynamic_alg
    ${rtm_LIBRARIES}
    yaml_help
    log_manager_producer
    reg_manager
    kinematics_alg
    basic_alg
)


install(FILES ./config/dynamic_payload_info.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/dynamics/
)

install(FILES ./config/dynamic_alg_rtm_type.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/dynamics/
)

install(FILES ./config/dynamic_alg_rtm_0kg.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/dynamics/
)

install(FILES ./config/dynamic_alg_rtm_1kg.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/dynamics/
)

install(FILES config/dynamic_alg_rtm_3.5kg.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/dynamics/
)

install(FILES config/dynamic_alg_rtm_3.5kg_custom.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/dynamics/
)

install(FILES config/dynamic_alg_rtm_7kg.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/dynamics/
)

install(FILES config/dynamic_alg_rtm_7kg_custom.yaml
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/dynamics/
)


