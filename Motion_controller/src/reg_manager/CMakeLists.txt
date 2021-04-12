cmake_minimum_required(VERSION 2.8.3)
project(reg_manager)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")

include_directories(
  include
  ../basic_alg/include
  ../common
  ../yaml_help/include
  ../log_manager/include
  ../nvram/include
  ${rtm_INCLUDE_DIRS}
)

aux_source_directory(src BASE_SRCS)

add_library(reg_manager SHARED
    ${BASE_SRCS}
)

target_link_libraries(reg_manager 
    ${rtm_LIBRARIES}
    yaml_help
    log_manager_producer
    nvram_handler 
)

install(FILES ./config/reg_manager.yaml 
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/component/
)
install(FILES ./config/r_reg.yaml 
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/reg/
)
install(FILES ./config/sr_reg.yaml 
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/reg/
)
install(FILES ./config/mr_reg.yaml 
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/reg/
)
install(FILES ./config/pr_reg.yaml 
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/reg/
)
install(FILES ./config/hr_reg.yaml 
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/reg/
)

