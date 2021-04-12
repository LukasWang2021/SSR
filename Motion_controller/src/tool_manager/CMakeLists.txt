cmake_minimum_required(VERSION 2.8.3)
project(tool_manager)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")

include_directories(
  include
  ../yaml_help/include
  ../log_manager/include
  ../basic_alg/include
  ../common
  ${rtm_INCLUDE_DIRS}
)

aux_source_directory(src BASE_SRCS)

add_library(tool_manager SHARED
    ${BASE_SRCS}
)

target_link_libraries(tool_manager
    ${rtm_LIBRARIES}
)

install(FILES ./config/tool_manager.yaml 
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/component/
)

install(FILES config/tool_info.yaml 
    DESTINATION ${CONFIG_FILE_OUTPUT_PATH}/robot/tool/ 
)



