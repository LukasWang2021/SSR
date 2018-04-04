cmake_minimum_required(VERSION 2.8.3)
project(prg_interpreter)

include_directories(${PROJECT_BINARY_DIR})

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  io_manager
)
 
set(CONFIG_DIR config)
#execute_process(COMMAND rm -r ${CMAKE_HOME_DIRECTORY}/../devel/${CONFIG_DIR}
#		COMMAND cp -r ${CMAKE_CURRENT_SOURCE_DIR}/${CONFIG_DIR} ${CMAKE_HOME_D#IRECTORY}/../devel)

if(${catkin_CROSS})
        set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS} ${depend_PATH_INC}/motion_plan/include ${depend_PATH_INC}/middleware_to_mem/include  ${depend_PATH_INC}/parameter_manager/include ${depend_PATH_INC}/safety/include ${depend_PATH_INC}/log_manager/include ${depend_PATH_INC}/service_manager/include ${depend_PATH_INC}/io_manager/include ${depend_PATH_INC}/ioboard/include ${depend_PATH_INC}/mem_share/include)
        set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS} ${depend_PATH_LIB}/libmotion_plan.so ${depend_PATH_LIB}/libcomm_interface.so ${depend_PATH_LIB}/libcore_interface.so  ${depend_PATH_LIB}/libparameter_manager.so ${depend_PATH_LIB}/libsafety.so ${depend_PATH_LIB}/libio_manager.so ${depend_PATH_LIB}/liblogger.so ${depend_PATH_LIB}/libmem_share.so)
        set(CMAKE_CXX_FLAGS "-DCROSS_PLATFORM")
        set(CMAKE_C_FLAGS "-DCROSS_PLATFORM")
else(${catkin_CROSS})
    set(NANOPB_DIR ~/my_work/nanopb)
    set(EXT_LIB nanomsg profiler pthread)
    include_directories(/home/fst/cross_workspace/src/service_manager/include )
endif(${catkin_CROSS})


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  DEPENDS io_manager
)
 
###########
## Build ##
###########
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT -lpthread -lxml2 -I/usr/local/roscrosstool/junlong/usr/include/libxml2")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -DPB_FIELD_32BIT")

#message(${CMAKE_CXX_FLAGS})
include_directories(include include/tp_interface include/proto include/common ${catkin_INCLUDE_DIRS})

#aux_source_directory(src/reg-shmi REG_SHMI)
aux_source_directory(src BASE_SRCS)

add_executable(prg_interpreter
              src/reg-shmi/forsight_op_reg_shmi.cpp
              src/reg-shmi/forsight_registers.cpp
              src/reg-shmi/forsight_op_shmi.cpp
              src/reg-shmi/forsight_peterson.cpp
              src/forsight_interpreter_shm.cpp
              src/forsight_innercmd.cpp
              src/forsight_xml_reader.cpp
              src/forsight_int_main.cpp
              src/forsight_basint.cpp
              src/forsight_innerfunc.cpp
              src/base_thread.cpp
              src/io_interface.cpp
              src/forsight_inter_control.cpp
               src/proto/base_types.pb.cpp
               src/proto/motionSL.pb.cpp
               src/proto/proto_parse.cpp
               ${NANOPB_DIR}/pb_decode.c  ${NANOPB_DIR}/pb_encode.c ${NANOPB_DIR}/pb_common.c )

## Specify libraries to link a library or executable target against
target_link_libraries(prg_interpreter
  ${catkin_LIBRARIES}
  io_manager
)

#############
## Install ##
#############

install(TARGETS prg_interpreter
   RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


install(DIRECTORY ${CONFIG_DIR}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#############
#Config File#
#############
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
