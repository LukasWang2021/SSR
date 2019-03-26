cmake_minimum_required(VERSION 2.8.3)
project(program_launching)

find_package(catkin REQUIRED COMPONENTS
        base_device
        io_manager
        parameter_manager
        log_manager
        error_monitor
        io_mapping
        process_comm
        modbus_manager
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

set(catkin_INCLUDE_DIRS
    ${catkin_INCLUDE_DIRS}
    ${depend_PATH_INC}/base_device/include
    ${depend_PATH_INC}/io_manager/include
    ${depend_PATH_INC}/common
    ${depend_PATH_INC}/parameter_manager/include
    ${depend_PATH_INC}/log_manager/include
    ${depend_PATH_INC}/xml_help/include 
    ${depend_PATH_INC}/error_monitor/include
    ${depend_PATH_INC}/io_mapping/include
    ${depend_PATH_INC}/process_comm/include
    ${depend_PATH_INC}/prg_interpreter/include
    ${depend_PATH_INC}/motion_control/include
    ${depend_PATH_INC}/thread_help/include
    ${depend_PATH_INC}/reg_manager/include
    ${depend_PATH_INC}/modbus_manager/include
    ${depend_PATH_INC}/fst_io_device/include
    ${depend_PATH_INC}/virtual_io_device/include
    ${depend_PATH_INC}/device_manager/include
    ${depend_PATH_INC}/protos/include
    ${depend_PATH_INC}/basic_alg/include
    ${depend_PATH_INC}/nvram/include
    ${depend_PATH_INC}/fst_spi/include
    )

catkin_package(
    INCLUDE_DIRS include
    #LIBRARIES fst_io_device
    DEPENDS base_device
    DEPENDS io_manager
    DEPENDS parameter_manager
    DEPENDS log_manager
    DEPENDS thread_help 
    DEPENDS error_monitor
    DEPENDS io_mapping
    DEPENDS process_comm
    DEPENDS modbus_manager
    DEPENDS fst_io_device
    DEPENDS basic_alg
    DEPENDS nvram 
    DEPENDS fst_spi 
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

aux_source_directory(src BASE_SRCS)

add_library(program_launching
        ${BASE_SRCS}
        )

target_link_libraries(program_launching
        ${catkin_LIBRARIES}
        base_device
        fst_io_device
        error_monitor
        logger
        thread_help
        parameter_manager
        io_mapping
        process_comm
        modbus_manager
        io_manager
        basic_alg
        )

add_executable(macro_test
    test/macro_test.cpp
)

target_link_libraries(macro_test
    ${catkin_LIBRARIES}
    program_launching
    io_mapping
    process_comm
    base_device
    io_manager
)

install(TARGETS program_launching
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(TARGETS	macro_test
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES config/program_launching.yaml
        DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/component_param/
        )

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
