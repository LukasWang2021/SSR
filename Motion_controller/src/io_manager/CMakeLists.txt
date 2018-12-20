cmake_minimum_required(VERSION 2.8.3)
project(io_manager)

find_package(catkin REQUIRED COMPONENTS
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
    ${depend_PATH_INC}/common
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES io_manager
    DEPENDS device_manager
    DEPENDS parameter_manager
    DEPENDS error_monitor
    DEPENDS xml_help
    DEPENDS fst_io_device
    DEPENDS thread_help
    DEPENDS modbus_manager
    DEPENDS virtual_io_device
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}  
    ${depend_PATH_INC}/base_device/include
    ${depend_PATH_INC}/common
    ${depend_PATH_INC}/device_manager/include
    ${depend_PATH_INC}/parameter_manager/include
    ${depend_PATH_INC}/log_manager/include
    ${depend_PATH_INC}/error_monitor/include
    ${depend_PATH_INC}/xml_help/include
    ${depend_PATH_INC}/fst_io_device/include
    ${depend_PATH_INC}/virtual_io_device/include
    ${depend_PATH_INC}/thread_help/include/
    ${depend_PATH_INC}/modbus_manager/include
    ${depend_PATH_INC}/protos/include
)

aux_source_directory(src BASE_SRCS)

add_library(io_manager
        ${BASE_SRCS}
        )

target_link_libraries(io_manager
        ${catkin_LIBRARIES}
        parameter_manager
        device_manager
        error_monitor
        xml_help
        fst_io_device
        virtual_io_device
        thread_help
        modbus_manager
        virtual_io_device
        )

add_executable(io_test
    test/io_test.cpp
)

target_link_libraries(io_test
    ${catkin_LIBRARIES}
    io_manager
    device_manager
)

install(TARGETS io_manager
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(TARGETS	io_test
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES config/io_manager.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/component_param/
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
