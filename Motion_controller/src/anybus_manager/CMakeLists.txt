cmake_minimum_required(VERSION 2.8.3)
project(anybus_manager)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    log_manager
    parameter_manager
    error_monitor
    thread_help
    xml_help
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

catkin_package( 
    INCLUDE_DIRS include
    LIBRARIES anybus_manager
    DEPENDS log_manager
    DEPENDS xml_help
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -fpermissive -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -D PB_FIELD_32BIT -D_GLIBCXX_USE_CXX11_ABI=0")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -D PB_FIELD_32BIT")

set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
    ${depend_PATH_INC}/log_manager/include/
    ${depend_PATH_INC}/log_service/include/
    ${depend_PATH_INC}/thread_help/include/
    ${depend_PATH_INC}/error_monitor/include/
    ${depend_PATH_INC}/parameter_manager/include/
    ${depend_PATH_INC}/xml_help/include/
    ${depend_PATH_INC}/common/
)

include_directories(
    include/
    include/anybus/
    include/anybus/abcc_abp/
    include/anybus/abcc_adapt/
    include/anybus/abcc_drv/inc/
    include/anybus/abcc_drv/src/
    include/anybus/abcc_drv/src/par/
    include/anybus/abcc_drv/src/par30/
    include/anybus/abcc_drv/src/serial/
    include/anybus/abcc_drv/src/spi/
    include/anybus/abcc_obj/
    include/anybus/abcc_obj/nw_obj/
    include/anybus/example_app/
    ${catkin_INCLUDE_DIRS}
    /usr/include/python2.7
)

aux_source_directory(src/ BASE_SRCS)
aux_source_directory(src/anybus/abcc_adapt/ ABCC_ADAPT_SRCS)
aux_source_directory(src/anybus/abcc_drv/src ABCC_DRV_SRCS)
aux_source_directory(src/anybus/abcc_drv/src/par/ ABCC_DRV_PAR_SRCS)
aux_source_directory(src/anybus/abcc_drv/src/par30/ ABCC_DRV_PAR30_SRCS)
aux_source_directory(src/anybus/abcc_drv/src/serial/ ABCC_DRV_SERIAL_SRCS)
aux_source_directory(src/anybus/abcc_drv/src/spi/ ABCC_DRV_SPI_SRCS)
aux_source_directory(src/anybus/abcc_obj/ ABCC_OBJ_SRCS)
aux_source_directory(src/anybus/abcc_obj/nw_obj/ ABCC_NW_OBJ_SRCS)
aux_source_directory(src/anybus/example_app/ EXAMPLE_APP_SRCS)

add_library(anybus_manager
    ${BASE_SRCS}
    ${ABCC_ADAPT_SRCS}
    ${ABCC_DRV_SRCS}
    ${ABCC_DRV_PAR_SRCS}
    ${ABCC_DRV_PAR30_SRCS}
    ${ABCC_DRV_SERIAL_SRCS}
    ${ABCC_DRV_SPI_SRCS}
    ${ABCC_OBJ_SRCS}
    ${ABCC_NW_OBJ_SRCS}
    ${EXAMPLE_APP_SRCS}
)

target_link_libraries(anybus_manager
    ${catkin_LIBRARIES}
    parameter_manager
    thread_help
    xml_help
    xml2
  )

install(TARGETS anybus_manager
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 )

install(FILES config/anybus_manager.yaml
    DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/runtime/component_param/
)


install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(test_anybus
    ${BASE_SRCS}
    ${ABCC_ADAPT_SRCS}
    ${ABCC_DRV_SRCS}
    ${ABCC_DRV_PAR_SRCS}
    ${ABCC_DRV_PAR30_SRCS}
    ${ABCC_DRV_SERIAL_SRCS}
    ${ABCC_DRV_SPI_SRCS}
    ${ABCC_OBJ_SRCS}
    ${ABCC_NW_OBJ_SRCS}
    ${EXAMPLE_APP_SRCS}
    src/test_ethercat.cpp
)

target_link_libraries(test_anybus 
    ${catkin_LIBRARIES}
    thread_help
    parameter_manager
    logger
    xml_help
)

install(TARGETS test_anybus
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/test)
