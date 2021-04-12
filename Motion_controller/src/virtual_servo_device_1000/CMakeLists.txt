cmake_minimum_required(VERSION 2.8.3)
project(virtual_servo_device_1000)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-rpath=/usr/lib/aarch64-linux-gnu")

include_directories(
        ./include
        ../core_comm/include
        ../xml_help/include
        ../yaml_help/include
        ../thread_help/include
        ../servo_comm/include
        ../common
        ${rtm_INCLUDE_DIRS}
)

aux_source_directory(./src SRC)
add_executable(virtual_servo_device_1000
	${SRC}
)

link_directories(
	../../install/lib
)
target_link_libraries(virtual_servo_device_1000
	${rtm_LIBRARIES}
	core_comm_bare
	servo_comm
	thread_help
	yaml_help
)

