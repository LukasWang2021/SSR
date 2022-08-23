cmake_minimum_required(VERSION 2.8.3)
project(foc_device)

include_directories(
	include
	../common
	../base_device/include/
	./foc_device/include/
	${rtm_INCLUDE_DIRS}
)


aux_source_directory(src SRC)
add_library(foc_device SHARED
        ${SRC}
)

target_link_libraries(foc_device
        ${rtm_LIBRARIES}
)






