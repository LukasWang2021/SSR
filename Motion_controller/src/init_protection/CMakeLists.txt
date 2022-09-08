cmake_minimum_required(VERSION 2.8.3)
project(init_protection)

include_directories(
	include
	${rtm_INCLUDE_DIRS}
)

aux_source_directory(src SRC)

add_library(init_protection SHARED
    ${SRC}
)

link_directories(
	../../install/lib
)

# target_link_libraries(init_protection
#     ${rtm_LIBRARIES}
#     controller
#     nvram_server
#     log_manager_consumer
# )