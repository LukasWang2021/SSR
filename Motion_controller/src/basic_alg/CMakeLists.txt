cmake_minimum_required(VERSION 2.8.3)
project(basic_alg)

include_directories(
	include
	${rtm_INCLUDE_DIRS}
	${CROSS_ROOT}/usr/local/include/clapack
)

aux_source_directory(src SRC)
add_library(basic_alg SHARED
	${SRC}
)

target_link_libraries(basic_alg
	${rtm_LIBRARIES}
	${USR_LOCAL_LIB_DIR}/libf2c.so
	${USR_LOCAL_LIB_DIR}/libblas.so
	${USR_LOCAL_LIB_DIR}/liblapack.so
)


