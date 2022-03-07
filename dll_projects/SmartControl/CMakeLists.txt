cmake_minimum_required(VERSION 2.8.3)
project(SmartControl)

include_directories(
	./Comm/
	./Comm/app/
	./Comm/basic_alg/
    ./Comm/nanomsg/
    ./Comm/nanopb/
	./Comm/protos/include/
    ./Comm/protos/include/event/
    ./Comm/protos/include/message_type/
    ./Comm/protos/include/protocol/
    ./Comm/protos/include/request/
    ./Comm/protos/include/response/
)

aux_source_directory(./Comm/app/ SRC_APP)
aux_source_directory(./Comm/basic_alg/ SRC_BASIC_ALG)
aux_source_directory(./Comm/nanomsg/ SRC_NANOMSG)
aux_source_directory(./Comm/nanopb/ SRC_NANOPB)
aux_source_directory(./Comm/protos/src/event/ SRC_PROTO_EVENT)
aux_source_directory(./Comm/protos/src/message_type/ SRC_PROTO_MSG)
aux_source_directory(./Comm/protos/src/protocol/ SRC_PROTO_PROTO)
aux_source_directory(./Comm/protos/src/request/ SRC_PROTO_REQ)
aux_source_directory(./Comm/protos/src/response/ SRC_PROTO_REP)

add_library(SmartControl SHARED
	${SRC_APP}
	${SRC_BASIC_ALG}
	${SRC_NANOMSG}
	${SRC_NANOPB}
	${SRC_PROTO_EVENT}
	${SRC_PROTO_MSG}
	${SRC_PROTO_PROTO}
	${SRC_PROTO_REQ}
	${SRC_PROTO_REP}
)

link_directories(
	./Comm/nanomsg
)

target_link_libraries(SmartControl
    libnanomsg.so
)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -O0 -Wno-unused-but-set-variable -Wno-unused-local-typedefs -fpermissive -lrt")

set_target_properties(SmartControl PROPERTIES OUTPUT_NAME "smart_control")
set (LIBRARY_OUTPUT_PATH ../lib_so)


