cmake_minimum_required(VERSION 2.8.3)
project(controller_ssr)

include_directories(
    ../common/
    ./
    ./app/
    ./basic_alg/
    ./nanomsg/
    ./nanopb/
    ./protos/include/
    ./protos/include/event/
    ./protos/include/message_type/
    ./protos/include/protocol/
    ./protos/include/request/
    ./protos/include/response/
)

aux_source_directory(./app/ SRC_APP)
aux_source_directory(./basic_alg/ SRC_BASIC_ALG)
aux_source_directory(./nanomsg/ SRC_NANOMSG)
aux_source_directory(./nanopb/ SRC_NANOPB)
aux_source_directory(./protos/src/event/ SRC_PROTO_EVENT)
aux_source_directory(./protos/src/message_type/ SRC_PROTO_MSG)
aux_source_directory(./protos/src/protocol/ SRC_PROTO_PROTO)
aux_source_directory(./protos/src/request/ SRC_PROTO_REQ)
aux_source_directory(./protos/src/response/ SRC_PROTO_REP)

add_library(controller SHARED
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
	./nanomsg
)

target_link_libraries(controller
    libnanomsg.so
)

set_target_properties(controller PROPERTIES OUTPUT_NAME "controller_ssr")






