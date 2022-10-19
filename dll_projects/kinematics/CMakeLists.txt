cmake_minimum_required(VERSION 2.8.3)
project(kinematics_ssr)

include_directories(
    ../common/
    ./
    ./app/include/
    ./basic_alg/include/
    ./clapack/include/
    ./kinematics_alg/include/
    ./transformation/include/
)


aux_source_directory(./app/src/ SRC_APP)
aux_source_directory(./basic_alg/src/ SRC_BASIC_ALG)
aux_source_directory(./kinematics_alg/src/ SRC_KINE)
aux_source_directory(./transformation/src/ SRC_TRANS)

add_library(kinematics SHARED
	${SRC_APP}
	${SRC_BASIC_ALG}
	${SRC_KINE}
	${SRC_TRANS}
)

link_directories(
	${dll_projects_SOURCE_DIR}/kinematics/clapack/lib/
)

target_link_libraries(kinematics
    ${dll_projects_SOURCE_DIR}/kinematics/clapack/lib/libf2c.a
    ${dll_projects_SOURCE_DIR}/kinematics/clapack/lib/blas.a
    ${dll_projects_SOURCE_DIR}/kinematics/clapack/lib/lapack.a
)

set_target_properties(kinematics PROPERTIES OUTPUT_NAME "kinematics_ssr")


