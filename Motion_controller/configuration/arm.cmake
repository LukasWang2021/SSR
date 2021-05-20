set(PLATFORM zcu102)
set(CROSS_ROOT /usr/local/crosstool/${PLATFORM})
set(GCC_PATH_ROOT /usr/local/aarch64-linux-gnu)

set(USR_LIB_DIR ${CROSS_ROOT}/usr/lib/aarch64-linux-gnu)
set(USR_LOCAL_LIB_DIR ${CROSS_ROOT}/usr/local/lib)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_C_COMPILER ${GCC_PATH_ROOT}/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${GCC_PATH_ROOT}/bin/aarch64-linux-gnu-g++)
set(CMAKE_FIND_ROOT_PATH 
    ${GCC_PATH_ROOT}
	${CROSS_ROOT} 
)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(rtm_INCLUDE_DIRS
    ${CROSS_ROOT}/usr/include
	${CROSS_ROOT}/usr/local/include 
	${CROSS_ROOT}/usr/include/libxml2
	${CROSS_ROOT}/usr/include/aarch64-linux-gnu
)

set(rtm_LIBRARY_DIRS 
	${USR_LIB_DIR}
)

set(CMAKE_LIBRARY_PATH
	${rtm_LIBRARY_DIRS}
)

set(rtm_LIBRARIES 
    ${USR_LIB_DIR}/libstdc++.so.6
	${USR_LIB_DIR}/libpthread.so
	${USR_LIB_DIR}/librt.so
	${USR_LIB_DIR}/libz.so
    ${USR_LIB_DIR}/liblzma.so
	${USR_LIB_DIR}/libxml2.so
	${USR_LIB_DIR}/libyaml-cpp.so
    ${USR_LOCAL_LIB_DIR}/libnanomsg.so
    ${USR_LIB_DIR}/libicuuc.so
    ${USR_LIB_DIR}/libicudata.so
    ${USR_LIB_DIR}/libdl.so
)


