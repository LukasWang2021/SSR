set(COMPILE_OPTION COMPILE_IN_X86)
set(PLATFORM x86_64-linux-gnu)
set(LIB_DIR /usr/lib/${PLATFORM})
set(LIB_LOCAL_DIR /usr/local/lib/${PLATFORM})


set(rtm_INCLUDE_DIRS
	/usr/include
	/usr/include/libxml2
)

set(rtm_LIBRARY_DIRS 
	${LIB_DIR}	
)

set(rtm_LIBRARIES
	${LIB_DIR}/libpthread.so
	${LIB_DIR}/libstdc++.so.6
	${LIB_DIR}/librt.so
	${LIB_DIR}/libz.so
	${LIB_DIR}/libxml2.so
	${LIB_DIR}/libyaml-cpp.so
	${LIB_LOCAL_DIR}/libnanomsg.so
)

