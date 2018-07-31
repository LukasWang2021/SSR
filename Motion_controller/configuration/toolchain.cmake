set(PLATFORM junlong)
set(GCCPATH "/usr/local/gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux")
set(GCCPREFIX "arm-linux-gnueabihf-")
set(CROSS_ROOT /usr/local/roscrosstool/${PLATFORM})
set(ROS_INSTALL_ROOT ${CROSS_ROOT}/opt/ros/indigo)
set(catkin_LIBRARY_DIRS ${ROS_INSTALL_ROOT}/lib)
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_C_COMPILER ${GCCPATH}/bin/${GCCPREFIX}gcc)
set(CMAKE_CXX_COMPILER ${GCCPATH}/bin/${GCCPREFIX}g++)
set(CMAKE_FIND_ROOT_PATH ${GCCPATH} ${CROSS_ROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(NANOPB_DIR ${CROSS_ROOT}/nanopb)

set(catkin_LIBRARYS_CROSS 
        ${ROS_INSTALL_ROOT}/lib/libroscpp.so 
		${ROS_INSTALL_ROOT}/lib/librosconsole.so  
		${ROS_INSTALL_ROOT}/lib/libroscpp_serialization.so
		${ROS_INSTALL_ROOT}/lib/librosconsole_log4cxx.so
		${ROS_INSTALL_ROOT}/lib/librostime.so
        ${ROS_INSTALL_ROOT}/lib/libxmlrpcpp.so
		${CROSS_ROOT}/usr/lib/arm-linux-gnueabihf/liblzma.so
		${CROSS_ROOT}/usr/lib/arm-linux-gnueabihf/libpthread.so
		${CROSS_ROOT}/usr/lib/arm-linux-gnueabihf/libstdc++.so.6
		${CROSS_ROOT}/usr/lib/arm-linux-gnueabihf/libboost_signals.so
		${CROSS_ROOT}/usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
		${CROSS_ROOT}/usr/lib/arm-linux-gnueabihf/libboost_system.so
		${CROSS_ROOT}/usr/lib/arm-linux-gnueabihf/libboost_thread.so
		${CROSS_ROOT}/usr/lib/arm-linux-gnueabihf/librt.so
		${CROSS_ROOT}/usr/lib/liblog4cxx.so		
		${CROSS_ROOT}/usr/lib/arm-linux-gnueabihf/libapr-1.so.0
		${CROSS_ROOT}/usr/lib/arm-linux-gnueabihf/libuuid.so
		${CROSS_ROOT}/usr/lib/arm-linux-gnueabihf/libexpat.so
		${CROSS_ROOT}/usr/lib/arm-linux-gnueabihf/libz.so
		${CROSS_ROOT}/usr/lib/arm-linux-gnueabihf/libxml2.so
		${CROSS_ROOT}/usr/local/lib/arm-linux-gnueabihf/libnanomsg.so
		)

set(catkin_INCLUDE_DIRS_CROSS
        ${CROSS_ROOT}/usr/include
        ${CROSS_ROOT}/usr/local/include
		${ROS_INSTALL_ROOT}/include
        ${CROSS_ROOT}/usr/include/libxml2
        ${NANOPB_DIR}
)

option(catkin_CROSS "cross compile" ON)

set(depend_PATH_INC ${CMAKE_HOME_DIRECTORY})
set(depend_PATH_LIB ${CMAKE_HOME_DIRECTORY}/../devel/lib)

