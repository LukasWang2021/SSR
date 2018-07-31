set(PLATFORM x86_64-linux-gnu)
set(ROS_INSTALL_ROOT /opt/ros/indigo)
set(catkin_LIBRARY_DIRS ${ROS_INSTALL_ROOT}/lib)

set(LIB_DIR /usr/lib/)
set(NANOMSG_LIB_DIR /usr/local/lib/${PLATFORM})
set(NANOPB_DIR /usr/local/lib/${PLATFORM}/nanopb)

set(catkin_LIBRARYS_x86
    ${ROS_INSTALL_ROOT}/lib/libroscpp.so 
	${ROS_INSTALL_ROOT}/lib/librosconsole.so  
	${ROS_INSTALL_ROOT}/lib/libroscpp_serialization.so
	${ROS_INSTALL_ROOT}/lib/librosconsole_log4cxx.so
	${ROS_INSTALL_ROOT}/lib/librostime.so
    ${ROS_INSTALL_ROOT}/lib/libxmlrpcpp.so
    ${LIB_DIR}/${PLATFORM}/libpthread.so
    ${LIB_DIR}/${PLATFORM}/libstdc++.so.6
    ${LIB_DIR}/${PLATFORM}/libboost_signals.so
    ${LIB_DIR}/${PLATFORM}/libboost_filesystem.so
    ${LIB_DIR}/${PLATFORM}/libboost_system.so
    ${LIB_DIR}/${PLATFORM}/libboost_thread.so
    ${LIB_DIR}/${PLATFORM}/librt.so
    ${LIB_DIR}/${PLATFORM}/libapr-1.so
    ${LIB_DIR}/${PLATFORM}/libuuid.so
    ${LIB_DIR}/${PLATFORM}/libexpat.so
    ${LIB_DIR}/${PLATFORM}/libz.so
    ${LIB_DIR}/${PLATFORM}/libxml2.so
    ${LIB_DIR}/liblog4cxx.so
    ${NANOMSG_LIB_DIR}/libnanomsg.so
)

set(catkin_INCLUDE_DIRS_x86
    /usr/include
    /usr/local/include
    /opt/ros/indigo/include/
    /usr/include/libxml2
    ${NANOPB_DIR}
)

option(catkin_CROSS "x86 compile" OFF)

set(depend_PATH_INC ${CMAKE_HOME_DIRECTORY})
set(depend_PATH_LIB ${CMAKE_HOME_DIRECTORY}/../devel/lib)


