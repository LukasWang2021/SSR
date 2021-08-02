cmake_minimum_required(VERSION 2.8.3)
project(tp_comm_test)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D PB_FIELD_32BIT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D PB_FIELD_32BIT")
SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/servo1001)

###################################################################################################servo1001
######### test_servo1001_servo_shutDown #########
add_executable(test_servo1001_servo_shutDown test_servo1001_servo_shutDown.cpp)
target_link_libraries(test_servo1001_servo_shutDown ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_switchOn #########
add_executable(test_servo1001_servo_switchOn test_servo1001_servo_switchOn.cpp)
target_link_libraries(test_servo1001_servo_switchOn ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_disableVoltage #########
add_executable(test_servo1001_servo_disableVoltage test_servo1001_servo_disableVoltage.cpp)
target_link_libraries(test_servo1001_servo_disableVoltage ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_enableOperation #########
add_executable(test_servo1001_servo_enableOperation test_servo1001_servo_enableOperation.cpp)
target_link_libraries(test_servo1001_servo_enableOperation ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_switchOnAndEnableOperation #########
add_executable(test_servo1001_servo_switchOnAndEnableOperation test_servo1001_servo_switchOnAndEnableOperation.cpp)
target_link_libraries(test_servo1001_servo_switchOnAndEnableOperation ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_disableOperation #########
add_executable(test_servo1001_servo_disableOperation test_servo1001_servo_disableOperation.cpp)
target_link_libraries(test_servo1001_servo_disableOperation ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_quickStop #########
add_executable(test_servo1001_servo_quickStop test_servo1001_servo_quickStop.cpp)
target_link_libraries(test_servo1001_servo_quickStop ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_resetFault #########
add_executable(test_servo1001_servo_resetFault test_servo1001_servo_resetFault.cpp)
target_link_libraries(test_servo1001_servo_resetFault ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_transCommState #########
add_executable(test_servo1001_servo_transCommState test_servo1001_servo_transCommState.cpp)
target_link_libraries(test_servo1001_servo_transCommState ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_readParameter #########
add_executable(test_servo1001_servo_readParameter test_servo1001_servo_readParameter.cpp)
target_link_libraries(test_servo1001_servo_readParameter ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_writeParameter #########
add_executable(test_servo1001_servo_writeParameter test_servo1001_servo_writeParameter.cpp)
target_link_libraries(test_servo1001_servo_writeParameter ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_moveVelocity #########
add_executable(test_servo1001_servo_moveVelocity test_servo1001_servo_moveVelocity.cpp)
target_link_libraries(test_servo1001_servo_moveVelocity ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_moveAbsolute #########
add_executable(test_servo1001_servo_moveAbsolute test_servo1001_servo_moveAbsolute.cpp)
target_link_libraries(test_servo1001_servo_moveAbsolute ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_triggerUploadParameters #########
add_executable(test_servo1001_servo_triggerUploadParameters test_servo1001_servo_triggerUploadParameters.cpp)
target_link_libraries(test_servo1001_servo_triggerUploadParameters ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_uploadParameters #########
add_executable(test_servo1001_servo_uploadParameters test_servo1001_servo_uploadParameters.cpp)
target_link_libraries(test_servo1001_servo_uploadParameters ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_triggerDownloadParameters #########
add_executable(test_servo1001_servo_triggerDownloadParameters test_servo1001_servo_triggerDownloadParameters.cpp)
target_link_libraries(test_servo1001_servo_triggerDownloadParameters ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_downloadParameters #########
add_executable(test_servo1001_servo_downloadParameters test_servo1001_servo_downloadParameters.cpp)
target_link_libraries(test_servo1001_servo_downloadParameters ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_isAsyncServiceFinish #########
add_executable(test_servo1001_servo_isAsyncServiceFinish test_servo1001_servo_isAsyncServiceFinish.cpp)
target_link_libraries(test_servo1001_servo_isAsyncServiceFinish ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_getCommState #########
add_executable(test_servo1001_servo_getCommState test_servo1001_servo_getCommState.cpp)
target_link_libraries(test_servo1001_servo_getCommState ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_getServoState #########
add_executable(test_servo1001_servo_getServoState test_servo1001_servo_getServoState.cpp)
target_link_libraries(test_servo1001_servo_getServoState ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_moveRelative #########
add_executable(test_servo1001_servo_moveRelative test_servo1001_servo_moveRelative.cpp)
target_link_libraries(test_servo1001_servo_moveRelative ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_resetEncoder #########
add_executable(test_servo1001_servo_resetEncoder test_servo1001_servo_resetEncoder.cpp)
target_link_libraries(test_servo1001_servo_resetEncoder ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_goHome #########
add_executable(test_servo1001_servo_goHome test_servo1001_servo_goHome.cpp)
target_link_libraries(test_servo1001_servo_goHome ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_abortHoming #########
add_executable(test_servo1001_servo_abortHoming test_servo1001_servo_abortHoming.cpp)
target_link_libraries(test_servo1001_servo_abortHoming ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_cpu_getVersion #########
add_executable(test_servo1001_cpu_getVersion test_servo1001_cpu_getVersion.cpp)
target_link_libraries(test_servo1001_cpu_getVersion ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_cpu_setCtrlPdoSync #########
add_executable(test_servo1001_cpu_setCtrlPdoSync test_servo1001_cpu_setCtrlPdoSync.cpp)
target_link_libraries(test_servo1001_cpu_setCtrlPdoSync ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_cpu_getCtrlPdoSync #########
add_executable(test_servo1001_cpu_getCtrlPdoSync test_servo1001_cpu_getCtrlPdoSync.cpp)
target_link_libraries(test_servo1001_cpu_getCtrlPdoSync ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_cpu_setSamplingSync #########
add_executable(test_servo1001_cpu_setSamplingSync test_servo1001_cpu_setSamplingSync.cpp)
target_link_libraries(test_servo1001_cpu_setSamplingSync ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_cpu_getSamplingSync #########
add_executable(test_servo1001_cpu_getSamplingSync test_servo1001_cpu_getSamplingSync.cpp)
target_link_libraries(test_servo1001_cpu_getSamplingSync ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_cpu_setSamplingInterval #########
add_executable(test_servo1001_cpu_setSamplingInterval test_servo1001_cpu_setSamplingInterval.cpp)
target_link_libraries(test_servo1001_cpu_setSamplingInterval ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_cpu_getSamplingInterval #########
add_executable(test_servo1001_cpu_getSamplingInterval test_servo1001_cpu_getSamplingInterval.cpp)
target_link_libraries(test_servo1001_cpu_getSamplingInterval ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_cpu_setSamplingMaxTimes #########
add_executable(test_servo1001_cpu_setSamplingMaxTimes test_servo1001_cpu_setSamplingMaxTimes.cpp)
target_link_libraries(test_servo1001_cpu_setSamplingMaxTimes ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_cpu_getSamplingMaxTimes #########
add_executable(test_servo1001_cpu_getSamplingMaxTimes test_servo1001_cpu_getSamplingMaxTimes.cpp)
target_link_libraries(test_servo1001_cpu_getSamplingMaxTimes ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_cpu_setSamplingChannel #########
add_executable(test_servo1001_cpu_setSamplingChannel test_servo1001_cpu_setSamplingChannel.cpp)
target_link_libraries(test_servo1001_cpu_setSamplingChannel ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_cpu_getSamplingChannel #########
add_executable(test_servo1001_cpu_getSamplingChannel test_servo1001_cpu_getSamplingChannel.cpp)
target_link_libraries(test_servo1001_cpu_getSamplingChannel ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_cpu_activateSamplingConfiguration #########
add_executable(test_servo1001_cpu_activateSamplingConfiguration test_servo1001_cpu_activateSamplingConfiguration.cpp)
target_link_libraries(test_servo1001_cpu_activateSamplingConfiguration ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_cpu_saveSamplingBufferData #########
add_executable(test_servo1001_cpu_saveSamplingBufferData test_servo1001_cpu_saveSamplingBufferData.cpp)
target_link_libraries(test_servo1001_cpu_saveSamplingBufferData ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_getServoCommInfo #########
add_executable(test_servo1001_servo_getServoCommInfo test_servo1001_servo_getServoCommInfo.cpp)
target_link_libraries(test_servo1001_servo_getServoCommInfo ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_cpu_getServoCpuCommInfo #########
add_executable(test_servo1001_cpu_getServoCpuCommInfo test_servo1001_cpu_getServoCpuCommInfo.cpp)
target_link_libraries(test_servo1001_cpu_getServoCpuCommInfo ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_servo_getServoDefinedInfo #########
add_executable(test_servo1001_servo_getServoDefinedInfo test_servo1001_servo_getServoDefinedInfo.cpp)
target_link_libraries(test_servo1001_servo_getServoDefinedInfo ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_cpu_setForceControlParam #########
add_executable(test_servo1001_cpu_setForceControlParam test_servo1001_cpu_setForceControlParam.cpp)
target_link_libraries(test_servo1001_cpu_setForceControlParam ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo1001_cpu_getForceControlParam #########
add_executable(test_servo1001_cpu_getForceControlParam test_servo1001_cpu_getForceControlParam.cpp)
target_link_libraries(test_servo1001_cpu_getForceControlParam ${rtm_LIBRARIES} pb tp_comm_test protos)
