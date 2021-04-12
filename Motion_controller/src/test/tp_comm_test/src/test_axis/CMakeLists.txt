cmake_minimum_required(VERSION 2.8.3)
project(tp_comm_test)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D PB_FIELD_32BIT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D PB_FIELD_32BIT")
SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/axis)


#############################################################################################################axis
######### test_axis_mcPower #########
add_executable(test_axis_mcPower  test_axis_mcPower.cpp)
target_link_libraries(test_axis_mcPower ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_axis_mcReset #########
add_executable(test_axis_mcReset  test_axis_mcReset.cpp)
target_link_libraries(test_axis_mcReset ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_axis_mcStop #########
add_executable(test_axis_mcStop  test_axis_mcStop.cpp)
target_link_libraries(test_axis_mcStop ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_axis_mcHalt #########
add_executable(test_axis_mcHalt  test_axis_mcHalt.cpp)
target_link_libraries(test_axis_mcHalt ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_axis_mcSetPosition #########
add_executable(test_axis_mcSetPosition  test_axis_mcSetPosition.cpp)
target_link_libraries(test_axis_mcSetPosition ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_axis_mcReadParameter #########
add_executable(test_axis_mcReadParameter  test_axis_mcReadParameter.cpp)
target_link_libraries(test_axis_mcReadParameter ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_axis_mcWriteParameter #########
add_executable(test_axis_mcWriteParameter  test_axis_mcWriteParameter.cpp)
target_link_libraries(test_axis_mcWriteParameter ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_axis_mcMoveAbsolute #########
add_executable(test_axis_mcMoveAbsolute  test_axis_mcMoveAbsolute.cpp)
target_link_libraries(test_axis_mcMoveAbsolute ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_axis_mcMoveVelocity #########
add_executable(test_axis_mcMoveVelocity  test_axis_mcMoveVelocity.cpp)
target_link_libraries(test_axis_mcMoveVelocity ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_axis_mcReadActualPosition #########
add_executable(test_axis_mcReadActualPosition  test_axis_mcReadActualPosition.cpp)
target_link_libraries(test_axis_mcReadActualPosition ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_axis_mcReadActualVelocity #########
add_executable(test_axis_mcReadActualVelocity  test_axis_mcReadActualVelocity.cpp)
target_link_libraries(test_axis_mcReadActualVelocity ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_axis_mcReadActualTorque #########
add_executable(test_axis_mcReadActualTorque  test_axis_mcReadActualTorque.cpp)
target_link_libraries(test_axis_mcReadActualTorque ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_axis_mcReadAxisInfo #########
add_executable(test_axis_mcReadAxisInfo  test_axis_mcReadAxisInfo.cpp)
target_link_libraries(test_axis_mcReadAxisInfo ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_axis_mcReadStatus #########
add_executable(test_axis_mcReadStatus  test_axis_mcReadStatus.cpp)
target_link_libraries(test_axis_mcReadStatus ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_axis_mcReadAxisError #########
add_executable(test_axis_mcReadAxisError  test_axis_mcReadAxisError.cpp)
target_link_libraries(test_axis_mcReadAxisError ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_axis_mcReadAxisErrorHistory #########
add_executable(test_axis_mcReadAxisErrorHistory  test_axis_mcReadAxisErrorHistory.cpp)
target_link_libraries(test_axis_mcReadAxisErrorHistory ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_axis_mcMoveRelative #########
add_executable(test_axis_mcMoveRelative  test_axis_mcMoveRelative.cpp)
target_link_libraries(test_axis_mcMoveRelative ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_axis_mcHome #########
add_executable(test_axis_mcHome  test_axis_mcHome.cpp)
target_link_libraries(test_axis_mcHome ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_axis_rtmAbortHoming #########
add_executable(test_axis_rtmAbortHoming  test_axis_rtmAbortHoming.cpp)
target_link_libraries(test_axis_rtmAbortHoming ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_axis_rtmReadAxisFdbPdoPtr #########
add_executable(test_axis_rtmReadAxisFdbPdoPtr  test_axis_rtmReadAxisFdbPdoPtr.cpp)
target_link_libraries(test_axis_rtmReadAxisFdbPdoPtr ${rtm_LIBRARIES} pb tp_comm_test protos)
