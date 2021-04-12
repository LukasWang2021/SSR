cmake_minimum_required(VERSION 2.8.3)
project(tp_comm_test)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D PB_FIELD_32BIT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D PB_FIELD_32BIT")
SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/servo_sampling)

##################################################################################################servo_sampling
######### test_servo_sampling_setSamplingConfiguration #########
add_executable(test_servo_sampling_setSamplingConfiguration   test_servo_sampling_setSamplingConfiguration.cpp)
target_link_libraries(test_servo_sampling_setSamplingConfiguration ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo_sampling_getSamplingConfiguration #########
add_executable(test_servo_sampling_getSamplingConfiguration   test_servo_sampling_getSamplingConfiguration.cpp)
target_link_libraries(test_servo_sampling_getSamplingConfiguration ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo_sampling_activateSamplingConfiguration #########
add_executable(test_servo_sampling_activateSamplingConfiguration   test_servo_sampling_activateSamplingConfiguration.cpp)
target_link_libraries(test_servo_sampling_activateSamplingConfiguration ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo_sampling_setSamplingSync #########
add_executable(test_servo_sampling_setSamplingSync   test_servo_sampling_setSamplingSync.cpp)
target_link_libraries(test_servo_sampling_setSamplingSync ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo_sampling_getSamplingSync #########
add_executable(test_servo_sampling_getSamplingSync   test_servo_sampling_getSamplingSync.cpp)
target_link_libraries(test_servo_sampling_getSamplingSync ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo_sampling_setSamplingChannel #########
add_executable(test_servo_sampling_setSamplingChannel   test_servo_sampling_setSamplingChannel.cpp)
target_link_libraries(test_servo_sampling_setSamplingChannel ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo_sampling_getSamplingChannel #########
add_executable(test_servo_sampling_getSamplingChannel   test_servo_sampling_getSamplingChannel.cpp)
target_link_libraries(test_servo_sampling_getSamplingChannel ${rtm_LIBRARIES} pb tp_comm_test protos)

######### test_servo_sampling_saveSamplingBufferData #########
add_executable(test_servo_sampling_saveSamplingBufferData   test_servo_sampling_saveSamplingBufferData.cpp)
target_link_libraries(test_servo_sampling_saveSamplingBufferData ${rtm_LIBRARIES} pb tp_comm_test protos)
