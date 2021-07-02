#pragma once
#include "comm_def.h"
#include "protoc.h"

extern bool rpc_valid;

#ifdef __cplusplus
extern "C"
{
#endif
/**
 * @brief Initialize the client of the requset_response mode.
 * @details Initialize the socket.\n
 * @param [in] server_ip the IP of the server.
 * @retval 0 Success.
 * @retval -1 Failed to initialize.
 */
COMM_INTERFACE_API uint64_t c_initRpc(char* server_ip);

//rpc_subcribe
/**
 * @brief Subscribe the topics from Controller.
 * @details 
 * @return error_code.
 */
COMM_INTERFACE_API uint64_t c_addTopic();

/**
 * @brief Unsubscribe the topics from Controller.
 * @details
 * @return error_code.
 */
COMM_INTERFACE_API uint64_t c_deleteTopic();

//rpc_controller
/**
 * @brief Get the versions of Controller and Servo.
 * @details
 * @param [out] version Major and minor verson.
 * - version[0]: major version of Controller,
 * - version[1]: minor version of Controller,
 * - version[2]: major version of Servo,
 * - version[3]: minor version of Servo,
 * @param [out] size The valid data number in version[4] array.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_getVersion(uint32_t version[4], int32_t* size);

/**
 * @brief Get the system time of Controller.
 * @details
 * @param [out] time The second of the system time.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_getSystemTime(uint64_t* time);


/**
 * @brief Set the working mode of Controller.
 * @details 1-auto, 2-manaul_slow,3-manual
 * @param [in] mode The mode of the controller.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_setWorkMode(uint32_t mode);

/**
 * @brief Get the working mode of Controller.
 * @details 1-auto, 2-manaul_slow,3-manual
 * @param [out] mode The mode of the controller.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_getWorkMode(uint32_t* mode);

/**
 * @brief Set the servo control mode of Controller.
 * @details 0-position, 1-force
 * @param [in] mode The control mode of the controller.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_setControlMode(uint32_t mode);

/**
 * @brief Get the servo control mode of Controller.
 * @details 0-position, 1-force
 * @param [out] mode The control mode of the controller.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_getControlMode(uint32_t* mode);

/**
 * @brief The the text description of a specified error code.
 * @details
 * @param [in] error_code The value of the error code.
 * @param [out] text_ptr The text description of the error code.
 * @param [out] size The size of the text.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_getErrorCodeDetail(uint64_t error_code, char* text_ptr, int* size);

/**
 * @brief Set the configuration values of the sampling channels in group.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [in] channel_index The index array of the channels.
 * @param [in] servo_index The index array of the servo.
 * @param [in] servo_param_index The index array of the specified parameter.
 * @param [in] size The valid size of the arrays.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_setSamplingChannelGroup(int32_t cpu_id, int32_t channel_index[16], int32_t servo_index[16], int32_t servo_param_index[16], int32_t size);

//rpc_file
/**
 * @brief Read a file.
 * @details 
 * @param [in] file_path_ptr The full path and the name of the file.
 * @param [out] file_data The byte stream of the file.
 * @param [out] byte_size_ptr The valid size of the byte stream.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_readFile(char* file_path_ptr, uint8_t file_data[65536], int* byte_size_ptr);

/**
 * @brief Write down a file.
 * @details
 * @param [in] file_path_ptr The full path and the name of the file.
 * @param [in] file_data The byte array of the file.
 * @param [in] byte_size The valid size of the byte array.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_writeFile(char* file_path_ptr, uint8_t file_data[65536], int byte_size);

//rpc_axis
/**
 * @brief Control the power stage(On or Off).
 * @details
 * @param [in] axis_id The ID of the axis.
 * @param [in] enable 
 * - 1: power is being enabled,
 * - 0: power is being disabled,
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisPower(int32_t axis_id, int32_t enable);

/**
 * @brief Reset all internal axis-related errors.
 * @details It makes the transition from the state 'ErrorStop' to 'Standstill' or 'Disabled'.
 * @param [in] axis_id The ID of the axis.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisReset(int32_t axis_id);

/**
 * @brief Stop the axis motion.
 * @details It commands a controlled motion stop and transfers the axis to the state 'Stopping'.\n
 *          While the axis is in state 'Stopping', no other commands can perform any motion on the same axis.\n
 *          The axis goes to state 'Standstill' when 'Velocity' zero is reached.
 * @param [in] axis_id The ID of the axis.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisStop(int32_t axis_id);

/**
 * @brief Halt the axis motion.
 * @details It commands a controlled motion stop. The axis is moved to the state 'DiscreteMotion', until the velocity is zero.\n
 *          The state is transferred to 'standstill' when the velocity is zero.\n
 *          The function is used to stop the axis under normal operation conditions.\n
 *          In non-buffered mode it is possible to set another motion command during deceleration of the axis, which will abort Halt.\n
 * @param [in] axis_id The ID of the axis.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisHalt(int32_t axis_id);

/**
 * @brief Set the offset position of the axis.
 * @details It shifts the coordinate system of an axis by manipulating both the set-point position as well as the\n
 *          actual position of an axis with the same value without any movement caused.\n
 * @param [in] axis_id The ID of the axis.
 * @param [in] position The actual position value of the axis. 
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisSetPosition(int32_t axis_id, double position);

/**
 * @brief Read the value of a specific parameter.
 * @details 
 * @param [in] axis_id The ID of the axis.
 * @param [in] param_index The index of the parameter.
 * @param [out] param_value The value of the parameter.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisReadParameter(int32_t axis_id, int32_t param_index, int32_t* param_value);

/**
 * @brief Write down the value of a specific parameter.
 * @details
 * @param [in] axis_id The ID of the axis.
 * @param [in] param_index The index of the parameter.
 * @param [in] param_value The value of the parameter.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisWriteParameter(int32_t axis_id, int32_t param_index, int32_t param_value);

/**
 * @brief Command a controlled motion to a specified absolute position.
 * @details The action completes with velocity zero if no further actions are pending.
 * @param [in] axis_id The ID of the axis.
 * @param [in] position The absolute position input. Unit: angle deg or mm.
 * @param [in] velocity The value of the maximum velocity. Unit: angle deg/s or mm/s.
 * @param [in] acc The value of the acceleration(always positive).  Unit: angle deg/s^2 or mm/s^2.
 * @param [in] dec The value of the deceleration(always positive). Unit: angle deg/s^2 or mm/s^2.
 * @param [in] jerk The value of the jerk(always positive). Unit: angle deg/s^3 or mm/s^3.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisMoveAbsolute(int32_t axis_id, double position, double velocity, double acc, double dec, double jerk);

/**
 * @brief Command a never ending controlled motion at a specified velocity.
 * @details To stop the motion, the FB has to be interrupted by another FB issuing a new command.
 * @param [in] axis_id The ID of the axis.
 * @param [in] direction
 * - 0: Positive direction.
 * - 1: Negative direction.
 * @param [in] velocity The value of the maximum velocity. Unit: angle deg/s or mm/s.
 * @param [in] acc The value of the acceleration(increasing energy of the motor). Unit: angle deg/s^2 or mm/s^2.
 * @param [in] dec The value of the deceleration(decreasing energy of the motor). Unit: angle deg/s^2 or mm/s^2.
 * @param [in] jerk The value of the jerk. Unit: angle deg/s^3 or mm/s^3.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisMoveVelocity(int32_t axis_id, int32_t direction, double velocity, double acc, double dec, double jerk);

/**
 * @brief Read the actual position.
 * @details
 * @param [in] axis_id The ID of the axis.
 * @param [out] position New absolute position. Unit: angle deg or mm.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisReadActualPosition(int32_t axis_id, double* position);

/**
 * @brief Read the actual velocity.
 * @details
 * @param [in] axis_id The ID of the axis.
 * @param [out] velocity The value of the actual velocity. Unit: angle deg/s or mm/s.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisReadActualVelocity(int32_t axis_id, double* velocity);

/**
 * @brief Read the actual torque.
 * @details
 * @param [in] axis_id The ID of the axis.
 * @param [out] torque The value of the actual torque. Unit: Nm.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisReadActualTorque(int32_t axis_id, double* torque);

/**
 * @brief Read the axis information.
 * @details
 * @param [in] axis_id The ID of the axis.
 * @param [out] simulation Simulation mode or not.
 * @param [out] comm_ready Communication is ready or not.
 * @param [out] ready_power_on Power is ready to switch on or not.
 * @param [out] power_on Power is on or off.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisReadAxisInfo(int32_t axis_id, int32_t* simulation, int32_t* comm_ready, int32_t* ready_power_on, int32_t* power_on);

/**
 * @brief Read the axis status.
 * @details
 * @param [in] axis_id The ID of the axis.
 * @param [out] status The status of the axis.
 * - AXIS_STATUS_UNKNOWN = 0,
 * - AXIS_STATUS_ERRORSTOP = 1,
 * - AXIS_STATUS_DISABLED = 2,
 * - AXIS_STATUS_STANDSTILL = 3,
 * - AXIS_STATUS_STOPPING = 4,
 * - AXIS_STATUS_HOMING = 5,
 * - AXIS_STATUS_DISCRETE_MOTION = 6,
 * - AXIS_STATUS_SYNCHRONIZED_MOTION = 8
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisReadAxisStatus(int32_t axis_id, int32_t* status);

/**
 * @brief Read the axis error.
 * @details
 * @param [in] axis_id The ID of the axis.
 * @param [out] error The current error.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisReadAxisError(int32_t axis_id, uint64_t* error);

/**
 * @brief Read the history buffer of errors.
 * @details
 * @param [in] axis_id The ID of the axis.
 * @param [out] error_list The maximum buffer size is eight.
 * @param [out] error_list_size The valid size of the buffer.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisReadAxisErrorHistory(int32_t axis_id, uint64_t error_list[8], int32_t* error_list_size);

/**
 * @brief Command a controlled motion to a specified relative position.
 * @details The action completes with velocity zero if no further actions are pending.
 * @param [in] axis_id The ID of the axis.
 * @param [in] position The relative position input. Unit: angle deg or mm.
 * @param [in] velocity The value of the maximum velocity. Unit: angle deg/s or mm/s.
 * @param [in] acc The value of the acceleration(always positive).  Unit: angle deg/s^2 or mm/s^2.
 * @param [in] dec The value of the deceleration(always positive). Unit: angle deg/s^2 or mm/s^2.
 * @param [in] jerk The value of the jerk(always positive). Unit: angle deg/s^3 or mm/s^3.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisMoveRelative(int32_t axis_id, double position, double velocity, double acc, double dec, double jerk);

/**
 * @brief Axis go home.
 * @details It commands the axis to perform the search home sequence.
 *          This option can only execute at state 'standstill'.
 *          The state is transferred to 'standstill' when the sequence is successfully.
 * @param [in] axis_id The ID of the axis.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisHome(int32_t axis_id);

/**
 * @brief Abort axis homing sequence.
 * @details This command abort the homing sequence and transfer state to 'stopping' and then  'standstill' when the velocity is zero
 * @param [in] axis_id The ID of the axis.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisAbortHoming(int32_t axis_id);

/**
 * @brief Axis go home.
 * @details It commands the axis to perform the search home sequence.
 *          This option can only execute at state 'standstill'.
 *          The state is transferred to 'standstill' when the sequence is successfully.
 * @param [in] axis_id The ID of the axis.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_axisResetEncoder(int32_t axis_id);

//rpc_group
/**
 * @brief Reset all internal group-related errors and. It also resets all axes in this group.
 * @details It makes the transition from the state GroupErrorStop to GroupStandby.
 * @param [in] group_id The ID of the group.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_groupReset(int32_t group_id);

/**
 * @brief Enable the group by controlling the power stage to be on.
 * @details It changes the state from GroupDisabled to GroupStandby.
 * @param [in] group_id The ID of the group.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_groupEnable(int32_t group_id);

/**
 * @brief Disable the group by controlling the power stage to be off.
 * @details It changes the state to GroupDisabled.
 * @param [in] group_id The ID of the group.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_groupDisable(int32_t group_id);

/**
 * @brief Get the axes group errors.
 * @details
 * @param [in] group_id The ID of the group.
 * @param [out] error The group errors.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_groupReadError(int32_t group_id, uint64_t* error);

/**
 * @brief Get the status of an axes group.
 * @details
 * @param [in] group_id The ID of the group.
 * @param [out] status Reference to the group state.
 * - GROUP_STATUS_UNKNOWN = 0,
 * - GROUP_STATUS_ERROR_STOP = 1,
 * - GROUP_STATUS_DISABLED = 2,
 * - GROUP_STATUS_STANDBY = 3,
 * - GROUP_STATUS_STOPPING = 4,
 * - GROUP_STATUS_HOMING = 5,
 * - GROUP_STATUS_MOVING = 6,
 * @param [out] in_position The position is reached or not.
 * - 0: The position is not reached.
 * - 1: The position is reached.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_groupReadStatus(int32_t group_id, int32_t* status, int32_t* in_position);

/**
 * @brief reset all the encoders in the group.
 * @details
 * @param [in] group_id The ID of the group.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_groupResetAllEncoder(int32_t group_id);

//rpc_sampling
/**
 * @brief Set the interval and maximum times of the sampling.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [in] interval The cyclical interval of the sampling.
 * @param [in] max_times The maximum times of a continuous sampling.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_setSamplingConfiguration(int32_t cpu_id, uint32_t interval, uint32_t max_times);

/**
 * @brief Get the interval and maximum times of the sampling.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [out] interval The cyclical interval of the sampling.
 * @param [out] max_times The maximum times of a continuous sampling.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_getSamplingConfiguration(int32_t cpu_id, uint32_t* interval, uint32_t* max_times);

/**
 * @brief Trigger the servo to read the configuration of the sampling.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_activateSamplingConfiguration(int32_t cpu_id);

/**
 * @brief Set the synchronization flag of the sampling.
 * @details Trigger the servo to start sampling.
 * @param [in] cpu_id CPU_ID.
 * @param [in] sync The synchroinzation flag to trigger.
 * - 0: Stop sampling.
 * - 1: Start sampling.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_setSamplingSync(int32_t cpu_id, uint32_t sync);

/**
 * @brief Get the synchronization flag of the sampling.
 * @details 
 * @param [in] cpu_id CPU_ID.
 * @param [out] sync The synchroinzation flag to trigger.
 * - 0: Sampling is finished.
 * - 1: Sampling is running.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_getSamplingSync(int32_t cpu_id, uint32_t* sync);

/**
 * @brief Set the configuration value of the sampling channel.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [in] channel_index The index of the channel.
 * @param [in] servo_index The index of the servo will be sampled on the specified channel.
 * @param [in] servo_param_index The value of the specified parameter will be sampled on the specified channel.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_setSamplingChannel(int32_t cpu_id, int32_t channel_index, int32_t servo_index, int32_t servo_param_index);

/**
 * @brief Get the configuration value of the sampling channel.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [out] value The MSB 16 bits represent servo index and the LSB 16 bits represent parameter index.
 * @param [out] size The valid siez of the value array.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_getSamplingChannel(int32_t cpu_id, uint32_t value[16], int32_t* size);

/**
 * @brief Save the sampling data to a specified file.
 * @details 
 * @param [in] cpu_id CPU_ID.
 * @param [in] file_path_ptr The full path and name of the file.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_saveSamplingBufferData(int32_t cpu_id, char* file_path_ptr);

//servo1001
/**
 * @brief Control the power stage to be off.
 * @details 
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoShutDown(int32_t cpu_id, int32_t servo_id);

/**
 * @brief Control the power stage to be on.
 * @details 
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoSwitchOn(int32_t cpu_id, int32_t servo_id);

/**
 * @brief Disable the power stage.
 * @details 
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoDisableVoltage(int32_t cpu_id, int32_t servo_id);

/**
 * @brief Enable operation after the power stage is on.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoEnableOperation(int32_t cpu_id, int32_t servo_id);

/**
 * @brief Control the power stage to be on and enable operation.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoSwitchOnAndEnableOperation(int32_t cpu_id, int32_t servo_id);

/**
 * @brief Disable operation.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoDisableOperation(int32_t cpu_id, int32_t servo_id);

/**
 * @brief Run the quick stop procedure.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoQuickStop(int32_t cpu_id, int32_t servo_id);

/**
 * @brief Clear all the faults.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoResetFault(int32_t cpu_id, int32_t servo_id);

/**
 * @brief Transfer the state of Core Communication.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @param [in] state The state of Core Communication.
 * - CORE_COMM_STATE_INIT = 0,
 * - CORE_COMM_STATE_PREOP = 1,
 * - CORE_COMM_STATE_SAFEOP = 2,
 * - CORE_COMM_STATE_OP = 3,
 * - CORE_COMM_STATE_UNKNOWN = 4,
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoTransCommState(int32_t cpu_id, int32_t servo_id, int32_t state);

/**
 * @brief Read the value of a specified parameter.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @param [in] param_index The index of the parameter.
 * @param [out] value The value of the parameter.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoReadParameter(int32_t cpu_id, int32_t servo_id, int32_t param_index, int32_t* value);

/**
 * @brief Write down the value of a specified parameter.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @param [in] param_index The index of the parameter.
 * @param [in] param_value The value of the parameter.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoWriteParameter(int32_t cpu_id, int32_t servo_id, int32_t param_index, int32_t param_value);

/**
 * @brief Command a never ending controlled motion at a specified velocity.
 * @details To stop the motion, the FB has to be interrupted by another FB issuing a new command.
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @param [in] velocity The value of the maximum velocity. The unit is revolutions/s.
 * @param [in] acc The value of the acceleration(increasing energy of the motor). The unit is revolutions/s^2.
 * @param [in] dec The value of the deceleration(decreasing energy of the motor). The unit is revolutions/s^2.
 * @param [in] jerk The value of the jerk(always positive). The unit is revolutions/s^3.
 * @param [in] direction
 * - 0: Positive direction.
 * - 1: Negative direction.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoMoveVelocity(int32_t cpu_id, int32_t servo_id, int32_t velocity, int32_t acc, int32_t dec, int32_t jerk, int32_t direction);

/**
 * @brief Command a controlled motion to a specified absolute position.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @param [in] position an absolute position. The unit is pulse.
 * @param [in] velocity The value of the maximum velocity. The unit is revolutions/s.
 * @param [in] acc The value of the acceleration(always positive). The unit is revolutions/s^2.
 * @param [in] dec The value of the deceleration(always positive). The unit is revolutions/s^2.
 * @param [in] jerk The value of the jerk(always positive). The unit is revolutions/s^3.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoMoveAbsolute(int32_t cpu_id, int32_t servo_id, int64_t position, int32_t velocity, int32_t acc, int32_t dec, int32_t jerk);


/**
 * @brief Trigger the servo to upload parameters.
 * @details Should be called first before reading the parameters(c_servo1001ServoUploadParameters).
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoTriggerUploadParameters(int32_t cpu_id, int32_t servo_id);

/**
 * @brief Read all the parameters upload from the servo.
 * @details Should be called after triggering the servo to upload parameters(c_servo1001ServoTriggerUploadParameters).
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @param [out] value int array.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoUploadParameters(int32_t cpu_id, int32_t servo_id, int32_t value[512]);

/**
 * @brief Trigger the servo to download parameters.
 * @details Should be called after parameters are prepared(c_servo1001ServoDownloadParameters).
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoTriggerDownloadParameters(int32_t cpu_id, int32_t servo_id);

/**
 * @brief Prepare the download parameters for the servo.
 * @details Should be called first before triggering the servo to download parameters(c_servo1001ServoTriggerDownloadParameters).
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @param [in] param_value The value array of the parameters.
 * @param [in] param_size The valid number of the array.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoDownloadParameters(int32_t cpu_id, int32_t servo_id, int32_t param_value[512], int32_t param_size);

/**
 * @brief Check the status of the servo operating on the parameters.
 * @details Should be called after triggering the servo to upload or download parameters.\n
 *          Sync being 0 indicates the servo is operating on the parameters.\n
 *          Sync being 1 indicates parameter operation is finished.\n  
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @param [out] sync 
 * - 0: The servo is operating on the paramters.
 * - 1: Parameter operation is finished.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoIsAsyncServiceFinish(int32_t cpu_id, int32_t servo_id, int32_t* sync);

/**
 * @brief Get the state of Core Communication.
 * @details 
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @param [out] state The state of Core Communication.
 * - CORE_COMM_STATE_INIT = 0,
 * - CORE_COMM_STATE_PREOP = 1,
 * - CORE_COMM_STATE_SAFEOP = 2,
 * - CORE_COMM_STATE_OP = 3,
 * - CORE_COMM_STATE_UNKNOWN = 4,
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoGetCommState(int32_t cpu_id, int32_t servo_id, int32_t* state);

/**
 * @brief Command a controlled motion to a specified relative position.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @param [in] position an absolute position. The unit is pulse.
 * @param [in] velocity The value of the maximum velocity. The unit is revolutions/s.
 * @param [in] acc The value of the acceleration(always positive). The unit is revolutions/s^2.
 * @param [in] dec The value of the deceleration(always positive). The unit is revolutions/s^2.
 * @param [in] jerk The value of the jerk(always positive). The unit is revolutions/s^3.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoMoveRelative(int32_t cpu_id, int32_t servo_id, int64_t position, int32_t velocity, int32_t acc, int32_t dec, int32_t jerk);

/**
 * @brief Axis go home.
 * @details It commands the axis to perform the search home sequence.
 *          This option can only execute at state 'standstill'.
 *          The state is transferred to 'standstill' when the sequence is successfully.
 * @param [in] axis_id The ID of the axis.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoHome(int32_t cpu_id, int32_t servo_id);

/**
 * @brief Abort axis homing sequence.
 * @details This command abort the homing sequence and transfer state to 'stopping' and then  'standstill' when the velocity is zero
 * @param [in] axis_id The ID of the axis.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoAbortHoming(int32_t cpu_id, int32_t servo_id);

/**
 * @brief Get the IDs of the applications among the servo communication.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @param [out] comm_reg_id The application ID of the communication register.
 * @param [out] service_id The application ID of the core service call.
 * @param [out] download_param_id The application ID of the download paramter structure.
 * @param [out] upload_param_id The application ID of the upload paramter structure.
 * @param [out] ctrl_pdo_id The application ID of the control PDO structure.
 * @param [out] fdb_pdo_id The application ID of the feedback PDO structure.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoGetServoCommInfo(int32_t cpu_id, int32_t servo_id, int32_t* comm_reg_id, int32_t* service_id,
	int32_t* download_param_id, int32_t* upload_param_id, int32_t* ctrl_pdo_id, int32_t* fdb_pdo_id);

/**
 * @brief Get the feedback of the servo defined data.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @param [in] req_data The request data.
 * @param [out] res_data The response data from the servo.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001ServoGetServoDefinedInfo(int32_t cpu_id, int32_t servo_id, int32_t req[7], int32_t res[7]);

/**
 * @brief Get the versions of Controller.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [out] major_version The value of the major version
 * @param [out] minor_version The value of the minor version.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001CpuGetVersion(int32_t cpu_id, int32_t* major_version, int32_t* minor_version);

/**
 * @brief Set the synchronization flag to control PDO.
 * @details After the flag being setted, the servo starts to read PDO and execute the motion.
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @param [in] sync The flag to enable the servo reading PDO.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001CpuSetCtrlPdoSync(int32_t cpu_id, int32_t servo_id, uint32_t sync);

/**
 * @brief Get the value of PDO synchronization.
 * @details 
 * @param [in] cpu_id CPU_ID.
 * @param [in] servo_id The ID of the servo.
 * @param [out] sync The flag to enable the servo reading PDO.
 * - 0: The servo has finished reading PDO.
 * - 1: The servo is reading PDO.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001CpuGetCtrlPdoSync(int32_t cpu_id, int32_t servo_id, uint32_t* sync);

/**
 * @brief Set the synchronization flag of the sampling.
 * @details Trigger the servo to start sampling.
 * @param [in] cpu_id CPU_ID.
 * @param [in] sync The synchroinzation flag to trigger.
 * - 0: Stop sampling.
 * - 1: Start sampling.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001CpuSetSamplingSync(int32_t cpu_id, uint32_t sync);

/**
 * @brief Get the synchronization flag of the sampling.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [out] sync The synchroinzation flag to trigger.
 * - 0: Sampling is finished.
 * - 1: Sampling is running.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001CpuGetSamplingSync(int32_t cpu_id, uint32_t* sync);

/**
 * @brief Set the interval of the sampling.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [in] interval The cyclical interval of the sampling.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001CpuSetSamplingInterval(int32_t cpu_id, uint32_t interval);

/**
 * @brief Get the interval of the sampling.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [out] interval The cyclical interval of the sampling.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001CpuGetSamplingInterval(int32_t cpu_id, uint32_t* interval);

/**
 * @brief Set the maximum times of the sampling.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [in] max_times The maximum times of a continuous sampling.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001CpuSetSamplingMaxTimes(int32_t cpu_id, uint32_t max_times);

/**
 * @brief Get the maximum times of the sampling.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [out] max_times The maximum times of a continuous sampling.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001CpuGetSamplingMaxTimes(int32_t cpu_id, uint32_t* max_times);

/**
 * @brief Set the configuration value of the sampling channel.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [in] channel_index The index of the channel.
 * @param [in] servo_index The index of the servo will be sampled on the specified channel.
 * @param [in] servo_param_index The value of the specified parameter will be sampled on the specified channel.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001CpuSetSamplingChannel(int32_t cpu_id, uint32_t channel_index, int32_t servo_index, int32_t servo_param_index);

/**
 * @brief Get the configuration value of the sampling channel.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [out] value The MSB 16 bits represent servo index and the LSB 16 bits represent parameter index.
 * @param [out] size The valid siez of the value array.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001CpuGetSamplingChannel(int32_t cpu_id, uint32_t value[16], int32_t* size);

/**
 * @brief Trigger the servo to read the configuration of the sampling.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001CpuActivateSamplingConfiguration(int32_t cpu_id);

/**
 * @brief Save the sampling data to a specified file.
 * @details Copy the sampling data to a specified file.
 * @param [in] cpu_id CPU_ID.
 * @param [in] file_path_ptr The full path and name of the file.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001CpuSaveSamplingBufferData(int32_t cpu_id, char* file_path_ptr);


/**
 * @brief Get the IDs of the applications among the cpu communication.
 * @details
 * @param [in] cpu_id CPU_ID.
 * @param [out] comm_reg_id The application ID of the communication register.
 * @param [out] sampling_buffer_id The application ID of the sampling data structure.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_servo1001CpuGetServoCpuCommInfo(int32_t cpu_id, int32_t* comm_reg_id, int32_t* sampling_buffer_id);

//IO
/**
 * @brief Get DI value.
 * @details
 * @param [in] port_offset The offset of the port.
 * @param [out] value The value of the port.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_ioReadDI(int32_t port_offset, int32_t* value);

/**
 * @brief Get DO value.
 * @details
 * @param [in] port_offset The offset of the port.
 * @param [out] value The value of the port.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_ioReadDO(int32_t port_offset, int32_t* value);

/**
 * @brief Write DO value.
 * @details
 * @param [in] port_offset The offset of the port.
 * @param [in] value The value of the port.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_ioWriteDO(int32_t port_offset, int32_t value);


//rpc_tool
/**
 * @brief Get the tool info.
 * @details
 * @param [in] id the id of the frame.
 * @param [out] x The x position.
 * @param [out] y The y position.
 * @param [out] z The z position.
 * @param [out] a The a posture.
 * @param [out] b The b posture.
 * @param [out] c The c posture.
 * @param [out] name The defined name by the cumstomer.
 * @param [out] comment The comment by the cumstomer.
 * @param [out] group_id The group id.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_tfUploadToolFrame(int32_t id, double* x, double* y, double* z, double* a, double* b, double* c, char* name, char* comment, int32_t* group_id);

/**
 * @brief Set the tool frame info.
 * @details
 * @param [in] id the id of the frame.
 * @param [in] x The x position.
 * @param [in] y The y position.
 * @param [in] z The z position.
 * @param [in] a The a posture.
 * @param [in] b The b posture.
 * @param [in] c The c posture.
 * @param [in] name The defined name by the cumstomer.
 * @param [in] comment The comment by the cumstomer.
 * @param [in] group_id The group id.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_tfDownloadToolFrame(int32_t id, double x, double y, double z, double a, double b, double c, char* name, char* comment, int32_t group_id = 0);

/**
 * @brief delete the tool frame info.
 * @details
 * @param [in] id the id of the frame.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_tfDeleteToolFrame(int32_t id);


//rpc_coord
/**
 * @brief Get the user coordinate frame info.
 * @details
 * @param [in] id the id of the frame.
 * @param [out] x The x position.
 * @param [out] y The y position.
 * @param [out] z The z position.
 * @param [out] a The a posture.
 * @param [out] b The b posture.
 * @param [out] c The c posture.
 * @param [out] name The defined name by the cumstomer.
 * @param [out] comment The comment by the cumstomer.
 * @param [out] group_id The group id.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_ufUploadUserFrame(int32_t id, double* x, double* y, double* z, double* a, double* b, double* c, char* name, char* comment, int32_t* group_id);

/**
 * @brief Set the coordinate frame info.
 * @details
 * @param [in] id the id of the frame.
 * @param [in] x The x position.
 * @param [in] y The y position.
 * @param [in] z The z position.
 * @param [in] a The a posture.
 * @param [in] b The b posture.
 * @param [in] c The c posture.
 * @param [in] name The defined name by the cumstomer.
 * @param [in] comment The comment by the cumstomer.
 * @param [in] group_id The group id.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_ufDownloadUserFrame(int32_t id, double x, double y, double z, double a, double b, double c, char* name, char* comment, int32_t group_id = 0);

/**
 * @brief delete the coordinate frame info.
 * @details
 * @param [in] id the id of the frame.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_ufDeleteUserFrame(int32_t id);



//rpc_motion_control

COMM_INTERFACE_API uint64_t c_mcSetGlobalVelRatio(int32_t vel_ratio);

COMM_INTERFACE_API uint64_t c_mcSetGlobalAccRatio(int32_t acc_ratio);

/**
 * @brief step move.
 * @details
 * @param [in] axis_id the axis reference.
 * @param [in] diretion 0-standing, 1-increase, 2-decrease.
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_mcDoStepManualMove(int32_t axis_id, int32_t direction);

COMM_INTERFACE_API uint64_t c_mcDoContinuousManualMove(int32_t axis_id, int32_t direction);

COMM_INTERFACE_API void c_mcDoContinuousManualToStandstill();

COMM_INTERFACE_API uint64_t c_mcDoGotoCartesianMove(int32_t group_index, double x, double y, double z, double a, double b, double c,
	int32_t arm_front_back, int32_t elbow_up_down, int32_t wrist_down_up, int32_t uf_id, int32_t tf_id);

COMM_INTERFACE_API uint64_t c_mcDoGotoJointMove(int32_t group_index, double j1, double j2, double j3, double j4, double j5, double j6, double j7, double j8, double j9);

/**
 * @brief Set the user coordinate frame.
 * @details
 * @param [in] coord_type "Joint/Base/World/User/Tool".
 * @return error_code
 */
COMM_INTERFACE_API uint64_t c_mcSetCoordinate(char* coord_type);

COMM_INTERFACE_API uint64_t c_mcSetUserCoordId(int32_t user_coord_id);

COMM_INTERFACE_API uint64_t c_mcSetToolId(int32_t tool_id);

COMM_INTERFACE_API uint64_t c_mcIgnoreLostZeroError();

COMM_INTERFACE_API uint64_t c_mcSetAllZeroPointOffsets();

COMM_INTERFACE_API uint64_t c_mcCalibrateAllZero();

COMM_INTERFACE_API uint64_t c_mcSetStep(double joint_step, double cartesian_step, double orientation_step);

COMM_INTERFACE_API uint64_t c_mcGetPostureByJoint(int32_t group_index, double j1, double j2, double j3, double j4, double j5, double j6, double j7, double j8, double j9,
	int32_t* arm_front_back, int32_t* elbow_up_down, int32_t* wrist_down_up);



#ifdef __cplusplus
}
#endif