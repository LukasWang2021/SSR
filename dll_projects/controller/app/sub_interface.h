#pragma once
#include "comm_def.h"
#include "protoc.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Initialize the client to subscribe.
 * @details Initialize the socket.\n
 *          Start a thread to receive publishing data from the server.\n
 * @param [in] server_ip the IP of the publishing server.
 * @retval 0 Success.
 * @retval -1 Failed to initialize.
 */
COMM_INTERFACE_API uint64_t c_initSub(char* server_ip);

/**
 * @brief The client exits.
 * @details
 * @retval 0 Success.
 * @retval -1 Failure.
 */
COMM_INTERFACE_API uint64_t c_exitSub(void);


/**
 * @brief Get all the axes feedback.
 * @details Array for axes.
 * @param [int] array_size The lengh of the array.
 * @param [out] isr The Interrupt Service Routine of servo.
 * @param [out] state The status of the axes.
 * - AXIS_STATUS_UNKNOWN = 0,
 * - AXIS_STATUS_ERRORSTOP = 1,
 * - AXIS_STATUS_DISABLED = 2,
 * - AXIS_STATUS_STANDSTILL = 3,
 * - AXIS_STATUS_STOPPING = 4,
 * - AXIS_STATUS_HOMING = 5,
 * - AXIS_STATUS_DISCRETE_MOTION = 6,
 * - AXIS_STATUS_SYNCHRONIZED_MOTION = 8
 * @param [out] position The positions of the axes. The unit is angle deg.
 * @param [out] velocity The velocities of the axes. The unit is angle deg/s
 * @param [out] torque The torques of the axes. The unit is Nm.
 * @retval 0 Success.
 * @retval -1 Failure.
 */
COMM_INTERFACE_API uint64_t c_getTopicAxisFeedback(uint32_t array_size, uint32_t isr[AXIS_NUM], uint32_t state[AXIS_NUM], double position[AXIS_NUM], double velocity[AXIS_NUM], double torque[AXIS_NUM]);

/**
 * @brief Get all the servos raw data.
 * @details Array for servos.
 * @param [int] array_size The lengh of the array.
 * @param [out] data The raw data of servo feedback. Each servo has 32 Int array.
 * @retval 0 Success.
 * @retval -1 Failure.
 */
COMM_INTERFACE_API uint64_t c_getTopicServoFeedback(uint32_t axis_size, int32_t data[AXIS_NUM][32]);

/**
 * @brief Get CPU related feedback.
 * @details Array for axes.
 * @param [int] array_size The lengh of the array.
 * @param [out] ctrl_pdo_sync Each axis has one control PDO synchronized flag.
 * - 1: The axis is running PDO.
 * - 0: The axis has no PDO.
 * @param [out] sampling_sync The sampling flag.
 * - 1: sampling is running.
 * - 0: sampling is finished.
 * @retval 0 Success.
 * @retval -1 Failure.
 */
COMM_INTERFACE_API uint64_t c_getTopicCpuFeedback(uint32_t array_size, uint32_t ctrl_pdo_sync[AXIS_NUM], uint32_t* sampling_sync);


/**
 * @brief Get all the axes feedback sampling by ISR.
 * @details Array for axes.
 * @param [int] array_size The lengh of the array.
 * @param [out] isr_count The Interrupt Service Routine of servo.
 * @param [out] state The status of the axes.
 * - AXIS_STATUS_UNKNOWN = 0,
 * - AXIS_STATUS_ERRORSTOP = 1,
 * - AXIS_STATUS_DISABLED = 2,
 * - AXIS_STATUS_STANDSTILL = 3,
 * - AXIS_STATUS_STOPPING = 4,
 * - AXIS_STATUS_HOMING = 5,
 * - AXIS_STATUS_DISCRETE_MOTION = 6,
 * - AXIS_STATUS_SYNCHRONIZED_MOTION = 8,
 * @param [out] position The positions of the axes. The unit is angle deg.
 * @param [out] velocity The velocities of the axes. The unit is angle deg/s.
 * @param [out] torque The torques of the axes. The unit is Nm.
 * @retval 0 Success.
 * @retval -1 Failure.
 */
COMM_INTERFACE_API uint64_t c_getAxisFeedBackByIsrCount(uint32_t array_size,
	                                                    uint32_t isr_count,
														uint32_t state[AXIS_NUM],
														double position[AXIS_NUM],
														double velocity[AXIS_NUM],
														double torque[AXIS_NUM]);


/**
 * @brief Get safety IO feedback.
 * @details .
 * @param [out] low The low byte for safety io.
 * @param [out] high The high byte for safety io.
 * @retval 0 Success.
 * @retval -1 Failure.
 */
COMM_INTERFACE_API uint64_t c_getSafetyIOFeedBack(uint32_t *low, uint32_t* high);

/**
 * @brief Get torque feedback.
 * @details .
 * @param [out] low The low byte for safety io.
 * @param [in] count of torque number.
 * @retval 0 Success.
 * @retval -1 Failure.
 */
COMM_INTERFACE_API uint64_t c_getTorqueFeedBack(double *torque, int size);

/**
 * @brief Get state of fio device from fio feedback information.
 * @details .
 * @param [out] state of fio device.
 * @retval 0 Success.
 * @retval -1 Failure.
 */
COMM_INTERFACE_API uint64_t c_getFioInfoFeedBackDevieState(uint32_t* state);

/**
 * @brief Get actual speed of fio motor.
 * @details .
 * @param [out] the actual speed back of fio motor.
 * @retval 0 Success.
 * @retval -1 Failure.
 */
COMM_INTERFACE_API uint64_t c_getFioInfoFeedBackActualVelocity(uint32_t* vel);
#ifdef __cplusplus
}
#endif