#ifndef CORE_COMM_SERVO_DATATYPE_H
#define CORE_COMM_SERVO_DATATYPE_H

/**
 * @file core_comm_servo_datatype.h
 * @brief The file includes the definitions of data structures relates to servo in CiA402 standard.
 * @author zhengyu.shen
 */

#include <stdint.h>

#define SERVO_SAMPLING_BUFFER_SIZE 0x400000 /**< The maximum byte size of servo cpu sampling buffer.*/
#define SERVO_PARAM_BUFFER_SIZE 512         /**< The maximum number of servo parameters.*/

/**
 * @brief Bit definition of the standard servo control word.
 * @details Defer to CiA402.
 */
typedef struct
{
    uint32_t switch_on:1;
    uint32_t enable_voltage:1;
    uint32_t quick_stop:1;
    uint32_t enable_operation:1;
    uint32_t operation_mode_spec1:3;
    uint32_t fault_reset:1;
    uint32_t halt:1;
    uint32_t operation_mode_spec2:1;
    uint32_t rsvd:22;
}ServoCtrl_b;
/**
 * @brief Definition of the standard servo control word.
 */
typedef union
{
    uint32_t all;
    ServoCtrl_b bit;
}ServoCtrl_u;
/**
 * @brief Bit definition of the standard servo state word.
 * @details Defer to CiA402.
 */
typedef struct
{
	uint32_t ready_to_switch_on:1;
	uint32_t switched_on:1;
	uint32_t operation_enabled:1;
	uint32_t fault:1;
	uint32_t voltage_enabled:1;
	uint32_t quick_stop:1;
	uint32_t switch_on_disabled:1;
    uint32_t warning:1;
    uint32_t rsvd1:1;
    uint32_t remote:1;
    uint32_t target_reached:1;
    uint32_t internal_limit_active:1;
    uint32_t operation_mode_spec:2;
    uint32_t rsvd:18;
}ServoState_b;
/**
 * @brief Definition of the standard servo state word.
 */
typedef union
{
	uint32_t all;
	ServoState_b bit;
}ServoState_u;
/**
 * @brief Definition of servo operation mode.
 * @details Defer to CiA402.
 */
typedef enum
{
    SERVO_OP_MODE_NO_MODE = 0,
    SERVO_OP_MODE_PROFILE_POSITION_MODE = 1,
    SERVO_OP_MODE_VELOCITY_MODE = 2,
    SERVO_OP_MODE_PROFILE_VELOCITY_MODE = 3,
    SERVO_OP_MODE_TORQUE_PROFILE_MODE = 4,
    SERVO_OP_MODE_HOMING_MODE = 6,
    SERVO_OP_MODE_INTERPOLATED_POSITION_MODE = 7,
    SERVO_OP_MODE_CYCLIC_SYNC_POSITION_MODE = 8,
    SERVO_OP_MODE_CYCLIC_SYNC_VELOCITY_MODE = 9,
    SERVO_OP_MODE_CYCLIC_SYNC_TORQUE_MODE = 10,
}ServoOpMode_e;
/**
 * @brief Definition of servo state machine.
 * @details Defer to CiA402.
 */
typedef enum
{
    SERVO_SM_START = 0,
    SERVO_SM_NOT_READY_TO_SWITCH_ON = 1,
    SERVO_SM_SWITCH_ON_DISABLED = 2,
    SERVO_SM_READY_TO_SWITCH_ON = 3,
    SERVO_SM_SWITCHED_ON = 4,
    SERVO_SM_OPERATION_ENABLED = 5,
    SERVO_SM_QUICK_STOP_ACTIVE = 6,
    SERVO_SM_FAULT_REACTION_ACTIVE = 7,
    SERVO_SM_FAULT = 8,
    SERVO_SM_UNKNOWN = 9,
}ServoSm_e;

#endif
