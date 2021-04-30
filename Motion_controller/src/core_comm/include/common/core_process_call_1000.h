#ifndef CORE_PROCESS_CALL_1000_H
#define CORE_PROCESS_CALL_1000_H

/**
 * @file core_process_call_1000.h
 * @brief The file includes the definition of the data type for core process call 1000 channel.
 * @author zhengyu.shen
 */

#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/core_process_call_base.h"
#include "./core_protocal_inc/core_comm_datatype.h"
#include "./core_protocal_inc/core_process_call_datatype.h"
#include <stdint.h>
#else
#include "common/core_process_call_base.h"
#include "common/core_comm_datatype.h"
#include "common/core_process_call_datatype.h"
#include <stdint.h>
#endif

/**
 * @brief Command type for controlling servo.
 * @details Defer to CiA402.
 */
typedef enum
{
    SERVO_CMD_TRANS_COMM_STATE = 0,                 /**< Ask servo slave switch communication state.*/
    SERVO_CMD_SHUT_DOWN = 1,                        /**< Ask servo switch to SERVO_SM_READY_TO_SWITCH_ON.*/
    SERVO_CMD_SWITCH_ON = 2,                        /**< Ask servo switch to SERVO_SM_SWITCHED_ON.*/
    SERVO_CMD_DISABLE_VOLTAGE = 3,                  /**< Ask servo switch to SERVO_SM_SWITCH_ON_DISABLED.*/
    SERVO_CMD_ENABLE_OPERATION = 4,                 /**< Ask servo switch to SERVO_SM_OPERATION_ENABLED.*/
    SERVO_CMD_SWITCH_ON_AND_ENABLE_OPERATION =5,    /**< Ask servo cross SERVO_SM_SWITCHED_ON and SERVO_SM_OPERATION_ENABLED automatically.*/
    SERVO_CMD_DISABLE_OPERATION = 6,                /**< Ask servo switch from SERVO_SM_OPERATION_ENABLED to SERVO_SM_SWITCHED_ON.*/
    SERVO_CMD_QUICK_STOP = 7,                       /**< Ask servo switch to SERVO_SM_QUICK_STOP_ACTIVE or SERVO_SM_SWITCH_ON_DISABLED.*/
    SERVO_CMD_FAULT_RESET = 8,                      /**< Ask servo switch from SERVO_SM_FAULT to SERVO_SM_SWITCH_ON_DISABLED.*/
    SERVO_CMD_RESET_FAULT_RESET = 9,                /**< Ask servo ready to respond for next SERVO_CMD_FAULT_RESET.*/
    SERVO_CMD_READ_PARAMETER = 10,                  /**< Read one parameter from servo.*/
    SERVO_CMD_WRITE_PARAMETER = 11,                 /**< Write one parameter to servo.*/
    SERVO_CMD_UPLOAD_PARAMETERS = 12,               /**< Read all parameters from servo.*/
    SERVO_CMD_DOWNLOAD_PARAMETERS = 13,             /**< Write all parameters to servo.*/
    SERVO_CMD_MOVE_VELOCITY = 14,                   /**< Ask servo move with specified velocity.*/
    SERVO_CMD_MOVE_ABSOLUTE = 15,                   /**< Ask servo move to specified position.*/
    SERVO_CMD_MOVE_RELATIVE = 16,                   /**< Ask servo move to a relative position.*/
    SERVO_CMD_HALT = 17,                            /**< Ask servo stop current motion, back to standstill.*/
    SERVO_CMD_RESET_ENCODER = 18,                   /**< Reset encoder error.*/
    SERVO_CMD_SERVO_DEFINED = 19,                   /**< Servo self defined data.*/
    SERVO_CMD_MAX = 20,                             /**< Number of servo command type.*/
    SERVO_CMD_UNKNOWN = 0xFF,                       /**< Invalid servo command.*/
}ServoCmd_e;
/**
 * @brief Data struct of a servo command.
 */
typedef struct
{
    int32_t cmd;        /**< Servo command type, choose in ServoCmd_e.*/
    int32_t param1;     /**< Servo command parameter 1, user defined.*/
    int32_t param2;     /**< Servo command parameter 2, user defined.*/
    int32_t param3;     /**< Servo command parameter 3, user defined.*/
    int32_t param4;     /**< Servo command parameter 4, user defined.*/
    int32_t param5;     /**< Servo command parameter 5, user defined.*/
    int32_t param6;     /**< Servo command parameter 6, user defined.*/
    int32_t param7;     /**< Servo command parameter 7, user defined.*/
    int32_t param8;     /**< Servo command parameter 8, user defined.*/
}CoreProcessCallAppData1000_t;
/**
 * @brief Initialize the core process call 1000 channel.
 * @details block_ptr.param1 is the index of servo which the channel is belong to.\n
 *          block_ptr.param2 is the byte size of CoreProcessCallAppData1000_t. \n
 *          block_ptr.param3 is the maximum times for a core process call sponsor to try receiving the response after send a request.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @return void
 */
void initCoreProcessCall1000(CommBlockData_t* block_ptr);

#endif

