#ifndef SERVO_INTERFACE_H
#define SERVO_INTERFACE_H

/**
 * @file servo_interface.h
 * @brief The file includes basic interfaces for operating servo for both controller and servo. 
 * @author zhengyu.shen
 */

#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/core_comm_datatype.h"
#include "./core_protocal_inc/core_comm_servo_datatype.h"
#include "./core_protocal_inc/comm_reg_0.h"
#include "./core_protocal_inc/core_process_call_1000.h"
#include "./core_protocal_inc/buffer_2000.h"
#include "./core_protocal_inc/buffer_2001.h"
#include "./core_protocal_inc/buffer_2002.h"
#include "./core_protocal_inc/circle_buffer_3000.h"
#include "./core_protocal_inc/circle_buffer_4000.h"
#include <stdint.h>
#else
#include "common/core_comm_datatype.h"
#include "common/core_comm_servo_datatype.h"
#include "common/comm_reg_0.h"
#include "common/core_process_call_1000.h"
#include "common/buffer_2000.h"
#include "common/buffer_2001.h"
#include "common/buffer_2002.h"
#include "common/circle_buffer_3000.h"
#include "common/circle_buffer_3001.h"
#include "common/circle_buffer_3002.h"
#include "common/circle_buffer_4000.h"
#include <stdint.h>
#endif

#define SERVO_COMM_APP_ID_COMM_REG          0       /**< Application ID of servo's register channel.*/
#define SERVO_COMM_APP_ID_SERVICE           1000    /**< Application ID of servo's core process call channel.*/
#define SERVO_COMM_APP_ID_UPLOAD_PARAM      2002    /**< Application ID of servo's upload buffer channel.*/
#define SERVO_COMM_APP_ID_DOWNLOAD_PARAM    2001    /**< Application ID of servo's download buffer channel.*/

/**
 * @brief Defines configuration of communication channels of a servo.
 */
typedef struct
{
    int32_t servo_index;    /**< Servo index.*/
    int32_t from;           /**< CPU id of the controller which controls the servo.*/
    int32_t to;             /**< CPU id of the servo cpu which contains the servo.*/
    CommBlockData_t* comm_reg_ptr;              /**< Pointer of register channel.*/
    CommBlockData_t* service_ptr;               /**< Pointer of core process call channel.*/
    CommBlockData_t* download_param_buffer_ptr; /**< Pointer of download buffer channel.*/
    CommBlockData_t* upload_param_buffer_ptr;   /**< Pointer of upload buffer channel.*/
    CommBlockData_t* ctrl_pdo_ptr;              /**< Pointer of circle buffer for control pdo channel.*/
    CommBlockData_t* fdb_pdo_ptr;               /**< Pointer of circle buffer for feedback pdo channel.*/
}ServoComm_t;
/**
 * @brief Create configuration object to handling a servo communication on controller side.
 * @details The returned object pointer should be freed if some one don't use it anymore.\n
 *          The API should only be called on the controller side.\n
 * @param [in] controller_id CPU id of controller.
 * @param [in] servo_id CPU id of the servo cpu.
 * @param [in] servo_index Servo index.
 * @return Pointer of the configuration object.
 */
ServoComm_t* createServoCommByController(int32_t controller_id, int32_t servo_id, int32_t servo_index);
/**
 * @brief Initialize the configuration object when the controller ask the servo transfer its communication state from INIT to PREOP.
 * @details Search the from and to lists of communication block data and select the matching block data to initialize channels.\n
 *          Four channels are initialized: register channel/core process call channel/download buffer channel/upload buffer channel.\n
 *          The API should only be called on the controller side.\n
 * @param [in] comm_ptr Pointer of the configuration object.
 * @param [in] from_block_ptr Pointer of the list of communication block data for all from channels.
 * @param [in] from_block_number Store the number of communication block data in the from list.
 * @param [in] to_block_ptr Pointer of the list of communication block data for all to channels.
 * @param [in] to_block_number Store the number of communication block data in the to list.
 * @retval true Operation succeed.
 * @retval false Operation failed.
 */
bool initServoCommInit2PreOpByController(ServoComm_t* comm_ptr,
                                                    CommBlockData_t* from_block_ptr, size_t from_block_number,
                                                    CommBlockData_t* to_block_ptr, size_t to_block_number);
/**
 * @brief Initialize the configuration object when the controller ask the servo transfer its communication state from PREOP to SAFEOP.
 * @details Search the from and to lists of communication block data and select the matching block data to initialize channels.\n
 *          One channel is initialized: feedback pdo channel.\n
 *          The API should only be called on the controller side.\n
 * @param [in] comm_ptr Pointer of the configuration object.
 * @param [in] to_block_ptr Pointer of the list of communication block data for all to channels.
 * @param [in] to_block_number Store the number of communication block data in the to list.
 * @param [in] app_id Application ID for the expected circle buffer channel.
 * @retval true Operation succeed.
 * @retval false Operation failed.
 */
bool initServoCommPreOp2SafeOpByController(ServoComm_t* comm_ptr, CommBlockData_t* to_block_ptr, size_t to_block_number, int32_t app_id);
/**
 * @brief Initialize the configuration object when the controller ask the servo transfer its communication state from SAFEOP to OP.
 * @details Search the from and to lists of communication block data and select the matching block data to initialize channels.\n
 *          One channel is initialized: control pdo channel.\n
 *          The API should only be called on the controller side.\n
 * @param [in] comm_ptr Pointer of the configuration object.
 * @param [in] from_block_ptr Pointer of the list of communication block data for all from channels.
 * @param [in] from_block_number Store the number of communication block data in the from list.
 * @param [in] app_id Application ID for the expected circle buffer channel.
 * @retval true Operation succeed.
 * @retval false Operation failed.
 */
bool initServoCommSafeOp2OpByController(ServoComm_t* comm_ptr, CommBlockData_t* from_block_ptr, size_t from_block_number, int32_t app_id);                                                     
/**
 * @brief Send a no-response core process call to servo.
 * @details Send a request to a servo and no servo response is expected.\n
 *          The API should only be called on the controller side.\n
 * @param [in] block_ptr Pointer of the servo core process call channel.
 * @param [in] req_data_ptr Pointer of the request data for the core process call.
 * @retval true Operation succeed.
 * @retval false Operation failed.
 */
bool doServoCmdFastService(CommBlockData_t* block_ptr, CoreProcessCallAppData1000_t* req_data_ptr);
/**
 * @brief Send a request-response core process call to servo.
 * @details Send a request to a servo and servo response is expected.\n
 *          The API should only be called on the controller side.\n
 * @param [in] block_ptr Pointer of the servo core process call channel.
 * @param [in] req_data_ptr Pointer of the request data for the core process call.
 * @param [in] enable_async Flag to set the sync/async property of the core process call. It is used internally, user should keep it in default value.
 * @param [out] res_data_ptr Pointer of the response data for the core process call.
 * @retval true Operation succeed.
 * @retval false Operation failed.
 */
bool doServoCmdNormalService(CommBlockData_t* block_ptr, 
                                     CoreProcessCallAppData1000_t* req_data_ptr, 
                                     CoreProcessCallAppData1000_t* res_data_ptr,
                                     bool enable_async = false);

/**
 * @brief Send an asynchronize request-response core process call to servo.
 * @details Send an asynchronize request to a servo and servo response is expected at once.\n
 *          A pointer of asynchronize flag is also returned to let user check the process of the real task.\n
 *          The core process call channel can not start another request until the asynchronize flag becomes to true.\n
 *          The API should only be called on the controller side.\n
 * @param [in] block_ptr Pointer of the servo core process call channel.
 * @param [in] req_data_ptr Pointer of the request data for the core process call.
 * @param [out] res_data_ptr Pointer of the response data for the core process call.
 * @param [out] async_ack_ptr_ptr Pointer of asynchronize flag.
 * @retval true Operation succeed.
 * @retval false Operation failed.
 */
bool doServoCmdAsyncService(CommBlockData_t* block_ptr, 
                                    CoreProcessCallAppData1000_t* req_data_ptr, 
                                    CoreProcessCallAppData1000_t* res_data_ptr, 
                                    int32_t** async_ack_ptr_ptr);
/**
 * @brief Check the state of asynchronize flag.
 * @details The API should only be called on the controller side.\n
 * @param [in] async_ack_ptr Pointer of asynchronize flag.
 * @retval true The asynchronize flag is true.
 * @retval false The asynchronize flag is false.
 */
bool isAsyncServiceDone(int32_t* async_ack_ptr);
/**
 * @brief Get the controller ID which controls the servo by servo index.
 * @details The API should only be called on the servo side.\n
 * @param [in] servo_index Servo index.
 * @param [in] from_block_ptr Pointer of the list of communication block data for all from channels.
 * @param [in] from_block_number Store the number of communication block data in the from list.
 * @param [out] controller_id Controller ID.
 * @retval true Some controller id is found.
 * @retval false No matching controller id is exist.
 */
bool getServoCommControllerIdByServoIndex(int32_t servo_index, CommBlockData_t* from_block_ptr, size_t from_block_number, int32_t& controller_id);
/**
 * @brief Create configuration object to handling a servo communication on servo side.
 * @details The returned object pointer should be freed if some one don't use it anymore.\n
 *          The API should only be called on the servo side.\n
 * @param [in] controller_id CPU id of controller.
 * @param [in] servo_id CPU id of the servo cpu.
 * @param [in] servo_index Servo index.
 * @return Pointer of the configuration object.
 */
ServoComm_t* createServoCommByServo(int32_t controller_id, int32_t servo_id, int32_t servo_index);
/**
 * @brief Initialize the configuration object when the servo transfer its communication state from INIT to PREOP.
 * @details Search the from and to lists of communication block data and select the matching block data to initialize channels.\n
 *          Four channels are initialized: register channel/core process call channel/download buffer channel/upload buffer channel.\n
 *          The API should only be called on the servo side.\n
 * @param [in] comm_ptr Pointer of the configuration object.
 * @param [in] from_block_ptr Pointer of the list of communication block data for all from channels.
 * @param [in] from_block_number Store the number of communication block data in the from list.
 * @param [in] to_block_ptr Pointer of the list of communication block data for all to channels.
 * @param [in] to_block_number Store the number of communication block data in the to list.
 * @retval true Operation succeed.
 * @retval false Operation failed.
 */
bool initServoCommInit2PreOpByServo(ServoComm_t* comm_ptr,
                                            CommBlockData_t* from_block_ptr, size_t from_block_number,
                                            CommBlockData_t* to_block_ptr, size_t to_block_number);
/**
 * @brief Initialize the configuration object when the servo transfer communication state from PREOP to SAFEOP.
 * @details Search the from and to lists of communication block data and select the matching block data to initialize channels.\n
 *          One channel is initialized: feedback pdo channel.\n
 *          The API should only be called on the servo side.\n
 * @param [in] comm_ptr Pointer of the configuration object.
 * @param [in] from_block_ptr Pointer of the list of communication block data for all from channels.
 * @param [in] from_block_number Store the number of communication block data in the from list.
 * @param [in] app_id Application ID for the expected circle buffer channel.
 * @retval true Operation succeed.
 * @retval false Operation failed.
 */
bool initServoCommPreOp2SafeOpByServo(ServoComm_t* comm_ptr, CommBlockData_t* from_block_ptr, size_t from_block_number, int32_t app_id);
/**
 * @brief Initialize the configuration object when the servo transfer communication state from SAFEOP to OP.
 * @details Search the from and to lists of communication block data and select the matching block data to initialize channels.\n
 *          One channel is initialized: control pdo channel.\n
 *          The API should only be called on the servo side.\n
 * @param [in] comm_ptr Pointer of the configuration object.
 * @param [in] to_block_ptr Pointer of the list of communication block data for all to channels.
 * @param [in] to_block_number Store the number of communication block data in the to list.
 * @param [in] app_id Application ID for the expected circle buffer channel.
 * @retval true Operation succeed.
 * @retval false Operation failed.
 */
bool initServoCommSafeOp2OpByServo(ServoComm_t* comm_ptr, CommBlockData_t* to_block_ptr, size_t to_block_number, int32_t app_id);
/**
 * @brief Set servo communication state.
 * @details The communication state should be set step by step like INIT->PREOP->SAFEOP->OP.\n
 *          The API should only be called on the servo side.\n
 * @param [in] comm_ptr Pointer of the configuration object.
 * @param [in] comm_state Communication state.
 * @retval true Operation succeed.
 * @retval false Operation failed.
 */
bool setServoCommState(ServoComm_t* comm_ptr, CoreCommState_e comm_state);
/**
 * @brief Get servo communication state.
 * @details The API can be called on both controller and servo sides.\n
 * @param [in] comm_ptr Pointer of the configuration object.
 * @return Communication state.
 */
CoreCommState_e getServoCommState(ServoComm_t* comm_ptr);
/**
 * @brief Free the configuration object.
 * @details The API can be called on both controller and servo sides.\n
 * @param [in] comm_ptr Pointer of the configuration object.
 * @return void
 */
void freeServoComm(ServoComm_t* comm_ptr);

#endif

