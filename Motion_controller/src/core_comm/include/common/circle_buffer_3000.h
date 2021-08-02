#ifndef CIRCLE_BUFFER_3000_H
#define CIRCLE_BUFFER_3000_H

/**
 * @file circle_buffer_3000.h
 * @brief The file includes the definition of the data type for circle buffer 3000 channel.
 * @author zhengyu.shen
 */

#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/circle_buffer_base.h"
#include "./core_protocal_inc/core_comm_datatype.h"
#include "./core_protocal_inc/circle_buffer_datatype.h"
#include "./core_protocal_inc/core_comm_servo_datatype.h"
#include <stdint.h>
#else
#include "common/circle_buffer_base.h"
#include "common/core_comm_datatype.h"
#include "common/circle_buffer_datatype.h"
#include "common/core_comm_servo_datatype.h"
#include <stdint.h>
#endif

/**
 * @brief Defines the data structure for standard servo feedback pdos.
 * @details Circle buffer 3000 is a circular buffer which is designed for handling feedback process data.\n
 */
typedef struct
{
    uint32_t time_stamp;     /**< Stores the current isr count of the servo. It must be the first element!*/
    ServoState_u state_word; /**< Stores the current servo state word.*/
    int32_t actual_op_mode;  /**< Stores the current servo operation mode.*/
    int32_t dummy;
    int64_t fdb_position;    /**< Stores the current servo position feedback, unit in encoder pulse.*/
    int32_t fdb_velocity;    /**< Stores the current servo velocity feedback, not used.*/
    int32_t fdb_torque;      /**< Stores the current servo torque feedback, not used.*/
    int32_t encoder_state;
    int32_t encoder_value;
}CircleBufferAppData3000_t;

/**
 * @brief Initialize the circle buffer 3000 channel.
 * @details block_ptr.param1 is the index of servo which the channel is belong to.\n
 *          block_ptr.param2 is the byte size of each element in the circle. \n
 *          block_ptr.param3 is the maximum byte size of all elements in the circle. It must be integer multiple of block_ptr.param2.\n
 *          block_ptr.param4 is the minimum byte size of all elements in the circle. It must be integer multiple of block_ptr.param2.\n
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @return void
 */
void initCircleBuffer3000(CommBlockData_t* block_ptr);

#endif

