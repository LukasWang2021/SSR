#ifndef CIRCLE_BUFFER_4000_H
#define CIRCLE_BUFFER_4000_H

/**
 * @file circle_buffer_4000.h
 * @brief The file includes the definition of the data type for circle buffer 4000 channel.
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
 * @brief Defines the data structure for standard servo control pdos in interpolated position mode.
 * @details Circle buffer 4000 is a circular buffer which is designed for handling control process data.\n
 */
typedef struct
{
    int64_t cmd_position;           /**< Stores the servo position command, unit in encoder pulse.*/
    int32_t feedforward_velocity;   /**< Stores the servo velocity feedforward, not used.*/
    int32_t feedforward_torque;     /**< Stores the servo torque feedforward, not used.*/
}CircleBufferAppData4000_t;

/**
 * @brief Initialize the circle buffer 4000 channel.
 * @details block_ptr.param1 is the index of servo which the channel is belong to.\n
 *          block_ptr.param2 is the byte size of each element in the circle. \n
 *          block_ptr.param3 is the maximum byte size of all elements in the circle. It must be integer multiple of block_ptr.param2.\n
 *          block_ptr.param4 is the minimum byte size of all elements in the circle. It must be integer multiple of block_ptr.param2.\n
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @return void
 */
void initCircleBuffer4000(CommBlockData_t* block_ptr);

#endif

