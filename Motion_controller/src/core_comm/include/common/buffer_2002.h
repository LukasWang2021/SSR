#ifndef BUFFER_2002_H
#define BUFFER_2002_H

/**
 * @file buffer_2002.h
 * @brief The file includes the definition of the data type for buffer 2002 channel.
 * @author zhengyu.shen
 */

#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/common_datatype.h"
#include "./core_protocal_inc/buffer_base.h"
#include "./core_protocal_inc/core_comm_datatype.h"
#include "./core_protocal_inc/core_comm_servo_datatype.h"
#include "./core_protocal_inc/buffer_datatype.h"
#include <stdint.h>
#else
#include "common_datatype.h"
#include "common/buffer_base.h"
#include "common/core_comm_datatype.h"
#include "common/core_comm_servo_datatype.h"
#include "common/buffer_datatype.h"
#include <stdint.h>
#endif


/**
 * @brief Defines the data structure for uploading servo parameters.
 */
typedef struct
{
    int32_t param[SERVO_PARAM_BUFFER_SIZE];   /**< Servo parameter list.*/
}BufferAppData2002_t;

/**
 * @brief Initialize the buffer 2002 channel.
 * @details block_ptr.param1 is the index of servo which the channel is belong to.
 *          block_ptr.param2 is the max byte size of the data section of the channel.  
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @return void
 */
void initBuffer2002(CommBlockData_t* block_ptr);

#endif

