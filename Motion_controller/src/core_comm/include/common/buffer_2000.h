#ifndef BUFFER_2000_H
#define BUFFER_2000_H

/**
 * @file buffer_2000.h
 * @brief The file includes the definition of the data type for buffer 2000 channel.
 * @author zhengyu.shen
 */

#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/buffer_base.h"
#include "./core_protocal_inc/core_comm_datatype.h"
#include "./core_protocal_inc/core_comm_servo_datatype.h"
#include "./core_protocal_inc/buffer_datatype.h"
#include <stdint.h>
#else
#include "common/buffer_base.h"
#include "common/core_comm_datatype.h"
#include "common/core_comm_servo_datatype.h"
#include "common/buffer_datatype.h"
#include <stdint.h>
#endif

/**
 * @brief BufferAppData2000_t defines the data structure for sampling.
 */
typedef struct
{
    int32_t sampling[SERVO_SAMPLING_BUFFER_SIZE];   /**< The array stores the sampling data.*/
}BufferAppData2000_t;

/**
 * @brief Initialize the buffer 2000 channel.
 * @details block_ptr.param1 is the max byte size of the data section of the channel.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @return void
 */
void initBuffer2000(CommBlockData_t* block_ptr);

#endif

