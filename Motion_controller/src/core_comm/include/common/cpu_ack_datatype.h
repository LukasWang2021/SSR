#ifndef CPU_ACK_DATATYPE_H
#define CPU_ACK_DATATYPE_H

/**
 * @file cpu_ack_datatype.h
 * @brief The file includes the definition of a cpu ack channel.
 * @author zhengyu.shen
 */

#include <stdint.h>

/**
 * @brief Defines the data structure for a cpu ack channel.
 */
typedef struct
{
    uint8_t data;   /**< Cpu ackownledgement data.*/
}CpuAckCommData_t;



#endif

