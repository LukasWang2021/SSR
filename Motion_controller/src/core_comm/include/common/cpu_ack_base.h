#ifndef CPU_ACK_BASE_H
#define CPU_ACK_BASE_H

/**
 * @file cpu_ack_base.h
 * @brief The file includes the APIs for operating core process call channel.
 * @author zhengyu.shen
 */

#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/core_comm_datatype.h"
#include "./core_protocal_inc/cpu_ack_datatype.h"
#else
#include "common/core_comm_datatype.h"
#include "common/cpu_ack_datatype.h"
#endif

/**
 * @brief Write data to a cpu ack channel.
 * @param [in] block_ptr Pointer of the configuration of a cpu ack channel.\n
 * @param [in] data_ptr Pointer of the data.
 * @retval true Operation is successful.
 * @retval false Operation is failed.
 */
bool setCpuAck(CpuAckBlockData_t* block_ptr, CpuAckCommData_t* data_ptr);
/**
 * @brief Read data from a cpu ack channel.
 * @param [in] block_ptr Pointer of the configuration of a cpu ack channel.\n
 * @param [in] data_ptr Pointer of the data.
 * @retval true Operation is successful.
 * @retval false Operation is failed.
 */
bool getCpuAck(CpuAckBlockData_t* block_ptr, CpuAckCommData_t* data_ptr);


#endif

