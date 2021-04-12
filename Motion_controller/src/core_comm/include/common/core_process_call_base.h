#ifndef CORE_PROCESS_CALL_BASE_H
#define CORE_PROCESS_CALL_BASE_H

/**
 * @file core_process_call_base.h
 * @brief The file includes the APIs for operating core process call channel.
 * @author zhengyu.shen
 */

#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/core_comm_datatype.h"
#include "./core_protocal_inc/core_process_call_datatype.h"
#include <stdint.h>
#else
#include "common/core_comm_datatype.h"
#include "common/core_process_call_datatype.h"
#include <stdint.h>
#endif

/**
 * @brief Initialize the share memory of the communication channel in core process call type.
 * @details Initialize both control and data sections of a communication channel in core process call type.\n
 * @param [in] block_ptr Pointer of the configuration of a communication channel.
 * @param [in] data_ptr Pointer of the initial setting of the control section.
 * @return void
 */
void initCoreProcessCallMemoryByFrom(CommBlockData_t* block_ptr, CoreProcessCallCommData_t* data_ptr);
/**
 * @brief Check if a core process call channel is idle to send a request.
 * @param [in] block_ptr Pointer of the configuration of a communication channel.
 * @retval true The channel is ready to send a request.
 * @retval false The channel is busy.
 */
bool isCoreProcessCallChannelIdle(CommBlockData_t* block_ptr);
/**
 * @brief Send a request.
 * @param [in] block_ptr Pointer of the configuration of a communication channel.
 * @param [in] data_ptr Pointer of the data to be sent.
 * @param [in] data_size Byte size of the data to be sent.
 * @param [in] enable_async Flag to set if the process call is synchronous or asynchronous.
 * @retval true Send a request in success.
 * @retval false Failed to send a request.
 */
bool sendCoreProcessCallRequest(CommBlockData_t* block_ptr, uint8_t* data_ptr, int32_t data_size, bool enable_async);
/**
 * @brief Receive a response.
 * @param [in] block_ptr Pointer of the configuration of a communication channel.
 * @param [in] data_ptr Pointer of the data to be received.
 * @param [in] data_size_ptr Byte size of the data to be received.
 * @param [in] timeout_ptr Flag to active time out function.
 * @retval true Receive a response in success.
 * @retval false Failed to receive a response.
 */
bool recvCoreProcessCallResponse(CommBlockData_t* block_ptr, uint8_t* data_ptr, int32_t* data_size_ptr, bool* timeout_ptr);
/**
 * @brief Check if some progress is finished which is triggered by an asynchronous request.
 * @param [in] block_ptr Pointer of the configuration of a communication channel.
 * @retval true The progress has finished.
 * @retval false The progress has not finished yet.
 */
bool isCoreProcessCallAsyncAckActived(CommBlockData_t* block_ptr);
/**
 * @brief Receive a request.
 * @param [in] block_ptr Pointer of the configuration of a communication channel.
 * @param [in] data_ptr Pointer of the data that has been received.
 * @param [in] data_size_ptr Byte size of the data that has been received.
 * @retval true Receive a request in success.
 * @retval false Failed to receive a request.
 */
bool recvCoreProcessCallRequest(CommBlockData_t* block_ptr, uint8_t* data_ptr, int32_t* data_size_ptr);
/**
 * @brief Send a response.
 * @param [in] block_ptr Pointer of the configuration of a communication channel.
 * @param [in] data_ptr Pointer of the data to be sent.
 * @param [in] data_size Byte size of the data to be sent.
 * @retval true Send a response in success.
 * @retval false Failed to send a response.
 */
bool sendCoreProcessCallResponse(CommBlockData_t* block_ptr, uint8_t* data_ptr, int32_t data_size);
/**
 * @brief Send an 'process done' acknowledgement to the sponsor who triggered an asynchronous request.
 * @param [in] block_ptr Pointer of the configuration of a communication channel.
 * @return void
 */
void sendCoreProcessCallAsyncAck(CommBlockData_t* block_ptr);

#endif

