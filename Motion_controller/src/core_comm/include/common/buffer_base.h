#ifndef BUFFER_BASE_H
#define BUFFER_BASE_H

/**
 * @file buffer_base.h
 * @brief The file includes the APIs for operating buffer channels.
 * @author zhengyu.shen
 */

#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/core_comm_datatype.h"
#include "./core_protocal_inc/buffer_datatype.h"
#include <stdint.h>
#else
#include "common/core_comm_datatype.h"
#include "common/buffer_datatype.h"
#include <stdint.h>
#endif

/**
 * @brief Initialize the share memory of the communication channel in buffer type.
 * @details Initialize both control and data sections of a communication channel in buffer type.\n
 * @param [in] block_ptr Pointer of the configuration of a communication channel.\n
 * @param [in] data_ptr Pointer of the initial setting of the control section.
 * @return void
 */
void initBufferMemoryByFrom(CommBlockData_t* block_ptr, BufferCommData_t* data_ptr);

/**
 * @brief Copy data from local to the share memory of the communication channel in buffer type.
 * @details Copy local data to the data section of a communication channel.\n
 *          The start address is the first byte of the data section.\n
 *          The operation is suit to buffer2001 and buffer2002.\n
 * @param [in] block_ptr Pointer of the configuration of a communication channel.\n
 * @param [in] data_ptr Pointer of the local data.
 * @param [in] data_byte_size The byte size of the local data that is to be copied.
 * @retval true Operation is success.
 * @retval false Operation is failed.
 */
bool setBufferType1(CommBlockData_t* block_ptr, uint8_t* data_ptr, int32_t data_byte_size);
/**
 * @brief Copy data from the data section of the communication channel in buffer type to local
 * @details Copy the data a in buffer channel to local.\n
 *          The start address is the first byte of the data section.\n
 *          The operation is suit to buffer2001 and buffer2002.\n
 * @param [in] block_ptr Pointer of the configuration of a communication channel.\n
 * @param [out] data_ptr Pointer of local memeory which is used to store the data in a buffer channel.
 * @param [out] data_byte_size_ptr Pointer of the variable to stored the valid byte size of data that is copied.
 * @retval true Operation is success.
 * @retval false Operation is failed.
 */
bool getBufferType1(CommBlockData_t* block_ptr, uint8_t* data_ptr, int32_t* data_byte_size_ptr);
/**
 * @brief Reset the data section of the communication channel in buffer type.
 * @details Only set the first valid byte of the data section to the first byte of the data section.
 *          The operation is suit to buffer2000.\n
 * @param [in] block_ptr Pointer of the configuration of a communication channel.\n
 * @return true void
 */
void resetBufferType2(CommBlockData_t* block_ptr);
/**
 * @brief Copy data from local to the share memory of the communication channel in buffer type.
 * @details Copy local data to the data section of a communication channel.\n
 *          The start address is the first valid byte of the data section.\n
 *          The first valid byte address grows automatically with data_byte_size after the api is called successfully.\n
 *          If the valid byte address is going to exceed the upper boundary, the api will return false.
 *          The operation is suit to buffer2000.\n
 * @param [in] block_ptr Pointer of the configuration of a communication channel.\n
 * @param [in] data_ptr Pointer of the local data.
 * @param [in] data_byte_size The byte size of the local data that is to be copied.
 * @retval true Operation is success.
 * @retval false Operation is failed.
 */
bool setBufferType2(CommBlockData_t* block_ptr, uint8_t* data_ptr, int32_t data_byte_size);
/**
 * @brief Copy data from the data section of the communication channel in buffer type to local
 * @details Copy the data a in buffer channel to local.\n
 *          The start address is the first byte of the data section.\n
 *          The operation is suit to buffer2000.\n
 * @param [in] block_ptr Pointer of the configuration of a communication channel.\n
 * @param [out] data_ptr Pointer of local memeory which is used to store the data in a buffer channel.
 * @param [out] data_byte_size_ptr Pointer of the variable to stored the valid byte size of data that is copied.
 * @retval true Operation is success.
 * @retval false Operation is failed.
 */
bool getBufferType2(CommBlockData_t* block_ptr, uint8_t* data_ptr, int32_t* data_byte_size_ptr);

#endif

