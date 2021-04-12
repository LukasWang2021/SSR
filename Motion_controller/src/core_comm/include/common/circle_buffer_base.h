#ifndef CIRCLE_BUFFER_BASE_H
#define CIRCLE_BUFFER_BASE_H

/**
 * @file circle_buffer_base.h
 * @brief The file includes the APIs for operating circle buffer channels.
 * @author zhengyu.shen
 */

#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/core_comm_datatype.h"
#include "./core_protocal_inc/circle_buffer_datatype.h"
#include <stdint.h>
#else
#include "common/core_comm_datatype.h"
#include "common/circle_buffer_datatype.h"
#include <stdint.h>
#endif

/**
 * @brief Initialize the share memory of the communication channel in circle buffer type.
 * @details Initialize both control and data sections of a communication channel in circle buffer type.\n
 * @param [in] block_ptr Pointer of the configuration of a communication channel.\n
 * @param [in] data_ptr Pointer of the initial setting of the control section.
 * @return void
 */
void initCircleBufferMemoryByFrom(CommBlockData_t* block_ptr, CircleBufferCommData_t* data_ptr);

/**
 * @brief Get the occupied byte size in the data section of a circle buffer channel.
 * @param [in] block_ptr Pointer of the configuration of a communication channel.\n
 * @return The byte size of occupied elements.
 */
int32_t getOccupiedByteSize(CommBlockData_t* block_ptr);
/**
 * @brief Get the number of occupied elements in the data section of a circle buffer channel.
 * @param [in] block_ptr Pointer of the configuration of a communication channel.\n
 * @return The number of the occupied elements.
 */
int32_t getOccupiedElementNumber(CommBlockData_t* block_ptr);
/**
 * @brief Get the spare byte size in the data section of a circle buffer channel.
 * @param [in] block_ptr Pointer of the configuration of a communication channel.\n
 * @return The byte size of spare elements.
 */
int32_t getSpareByteSize(CommBlockData_t* block_ptr);
/**
 * @brief Get the number of spare elements in the data section of a circle buffer channel.
 * @param [in] block_ptr Pointer of the configuration of a communication channel.\n
 * @return The number of the spare elements.
 */
int32_t getSpareElementNumber(CommBlockData_t* block_ptr);
/**
 * @brief Push local data to the tail of the circle buffer section.
 * @details The operation is suit to circle buffer 4000.\n
 * @param [in] block_ptr Pointer of the configuration of a communication channel.\n
 * @param [in] domain_ptr Pointer of the local pdo domain.
 * @param [in] element_number The number of pdo elements that is to be pushed in.
 * @retval true Operation is success.
 * @retval false Operation is failed.
 */
bool pushCircleBufferType1(CommBlockData_t* block_ptr, uint8_t* domain_ptr, int32_t element_number);
/**
 * @brief Pull out the data in the head of the circle buffer section.
 * @details The operation is suit to circle buffer 4000.\n
 * @param [in] block_ptr Pointer of the configuration of a communication channel.
 * @param [in] element_number The number of pdo elements that is to be pulled out.
 * @param [out] domain_ptr Pointer of the local pdo domain which is going to store the pdo data from channel.
 * @retval true Operation is success.
 * @retval false Operation is failed.
 */
bool pullCircleBufferType1(CommBlockData_t* block_ptr, uint8_t* domain_ptr, int32_t element_number);
/**
 * @brief Get the previous byte offset of the head.
 * @details The operation is suit to circle buffer 3000.\n
 *          The API is used by real servo.\n         
 * @param [in] block_ptr Pointer of the configuration of a communication channel.\n
 * @retval true Operation is success.
 * @retval false Operation is failed.
 */
int32_t getPreviousHeadByteOffset(CommBlockData_t* block_ptr);
/**
 * @brief Get the previous byte offset of the tail.
 * @details The operation is suit to circle buffer 3000.\n
 *          The API is used by real servo.\n         
 * @param [in] block_ptr Pointer of the configuration of a communication channel.\n
 * @retval true Operation is success.
 * @retval false Operation is failed.
 */
int32_t getPreviousTailByteOffset(CommBlockData_t* block_ptr);
/**
 * @brief Push local pdo element to the tail of the circle buffer section.
 * @details The operation is suit to circle buffer 3000.\n
 * @param [in] block_ptr Pointer to the configuration of a communication channel.\n
 * @param [in] domain_ptr Pointer of the local pdo domain.
 * @retval true Operation is success.
 * @retval false Operation is failed.
 */
bool pushCircleBufferType2(CommBlockData_t* block_ptr, uint8_t* domain_ptr);
/**
 * @brief Pull out one pdo element from the head of the circle buffer section.
 * @details The operation is suit to circle buffer 3000.\n
 * @param [in] block_ptr Pointer to the configuration of a communication channel.
 * @param [out] domain_ptr Pointer of the local pdo domain which is going to store the pdo data from channel.
 * @param [out] time_stamp_ptr Pointer of the variable which stores the servo isr count of the pdo element one gets.
 * @retval true Operation is success.
 * @retval false Operation is failed.
 */
bool pullCircleBufferType2ByHead(CommBlockData_t* block_ptr, uint8_t* domain_ptr, uint32_t* time_stamp_ptr);
/**
 * @brief Pull out one pdo element from the circle buffer section according to a specified servo isr count.
 * @details The operation is suit to circle buffer 3000.\n
 * @param [in] block_ptr Pointer of the configuration of a communication channel.
 * @param [out] domain_ptr Pointer of the local pdo domain which is going to store the pdo data from channel.
 * @param [out] time_stamp Servo isr count.
 * @retval true Operation is success.
 * @retval false Operation is failed.
 */
bool pullCircleBufferType2ByTimeStamp(CommBlockData_t* block_ptr, uint8_t* domain_ptr, uint32_t time_stamp);


#endif

