#ifndef BOARDCAST_BASE_H
#define BOARDCAST_BASE_H

/**
 * @file boardcast_base.h
 * @brief The file includes the APIs for operating boardcast channel.
 * @author zhengyu.shen
 */

#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/core_comm_datatype.h"
#include "./core_protocal_inc/boardcast_datatype.h"
#else
#include "common/core_comm_datatype.h"
#include "common/boardcast_datatype.h"
#endif

/**
 * @brief Write data to boardcast channel.
 * @param [in] memory_ptr Pointer of the expected address to write data.
 * @param [in] data_ptr Pointer of the expected data to write.
 * @retval true Operation succeed.
 * @retval false Operation failed.
 */
bool setBoardcast(char* memory_ptr, BoardcastCommData_t* data_ptr);
/**
 * @brief Read data from boardcast channel.
 * @param [in] memory_ptr Pointer of the expected address to read data.
 * @param [out] data_ptr Pointer of the expected data to read.
 * @retval true Operation succeed.
 * @retval false Operation failed.
 */
bool getBoardcast(char* memory_ptr, BoardcastCommData_t* data_ptr);


#endif
