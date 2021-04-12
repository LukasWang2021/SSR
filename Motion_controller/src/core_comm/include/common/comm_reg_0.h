#ifndef COMM_REG_0_H
#define COMM_REG_0_H

/**
 * @file comm_reg_0.h
 * @brief The file includes the definition of the register channel for establishing the inter core communication.
 * @author zhengyu.shen
 */

#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/core_comm_datatype.h"
#include <stdint.h>
#else
#include "common/core_comm_datatype.h"
#include <stdint.h>
#endif

#define COMM_REG0_STATE_PTR                  ((int8_t*)(block_ptr->memory_ptr))     /**< The address to store the communication state of slave.*/
#define COMM_REG0_ERROR_FLAG_PTR             ((int8_t*)(block_ptr->memory_ptr + 4)) /**< The address to store the error flag if there is something wrong in establishing the communication.*/

/**
 * @brief Defines the data structure for the register channel for establishing the inter core communication.
 */
typedef struct
{
    int32_t state;      /**< Communication state of slave. Define 0: INIT; 1: PREOP; 2: SAFEOP; 3: OP*/
    int32_t error_flag; /**< Communication error flag of slave. Define 0: no error; 1: error*/
}CommRegAppData0_t;

/**
 * @brief Initialize the register 0 channel.
 * @details block_ptr.param1~block_ptr.param8 is not used.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @return void
 */
void initCommReg0(CommBlockData_t* block_ptr);
/**
 * @brief Set slave communication state.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [in] state Communication state. Define 0: INIT; 1: PREOP; 2: SAFEOP; 3: OP.
 * @return void
 */
void setCommReg0State(CommBlockData_t* block_ptr, int32_t state);
/**
 * @brief Get slave communication state.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [out] state_ptr Pointer of the communication state. Define 0: INIT; 1: PREOP; 2: SAFEOP; 3: OP.
 * @return void
 */
void getCommReg0State(CommBlockData_t* block_ptr, int32_t* state_ptr);
/**
 * @brief Set slave communication error flag.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [in] error_flag Communication error flag. Define 0: no error; 1: error.
 * @return void
 */
void setCommReg0ErrorFlag(CommBlockData_t* block_ptr, int32_t error_flag);
/**
 * @brief Get slave communication error flag.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [out] error_flag_ptr Pointer to communication error flag. Define 0: no error; 1: error.
 * @return void
 */
void getCommReg0ErrorFlag(CommBlockData_t* block_ptr, int32_t* error_flag_ptr);



#endif

