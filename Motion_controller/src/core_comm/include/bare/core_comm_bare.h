#ifndef CORE_COMM_BARE_H
#define CORE_COMM_BARE_H

/**
 * @file core_comm_bare.h
 * @brief The file includes the example APIs for establishing inter core communication on bare core.
 * @author zhengyu.shen
 */

#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/common_error_code.h"
#include "./core_protocal_inc/core_comm_datatype.h"
#else
#include "common_error_code.h"
#include "common/core_comm_datatype.h"
#endif

extern int32_t g_cpu_id;                    /**< The cpu id of the bare core.*/
extern bool g_core_comm_config_ready;       /**< Flag to show if the configuration data of the inter core communition is available.*/
extern CommBlockData_t* g_from_block_ptr;   /**< The pointer of the specified configuration data which originate from the bare core.*/
extern CommBlockData_t* g_to_block_ptr;     /**< The pointer of the specified configuration data which direct to the bare core.*/
extern size_t g_from_block_number;          /**< The number of communication channels which originate from the bare core.*/
extern size_t g_to_block_number;            /**< The number of communication channels which direct to the bare core.*/

/**
 * @brief Check if the master is booted and the configuration data of the inter core communication is available.
 * @retval true Configuration data is available.
 * @retval false Configuration data is not available.
 */
bool isMasterBooted();

/**
 * @brief Establish the inter core communication channel according to the configuration data.
 * @details Initialize local data and establish the communication channel for local side.\n
 *          Set acknowledge flag to notify the master that the bare core side is ready to communicate.\n
 * @return If no error happen, return SUCCESS, otherwise return some error code.
 */
ErrorCode bootAsSlave();



#endif

