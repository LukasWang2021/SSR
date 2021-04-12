#ifndef CORE_COMM_CONFIG_BASE_H
#define CORE_COMM_CONFIG_BASE_H

/**
 * @file core_comm_config_base.h
 * @brief The file includes the APIs for establishing the inter core communication.
 * @author zhengyu.shen
 */

#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/core_comm_datatype.h"
#include <stdint.h>
#else
#include "common/core_comm_datatype.h"
#include <stdint.h>
#endif


/**
 * @brief Initialize all pointers of communication channels according to configuration.
 * @param [in,out] core_comm_config_ptr Pointer of the configuration data of all channels.
 * @param [in] comm_ptr Pointer of the base address of the share memeory for inter core communication.
 * @return void
 */
void initCoreCommConfigMemeoryPtr(CoreCommConfig_t* core_comm_config_ptr, char* comm_ptr);
/**
 * @brief Extract the information of all channels related to the specified cpu id and initialize corresponding channels.
 * @param [in] core_comm_config_ptr Pointer of the configuration data of all channels.
 * @param [in] cpu_id Cpu id.
 * @param [out] from_block_ptr_ptr Pointer of the list of communication block data which start from the specified cpu id.
 * @param [out] from_block_number_ptr Pointer of the variable to store the number of communication block data in the from list.
 * @param [out] to_block_ptr_ptr Pointer of the list of communication block data which end to the specified cpu id.
 * @param [out] to_block_number_ptr Pointer tof the variable to store the number of communication block data in the to list.
 * @return void
 */
void initLocalChannel(CoreCommConfig_t* core_comm_config_ptr, int32_t cpu_id, 
                           CommBlockData_t** from_block_ptr_ptr, size_t* from_block_number_ptr,
                           CommBlockData_t** to_block_ptr_ptr, size_t* to_block_number_ptr);
/**
 * @brief Get the information of boardcast block by from cpu id. 
 * @param [in] config_ptr Pointer of the configuration data of all channels.
 * @param [in] from_id From cpu id.
 * @param [out] number_ptr Pointer of the variable to store the number of boardcast block found. It should always be 1.
 * @return The pointer of the configuration of boardcast block. It should be freed after use.
 */
BoardcastBlockData_t* getBoardcastBlockDataByFrom(CoreCommConfig_t* config_ptr, int32_t from_id, size_t* number_ptr);
/**
 * @brief Get the information of cpu ack block by from cpu id. 
 * @param [in] config_ptr Pointer of the configuration data of all channels.
 * @param [in] from_id From cpu id.
 * @param [out] number_ptr Pointer of the variable to store the number of cpu ack block found.
 * @return The pointer of the list of the configuration of cpu ack blocks. It should be freed after use.
 */
CpuAckBlockData_t* getCpuAckBlockDataByFrom(CoreCommConfig_t* config_ptr, int32_t from_id, size_t* number_ptr);  
/**
 * @brief Get the information of cpu ack block by to cpu id. 
 * @param [in] config_ptr Pointer of the configuration data of all channels.
 * @param [in] to_id To cpu id.
 * @param [out] number_ptr Pointer of the variable to store the number of cpu ack block found.
 * @return The pointer of the list of the configuration of cpu ack blocks. It should be freed after use.
 */
CpuAckBlockData_t* getCpuAckBlockDataByTo(CoreCommConfig_t* config_ptr, int32_t to_id, size_t* number_ptr);
/**
 * @brief Get the information of communication block by from cpu id. 
 * @param [in] config_ptr Pointer of the configuration data of all channels.
 * @param [in] from_id From cpu id.
 * @param [out] number_ptr Pointer of the variable to store the number of communication block found.
 * @return The pointer of the list of the configuration of communication blocks. It should be freed after use.
 */
CommBlockData_t* getCommBlockDataByFrom(CoreCommConfig_t* config_ptr, int32_t from_id, size_t* number_ptr);    
/**
 * @brief Get the information of communication block by to cpu id. 
 * @param [in] config_ptr Pointer of the configuration data of all channels.
 * @param [in] to_id To cpu id.
 * @param [out] number_ptr Pointer of the variable to store the number of communication block found.
 * @return The pointer of the list of the configuration of communication blocks. It should be freed after use.
 */
CommBlockData_t* getCommBlockDataByTo(CoreCommConfig_t* config_ptr, int32_t to_id, size_t* number_ptr);

#endif

