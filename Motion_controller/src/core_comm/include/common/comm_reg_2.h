#ifndef COMM_REG_2_H
#define COMM_REG_2_H

/**
 * @file comm_reg_2.h
 * @brief The file includes the definition of the stator parameters for MMT.
 * @author feng.wu
 */

#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/core_comm_datatype.h"
#include <stdint.h>
#else
#include "common/core_comm_datatype.h"
#include <stdint.h>
#endif

#define COMM_REG2_PARAMETER_NUMBER 512
#define COMM_REG2_UPDATE_FLAG_PTR                  ((int8_t*)(block_ptr->memory_ptr))         /**< The address to store the updateing flag.*/
#define COMM_REG2_PARAMETERS_PTR                  ((int8_t*)(block_ptr->memory_ptr + 4))     /**< The address to store the parameters of the stators.*/


/**
 * @brief Defines the data structure for force control parameters.
 * @details The register section provides the updating of the force control parameters.\n
 */
typedef struct
{
    int32_t parameter[COMM_REG2_PARAMETER_NUMBER]; /**< It stores the paramters of all the stators*/
}CommRegForceControlParam_t;

typedef struct
{
    uint32_t update_flag;    /**< Indicates the the parameters are updated.*/
    CommRegForceControlParam_t force_control;  /**< It stores the paramters of all the force control*/
}CommRegAppData2_t;

/**
 * @brief Initialize the register 2 channel.
 * @details block_ptr.param1~block_ptr.param8 is not used.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @return void
 */
void initCommReg2(CommBlockData_t* block_ptr);
/**
 * @brief Set the flag to indicate the update stator parameters.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [in] index The index of the flag array.
 * @param [in] value The value of the flag.
 * @return bool
 */
bool setCommReg2UpdateFlag(CommBlockData_t* block_ptr, uint32_t value);
/**
 * @brief Get the value of the flag.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [in] index The index of the flag array.
 * @param [out] value_ptr Pointer to the value of the flag.
 * @return bool
 */
bool getCommReg2UpdateFlag(CommBlockData_t* block_ptr, uint32_t* value_ptr);

/**
 * @brief Set the stator parameters.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [in] index The index of the stator.
 * @param [in] data_ptr Pointer of the local data.
 * @param [in] data_byte_size The byte size of the local data that is to be copied.
 * @return bool
 */
bool setCommReg2Parameters(CommBlockData_t* block_ptr, uint8_t* data_ptr, uint32_t data_byte_size);

/**
 * @brief Set the stator parameters.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [in] index The index of the stator.
 * @param [out] data_ptr Pointer of local memeory which is used to store the data.
 * @param [out] data_byte_size_ptr Pointer of the variable to stored the valid byte size of data that is copied.
 * @return bool
 */
bool getCommReg2Parameters(CommBlockData_t* block_ptr, uint8_t* data_ptr, uint32_t* data_byte_size_ptr);



#endif

