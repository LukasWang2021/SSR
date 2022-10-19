#ifndef COMM_REG_3_H
#define COMM_REG_3_H

/*
 * @file comm_reg_3.h
 * @brief 这个文件定义了力矩传感器数据标定接口
 * @author xzc
 */
#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/core_comm_datatype.h"
#include <stdint.h>
#else
#include "common/core_comm_datatype.h"
#include <stdint.h>
#endif
#define COMM_REG3_DATA_NUMBER 6 //力矩数量6
#define COMM_REG3_UPDATE_FLAG_PTR     ((int8_t*)(block_ptr->memory_ptr))         /**< The address to store the updateing flag.*/
#define COMM_REG3_DATA_PTR            ((int8_t*)(block_ptr->memory_ptr + 4))     /**< The address to store the torque data.*/
typedef struct
{
    int32_t data[COMM_REG3_DATA_NUMBER];//力矩数据数组e
}CommRegTorqueData_t;
typedef struct
{
    uint32_t update_flag;    /**< Indicates the the parameters are updated.*/
    CommRegTorqueData_t F_torque;
}CommRegAppData3_t;
/* @brief Initialize the register 3 channel.
 * @details block_ptr.param1~block_ptr.param8 is not used.
 * @param [in] block_ptr Pointer of the configuration data of the channe3.*/
void initCommReg3(CommBlockData_t* block_ptr);
/* @brief Set the flag to indicate the update Reg3
 * @param [in] block_ptr Pointer of the configuration data of the channe3.
 * @param [in] index The index of the flag array.
 * @param [in] value The value of the flag.*/
bool setCommReg3UpdateFlag(CommBlockData_t* block_ptr, uint32_t value);
/* @brief Get the value of the flag.
 * @param [in] block_ptr Pointer of the configuration data of the channe3.
 * @param [out] value_ptr Pointer to the value of the flag.*/
bool getCommReg3UpdateFlag(CommBlockData_t* block_ptr, uint32_t* value_ptr);

/* @brief get the Reg3
 * @param [in] block_ptr Pointer of the configuration data of the channe3.
 * @param [out] data_ptr Pointer of local memeory which is used to store the data.
 * @param [out] data_byte_size_ptr Pointer of the variable to stored the valid byte size of data that is copied.*/
bool getCommReg3Data(CommBlockData_t* block_ptr, uint8_t* data_ptr, uint32_t* data_byte_size_ptr);
#endif

