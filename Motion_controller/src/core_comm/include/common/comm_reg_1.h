#ifndef COMM_REG_1_H
#define COMM_REG_1_H

/**
 * @file comm_reg_1.h
 * @brief The file includes the definition of the register channel for standard servo cpu communication.
 * @author zhengyu.shen
 */

#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/core_comm_datatype.h"
#include <stdint.h>
#else
#include "common/core_comm_datatype.h"
#include <stdint.h>
#endif

#define COMM_REG1_SAMPLING_CHANNEL_NUMBER 16
#define COMM_REG1_CTRL_PDO_SYNC_NUMBER 16

#define COMM_REG1_MAJOR_VERSION_PTR                  ((int8_t*)(block_ptr->memory_ptr))         /**< The address to store the major version of servo.*/
#define COMM_REG1_MINOR_VERSION_PTR                  ((int8_t*)(block_ptr->memory_ptr + 4))     /**< The address to store the minor version of servo.*/
#define COMM_REG1_SAMPLING_SYNC_PTR                  ((int8_t*)(block_ptr->memory_ptr + 8))     /**< The address to store the sampling synchronization control word.*/
#define COMM_REG1_SAMPLING_CFG_PTR                   ((int8_t*)(block_ptr->memory_ptr + 12))    /**< The address to store the sampling configuration control word.*/
#define COMM_REG1_SAMPLING_INTERVAL_PTR              ((int8_t*)(block_ptr->memory_ptr + 16))    /**< The address to store the sampling interval, unit in us.*/
#define COMM_REG1_SAMPLING_MAX_TIMES_PTR             ((int8_t*)(block_ptr->memory_ptr + 20))    /**< The address to store the max sampling times.*/
#define COMM_REG1_SAMPLING_CHANNEL_PTR               ((int8_t*)(block_ptr->memory_ptr + 24))    /**< The address to store the configuration of 16 sampling channels.*/
#define COMM_REG1_CTRL_PDO_SYNC_PTR                  ((int8_t*)(block_ptr->memory_ptr + 88))    /**< The address to store the pdo synchronization control words.*/
#define COMM_REG1_CONTROL_MODE_PTR               ((int8_t*)(block_ptr->memory_ptr + 152))

/**
 * @brief Defines the data structure for standard servo cpu level communication.
 * @details The register section provides various functions including version, sampling and pdo synchronization.\n
 */
typedef struct
{
    uint32_t major_version;         /**< The major version of servo.*/
    uint32_t minor_version;         /**< The minor version of servo.*/
    uint32_t sampling_sync;         /**< The sampling synchronization control word.*/
    uint32_t sampling_cfg;          /**< The sampling configuration control word.*/
    uint32_t sampling_interval;     /**< The sampling interval, unit in us.*/
    uint32_t sampling_max_times;    /**< The max sampling times.*/
    uint32_t sampling_channel[COMM_REG1_SAMPLING_CHANNEL_NUMBER];   /**< The configurations of sampling channel.*/ 
    uint32_t ctrl_pdo_sync[COMM_REG1_CTRL_PDO_SYNC_NUMBER];         /**< The pdo sync control words.*/
    uint32_t control_mode;           /**< The control words to define position or force.*/
}CommRegAppData1_t;

/**
 * @brief Initialize the register 1 channel.
 * @details block_ptr.param1~block_ptr.param8 is not used.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @return void
 */
void initCommReg1(CommBlockData_t* block_ptr);
/**
 * @brief Set servo major version.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [in] major_version Major version of the servo device.
 * @return void
 */
void setCommReg1MajorVersion(CommBlockData_t* block_ptr, uint32_t major_version);
/**
 * @brief Get servo major version.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [out] major_version_ptr Pointer to major version of the servo device.
 * @return void
 */
void getCommReg1MajorVersion(CommBlockData_t* block_ptr, uint32_t* major_version_ptr);
/**
 * @brief Set servo minor version.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [in] minor_version Minor version of the servo device.
 * @return void
 */
void setCommReg1MinorVersion(CommBlockData_t* block_ptr, uint32_t minor_version);
/**
 * @brief Get servo minor version.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [out] minor_version_ptr Pointer to minor version of the servo device.
 * @return void
 */
void getCommReg1MinorVersion(CommBlockData_t* block_ptr, uint32_t* minor_version_ptr);
/**
 * @brief Set pdo synchronization control word.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [in] index The index of control word, in range of 0~7.
 * @param [in] ctrl_pdo_sync The value of the control word.
 * @return void
 */
void setCommReg1CtrlPdoSync(CommBlockData_t* block_ptr, uint32_t index, uint32_t ctrl_pdo_sync);
/**
 * @brief Get pdo synchronization control word.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [in] index The index of control word, in range of 0~7.
 * @param [out] ctrl_pdo_sync_ptr Pointer to the value of the control word.
 * @return void
 */
void getCommReg1CtrlPdoSync(CommBlockData_t* block_ptr, uint32_t index, uint32_t* ctrl_pdo_sync_ptr);
/**
 * @brief Set sampling synchronization control word.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [in] sampling_sync The value of the control word.
 * @return void
 */
void setCommReg1SamplingSync(CommBlockData_t* block_ptr, uint32_t sampling_sync);
/**
 * @brief Get sampling synchronization control word.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [out] sampling_sync_ptr Pointer to the value of the control word.
 * @return void
 */
void getCommReg1SamplingSync(CommBlockData_t* block_ptr, uint32_t* sampling_sync_ptr);
/**
 * @brief Set sampling configuration control word.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [in] sampling_cfg The value of the control word.
 * @return void
 */
void setCommReg1SamplingCfg(CommBlockData_t* block_ptr, uint32_t sampling_cfg);
/**
 * @brief Get sampling configuration control word.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [out] sampling_cfg_ptr Pointer to the value of the control word.
 * @return void
 */
void getCommReg1SamplingCfg(CommBlockData_t* block_ptr, uint32_t* sampling_cfg_ptr);
/**
 * @brief Set sampling interval.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [in] sampling_interval The sampling interval time in us.
 * @return void
 */
void setCommReg1SamplingInterval(CommBlockData_t* block_ptr, uint32_t sampling_interval);
/**
 * @brief Get sampling interval.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [out] sampling_interval_ptr Pointer to the value of the sampling interval time in us.
 * @return void
 */
void getCommReg1SamplingInterval(CommBlockData_t* block_ptr, uint32_t* sampling_interval_ptr);
/**
 * @brief Set sampling maximum times.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [in] sampling_max_times The sampling maximum times.
 * @return void
 */
void setCommReg1SamplingMaxTimes(CommBlockData_t* block_ptr, uint32_t sampling_max_times);
/**
 * @brief Get sampling maximum times.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [out] sampling_max_times_ptr Pointer to the value of the sampling maximum times.
 * @return void
 */
void getCommReg1SamplingMaxTimes(CommBlockData_t* block_ptr, uint32_t* sampling_max_times_ptr);
/**
 * @brief Set sampling channel configuration.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [in] channel_index Index of sampling channel in range of 0~15.
 * @param [in] channel_value The sampling channel configuration.
 * @return void
 */
void setCommReg1SamplingChannel(CommBlockData_t* block_ptr, uint32_t channel_index, uint32_t channel_value);
/**
 * @brief Get sampling channel configuration.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [in] channel_index Index of sampling channel in range of 0~15.
 * @param [out] channel_value_ptr Pointer to the value of the sampling channel configuration.
 * @return void
 */
void getCommReg1SamplingChannel(CommBlockData_t* block_ptr, uint32_t channel_index, uint32_t* channel_value_ptr);

/**
 * @brief Set the control mode.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [in] control_mode The control mode.
 * @return void
 */
void setCommReg1ControlMode(CommBlockData_t* block_ptr, uint32_t control_mode);
/**
 * @brief Get the control mode.
 * @param [in] block_ptr Pointer of the configuration data of the channel.
 * @param [out] control_mode_ptr Pointer to the value of the control mode.
 * @return void
 */
void getCommReg1ControlMode(CommBlockData_t* block_ptr, uint32_t* control_mode_ptr);


#endif

