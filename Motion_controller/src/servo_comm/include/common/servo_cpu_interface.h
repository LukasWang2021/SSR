#ifndef SERVO_CPU_INTERFACE_H
#define SERVO_CPU_INTERFACE_H

/**
 * @file servo_cpu_interface.h
 * @brief The file includes basic interfaces for operating servo cpu for both controller and servo. 
 * @author zhengyu.shen
 */

#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/core_comm_datatype.h"
#include "./core_protocal_inc/comm_reg_1.h"
#include "./core_protocal_inc/comm_reg_2.h"
#include <stdint.h>
#else
#include "common/core_comm_datatype.h"
#include "common/comm_reg_1.h"
#include "common/comm_reg_2.h"
#include "common/comm_reg_3.h"
#include <stdint.h>
#endif


#define SERVO_CPU_COMM_APP_ID_COMM_REG  1       /**< The application id of the register channel for Servo CPU communication.*/
#define SERVO_CPU_COMM_APP_ID_SAMPLING  2000    /**< The application id of the buffer channel for Servo CPU sampling.*/
#define SERVO_CPU_COMM_APP_ID_PARAM_REG  2      /**< The application id of the register channel for external parameter communication.*/
#define SERVO_CPU_COMM_APP_ID_TSD_REG  3  //Torque Sensor Data---力矩传感器数据寄存器
/**
 * @brief Defines configuration of communication channels of a servo cpu.
 */
typedef struct
{
    int32_t from;   /**< CPU id of the master.*/ 
    int32_t to;     /**< CPU id of the servo cpu.*/
    CommBlockData_t* comm_reg_ptr;          /**< Pointer of register channel.*/
    CommBlockData_t* sampling_buffer_ptr;   /**< Pointer of sampling buffer channel.*/
    CommBlockData_t* param_reg_ptr;         /**< Pointer of the external parameter channel.*/
    CommBlockData_t* torque_reg_ptr;            /** <Pointer of the Torque sensor data channel.*/
}ServoCpuComm_t;
/**
 * @brief Create configuration object to handling servo cpu communication on controller side.
 * @details The returned object pointer should be freed if some one don't use it anymore.\n
 *          The API should only be called on the controller side.\n
 * @param [in] controller_id CPU id of controller.
 * @param [in] servo_id CPU id of the servo cpu.
 * @return Pointer of the configuration object.
 */
ServoCpuComm_t* createServoCpuCommByController(int32_t controller_id, int32_t servo_id);
/**
 * @brief Initialize the configuration object of a servo cpu on controller side.
 * @details Search the from and to lists of communication block data and select the matching block data to initialize the configuration object.\n
 *          The API should only be called on the controller side.\n
 * @param [in] comm_ptr Pointer of the configuration object.
 * @param [in] from_block_ptr Pointer of the list of communication block data for all from channels.
 * @param [in] from_block_number Store the number of communication block data in the from list.
 * @param [in] to_block_ptr Pointer of the list of communication block data for all to channels.
 * @param [in] to_block_number Store the number of communication block data in the to list.
 * @retval true Operation succeed.
 * @retval false Operation failed.
 */
bool initServoCpuCommByController(ServoCpuComm_t* comm_ptr,
                                            CommBlockData_t* from_block_ptr, size_t from_block_number,
                                            CommBlockData_t* to_block_ptr, size_t to_block_number); 
/**
 * @brief Get the major version of a servo cpu.
 * @details The API should only be called on the controller side.\n
 * @param [in] comm_ptr The configuration object of a servo cpu.
 * @return Major version of a servo cpu.
 */
uint32_t getServoCpuCommMajorVersion(ServoCpuComm_t* comm_ptr);
/**
 * @brief Get the minor version of a servo cpu.
 * @details The API should only be called on the controller side.\n
 * @param [in] comm_ptr The configuration object of a servo cpu.
 * @return Minor version of a servo cpu.
 */
uint32_t getServoCpuCommMinorVersion(ServoCpuComm_t* comm_ptr);
/**
 * @brief Set pdo synchronization control word.
 * @details The API should only be called on the controller side.\n
 * @param [in] comm_ptr The configuration object of a servo cpu.
 * @param [in] index The index of the pdo synchronization control word.
 * @param [in] ctrl_pdo_sync The value of the pdo synchronization controlword.
 * @return void
 */
void setServoCpuCommCtrlPdoSync(ServoCpuComm_t* comm_ptr, uint32_t index, uint32_t ctrl_pdo_sync);
/**
 * @brief Set sampling configuration control word.
 * @details The API should only be called on the controller side.\n
 * @param [in] comm_ptr The configuration object of a servo cpu.
 * @param [in] sampling_cfg The value of the sampling configuration control word.
 * @return void
 */
void setServoCpuCommSamplingCfg(ServoCpuComm_t* comm_ptr, uint32_t sampling_cfg);
/**
 * @brief Set sampling interval word.
 * @details The API should only be called on the controller side.\n
 * @param [in] comm_ptr The configuration object of a servo cpu.
 * @param [in] sampling_interval The value of the sampling interval word, unit in us.
 * @return void
 */
void setServoCpuCommSamplingInterval(ServoCpuComm_t* comm_ptr, uint32_t sampling_interval);
/**
 * @brief Set sampling max times word.
 * @details The API should only be called on the controller side.\n
 * @param [in] comm_ptr The configuration object of a servo cpu.
 * @param [in] sampling_max_times The value of the max times word.
 * @return void
 */
void setServoCpuCommSamplingMaxTimes(ServoCpuComm_t* comm_ptr, uint32_t sampling_max_times);
/**
 * @brief Set sampling channel configuration word.
 * @details The channel configuration word is in compose of two parts.\n
 *          The MSB16bit is the servo index, the LSB16bit the servo parameter index.\n
 *          The API should only be called on the controller side.\n
 * @param [in] comm_ptr The configuration object of a servo cpu.
 * @param [in] channel_index The index of sampling channel.
 * @param [in] channel_value The value of sampling channel configuration.
 * @return void
 */
void setServoCpuCommSamplingChannel(ServoCpuComm_t* comm_ptr, uint32_t channel_index, uint32_t channel_value);
/**
 * @brief Copy all valid sampling data from sampling buffer channel to local memory.
 * @details The API should only be called on the controller side.\n
 * @param [in] comm_ptr The configuration object of a servo cpu.
 * @param [in] data_ptr Pointer of the local memory to store sampling data.
 * @param [in] data_byte_size_ptr Byte size of valid sampling data.
 * @return void
 */
void getServoCpuCommSamplingBuffer(ServoCpuComm_t* comm_ptr, uint8_t* data_ptr, int32_t* data_byte_size_ptr);

/**
 * @brief Create configuration object to handling servo cpu communication on servo side.
 * @details On servo side, it is not necessary to known who connects to the servo cpu so that only servo cpu id is needed.\n
 *          The returned object pointer should be freed if some one don't use it anymore.\n
 *          The API should only be called on the servo side.\n
 * @param [in] servo_id CPU id of servo.
 * @return Pointer of the configuration object.
 */
ServoCpuComm_t* createServoCpuCommByServo(int32_t servo_id);
/**
 * @brief Initialize the configuration object of a servo cpu on servo side.
 * @details Search the from and to lists of communication block data and select the matching block data to initialize the configuration object.\n
 *          The API should only be called on the servo side.\n
 * @param [in] comm_ptr Pointer of the configuration object.
 * @param [in] from_block_ptr Pointer of the list of communication block data for all channels.
 * @param [in] from_block_number Store the number of communication block data in the from list.
 * @param [in] to_block_ptr Pointer of the list of communication block data for all channels.
 * @param [in] to_block_number Store the number of communication block data in the to list.
 * @return void
 */
bool initServoCpuCommByServo(ServoCpuComm_t* comm_ptr,
                                    CommBlockData_t* from_block_ptr, size_t from_block_number,
                                    CommBlockData_t* to_block_ptr, size_t to_block_number);
/**
 * @brief Get sampling configuration control word.
 * @details The API should only be called on the servo side.\n
 * @param [in] comm_ptr The configuration object of a servo cpu.
 * @return The value of the sampling configuration control word.
 */
uint32_t getServoCpuCommSamplingCfg(ServoCpuComm_t* comm_ptr);
/**
 * @brief Get sampling interval word.
 * @details The API should only be called on the servo side.\n
 * @param [in] comm_ptr The configuration object of a servo cpu.
 * @return The value of the sampling interval word, unit in us.
 */
uint32_t getServoCpuCommSamplingInterval(ServoCpuComm_t* comm_ptr);
/**
 * @brief Get sampling max times word.
 * @details The API should only be called on the servo side.\n
 * @param [in] comm_ptr The configuration object of a servo cpu.
 * @return The value of the max times word.
 */
uint32_t getServoCpuCommSamplingMaxTimes(ServoCpuComm_t* comm_ptr);
/**
 * @brief Get sampling channel configuration word.
 * @details The channel configuration word is in compose of two parts.\n
 *          The MSB16bit is the servo index, the LSB16bit the servo parameter index.\n
 *          The API should only be called on the servo side.\n
 * @param [in] comm_ptr The configuration object of a servo cpu.
 * @param [in] channel_index The index of sampling channel.
 * @return The value of sampling channel configuration.
 */
uint32_t getServoCpuCommSamplingChannel(ServoCpuComm_t* comm_ptr, uint32_t channel_index);
/**
 * @brief Set the major version of a servo cpu.
 * @details The API should only be called on the servo side.\n
 * @param [in] comm_ptr The configuration object of a servo cpu.
 * @param [in] major_version Major version of a servo cpu.
 * @return void
 */
void setServoCpuCommMajorVersion(ServoCpuComm_t* comm_ptr, uint32_t major_version);
/**
 * @brief Set the minor version of a servo cpu.
 * @details The API should only be called on the servo side.\n
 * @param [in] comm_ptr The configuration object of a servo cpu.
 * @param [in] minor_version Minor version of a servo cpu.
 * @return void
 */
void setServoCpuCommMinorVersion(ServoCpuComm_t* comm_ptr, uint32_t minor_version);
/**
 * @brief Get pdo synchronization control word.
 * @details The API can be called on both controller and servo cpu sides.\n
 * @param [in] comm_ptr The configuration object of a servo cpu.
 * @param [in] index The index of the pdo synchronization control word.
 * @return The value of the pdo synchronization control word.
 */
uint32_t getServoCpuCommCtrlPdoSync(ServoCpuComm_t* comm_ptr, uint32_t index);
/**
 * @brief Set sampling synchronization control word.
 * @details The API can be called on both controller and servo cpu sides.\n
 * @param [in] comm_ptr The configuration object of a servo cpu.
 * @param [in] sampling_sync The value of sampling synchronization control word.
 * @return void
 */
void setServoCpuCommSamplingSync(ServoCpuComm_t* comm_ptr, uint32_t sampling_sync);
/**
 * @brief Get sampling synchronization control word.
 * @details The API can be called on both controller and servo cpu sides.\n
 * @param [in] comm_ptr The configuration object of a servo cpu.
 * @return The value of sampling synchronization control word.
 */
uint32_t getServoCpuCommSamplingSync(ServoCpuComm_t* comm_ptr);

/**
 * @brief Set the control mode
 * @details The API should only be called on the controller side.\n
 * @param [in] comm_ptr The configuration object of a servo cpu.
 * @param [in] control_mode The value of the control mode.
 * @return void
 */
void setServoCpuCommControlMode(ServoCpuComm_t* comm_ptr, uint32_t control_mode);
/**
 * @brief Get the control mode.
 * @details The API should only be called on the servo cpu side.\n
 * @param [in] comm_ptr The configuration object of a servo cpu.
 * @return The control value.
 */
uint32_t getServoCpuCommControlMode(ServoCpuComm_t* comm_ptr);

bool setServoCpuCommForceControlUpdateFlag(ServoCpuComm_t* comm_ptr, uint32_t value);
bool getServoCpuCommForceControlUpdateFlag(ServoCpuComm_t* comm_ptr, uint32_t* value_ptr);
bool setServoCpuCommForceControlParameters(ServoCpuComm_t* comm_ptr, uint8_t* data_ptr, uint32_t data_byte_size);
bool getServoCpuCommForceControlParameters(ServoCpuComm_t* comm_ptr, uint8_t* data_ptr, uint32_t* data_byte_size_ptr);

bool setServoCpuCommTorqueSensorUpdateFlag(ServoCpuComm_t* comm_ptr, uint32_t value);
bool getServoCpuCommTorqueSensorUpdateFlag(ServoCpuComm_t* comm_ptr, uint32_t* value_ptr);
bool getServoCpuCommTorqueSensorData(ServoCpuComm_t* comm_ptr, uint8_t* data_ptr, uint32_t* data_byte_size_ptr);
/**
 * @brief Free the configuration object of a servo cpu.
 * @details The API can be called on both controller and servo cpu sides.\n
 * @param [in] comm_ptr The configuration object of a servo cpu. 
 * @return void
 */
void freeServoCpuComm(ServoCpuComm_t* comm_ptr);


#endif

