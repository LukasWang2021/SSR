#ifndef SERVO_CPU_COMM_BASE_H
#define SERVO_CPU_COMM_BASE_H

/**
 * @file servo_cpu_comm_base.h
 * @brief The file defines the object of a servo cpu.
 * @author zhengyu.shen
 */

#include "common/servo_cpu_interface.h"
#include "common_error_code.h"
#include "common/comm_reg_1.h"
#include "common/buffer_2000.h"
#include <string>
#include <vector>
/**
 * @brief servo_comm_space includes all servo device related implementation.
 */
namespace servo_comm_space
{
/*
 * @brief Defines communication configuration of a servo cpu. 
 */
typedef struct
{
	int32_t comm_reg_id;           /**< Application ID of the register channel of the servo cpu.*/ 
	int32_t sampling_buffer_id;    /**< Application ID of the sampling buffer channel of the servo cpu.*/
}ServoCpuCommInfo_t;

typedef enum
{
    CONTROL_MODE_POSITION         = 0,
    CONTROL_MODE_FORCE            = 1,
}ServoControlMode;

/**
 * @brief ServoCpuCommBase is the object to handle the servo cpu communication on controller side.
 */
class ServoCpuCommBase
{
public:
    /**
     * @brief Constructor of the class.
     * @param [in] controller_id CPU id of controller.
     * @param [in] servo_id CPU id of the servo cpu.
     */
    ServoCpuCommBase(int32_t controller_id, int32_t servo_id);
    /**
     * @brief Destructor of the class.
     */    
    ~ServoCpuCommBase();    
    /**
     * @brief Initialize the communication configuration object.
     * @param [in] from_block_ptr Pointer of the list of communication block data for all from channels.
     * @param [in] from_block_number Store the number of communication block data in the from list.
     * @param [in] to_block_ptr Pointer of the list of communication block data for all to channels.
     * @param [in] to_block_number Store the number of communication block data in the to list.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    bool init(CommBlockData_t* from_block_ptr, size_t from_block_number, CommBlockData_t* to_block_ptr, size_t to_block_number);
    /**
     * @brief Get the major version of the servo cpu.
     * @return Major version of the servo cpu.
     */
    uint32_t getMajorVersion();
    /**
     * @brief Get the minor version of the servo cpu.
     * @return Minor version of the servo cpu.
     */
    uint32_t getMinorVersion();
    /**
     * @brief Set pdo synchronization control word.
     * @param [in] index The index of the pdo synchronization control word.
     * @param [in] ctrl_pdo_sync The value of the pdo synchronization controlword.
     * @return void
     */
    void setCtrlPdoSync(uint32_t index, uint32_t ctrl_pdo_sync);
    /**
     * @brief Get pdo synchronization control word.
     * @param [in] index The index of the pdo synchronization control word.
     * @return The value of the pdo synchronization control word.
     */
    uint32_t getCtrlPdoSync(uint32_t index);
    /**
     * @brief Set sampling synchronization control word.
     * @param [in] sampling_sync The value of sampling synchronization control word.
     * @return void
     */
    void setSamplingSync(uint32_t sampling_sync);
    /**
     * @brief Get sampling synchronization control word.
     * @return The value of sampling synchronization control word.
     */
    uint32_t getSamplingSync();
    /**
     * @brief Active sampling configuration.
     * @details Set and keep the sampling configuration word to 1 and reset it to 0 at the end.
     * @param [in] pulse_width The time span to hold the sampling configuration word in value 1.
     * @return void
     */
    void enableSamplingCfg(uint32_t pulse_width = 10000);
    /**
     * @brief Set sampling interval word.
     * @param [in] sampling_interval The value of the sampling interval word, unit in us.
     * @return void
     */   
    void setSamplingInterval(uint32_t sampling_interval);
    /**
     * @brief Get sampling interval word.
     * @return The value of the sampling interval word, unit in us.
     */
    uint32_t getSamplingInterval();
    /**
     * @brief Set sampling max times word.
     * @param [in] sampling_max_times The value of the max times word.
     * @return void
     */    
    void setSamplingMaxTimes(uint32_t sampling_max_times);
    /**
     * @brief Get sampling max times word.
     * @return The value of the max times word.
     */
    uint32_t getSamplingMaxTimes();
    /**
     * @brief Set sampling channel configuration word.
     * @details The channel configuration word is in compose of two parts.\n
     *          The MSB16bit is the servo index, the LSB16bit the servo parameter index.\n
     * @param [in] channel_index The index of sampling channel.
     * @param [in] channel_value The value of sampling channel configuration.
     * @return void
     */    
    void setSamplingChannel(uint32_t channel_index, uint32_t channel_value);    
    /**
     * @brief Get all sampling channel configuration words.
     * @details The channel configuration word is in compose of two parts.\n
     *          The MSB16bit is the servo index, the LSB16bit the servo parameter index.\n
     * @return The all sampling channel configurations.
     */
    std::vector<uint32_t> getSamplingChannel();
    /**
     * @brief Copy all valid sampling data from sampling buffer channel to local memory.
     * @param [in] data_ptr Pointer of the local memory to store sampling data.
     * @param [in] data_byte_size_ptr Byte size of valid sampling data.
     * @return void
     */ 
    void getSamplingBufferData(uint8_t* data_ptr, int32_t* data_byte_size_ptr);
    /**
     * @brief Get configuration of the servo cpu communication.
     * @param [out] info Configuration of the servo cpu communication.
     * @return void
     */
	void getServoCpuCommInfo(ServoCpuCommInfo_t* info);

     /**
     * @brief Ask servo switch to force/position control mode.
     * @return void
     */    
    void setServoControlMode(ServoControlMode control_mode);
    /**
     * @brief Get the servo control mode.
     * @return The value of the control mode.
     */  
    uint32_t getServoControlMode();


private:
    ServoCpuCommBase();
    ServoCpuComm_t* comm_ptr_;  /**< Servo cpu communication configuration object.*/
};

}


#endif

