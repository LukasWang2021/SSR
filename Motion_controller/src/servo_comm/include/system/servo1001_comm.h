#ifndef SERVO1001_COMM_H
#define SERVO1001_COMM_H

/**
 * @file servo1001_comm.h
 * @brief The file defines the object of servo1001 device.
 * @author youming.wu
 */

#include "system/servo_comm_base.h"
#include "system/servo_cpu_comm_base.h"
#include "common_datatype.h"
/**
 * @brief servo_comm_space includes all servo device related implementation.
 */
namespace servo_comm_space
{
/**
 * @brief Servo1001 is the abstract device of the real servo1001 device. 
 * @details A Servo1001 device consists of 1 servo cpu 10 servo device. 
 */
class Servo1001
{
public:
    /**
     * @brief Constructor of the class.
     * @param [in] controller_id CPU ID of controller which controls the device.
     * @param [in] servo_id CPU ID of servo cpu which the device is on.
     */
    Servo1001(int32_t controller_id, int32_t servo_id);
    /**
     * @brief Destructor of the class.
     */     
    ~Servo1001();
    /**
     * @brief Check if the servo1001 device is constructed correctly.
     * @retval true The servo1001 device is constructed correctly.
     * @retval false The servo1001 device is not constructed correctly.
     */
    bool isValid(void);
    /**
     * @brief Get the handler of the servo cpu.
     * @return Pointer of the configuration object of the servo cpu.
     */    
    ServoCpuCommBase* getCpuCommPtr(void);
    /**
     * @brief Get the handler of some servo by servo index.
     * @param [in] servo_index Servo index in range 0~15.
     * @return Pointer of the configuration object of some servo.
     */     
    ServoCommBase* getServoCommPtr(size_t servo_index);

    /**
     * @brief Config settings when the controller ask the servo transfer its communication state from INIT to PREOP.
     * @param [in] from_block_ptr Pointer of the list of communication block data for all from channels.
     * @param [in] from_block_number Store the number of communication block data in the from list.
     * @param [in] to_block_ptr Pointer of the list of communication block data for all to channels.
     * @param [in] to_block_number Store the number of communication block data in the to list.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    bool prepareInit2PreOp(CommBlockData_t* from_block_ptr, size_t from_block_number, CommBlockData_t* to_block_ptr, size_t to_block_number);
    /**
     * @brief Config settings when the controller ask the servo transfer its communication state from PREOP to SAFEOP.
     * @param [in] to_block_ptr Pointer of the list of communication block data for all to channels.
     * @param [in] to_block_number Store the number of communication block data in the to list.
     * @param [in] fdb_pdo_app_id Application ID for the expected feedback channel in circle buffer type.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    bool preparePreOp2SafeOp(CommBlockData_t* to_block_ptr, size_t to_block_number);
    /**
     * @brief Config settings when the controller ask the servo transfer its communication state from SAFEOP to OP.
     * @param [in] from_block_ptr Pointer of the list of communication block data for all from channels.
     * @param [in] from_block_number Store the number of communication block data in the from list.
     * @param [in] ctrl_pdo_app_id Application ID for the expected control channel in circle buffer type.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
    bool prepareSafeOp2Op(CommBlockData_t* from_block_ptr, size_t from_block_number);
    
    /**
     * @brief Initialize the servo cpu communication configuration object.
     * @param [in] from_block_ptr Pointer of the list of communication block data for all from channels.
     * @param [in] from_block_number Store the number of communication block data in the from list.
     * @param [in] to_block_ptr Pointer of the list of communication block data for all to channels.
     * @param [in] to_block_number Store the number of communication block data in the to list.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */   
    bool initServoCpuComm(CommBlockData_t* from_block_ptr, size_t from_block_number, CommBlockData_t* to_block_ptr, size_t to_block_number);

    /**
     * @brief Command servo to transfer communication state.
     * @details Servo response is expected.\n
     * @param [in] expected_state Expected communication state.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    bool doServoCmdTransCommState(CoreCommState_e expected_state);

    /**
     * @brief Check all servos in the expected state.
     * @param [in] expected_comm_state The specified expected state to check.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */ 
    bool isAllServosInExpectedCommState(CoreCommState_e expected_comm_state);

private:

    bool is_valid_;                 /**< Flag to show if the Servo1001 is valid.*/
    ServoCpuCommBase* cpu_ptr_;     /**< Pointer of the configuration object of the servo cpu.*/
    ServoCommBase* servo_ptr_[AXIS_NUM];   /**< Pointer of the configuration object of some servo.*/
        
    Servo1001();
};

}

#endif

