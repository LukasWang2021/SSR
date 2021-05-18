#ifndef AXIS_FDB_H
#define AXIS_FDB_H

/**
 * @file axis_fdb.h
 * @brief The file is the header file of class "AxisFdb".
 * @author Feng.Wu
 */
 
#include <vector>
#include <string.h>  
#include "common_error_code.h"
#include "system/servo_comm_base.h"
#include "common/core_comm_servo_datatype.h"
#include "axis_datatype.h"
#include "log_manager_producer.h"

/**
 * @brief axis_space includes all axis related definitions and implementation.
 */
namespace axis_space {

/**
 * @brief AxisFdb handles the feedback of the servo.
 * @details
 */
class AxisFdb {
  public:
    /**
     * @brief Constructor of the class.
     */
    AxisFdb(void);
    /**
     * @brief Destructor of the class. 
     */ 
    ~AxisFdb(void);

    /**
     * @brief Initializatin.
     * @details Choose the function to hanle the feedback data according to the fdb_application_id got from servo_comm_ptr.\n
     *          The feedback data structure might be different upon the applications.
     * @param [in] id The reference of the axis.
     * @param [in] servo_comm_ptr The pointer to communicate with servos.
     * @retval true success.
     * @retval false Failed to initialize.
     */
    bool init(int32_t id, servo_comm_space::ServoCommBase* servo_comm_ptr);

    /**
     * @brief To update the feedback of the servo.
     * @details 
     * @param [out] current_time_stamp_ptr Get the current time stamp of the current feedback data.
     * @return void.
     */
    void processFdbPdoCurrent(uint32_t* current_time_stamp_ptr);

    /**
     * @brief To update the feedback of the servo.
     * @details It is called after processFdbPdoCurrent.\n
     *          The parameter input is from processFdbPdoCurrent in order to guarantee the feedback synchronization.\n
     * @param [in] expect_time_stamp Get the specified feedback data according to the specified time stamp.
     * @return void.
     */
    void processFdbPdoSync(uint32_t expect_time_stamp);

    /**
     * @brief Read the actual position.
     * @return The value of the servo position. Unit: pulse.
     */
	int64_t getServoPosition(void);

    /**
     * @brief Read the actual velocity.
     * @return The value of the servo velocity. Unit: revolutions/s.
     */
	int32_t getServoVelocity(void);

    /**
     * @brief Read the actual torque.
     * @return The value of the servo torque. Unit: 0.01N*m.
     */
	int32_t getServoTorque(void);

    /**
     * @brief Get the stutus of the servo.
     * @return ServoSm_e 
     * - SERVO_SM_START = 0,
     * - SERVO_SM_NOT_READY_TO_SWITCH_ON = 1,
     * - SERVO_SM_SWITCH_ON_DISABLED = 2,
     * - SERVO_SM_READY_TO_SWITCH_ON = 3,
     * - SERVO_SM_SWITCHED_ON = 4,
     * - SERVO_SM_OPERATION_ENABLED = 5,
     * - SERVO_SM_QUICK_STOP_ACTIVE = 6,
     * - SERVO_SM_FAULT_REACTION_ACTIVE = 7,
     * - SERVO_SM_FAULT = 8,
     * - SERVO_SM_UNKNOWN = 9,
     */
	ServoSm_e getServoState(void);

    /**
     * @brief Get the operation mode of the servo.
     * @return ServoOpMode_e 
     * - SERVO_OP_MODE_NO_MODE = 0,
     * - SERVO_OP_MODE_PROFILE_POSITION_MODE = 1,
     * - SERVO_OP_MODE_VELOCITY_MODE = 2,
     * - SERVO_OP_MODE_PROFILE_VELOCITY_MODE = 3,
     * - SERVO_OP_MODE_TORQUE_PROFILE_MODE = 4,
     * - SERVO_OP_MODE_HOMING_MODE = 6,
     * - SERVO_OP_MODE_INTERPOLATED_POSITION_MODE = 7,
     * - SERVO_OP_MODE_CYCLIC_SYNC_POSITION_MODE = 8,
     * - SERVO_OP_MODE_CYCLIC_SYNC_VELOCITY_MODE = 9,
     * - SERVO_OP_MODE_CYCLIC_SYNC_TORQUE_MODE = 10,
     */
    ServoOpMode_e getServoOpMode(void);

    int32_t getEncoderState(void);

    /**
     * @brief Check if the target position is reached.
     * @retval false The target position is not reached.
     * @retval true The target position is reached.
     */
	bool isTargetReached(void);

    /**
     * @brief Check if the homing option is complete successfully.
     * @retval false The homing position is failed.
     * @retval true The homing position is OK.
     */
    bool isHomingSuccess(void);

    /**
     * @brief Check if the homing procedure is in progress.
     * @retval false Not in progress.
     * @retval true In progress.
     */
    bool isHoming(void);

    /**
     * @brief Get the pointer of the feedback PDO.
     * @param [out] size The valid size of the data.
     * @return Pointer of the data.
     */
	uint8_t* getFdbPdoPtr(int32_t* size);
	
  
  private:    
  	servo_comm_space::ServoCommBase* servo_comm_ptr_;
	void* pdo_ptr_;
	int32_t pdo_size_;
	int32_t fdb_pdo_id_;
	ServoSm_e servo_state_;
	
	int64_t position_;
    int64_t cmd_position_;
	int32_t velocity_;
	int32_t torque_;
    ServoState_u state_word_;
    ServoOpMode_e servo_op_mode_;
    int32_t encoder_state_;
    int32_t digital_input_;

    int32_t stepper_currentA_;
    int32_t stepper_currentB_;

    int32_t id_;

    typedef void (AxisFdb::*HandleFdbCurrentFuncPtr)(uint32_t* current_time_stamp_ptr);
    typedef void (AxisFdb::*HandleFdbSyncFuncPtr)(uint32_t expect_time_stamp);
	typedef struct
	{
	    int32_t app_id;
        HandleFdbCurrentFuncPtr fdb_current_func_ptr;
        HandleFdbSyncFuncPtr fdb_sync_func_ptr;
	}FdbPdo;
	std::vector<FdbPdo> fdb_table_;
    HandleFdbCurrentFuncPtr fdb_current_func_ptr_;
    HandleFdbSyncFuncPtr fdb_sync_func_ptr_;
	void initFdbTable(void);

	void handleFdbCurrentCircleBuffer3000(uint32_t* current_time_stamp_ptr);
    void handleFdbSyncCircleBuffer3000(uint32_t expect_time_stamp);

    //handle servo feedback
    void handleFdbCurrentCircleBuffer3001(uint32_t* current_time_stamp_ptr);
    void handleFdbSyncCircleBuffer3001(uint32_t expect_time_stamp);

    //handle stepper feedback
    void handleFdbCurrentCircleBuffer3002(uint32_t* current_time_stamp_ptr);
    void handleFdbSyncCircleBuffer3002(uint32_t expect_time_stamp);

};

}
#endif
