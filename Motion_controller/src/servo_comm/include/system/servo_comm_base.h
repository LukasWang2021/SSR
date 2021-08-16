#ifndef SERVO_COMM_BASE_H
#define SERVO_COMM_BASE_H

/**
 * @file servo_comm_base.h
 * @brief The file defines the object of a servo.
 * @author zhengyu.shen
 */

#include "common/servo_interface.h"
#include "common_error_code.h"
#include "common/buffer_2001.h"
#include "common/buffer_2002.h"
#include <string>
/**
 * @brief servo_comm_space includes all servo device related implementation.
 */
namespace servo_comm_space
{
/**
 * @brief Parameter indexs of servo that are often used.
 */
typedef enum{
    SERVO_PARAM_ENCODER_OFFSET_LSB = 74,    /**< Encoder offset LSB-32bit.*/
    SERVO_PARAM_ENCODER_OFFSET_MSB = 75,    /**< Encoder offset MSB-32bit.*/
    SERVO_PARAM_CURRENT_ERROR = 149,        /**< Current error code.*/
    SERVO_PARAM_HISTORY_ERROR = 150,        /**< The latest histroy error code.*/
    SERVO_PARAM_OP_MODE = 451,              /**< The expected operation mode.*/
    SERVO_PARAM_OP_MODE_DISPLAY = 459,      /**< The acutal operation mode.*/
    SERVO_PARAM_CTRL_PDO_SYNC_INDEX = 500,  /**< The index of control pdo sync word for the servo.*/
}ServoParam_e;
/*
 * @brief Defines communication configuration of a servo. 
 */
typedef struct
{
    int32_t from;               /**< CPU ID of the controller which controls the servo.*/
    int32_t to;                 /**< CPU ID of the servo cpu which the servo is on.*/
    int32_t servo_index;        /**< Servo index.*/
    int32_t comm_reg_id;        /**< Application ID of the register channel of the servo.*/
	int32_t service_id;         /**< Application ID of the core process call channel of the servo.*/
	int32_t download_param_id;  /**< Application ID of the download buffer channel of the servo.*/
	int32_t upload_param_id;    /**< Application ID of the upload buffer channel of the servo.*/
	int32_t ctrl_pdo_id;        /**< Application ID of the control pdo circle buffer channel of the servo.*/
	int32_t fdb_pdo_id;         /**< Application ID of the feedback pdo circle buffer channel of the servo.*/
}ServoCommInfo_t;
/**
 * @brief ServoCommBase is the object to handle the servo communication on controller side.
 */
class ServoCommBase
{
public:
    /**
     * @brief Constructor of the class.
     * @param [in] controller_id CPU id of controller.
     * @param [in] servo_id CPU id of the servo cpu.
     * @param [in] servo_index Servo index.
     */    
    ServoCommBase(int32_t controller_id, int32_t servo_id, int32_t servo_index);
    /**
     * @brief Destructor of the class.
     */ 
    ~ServoCommBase();    
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
    bool preparePreOp2SafeOp(CommBlockData_t* to_block_ptr, size_t to_block_number, int32_t fdb_pdo_app_id);
    /**
     * @brief Config settings when the controller ask the servo transfer its communication state from SAFEOP to OP.
     * @param [in] from_block_ptr Pointer of the list of communication block data for all from channels.
     * @param [in] from_block_number Store the number of communication block data in the from list.
     * @param [in] ctrl_pdo_app_id Application ID for the expected control channel in circle buffer type.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
    bool prepareSafeOp2Op(CommBlockData_t* from_block_ptr, size_t from_block_number, int32_t ctrl_pdo_app_id);
    /**
     * @brief Get servo communication state.
     * @return Communication state.
     */
    CoreCommState_e getCommState();
    /**
     * @brief Send a SERVO_CMD_SHUT_DOWN command to servo.
     * @details No servo response is expected.\n
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    bool emitServoCmdShutDown();
    /**
     * @brief Send a SERVO_CMD_SWITCH_ON command to servo.
     * @details No servo response is expected.\n
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
    bool emitServoCmdSwitchOn();
    /**
     * @brief Send a SERVO_CMD_DISABLE_VOLTAGE command to servo.
     * @details No servo response is expected.\n
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */     
    bool emitServoCmdDisableVoltage();
    /**
     * @brief Send a SERVO_CMD_ENABLE_OPERATION command to servo.
     * @details No servo response is expected.\n
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
    bool emitServoCmdEnableOperation();
    /**
     * @brief Send a SERVO_CMD_SWITCH_ON_AND_ENABLE_OPERATION command to servo.
     * @details No servo response is expected.\n
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
    bool emitServoCmdSwitchOnAndEnableOperation();
    /**
     * @brief Send a SERVO_CMD_DISABLE_OPERATION command to servo.
     * @details No servo response is expected.\n
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
    bool emitServoCmdDisableOperation();
    /**
     * @brief Send a SERVO_CMD_QUICK_STOP command to servo.
     * @details No servo response is expected.\n
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */     
    bool emitServoCmdQuickStop();
    /**
     * @brief Send a SERVO_CMD_FAULT_RESET command to servo.
     * @details No servo response is expected.\n
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
    bool emitServoCmdFaultReset();   
    /**
     * @brief Send a SERVO_CMD_RESET_FAULT_RESET command to servo.
     * @details No servo response is expected.\n
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
    bool emitServoCmdResetFaultReset();
    /**
     * @brief Command servo to transfer communication state.
     * @details Servo response is expected.\n
     * @param [in] expected_state Expected communication state.
     * @retval SUCCESS Execute the process successfully.
     * @retval CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED Failed to send request.
     * @retval CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED Failed to execute the process.
     */
    ErrorCode doServoCmdTransCommState(CoreCommState_e expected_state);
    /**
     * @brief Read a servo parameter from servo.
     * @details Servo response is expected.\n
     * @param [in] param_index Servo parameter index.
     * @param [in] param_value_ptr Pointer of servo parameter value.
     * @retval SUCCESS Execute the process successfully.
     * @retval CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED Failed to send request.
     * @retval CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED Failed to execute the process.
     */
    ErrorCode doServoCmdReadParameter(int32_t param_index, int32_t* param_value_ptr);
    /**
     * @brief Write a servo parameter to servo.
     * @details Servo response is expected.\n
     * @param [in] param_index Servo parameter index.
     * @param [in] param_value Servo parameter value.
     * @retval SUCCESS Execute the process successfully.
     * @retval CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED Failed to send request.
     * @retval CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED Failed to execute the process.
     */    
    ErrorCode doServoCmdWriteParameter(int32_t param_index, int32_t param_value);
    /**
     * @brief Command servo to move in a specified velocity.
     * @details Servo response is expected.\n
     * @param [in] target_velocity Target velocity, always positive value. Unit in revolutions/s.
     * @param [in] acc Acceleration to the target velocity. Unit in revolutions/s^2.
     * @param [in] dec Deceleration to the target velocity. Unit in revolutions/s^2.
     * @param [in] jerk Jerk. Unit in revolutions/s^3.
     * @param [in] direction The direction of the target velocity. 0 is positive, 1 is negative.
     * @retval SUCCESS Execute the process successfully.
     * @retval CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED Failed to send request.
     * @retval CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED Failed to execute the process.
     */     
    ErrorCode doServoCmdMoveVelocity(int32_t target_velocity, int32_t acc, int32_t dec, int32_t jerk, int32_t direction);
    /**
     * @brief Command servo to move to a absolute position.
     * @details Servo response is expected.\n
     * @param [in] target_position Target position, Unit in pulse.
     * @param [in] target_velocity Maximum velocity, always positive value. Unit in revolutions/s.
     * @param [in] acc Acceleration to the target velocity. Unit in revolutions/s^2.
     * @param [in] dec Deceleration to the target velocity. Unit in revolutions/s^2.
     * @param [in] jerk Jerk. Unit in revolutions/s^3.
     * @retval SUCCESS Execute the process successfully.
     * @retval CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED Failed to send request.
     * @retval CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED Failed to execute the process.
     */ 
    ErrorCode doServoCmdMoveAbsolute(int64_t target_position, int32_t target_velocity, int32_t acc, int32_t dec, int32_t jerk);
    
    /**
     * @brief Command servo to move to a relative position.
     * @details Servo response is expected.\n
     * @param [in] target_position Target position, Unit in pulse.
     * @param [in] target_velocity Maximum velocity, always positive value. Unit in revolutions/s.
     * @param [in] acc Acceleration to the target velocity. Unit in revolutions/s^2.
     * @param [in] dec Deceleration to the target velocity. Unit in revolutions/s^2.
     * @param [in] jerk Jerk. Unit in revolutions/s^3.
     * @retval SUCCESS Execute the process successfully.
     * @retval CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED Failed to send request.
     * @retval CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED Failed to execute the process.
     */ 
    ErrorCode doServoCmdMoveRelative(int64_t target_position, int32_t target_velocity, int32_t acc, int32_t dec, int32_t jerk);

    /**
     * @brief Command servo homing.
     * @details Servo response is expected.\n
     * @retval SUCCESS Execute the process successfully.
     * @retval CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED Failed to send request.
     * @retval CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED Failed to execute the process.
     */ 
    ErrorCode doServoCmdHoming(void);

    /**
     * @brief Command servo abort homing.
     * @details Servo response is expected.\n
     * @retval SUCCESS Execute the process successfully.
     * @retval CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED Failed to send request.
     * @retval CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED Failed to execute the process.
     */ 
    ErrorCode doServoCmdAbortHoming(void);

    /**
     * @brief Command servo halt.
     * @details Servo response is expected.\n
     * @retval SUCCESS Execute the process successfully.
     * @retval CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED Failed to send request.
     * @retval CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED Failed to execute the process.
     */ 
    ErrorCode doServoCmdHalt(void);

    /**
     * @brief Command reset encoder error.
     * @details Servo response is expected.\n
     * @retval SUCCESS Execute the process successfully.
     * @retval CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED Failed to send request.
     * @retval CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED Failed to execute the process.
     */ 
    ErrorCode doServoCmdResetEncoder(void);

    /**
     * @brief Command to get servo defined info.
     * @details Servo response is expected.\n
     * @retval SUCCESS Execute the process successfully.
     * @retval CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED Failed to send request.
     * @retval CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED Failed to execute the process.
     */ 
    ErrorCode doServoCmdGetServoDefinedInfo(int32_t* req_data_ptr, int32_t* res_data_ptr);

    /**
     * @brief Command to set zero position.
     * @details Servo response is expected.\n
     * @retval SUCCESS Execute the process successfully.
     * @retval CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED Failed to send request.
     * @retval CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED Failed to execute the process.
     */ 
    ErrorCode doServoCmdSetZeroOffset(int32_t encoder_value);

    /**
     * @brief Command servo to upload all its parameters to upload buffer channel.
     * @details It is an asynchronize core process call. Servo's response should return at once after controller sponsor the request.\n
     *          Servo will notify the controller by asynchronize flag after it uploads all its parameters to the upload buffer channel.\n
     * @param [out] async_ack_ptr_ptr Pointer of asynchronize flag.
     * @retval SUCCESS Execute the process successfully.
     * @retval CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED Failed to send request.
     * @retval CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED Failed to execute the process.
     */
    ErrorCode triggerServoCmdUploadParameters(int32_t** async_ack_ptr_ptr);
    /**
     * @brief Copy data from upload buffer channel to local memory.
     * @param [out] params_ptr Pointer of upload data.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
    bool uploadServoParameters(BufferAppData2002_t* params_ptr);
    /**
     * @brief Copy data from local memory to download buffer channel.
     * @param [in] params_ptr Pointer of download data.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
    bool downloadServoParameters(const BufferAppData2001_t* params_ptr);
    /**
     * @brief Command servo to download all its parameters from download buffer channel.
     * @details It is an asynchronize core process call. Servo's response should return at once after controller sponsor the request.\n
     *          Servo will notify the controller by asynchronize flag after it downloads all its parameters from the download buffer channel.\n
     * @param [out] async_ack_ptr_ptr Pointer of asynchronize flag.
     * @retval SUCCESS Execute the process successfully.
     * @retval CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED Failed to send request.
     * @retval CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED Failed to execute the process.
     */    
    ErrorCode triggerServoCmdDownloadParameters(int32_t** async_ack_ptr_ptr);
    /**
     * @brief Check if the asynchronize process is finished.
     * @param [in] async_ack_ptr Pointer of asynchronize flag.
     * @retval true Asynchronize process has finished.
     * @retval false Asynchronize process is running.
     */    
    bool isServoAsyncServiceFinish(int32_t* async_ack_ptr);
    /**
     * @brief Get the latest data frame from feedback circle buffer channel.
     * @param [out] pdo_data_ptr Pointer of data frame.
     * @param [out] current_time_stamp_ptr Pointer of the time stamp of the frame.
     * @return void
     */ 
    void processFdbPdoCurrent(uint8_t* pdo_data_ptr, uint32_t* current_time_stamp_ptr);
    /**
     * @brief Get the data frame from feedback circle buffer channel by specified time stamp.
     * @param [out] pdo_data_ptr Pointer of data frame.
     * @param [in] expect_time_stamp Expected time stamp of the frame.
     * @return void
     */    
    void processFdbPdoSync(uint8_t* pdo_data_ptr, uint32_t expect_time_stamp);
    /**
     * @brief Push data frame to control circle buffer channel.
     * @param [in] pdo_data_ptr Pointer of the list of data frames.
     * @param [in] expect_element_number The expected number of data frames to be pushed in.
     * @param [in] actual_element_number_ptr Pointer of the actual number of data frames that have been pushed in.
     * @return void
     */ 
    void processCtrlPdoBufferMode(uint8_t* pdo_data_ptr, int32_t expect_element_number, int32_t* actual_element_number_ptr);
    /**
     * @brief Clear data in control circle buffer channel.
     * @return void
     */ 
    void clearCtrlPdoBuffer();
    /**
     * @brief Get servo state and state word.
     * @param [out] state_word Servo state word.
     * @return Servo state.
     */
    ServoSm_e getServoState(ServoState_u& state_word);
    /**
     * @brief Get servo state expressed in string format.
     * @param [in] sm Servo state.
     * @return Servo state in string.
     */
    std::string getServoStateString(ServoSm_e sm);
    /**
     * @brief Get configuration of the servo communication.
     * @param [out] info Configuration of the servo communication.
     * @return void
     */
    void getServoCommInfo(ServoCommInfo_t* info);
private:
    ServoCommBase();
    ServoComm_t* comm_ptr_; /**< Servo communication configuration object.*/
};

}


#endif

