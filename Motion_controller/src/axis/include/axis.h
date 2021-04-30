#ifndef AXIS_H
#define AXIS_H

/**
 * @file axis.h
 * @brief The file is the header file of class "Axis".
 * @author Feng.Wu
 */

#include <mutex>
#include "axis_sm.h"
#include "axis_fdb.h"
#include "axis_conversion.h"
#include "common_error_code.h"
#include "system/servo_comm_base.h"
#include "log_manager_producer.h"
#include "algorithm_base.h"
#include "system_model_manager.h"
#include "system/servo_cpu_comm_base.h"
#include "function_block.h"
#include "trajectory_block.h"
#include "fb_queue.h"
#include "tb_queue.h"

/**
 * @brief axis_space includes all axis related definitions and implementation.
 */
namespace axis_space {

/**
 * @brief Axis is the basic controlled object.
 * @details The interfaces are referenced from PLCopen.
 */
class Axis {
  public:
    /**
     * @brief Constructor of the class.
     * @param [in] id The reference to axis.
     * @param [in] type The type of axis.
     */
    Axis(int32_t id, AxisType_e type);
    /**
     * @brief Destructor of the class. 
     */    
    virtual ~Axis(void);

    /**
     * @brief Initialize axis.
     * @details Initialize internal components.\n
     *          Initialize all the supported algorithms.\n
     * @param [in] cpu_comm_ptr The pointer to communicate with the other cpu.
     * @param [in] servo_comm_ptr The pointer to communicate with servos.
     * @param [in] db_ptr The pointer of the parameters of the axis model.
     * @param [in] axis_config_ptr The pointer of the configuration of the axis.
     * @retval true success.
     * @retval false Failed to initialize.
     */
    bool init(servo_comm_space::ServoCpuCommBase* cpu_comm_ptr, servo_comm_space::ServoCommBase* servo_comm_ptr, 
        system_model_space::AxisModel_t* db_ptr, system_model_space::AxisConfig_t* axis_config_ptr);

    /**
     * @brief Load the algorithms.
     * @details
     * @retval false Failed.
     * @retval true Success.
     */
    bool reloadAlgorithms(void);

    //control
    /**
     * @brief Control the power stage(On or Off).
     * @param [in] enable
     * - false Power is being enabled,
     * - true Power is being disabled,
     * @return error_code.
     */
	virtual ErrorCode mcPower(bool enable) = 0;

    /**
     * @brief Reset all internal axis-related errors.
     * @details It makes the transition from the state 'ErrorStop' to 'Standstill' or 'Disabled'.
     * @return error_code
     */
	ErrorCode mcReset(void);

    /**
     * @brief Stop the axis motion.
     * @details It commands a controlled motion stop and transfers the axis to the state 'Stopping'.\n
     *          While the axis is in state 'Stopping', no other commands can perform any motion on the same axis.\n
     *          The axis goes to state 'Standstill' when 'Velocity' zero is reached.
     * @return error_code
     */
	ErrorCode mcStop(void);

    /**
     * @brief Halt the axis motion.
     * @details It commands a controlled motion stop. The axis is moved to the state 'DiscreteMotion', until the velocity is zero.\n
     *          The state is transferred to 'standstill' when the velocity is zero.\n
     *          The function is used to stop the axis under normal operation conditions.\n
     *          In non-buffered mode it is possible to set another motion command during deceleration of the axis, which will abort Halt.\n
     * @return error_code
     */
	ErrorCode mcHalt(void);

    /**
     * @brief Set the offset position of the axis.
     * @details It shifts the coordinate system of an axis by manipulating both the set-point position as well as the\n
     *          actual position of an axis with the same value without any movement caused.\n
     * @param [in] position The actual position value of the axis. The unit is angle pulse.
     * @return error_code
     */
	ErrorCode mcSetPosition(double position);

    /**
     * @brief Read the value of a specific parameter.
     * @details 
     * @param [in] param_index The index of the parameter.
     * @param [out] param_value The value of the parameter.
     * @return error_code
     */
	ErrorCode mcReadParamter(int32_t param_index, int32_t &param_value);

    /**
     * @brief Write down the value of a specific parameter.
     * @details
     * @param [in] param_index The index of the parameter.
     * @param [in] param_value The value of the parameter.
     * @return error_code
     */
	ErrorCode mcWriteParamter(int32_t param_index, int32_t param_value);

	//moving
	/**
     * @brief Command a controlled motion to a specified absolute position.
     * @details The action completes with velocity zero if no further actions are pending.
     * @param [in] position The absolute position input. Unit: angle pulse or um.
     * @param [in] velocity The value of the maximum velocity. Unit: angle pulse/s or um/s.
     * @param [in] acc The value of the acceleration(always positive).  Unit: angle pulse/s^2.
     * @param [in] dec The value of the deceleration(always positive). Unit: angle pulse/s^2.
     * @param [in] jerk The value of the jerk(always positive). Unit: angle pulse/s^3.
     * @return error_code
     */
	ErrorCode mcMoveAbsolute(double position, double velocity, double acc, double dec, double jerk);

	/**
     * @brief Command a controlled motion to a specified relative position.
     * @details The action completes with velocity zero if no further actions are pending.
     * @param [in] position The absolute position input. Unit: angle pulse or um.
     * @param [in] velocity The value of the maximum velocity. Unit: angle pulse/s or um/s.
     * @param [in] acc The value of the acceleration(always positive).  Unit: angle pulse/s^2.
     * @param [in] dec The value of the deceleration(always positive). Unit: angle pulse/s^2.
     * @param [in] jerk The value of the jerk(always positive). Unit: angle pulse/s^3.
     * @return error_code
     */
	ErrorCode mcMoveRelative(double position, double velocity, double acc, double dec, double jerk);

	/**
     * @brief Command a controlled motion to the home position.
     * @details The action completes with velocity zero and axis on home position.
     * @return error_code
     */
	ErrorCode mcHome(void);

    /**
     * @brief Command a never ending controlled motion at a specified velocity.
     * @details To stop the motion, the FB has to be interrupted by another FB issuing a new command.
     * @param [in] velocity The value of the maximum velocity. Unit: angle pulse/s or um/s.
     * @param [in] acc The value of the acceleration(increasing energy of the motor). Unit: angle pulse/s^2.
     * @param [in] dec The value of the deceleration(decreasing energy of the motor). Unit: angle pulse/s^2.
     * @param [in] jerk The value of the jerk. Unit: angle pulse/s^3.
     * @param [in] direction
     * - AXIS_DIRECTION_POSITIVE: Positive direction.
     * - AXIS_DIRECTION_NEGATIVE: Negative direction.
     * @return error_code
     */
    ErrorCode mcMoveVelocity(double velocity, double acc, double dec, double jerk, AxisDirection_e direction);
	
    //status
    /**
     * @brief Read the actual position.
     * @details
     * @param [out] position New absolute position. Unit: angle pulse or um.
     * @return error_code
     */
	ErrorCode mcReadActualPosition(double &position);

    /**
     * @brief Read the actual velocity.
     * @details
     * @param [out] velocity The value of the actual velocity. Unit: angle pulse/s or um/s.
     * @return error_code
     */
	ErrorCode mcReadActualVelocity(double &velocity);

    /**
     * @brief Read the actual torque.
     * @details
     * @param [out] torque The value of the actual torque. Unit: Nm.
     * @return error_code
     */
	ErrorCode mcReadActualTorque(double &torque);

    /**
     * @brief Read the axis information.
     * @details
     * @param [out] info The information of the servo.
     * - simulation Simulation mode or not.
     * - comm_ready Communication is ready or not.
     * - ready_power_on Power is ready to switch on or not.
     * - power_on Power is on or off.
     * @return error_code
     */
	ErrorCode mcReadAxisInfo(AxisInfo_b &info);

    /**
     * @brief Read the axis status.
     * @details
     * @param [out] status The status of the axis.
     * - AXIS_STATUS_UNKNOWN = 0,
     * - AXIS_STATUS_ERRORSTOP = 1,
     * - AXIS_STATUS_DISABLED = 2,
     * - AXIS_STATUS_STANDSTILL = 3,
     * - AXIS_STATUS_STOPPING = 4,
     * - AXIS_STATUS_HOMING = 5,
     * - AXIS_STATUS_DISCRETE_MOTION = 6,
     * - AXIS_STATUS_CONTINUOUS_MOTION = 7,
     * - AXIS_STATUS_SYNCHRONIZED_MOTION = 8
     * @return error_code
     */
	ErrorCode mcReadStatus(AxisStatus_e &status);

    /**
     * @brief Read the axis error.
     * @details
     * @param [out] error The current error.
     * @return error_code
     */
	ErrorCode mcReadAxisError(int32_t &error);

    /**
     * @brief Read the history buffer of errors.
     * @details
     * @param [in] size The valid size of the data.
     * @param [out] error_ptr The point of the error array.
     * @return error_code
     */
    ErrorCode rtmReadAxisErrorHistory(int32_t *error_ptr, int32_t size);
    
    /**
     * @brief Abort the hoiming option.
     * @details 
     * @return error_code
     */
    ErrorCode rtmAbortHoming(void);

    /**
     * @brief Get the pointer of the feedback PDO.
     * @details It is used to be called in publishing feedback data.
     * @param [out] size The valid size of the data.
     * @return Pointer of the data.
     */
	uint8_t* rtmReadAxisFdbPdoPtr(int32_t* size);

    /**
     * @brief Get the pointer of the feedback PDO.
     * @details It is used to be called in publishing feedback data.
     * @param [out] size The valid size of the data.
     * @return Pointer of the data.
     */
	ErrorCode rtmResetEncoder(void);

    /**
     * @brief Check if the target position is reached.
     * @details It is used to be called in a group.
     * @retval false The target position is not reached.
     * @retval true The target position is reached.
     */
    bool rtmIsTargetReached(void);

    /**
     * @brief Get the stutus of the servo.
     * @details It is used to be called in a group.
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
    ServoSm_e readServoStatus(void);

    /**
     * @brief Get the pointer of servo communication.
     * @details It is used to be called in a group.
     * @return Pointer.
     */
    servo_comm_space::ServoCommBase* getServoCommPtr(void);   

    /**
     * @brief Get the pointer of unit conversion between axis and motor.
     * @details It is used to be called in a group.
     * @return Pointer.
     */
    AxisConversion* getAxisConvPtr(void);

    /**
     * @brief Get the ID of the axis.
     * @details It is used to be called in a group.
     * @return The ID of the axis.
     */
    int32_t getID(void);

    /**
     * @brief Get the type of the axis.
     * @details It is used to be called in a group.
     * @return The type of the axis.
     */
    AxisType_e getType(void);

    /**
     * @brief Notify the axis to be in a group.
     * @details It is used to be called in a group.\n
     *          The state machine will be disable if the axis is added to a group.\n
     *          The status of the axis will stay in "Unknown".\n
     * @return void
     */
    void setAxisInGroup(bool is_in_group);

    /**
     * @brief Indicate the axis is in a group or not.
     * @details It is used to be called in a group.\n
     *          The state machine will be disable if the axis is added to a group.\n
     *          The status of the axis will stay in "Unknown".\n
     * @retval true The axis is in a group.
     * @retval false The axis is not in a group.
     */
    bool isAxisInGroup(void);

    /**
     * @brief Inform the state machine that the internal error happens.
     * @details Set error in order to process the state to ErrorStop.
     * @return void
     */
    void setError(ErrorCode err);

    /**
     * @brief Clear the internal error flag.
     * @details It can be called when reset.
     * @return void
     */
    void clearError(void);

    /**
     * @brief Lock the data of control PDO.
     * @details 
     * @return void.
     */
	void lockCtrlPdoMutex(void);

    /**
     * @brief Unlock the data of control PDO.
     * @details 
     * @return void.
     */
	void unlockCtrlPdoMutex(void);

    /**
     * @brief Lock the data of feedback PDO.
     * @details 
     * @return void.
     */
    void lockFdbPdoMutex(void);

    /**
     * @brief Unlock the data of control PDO.
     * @details 
     * @return void.
     */
	void unlockFdbPdoMutex(void);

    /**
     * @brief Run the state machine to update the status of the axis.
     * @details It should be cyclically called by Controller if the axis is an independent controlled object.
     * @return void.
     */
	void processStateMachine(void);

    /**
     * @brief To update the feedback of the servo.
     * @details It should be cyclically called by Controller.
     * @param [out] current_time_stamp_ptr Get the current time stamp of the current feedback data.
     * @return void.
     */
    void processFdbPdoCurrent(uint32_t* current_time_stamp_ptr);

    /**
     * @brief To update the feedback of the servo.
     * @details It should be cyclically called by Controller.\n
     *          It is called after processFdbPdoCurrent.\n
     *          The parameter input is from processFdbPdoCurrent in order to guarantee the feedback synchronization.\n
     * @param [in] expect_time_stamp Get the specified feedback data according to the specified time stamp.
     * @return void.
     */
    void processFdbPdoSync(uint32_t expect_time_stamp);

    /**
     * @brief Initialize the axis application.
     * @details Virtual function. It should be implemented in the axis subclass.
     * @retval false Failed.
     * @retval true Success.
     */
    virtual bool initApplication(void) = 0;

    /**
     * @brief Load the model parameters of the axis application.
     * @details Virtual function. It should be implemented in the axis subclass.
     * @retval false Failed to load parameters.
     * @retval true Success.
     */
    virtual bool reloadSystemModel(void) = 0;

    /**
     * @brief Push a Function Block to the queue.
     * @details Virtual function. It should be implemented in the axis subclass.
     * @retval false Failed to push a FB.
     * @retval true Success.
     */
    virtual bool pushBackFB(void* fb_ptr) = 0;

    /**
     * @brief Get the status of the Function Block Queue.
     * @details Virtual function. It should be implemented in the axis subclass.
     * @return 
     * - FBQ_STATUS_EMPTY = 0,      
     * - FBQ_STATUS_NOT_FULL = 1,
     * - FBQ_STATUS_FULL = 2,      
     */
    virtual base_space::FBQueueStatus_e getFBQStatus() = 0;

    /**
     * @brief Compute the Function Block.
     * @details Virtual function. It should be implemented in the axis subclass.
     * @return void
     */
    virtual void processFBQ() = 0;

    /**
     * @brief Send the Trajectory Block queue to the servo.
     * @details Virtual function. It should be implemented in the axis subclass.
     * @return void
     */
    virtual void processTBQ() = 0;

    /**
     * @brief Clear all the Block Queues.
     * @details Virtual function. It should be implemented in the axis subclass.\n
     *          It is called under power_on, reset and stop.
     * @return void
     */
    virtual void clearBQ() = 0;

  protected: 
    AxisConversion conv_;                                      /**< The unit conversion between axis and motor.*/
    std::vector<base_space::AlgorithmBase*> support_alg_list_; /**< The list of the pointers of the supported algorithms.*/
    std::vector<base_space::AlgorithmBase*> current_alg_list_; /**< The list of the pointers of the current algorithms.*/
    servo_comm_space::ServoCpuCommBase* cpu_comm_ptr_;         /**< The pointer to communicate with the other cpu.*/
    servo_comm_space::ServoCommBase* servo_comm_ptr_;          /**< The pointer to communicate with servos.*/
    system_model_space::AxisConfig_t* axis_config_ptr_;        /**< The pointer of the configuration of the axis.*/
    system_model_space::AxisModel_t* db_ptr_;                  /**< The pointer of the parameters of the axis model.*/
    AxisSm sm_;                                                /**< The state machine of the axis.*/

  private:    	
	AxisFdb fdb_;
    int32_t id_;
    ErrorCode axis_error_;
    bool is_in_group_;
    AxisType_e type_;
	        
    std::mutex ctrl_pdo_mutex_;
    std::mutex fdb_pdo_mutex_;
};
}
#endif
