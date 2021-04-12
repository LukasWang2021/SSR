#ifndef AXIS_SM_H
#define AXIS_SM_H

/**
 * @file axis_sm.h
 * @brief The file is the header file of class "AxisSm".
 * @author Feng.Wu
 */

#include "common_error_code.h"
#include "axis_datatype.h"
#include "system_model_manager.h"
#include "axis_fdb.h"
#include "log_manager_producer.h"
#include "system/servo_comm_base.h"
#include "axis_application_param_1000.h"
#include "error_queue.h"

/**
 * @brief axis_space includes all axis related definitions and implementation.
 */
namespace axis_space {

/**
 * @brief AxisSm process the state machine of the axis.
 * @details
 */
class AxisSm {
  public:
    /**
     * @brief Constructor of the class.
     */
    AxisSm(void);
    /**
     * @brief Destructor of the class. 
     */ 
    ~AxisSm(void);

    /**
     * @brief Initializatin.
     * @details Extract the related parameters from the model data.\n
     * @param [in] id The reference of the axis.
     * @param [in] fdb_ptr The pointer of the feedback data from the servo.
     * @param [in] servo_comm_ptr The pointer of the servo communication object.
     * @param [in] db_ptr The pointer of the parameters of the axis model.
     * @retval true success.
     * @retval false Failed to initialize.
     */
    bool init(int32_t id, AxisFdb* fdb_ptr, servo_comm_space::ServoCommBase* servo_comm_ptr, system_model_space::AxisModel_t* db_ptr);

    /**
     * @brief Run the state machine to update the status of the axis.
     * @details It should be cyclically called.
     * @return void.
     */
    void processStatemachine(void);

    /**
     * @brief Transfer the state of the axis to Discrete Motion.
     * @details 
     * @retval  true Succeed to transfer the state.
     * @retval  false Failed.
     */
	bool transferStateToDiscreteMotion(void);

    /**
     * @brief Transfer the state of the axis to Continuous Motion.
     * @details 
     * @retval  true Succeed to transfer the state.
     * @retval  false Failed.
     */
	bool transferStateToContinuousMotion(void);

    /**
     * @brief Transfer the state of the axis to Synchronized Motion.
     * @details 
     * @retval  true Succeed to transfer the state.
     * @retval  false Failed.
     */
	bool transferStateToSynchronizedMotion(void);

    /**
     * @brief Transfer the state of the axis to Homing.
     * @details 
     * @retval  true Succeed to transfer the state.
     * @retval  false Failed.
     */
	bool transferStateToHoming(void);

    /**
     * @brief Set the valid status of the state machine.
     * @details 
     * @param [in] is_valid The valid state.
     * @return void
     */
    void setValid(bool is_valid);
    
    /**
     * @brief Inform the state machine that the internal error happens in the axis.
     * @details Set error in order to process the state to ErrorStop.
     * @return void
     */
	void setError(void);

    /**
     * @brief Clear the internal error flag.
     * @details It can be called when reset.
     * @return void
     */
	void clearError(void);

    /**
     * @brief Read the axis status.
     * @details
     * @return 
     * - AXIS_STATUS_UNKNOWN = 0,
     * - AXIS_STATUS_ERRORSTOP = 1,
     * - AXIS_STATUS_DISABLED = 2,
     * - AXIS_STATUS_STANDSTILL = 3,
     * - AXIS_STATUS_STOPPING = 4,
     * - AXIS_STATUS_HOMING = 5,
     * - AXIS_STATUS_DISCRETE_MOTION = 6,
     * - AXIS_STATUS_CONTINUOUS_MOTION = 7,
     * - AXIS_STATUS_SYNCHRONIZED_MOTION = 8
     */
    AxisStatus_e getAxisStatus(void);

    /**
     * @brief Get the text description of the axis status.
     * @details Mostly for the logs.
     * @param [in] axis_status The status of the axis.
     * - AXIS_STATUS_UNKNOWN = 0,
     * - AXIS_STATUS_ERRORSTOP = 1,
     * - AXIS_STATUS_DISABLED = 2,
     * - AXIS_STATUS_STANDSTILL = 3,
     * - AXIS_STATUS_STOPPING = 4,
     * - AXIS_STATUS_HOMING = 5,
     * - AXIS_STATUS_DISCRETE_MOTION = 6,
     * - AXIS_STATUS_CONTINUOUS_MOTION = 7,
     * - AXIS_STATUS_SYNCHRONIZED_MOTION = 8
     * @return The text description of the status.
     */
    std::string getAxisStateString(AxisStatus_e axis_status);

    /**
     * @brief Get the permission to process the control PDO under the certain status.
     * @details 
     * @retval true Processing the control PDO is enable.
     * @retval false It is not permited to process the control PDO under the current axis status.
     */
    bool isCtrlPdoEnable(void);
  
  private:
	AxisStatus_e axis_state_;
    AxisFdb* fdb_ptr_;
    servo_comm_space::ServoCommBase* servo_comm_ptr_;
	bool is_err_exist_;
    int32_t id_;
    int32_t target_reached_count_;
    int32_t target_reached_max_;
    bool ctrl_pdo_enble_;
    bool is_valid_;
    
	void processError(void);
    void processAxisState(void);
};

}
#endif
