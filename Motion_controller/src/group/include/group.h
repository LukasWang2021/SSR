#ifndef GROUP_H
#define GROUP_H

/**
 * @file group.h
 * @brief The file is the header file of class "Group".
 * @author Feng.Wu
 */

#include <mutex>
#include "group_sm.h"
#include "common_error_code.h"
#include "system/servo_comm_base.h"
#include "log_manager_producer.h"
#include "system_model_manager.h"
#include "error_queue.h"
#include "function_block.h"
#include "trajectory_block.h"
#include "fb_queue.h"
#include "tb_queue.h"

/**
 * @brief group_space includes all group related definitions and implemmcGroupEnableentation.
 */
namespace group_space {

/**
 * @brief Group is the basic controlled object.
 * @details The interfaces are referenced from PLCopen.
 */
class Group {
  public:
    /**
     * @brief Constructor of the class.
     * @param [in] id The reference to group.
     */ 
    Group(int32_t id);
    /**
     * @brief Destructor of the class. 
     */    
    virtual ~Group(void);

    /**
     * @brief Initialize group.
     * @details Initialize internal components.\n
     *          Initialize all the supported algorithms.\n
     * @param [in] cpu_comm_ptr The pointer to communicate with the other cpu.
     * @param [in] db_ptr The pointer of the parameters of the group model.
     * @param [in] group_config_ptr The pointer of the configuration of the group.
     * @retval true success.
     * @retval false Failed to initialize.
     */
    bool init(servo_comm_space::ServoCpuCommBase* cpu_comm_ptr,  system_model_space::GroupModel_t* db_ptr,
        system_model_space::GroupConfig_t* group_config_ptr);

    /**
     * @brief Load the algorithms.
     * @details
     * @retval false Failed.
     * @retval true Success.
     */
    bool reloadAlgorithms(void);

    /**
     * @brief It adds one axis to a group.
     * @details It is valid under no movement.\n
     * @param [in] id_in_group Identifies the order in the group of the added axis.
     * @param [in] axis_ref The reference to the axis to be added.
     * @return error_code.
     */
    ErrorCode mcAddAxisToGroup(int32_t id_in_group, axis_space::Axis &axis_ref);

    /**
     * @brief It removes one axis from the group.
     * @details It is valid under no movement.\n
     * @param [in] id_in_group Identifies the axis in the group.
     * @return error_code.
     */
    ErrorCode mcRemoveAxisFromGroup(int32_t id_in_group);

    /**
     * @brief It removes all axes from the group.
     * @details It is valid under no movement.\n
     * @return error_code.
     */
    ErrorCode mcUngroupAllAxes(void);

    /**
     * @brief Reset all internal group-related errors and. It also resets all axes in this group.
     * @details It makes the transition from the state GroupErrorStop to GroupStandby.
     * @return error_code
     */
    ErrorCode mcGroupReset(void);

    /**
     * @brief Enable the group by controlling the power stage to be on.
     * @details It changes the state from GroupDisabled to GroupStandby.
     * @return error_code
     */
    ErrorCode mcGroupEnable(void);

    /**
     * @brief Disable the group by controlling the power stage to be off.
     * @details It changes the state to GroupDisabled.
     * @return error_code
     */
    ErrorCode mcGroupDisable(void);

    /**
     * @brief It commands a controlled motion stop and transfers the axes group to the state GroupStopping.
     * @details It aborts any ongoing Function Block execution.
     *          While the axes group is in state GroupStopping, no other FB can perform any motion on the same axes group.
     *          After the axes group has reached velocity zero, the axes group goes to the state GroupStandby.
     *          This command can only be aborted by c_groupDisable.
     * @param [in] dec The value of the deceleration. Unit: angle degree/s^2 or mm/s^2.
     * @param [in] jerk The value of the jerk. Unit: angle degree/s^3 or mm/s^3.
     * @return error_code
     */
    ErrorCode mcGroupStop(double dec, double jerk);

    /**
     * @brief It commands a controlled motion stop.
     * @details Virtual function. It should be implemented in the Group subclass.\n
     *          It aborts any ongoing Function Block execution.
     *          AxesGroup is moved to the state GroupMoving, until the velocity is zero.
     *          When the velocity is zero, the state is transferred to GroupStandby.
     * @param [in] dec The value of the deceleration. Unit: angle degree/s^2 or mm/s^2.
     * @param [in] jerk The value of the jerk. Unit: angle degree/s^3 or mm/s^3.
     * @return error_code
     */
    virtual ErrorCode mcGroupHalt(double dec, double jerk) = 0;

    /**
     * @brief It interrupts the on-going motion and stops the group from moving.
     * @details Virtual function. It should be implemented in the Group subclass.\n
     *          It stores all the relevant track or path information internally at the moment it becomes active.
     *          The AxesGroup statys in the original state even if the velocity zero is reached.
     * @param [in] dec The value of the deceleration. Unit: angle degree/s^2 or mm/s^2.
     * @param [in] jerk The value of the jerk. Unit: angle degree/s^3 or mm/s^3.
     * @return error_code
     */
    virtual ErrorCode mcGroupInterrupt(double dec, double jerk) = 0;

    /**
     * @brief It transfers the program back to the situation at issuing c_groupInterrupt.
     * @details It uses internally the data set as stored at issuing c_groupInterrupt, 
     *          and tranfer the control on the group back to the original FB doing the movement on the axes group.
     * @return error_code
     */
    virtual ErrorCode mcGroupContinue(void) = 0;

    /**
     * @brief It sets the position of all axes in a group without moving the axes.
     * @details Virtual function. It should be implemented in the Group subclass.\n
     *          The new coordinates are described in an array of position, with the coordinate system input the according coordinate system is selected.\n
     *          It can be seen as a way of referencing or a transformation.\n
     *          It shifts the position of the addressed coordinate system and affects the higher level coordinate systems(so if ACS is select, MCS and PCS are affacted).
     * @param [in] position Array of absolute end position for each dimension in the specified coordinate system.
     * @param [in] relative Mode of position inputs.
     * - 0: Absolute.
     * - 1: Relative
     * @param [in] coord_type Reference to the applicable coordinate system.
     * - COORD_TYPE_ACS = 0,
     * - COORD_TYPE_MCS = 1,
     * - COORD_TYPE_PCS = 2,
     * @return error_code
     */
    virtual ErrorCode mcGroupSetPosition(const std::vector<double> &position, bool relative, CoordType_e coord_type) = 0;
  
    /**
     * @brief It commands a movement of an axes group to the specified absolute position in the specified coordinate system.
     * @details Virtual function. It should be implemented in the Group subclass.\n
     *          It does not take care of how (on which path) the target position is reached.\n
     * @param [in] position Array of absolute end position for each dimension in the specified coordinate system.
     * @param [in] coord_type Reference to the applicable coordinate system.
     * - COORD_TYPE_ACS = 0,
     * - COORD_TYPE_MCS = 1,
     * - COORD_TYPE_PCS = 2,
     * @param [in] vel_pct The percent of the maximum velocity for the path for the coordinate system in which the path is defined. The unit is %.
     * @param [in] acc_pct The percent of the maximum acceleration. Always positive. The unit is %.
     * @param [in] jerk Maximum jerk. Always positive. The unit is mm/s^3.
     * @return error_code
     */
    virtual ErrorCode mcMoveDirectAbsolute(const std::vector<double> &position, CoordType_e coord_type,
        double vel_pct, double acc_pct, double jerk) = 0;

    /**
     * @brief It commands an interpolated linear movement on an axes group from the actual position of the TCP to an absolute position in specified coordinate system.
     * @details Virtual function. It should be implemented in the Group subclass.\n
     * @param [in] position Array of absolute end position for each dimension in the specified coordinate system.
     * @param [in] coord_type Reference to the applicable coordinate system.
     * - COORD_TYPE_ACS = 0,
     * - COORD_TYPE_MCS = 1,
     * - COORD_TYPE_PCS = 2,
     * @param [in] velocity Maximum velocity for the path for the coordinate system in which the path is defined. The unit is mm/s.
     * @param [in] acc Maximum acceleration. Always positive. The unit is mm/s^2.
     * @param [in] dec Maximum deceleration. Always positive. The unit is mm/s^2.
     * @param [in] jerk Maximum jerk. Always positive. The unit is mm/s^3.
     * @return error_code
     */
    virtual ErrorCode mcMoveLinearAbsolute(const std::vector<double> &position, CoordType_e coord_type, 
        double velocity, double acc, double dec, double jerk) = 0;

    /**
     * @brief Get the actual position in the selected coordinate system of an axes group.
     * @details Virtual function. It should be implemented in the Group subclass.\n
     * @param [in] coord_type Reference to the applicable coordinate system.
     * - COORD_TYPE_ACS = 0,
     * - COORD_TYPE_MCS = 1,
     * - COORD_TYPE_PCS = 2,
     * @param [out] position Array of absolute end position for each dimension in the specified coordinate system. Unit: deg or mm.
     * @return error_code
     */
    virtual ErrorCode mcGroupReadActualPosition(CoordType_e coord_type, std::vector<double> &position) = 0;

    /**
     * @brief Get the actual velocity in the selected coordinate system of an axes group.
     * @details Virtual function. It should be implemented in the Group subclass.\n
     * @param [in] coord_type Reference to the applicable coordinate system.
     * - COORD_TYPE_ACS = 0,
     * - COORD_TYPE_MCS = 1,
     * - COORD_TYPE_PCS = 2,
     * @param [out] velocity Array of absolute end velocity for each dimension in the specified coordinate system. Unit: deg or mm.
     * @return error_code
     */
    virtual ErrorCode mcGroupReadActualVelocity(CoordType_e coord_type, std::vector<double> &velocity) = 0;

    /**
     * @brief Get the axes group errors.
     * @details
     * @return The group errors
     */
    ErrorCode mcGroupReadError(void);

    /**
     * @brief Get the status of an axes group.
     * @details
     * @param [out] status Reference to the group state.
     * - GROUP_STATUS_UNKNOWN = 0,
     * - GROUP_STATUS_ERROR_STOP = 1,
     * - GROUP_STATUS_DISABLED = 2,
     * - GROUP_STATUS_STANDBY = 3,
     * - GROUP_STATUS_STOPPING = 4,
     * - GROUP_STATUS_HOMING = 5,
     * - GROUP_STATUS_MOVING = 6,
     * @param [out] in_position The position is reached or not.
     * - 0: The position is not reached.
     * - 1: The position is reached.
     * @return error_code
     */
    ErrorCode mcGroupReadStatus(GroupStatus_e &status, bool &in_position);

    /**
     * @brief Get the reference of axis according the id in an axes group.
     * @details 
     * @param [in] id_in_group The order in the group of the added axis.
     * @param [in] axis_ref The reference to the axis to be added.     
     * @return error_code
     */
    ErrorCode mcGroupReadConfiguration(int32_t id_in_group, axis_space::Axis &axis_ref);

    /**
     * @brief Run the state machine to update the status of the group.
     * @details It should be cyclically called by Controller.
     * @return void.
     */
    void processStateMachine(void);

    /**
     * @brief To update the feedback of the servo and the status of the group.
     * @details It should be cyclically called by Controller.
     * @return void.
     */
    void processFdbPdo(void);

    /**
     * @brief Get the ID of the group.
     * @details 
     * @return The ID of the group.
     */
    int32_t getID();

    /**
     * @brief Inform the state machine that the internal error happens in the group.
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
     * @brief Initialize the group application.
     * @details Virtual function. It should be implemented in the Group subclass.
     * @retval false Failed to load parameters.
     * @retval true Success.
     */
    virtual bool initApplication(void) = 0;

    /**
     * @brief Load the model parameters of the group application.
     * @details Virtual function. It should be implemented in the axis subclass.
     * @retval false Failed to load parameters.
     * @retval true Success.
     */
    virtual bool reloadSystemModel(void) = 0;

    /**
     * @brief Push a Function Block to the queue.
     * @details Virtual function. It should be implemented in the Group subclass.
     * @retval false Failed to push a FB.
     * @retval true Success.
     */
    virtual bool pushBackFB(void* fb_ptr) = 0;

    /**
     * @brief Get the status of the Function Block Queue.
     * @details Virtual function. It should be implemented in the Group subclass.
     * @return 
     * - FBQ_STATUS_EMPTY = 0,      
     * - FBQ_STATUS_NOT_FULL = 1,
     * - FBQ_STATUS_FULL = 2,      
     */
    virtual base_space::FBQueueStatus_e getFBQStatus() = 0;

    /**
     * @brief Compute the Function Block.
     * @details Virtual function. It should be implemented in the Group subclass.
     * @return void
     */
    virtual void processFBQ() = 0;

    /**
     * @brief Send the Trajectory Block queue to the servo.
     * @details Virtual function. It should be implemented in the Group subclass.
     * @return void
     */
    virtual void processTBQ() = 0;

    /**
     * @brief Clear all the Block Queues.
     * @details Virtual function. It should be implemented in the Group subclass.\n
     *          It is called under power_on, reset and stop.
     * @return void
     */
    virtual void clearBQ() = 0;

  protected:
    servo_comm_space::ServoCpuCommBase* cpu_comm_ptr_;         /**< The pointer to communicate with the other cpu.*/
    system_model_space::GroupConfig_t* group_config_ptr_;      /**< The pointer of the configuration of the group.*/
    system_model_space::GroupModel_t* db_ptr_;                 /**< The pointer of the parameters of the group model.*/
    std::map<int32_t, axis_space::Axis*> axis_group_;              /**< The list of the axes in the group.*/
	  GroupSm sm_;                                               /**< The state machine of the group.*/

  private:   
    GroupFdb fdb_;
    int32_t id_;
    ErrorCode group_error_;
        
    std::mutex ctrl_pdo_mutex_;
    std::mutex fdb_pdo_mutex_;

};

}
#endif
