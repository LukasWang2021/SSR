#ifndef GROUP_SM_H
#define GROUP_SM_H

/**
 * @file group_sm.h
 * @brief The file is the header file of class "GroupSm".
 * @author Feng.Wu
 */

#include "common_error_code.h"
#include "group_datatype.h"
#include "system_model_manager.h"
#include "log_manager_producer.h"
#include "axis.h"
#include "group_fdb.h"
#include "group_application_param_1000.h"
#include "error_queue.h"


/**
 * @brief group_space includes all group related definitions and implementation.
 */
namespace group_space {

/**
 * @brief GroupSm process the state machine of the group.
 * @details
 */
class GroupSm {
  public:
    /**
     * @brief Constructor of the class.
     */
    GroupSm(void);
    /**
     * @brief Destructor of the class. 
     */ 
    ~GroupSm(void);

    /**
     * @brief Initializatin.
     * @details Extract the parameters from the model data.\n
     * @param [in] id The reference of the group.
     * @param [in] fdb_ptr The pointer of the feedback data from the servos.
     * @param [in] db_ptr The pointer of the parameters of the group model.
     * @param [in] axis_group_ptr The pointer to the list of the axes in the group.
     * @retval true success.
     * @retval false Failed to initialize.
     */
    bool init(int32_t id, GroupFdb*  fdb_ptr, system_model_space::GroupModel_t* db_ptr, std::map<int, axis_space::Axis*>* axis_group_ptr);

    /**
     * @brief Run the state machine to update the status of the group.
     * @details It should be cyclically called.
     * @return void.
     */
    void processStatemachine(void);

    /**
     * @brief Transfer the state of the group to Stopping.
     * @details 
     * @retval  true Succeed to transfer the state.
     * @retval  false Failed.
     */
	bool transferStateToGroupStopping(void);

    /**
     * @brief Transfer the state of the group to Homing.
     * @details 
     * @retval  true Succeed to transfer the state.
     * @retval  false Failed.
     */
	bool transferStateToGroupHoming(void);

    /**
     * @brief Transfer the state of the group to Moving.
     * @details 
     * @retval  true Succeed to transfer the state.
     * @retval  false Failed.
     */
	bool transferStateToGroupMoving(void);

    /**
     * @brief Inform the state machine that the internal error happens in the group.
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
     * @brief Get the stutus of the group.
     * @return GroupStatus_e 
     * - GROUP_STATUS_UNKNOWN = 0,
     * - GROUP_STATUS_ERROR_STOP = 1,  
     * - GROUP_STATUS_DISABLED = 2,   
     * - GROUP_STATUS_STANDBY = 3,     
     * - GROUP_STATUS_STOPPING = 4,   
     * - GROUP_STATUS_HOMING = 5,   
     * - GROUP_STATUS_MOVING = 6, 
     */
    GroupStatus_e getGroupStatus(void);

    /**
     * @brief Get the text description of the group status.
     * @details Mostly for the logs.
     * @param [in] group_state The status of the group.
     * - GROUP_STATUS_UNKNOWN = 0,
     * - GROUP_STATUS_ERROR_STOP = 1,  
     * - GROUP_STATUS_DISABLED = 2,   
     * - GROUP_STATUS_STANDBY = 3,     
     * - GROUP_STATUS_STOPPING = 4,   
     * - GROUP_STATUS_HOMING = 5,   
     * - GROUP_STATUS_MOVING = 6, 
     * @return The text description of the status.
     */
    std::string getGroupStatusString(GroupStatus_e group_state);

    /**
     * @brief Get the permission to process the control PDO under the certain status.
     * @details 
     * @retval true Processing the control PDO is enable.
     * @retval false It is not permited to process the control PDO under the current status.
     */
    bool isCtrlPdoEnable(void);
  
  private:
    GroupFdb* fdb_ptr_;
    system_model_space::GroupModel_t* db_ptr_;
    std::map<int, axis_space::Axis*>* axis_group_ptr_;
	GroupStatus_e group_state_;
	bool is_err_exist_;
    int32_t id_;
    int32_t target_reached_count_;
    int32_t target_reached_max_;
    bool ctrl_pdo_enble_;

	void processError(void);
    void processGroupState(void);
};

}
#endif
