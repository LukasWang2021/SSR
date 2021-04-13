#ifndef GROUP_FDB_H
#define GROUP_FDB_H

/**
 * @file group_fdb.h
 * @brief The file is the header file of class "GroupFdb".
 * @author Feng.Wu
 */

#include <vector>
#include <string.h>  
#include "common_error_code.h"
#include "group_datatype.h"
#include "log_manager_producer.h"
#include "axis.h"

/**
 * @brief group_space includes all group related definitions and implementation.
 */
namespace group_space {

/**
 * @brief GroupFdb handles the feedback of the servo.
 * @details
 */
class GroupFdb {
  public:
    /**
     * @brief Constructor of the class.
     */ 
    GroupFdb(void);
    /**
     * @brief Destructor of the class. 
     */ 
    ~GroupFdb(void);

    /**
     * @brief Initializatin.
     * @details 
     * @param [in] id The reference of the axis.
     * @param [in] axis_group_ptr The pointer to the list of the axes in the group.
     * @retval true success.
     * @retval false Failed to initialize.
     */
    bool init(int32_t id, std::map<int, axis_space::Axis*>* axis_group_ptr);

    /**
     * @brief To update the feedback of the servo.
     * @details 
     * @return void.
     */
    void processAxesFdb();

    /**
     * @brief Get the stutus of the servos.
     * @return ServoSm_e 
     * - SERVO_SM_SWITCH_ON_DISABLED = 2,
     * - SERVO_SM_READY_TO_SWITCH_ON = 3,
     * - SERVO_SM_SWITCHED_ON = 4,
     * - SERVO_SM_OPERATION_ENABLED = 5,
     * - SERVO_SM_FAULT = 8,
     */
    ServoSm_e getServosStatus(void);

    /**
     * @brief Get the error which trigger the FAULT status.
     * @details 
     * @return The axis error.
     */
    ErrorCode getAxisError(void);

    /**
     * @brief Check if the target positions of all axes are reached.
     * @retval false The target position is not reached.
     * @retval true The target position is reached.
     */
    bool isTargetReached(void);
  
  private:    
    std::map<int, axis_space::Axis*>* axis_group_ptr_;
    ServoSm_e servos_status_;
    ErrorCode axis_error_;

    int32_t id_;
};

}
#endif
