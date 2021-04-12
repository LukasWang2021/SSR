#ifndef SYSTEM_MODEL_MANAGER_H
#define SYSTEM_MODEL_MANAGER_H

/**
 * @file system_model_manager.h
 * @brief The file defines the class SystemModelManager which provides interface for the system model.
 * @author zhengyu.shen
 */

#include "common_error_code.h"
#include "model_base.h"
#include "servo_base.h"
#include "axes_config.h"
#include "groups_config.h"
#include "system_model_manager_config.h"
#include <string>
#include <vector>
#include <map>
/**
 * @brief system_model_space includes all system model related implementation.
 */
namespace system_model_space
{
/**
 * @brief Defines actuator model.
 */
typedef struct
{
    ServoBase* servo_ptr;   /**< Pointer of servo model.*/
}ActuatorModel_t;
/**
 * @brief Defines axis model.
 */
typedef struct
{
    ModelBase* application_ptr; /**< Pointer of axis application model.*/
    ActuatorModel_t actuator;   /**< Pointer of actuator model.*/
}AxisModel_t;
/**
 * @brief Defines group model.
 */
typedef struct
{
    ModelBase* kinematics_ptr;          /**< Pointer of group kinematics model.*/
    ModelBase* dynamics_ptr;            /**< Pointer of group dynamics model.*/
    ModelBase* application_ptr;         /**< Pointer of group application model.*/
    std::vector<AxisModel_t*> axis_set; /**< Pointer of axis model.*/
    std::map<std::string, ModelBase*> algorithm_set;    /**< Pointer of group algorithm model.*/
}GroupModel_t;
/**
 * @brief SystemModelManager is the interface class for system model.
 */
class SystemModelManager
{
public:
    /**
     * @brief Constructor of the class.
     */    

    SystemModelManager();
    /**
     * @brief Destructor of the class.
     */
    ~SystemModelManager();
    /**
     * @brief Load axes_config.xml and groups_config.xml.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    bool init();
    /**
     * @brief Load all model configuration files. 
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
    bool load();
    /**
     * @brief Save all model configuration to files
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
    bool save();
    /**
     * @brief Get the handler for AxesConfig.
     * @return Pointer of AxesConfig object.
     */ 
    AxesConfig* getAxesConfig();
    /**
     * @brief Get the handler for GroupsConfig.
     * @return Pointer of GroupsConfig object.
     */     
    GroupsConfig* getGroupsConfig();
    /**
     * @brief Get the pointer of some axis model.
     * @param [in] axis_id Axis id.
     * @return Pointer of axis model.
     */     
    AxisModel_t* getAxisModel(int32_t axis_id);
    /**
     * @brief Get the pointer of some group model.
     * @param [in] group_id Group id.
     * @return Pointer of group model.
     */ 
    GroupModel_t* getGroupModel(int32_t group_id);
    
private:
    AxesConfig* axes_config_ptr_;
    GroupsConfig* groups_config_ptr_;
    std::map<int32_t, AxisModel_t> axis_model_set_;
    std::map<int32_t, GroupModel_t> group_model_set_;
    SystemModelManagerConfig config_;
};

}

#endif


