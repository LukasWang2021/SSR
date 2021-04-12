#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H

/**
 * @file controller_config.h
 * @brief The file is the header file of class "ControllerConfig".
 * @author Feng.Wu
 */

#include "yaml_help.h"
#include <string>

/**
 * @brief user_space includes the user level implementation.
 */
namespace user_space
{

/**
 * @brief ControllerConfig stores the parameters of the controller.
 * @details 
 */
class ControllerConfig
{
public:
    /**
     * @brief Constructor of the class.
     */
    ControllerConfig();
    /**
     * @brief Destructor of the class. 
     */  
    ~ControllerConfig();

    /**
     * @brief Load all the paramters.
     * @retval true Load all the parameters successfully.
     * @retval false Failed to load all the parameters.
     */
    bool load();
    /**
     * @brief Save all the paramters.
     * @retval true Save all the parameters successfully.
     * @retval false Failed to save all the parameters.
     */
    bool save();

    int log_level_;                 /**< The level of the log information.*/
    int routine_cycle_time_;        /**< The cycle time of the routine thread. The unit is us.*/
    int realtime_cycle_time_;       /**< The cycle time of the realtime thread. The unit is us.*/
    int routine_thread_priority_;   /**< The priority of the routine thread.*/
    int realtime_thread_priority_;  /**< The priority of the realtime thread.*/
    bool dio_exist_;
    bool aio_exist_;

private:
    base_space::YamlHelp yaml_help_;
    std::string file_path_;
};
}


#endif

