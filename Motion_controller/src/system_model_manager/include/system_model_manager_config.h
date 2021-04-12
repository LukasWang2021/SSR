#ifndef SYSTEM_MODEL_MANAGER_CONFIG_H
#define SYSTEM_MODEL_MANAGER_CONFIG_H

/**
 * @file system_model_manager_config.h
 * @brief The file defines the class SystemModelManagerConfig which provides configuration for the system_model moduel.
 * @author zhengyu.shen
 */

#include "yaml_help.h"
#include <string>
/**
 * @brief system_model_space includes all system model related implementation.
 */
namespace system_model_space
{
class SystemModelManagerConfig
{
public:
    /**
     * @brief Constructor of the class.
     */    
    SystemModelManagerConfig();
    /**
     * @brief Destructor of the class.
     */
    ~SystemModelManagerConfig();
    /**
     * @brief Load system_model_manager.yaml.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */    
    bool load();
    /**
     * @brief Save configuration to system_model_manager.yaml.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */     
    bool save();

    std::string system_model_root_dir_; /**< Absolute path of the root directory of all sysytem models.*/

private:
    base_space::YamlHelp yaml_help_;
    std::string file_path_;
};

}

#endif
