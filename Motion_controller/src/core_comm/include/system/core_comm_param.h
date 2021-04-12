#ifndef CORE_COMM_PARAM_H
#define CORE_COMM_PARAM_H

/**
 * @file core_comm_param.h
 * @brief The file includes the class for parsing the configuration file of the 'core_comm' module.
 * @author zhengyu.shen
 */

#include "yaml_help.h"
#include <string>

/**
 * @brief core_comm_space includes all inter core communication related implementation.
 */
namespace core_comm_space
{
/**
 * @brief CoreCommParam is the object to handle the configuration file of the 'core_comm' module.
 */
class CoreCommParam
{
public:
    /**
     * @brief Constructor of the class.
     */    
    CoreCommParam();
    /**
     * @brief Destructor of the class.
     */    
    ~CoreCommParam();
    /**
     * @brief Load parameter into memory from file.
     * @details The Caller must be the master of the inter core communication.
     * @retval true Operation is sucessful.
     * @retval flase Operation is failed.
     */
    bool loadParam();
    /**
     * @brief Load parameter into memory from file.
     * @details The Caller must be some slave of the inter core communication.
     * @retval true Operation is sucessful.
     * @retval false Operation is failed.
     */    
    bool loadParamSlave();
    /**
     * @brief Write parameter into file from local memory.
     * @details The Caller must be the master of the inter core communication.
     * @retval true Operation is sucessful.
     * @retval false Operation is failed.
     */    
    bool saveParam();

    // param to load & save
    std::string device_path_;   /**< The absolute path for share memory.*/
    std::string config_file_;   /**< Name of the configuration file of inter core communication.*/
    int32_t cpu_id_;            /**< Cpu id of the caller.*/
private:
    base_space::YamlHelp yaml_help_;    /**< File parser for YAML format.*/
    std::string file_path_;             /**< The absolute file path of the configuration file of inter core communication.*/
};

}


#endif

