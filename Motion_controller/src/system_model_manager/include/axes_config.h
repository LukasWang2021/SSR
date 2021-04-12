#ifndef AXES_CONFIG_H
#define AXES_CONFIG_H

/**
 * @file axes_config.h
 * @brief The file defines the class handling axes_config.xml.
 * @author zhengyu.shen
 */

#include "xml_help.h"
#include <string>
#include <vector>
/**
 * @brief system_model_space includes all system model related implementation.
 */
namespace system_model_space
{
/**
 * @brief Defines node information in path /AxesConfig/AxisConfig/Actuator.
 */
typedef struct
{
    std::string servo;  /**< Node value in path /AxesConfig/AxisConfig/Actuator/Servo.*/
}ActuatorConfig_t;
/**
 * @brief Defines node information in path /AxesConfig/AxisConfig.
 */
typedef struct
{
    int32_t axis_id;        /**< Node value in path /AxesConfig/AxisConfig/AxisId.*/
    int32_t core_id;        /**< Node value in path /AxesConfig/AxisConfig/CoreId.*/
    int32_t servo_id;       /**< Node value in path /AxesConfig/AxisConfig/ServoId.*/
    std::string servo_type; /**< Node value in path /AxesConfig/AxisConfig/ServoType.*/
    std::string root_dir;   /**< Node value in path /AxesConfig/AxisConfig/RootDir.*/
    std::string application;/**< Node value in path /AxesConfig/AxisConfig/Application.*/
    ActuatorConfig_t actuator;                  /**< Node value in path /AxesConfig/AxisConfig/Actuator.*/
}AxisConfig_t;
/**
 * @brief Defines the class to parsing axes_config.xml.
 */
class AxesConfig
{
public:
    /**
     * @brief Constructor of the class.
     * @param [in] file_path The absoulte file path of axes_config.xml.
     */        
    AxesConfig(std::string file_path);
    /**
     * @brief Destructor of the class.
     */
    ~AxesConfig();
    /**
     * @brief Parse the axes_config.xml, turn it into data struct std::vector<AxisConfig_t>.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    bool load();
    /**
     * @brief Check if the axes_config.xml has loaded correctly.
     * @retval true Configuration data is valid.
     * @retval false Configuration data is not valid.
     */    
    bool isValid();
    /**
     * @brief Get the reference of the configuration data.
     * @return The reference of configuration data.
     */     
    std::vector<AxisConfig_t>& getRef();
    
private:
    std::string file_path_;     /**< The absolute file path of axes_config.xml.*/
    bool is_valid_;             /**< The flag to show the validity of axis_config_.*/
    std::vector<AxisConfig_t> axis_config_; /**< Stores the configuration data.*/
    base_space::XmlHelp xml_help_;  /**< XML parsing tool.*/

    bool loadAxisConfig(const xmlDocPtr doc_ptr, const xmlNodePtr child_node_ptr, AxisConfig_t& axis_config);
};

}



#endif

