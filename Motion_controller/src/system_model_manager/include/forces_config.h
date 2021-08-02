#ifndef FORCES_CONFIG_H
#define FORCES_CONFIG_H

/**
 * @file forces_config.h
 * @brief The file defines the class handling force_config.xml.
 * @author feng.wu
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
 * @brief Defines node information in path /ForcesConfig/ForceConfig.
 */
typedef struct
{
    int32_t force_id;       /**< Node value in path /ForcesConfig/ForceConfig/ForceId.*/
    std::string root_dir;   /**< Node value in path /ForcesConfig/ForceConfig/RootDir.*/
    std::string parameter;  /**< Node value in path /ForcesConfig/ForceConfig/Parameter.*/
}ForceConfig_t;
/**
 * @brief Defines the class to parsing force_config.xml.
 */
class ForcesConfig
{
public:
    /**
     * @brief Constructor of the class.
     * @param [in] file_path The absoulte file path of force_config.xml.
     */        
    ForcesConfig(std::string file_path);
    /**
     * @brief Destructor of the class.
     */
    ~ForcesConfig();
    /**
     * @brief Parse the force_config.xml, turn it into data struct std::vector<AxisConfig_t>.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    bool load();
    /**
     * @brief Check if the force_config.xml has loaded correctly.
     * @retval true Configuration data is valid.
     * @retval false Configuration data is not valid.
     */    
    bool isValid();
    /**
     * @brief Get the reference of the configuration data.
     * @return The reference of configuration data.
     */     
    std::vector<ForceConfig_t>& getRef();
    
private:
    std::string file_path_;     /**< The absolute file path of force_config.xml.*/
    bool is_valid_;             /**< The flag to show the validity of force_config_.*/
    std::vector<ForceConfig_t> force_config_; /**< Stores the configuration data.*/
    base_space::XmlHelp xml_help_;  /**< XML parsing tool.*/

    bool loadForceConfig(const xmlDocPtr doc_ptr, const xmlNodePtr child_node_ptr, ForceConfig_t& force_config);
};

}



#endif

