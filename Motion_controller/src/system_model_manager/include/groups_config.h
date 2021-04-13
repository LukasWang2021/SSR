#ifndef GROUPS_CONFIG_H
#define GROUPS_CONFIG_H

/**
 * @file groups_config.h
 * @brief The file defines the class handling groups_config.xml.
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
 * @brief Defines node information in path /GroupsConfig/GroupConfig.
 */
typedef struct
{
    int32_t group_id;               /**< Node value in path /GroupsConfig/GroupConfig/GroupId.*/
    std::vector<int32_t> axis_id;   /**< Node value in path /GroupsConfig/GroupConfig/AxesId.*/
    std::string root_dir;           /**< Node value in path /GroupsConfig/GroupConfig/RootDir.*/
    std::string kinematics;         /**< Node value in path /GroupsConfig/GroupConfig/Kinematics.*/
    std::string application;        /**< Node value in path /GroupsConfig/GroupConfig/Application.*/
}GroupConfig_t;
/**
 * @brief Defines the class to parsing groups_config.xml.
 */
class GroupsConfig
{
public:
    /**
     * @brief Constructor of the class.
     * @param [in] file_path The absoulte file path of groups_config.xml.
     */     
    GroupsConfig(std::string file_path);
    /**
     * @brief Destructor of the class.
     */    
    ~GroupsConfig();
    /**
     * @brief Parse the groups_config.xml, turn it into data struct std::vector<GroupConfig_t>.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    bool load();
    /**
     * @brief Check if the groups_config.xml has loaded correctly.
     * @retval true Configuration data is valid.
     * @retval false Configuration data is not valid.
     */     
    bool isValid();
    /**
     * @brief Get the reference of the configuration data.
     * @return The reference of configuration data.
     */    
    std::vector<GroupConfig_t>& getRef();
    
private:
    std::string file_path_; /**< The absolute file path of groups_config.xml.*/
    bool is_valid_;         /**< The flag to show the validity of group_config_.*/
    std::vector<GroupConfig_t> group_config_;   /**< Stores the configuration data.*/
    base_space::XmlHelp xml_help_;              /**< XML parsing tool.*/

    bool loadGroupConfig(const xmlDocPtr doc_ptr, const xmlNodePtr child_node_ptr, GroupConfig_t& group_config);
    bool loadAxesId(const xmlDocPtr doc_ptr, const xmlNodePtr child_node_ptr, std::vector<int32_t>& axis_id);
    bool loadAlgorithms(const xmlDocPtr doc_ptr, const xmlNodePtr child_node_ptr, std::vector<std::string>& algorithm);
    bool loadDefaultAlgorithms(const xmlDocPtr doc_ptr, const xmlNodePtr child_node_ptr, std::vector<std::string>& default_algorithm);
};

}



#endif

