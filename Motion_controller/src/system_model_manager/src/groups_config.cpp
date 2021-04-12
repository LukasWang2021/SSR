#include "groups_config.h"


using namespace system_model_space;
using namespace base_space;
using namespace std;


GroupsConfig::GroupsConfig(std::string file_path):
    file_path_(file_path), is_valid_(false)
{

}

GroupsConfig::~GroupsConfig()
{

}

bool GroupsConfig::load()
{
    xmlDocPtr doc_ptr;
    doc_ptr = xmlParseFile(file_path_.c_str());
    if(doc_ptr == NULL)
    {
        return false;
    } 
    group_config_.clear();
    xmlXPathObjectPtr node_set = xml_help_.getNodeObject(doc_ptr, BAD_CAST("/GroupsConfig/GroupConfig"));   
    for(int i = 0; i < node_set->nodesetval->nodeNr; i++)
    {
        GroupConfig_t group_config;
        if(!loadGroupConfig(doc_ptr, node_set->nodesetval->nodeTab[i], group_config))
        {
            return false;
        }
        group_config_.push_back(group_config);
    }
    xmlFree(node_set);
    xmlFreeDoc(doc_ptr);
    is_valid_ = true;
    return true;
}

bool GroupsConfig::isValid()
{
    return is_valid_;
}

std::vector<GroupConfig_t>& GroupsConfig::getRef()
{
    return group_config_;
}

bool GroupsConfig::loadGroupConfig(const xmlDocPtr doc_ptr, const xmlNodePtr child_node_ptr, GroupConfig_t& group_config)
{
	if(!xml_help_.getDataFromNode<int32_t>(doc_ptr, child_node_ptr, group_config.group_id, BAD_CAST("./GroupId"))
        || !xml_help_.getDataFromNode<std::string>(doc_ptr, child_node_ptr, group_config.root_dir, BAD_CAST("./RootDir"))
        || !xml_help_.getDataFromNode<std::string>(doc_ptr, child_node_ptr, group_config.kinematics, BAD_CAST("./Kinematics"))
        || !xml_help_.getDataFromNode<std::string>(doc_ptr, child_node_ptr, group_config.dynamics, BAD_CAST("./Dynamics"))
        || !xml_help_.getDataFromNode<std::string>(doc_ptr, child_node_ptr, group_config.application, BAD_CAST("./Application")))
    {
        return false;
    }
    if(!loadAxesId(doc_ptr, child_node_ptr, group_config.axis_id))
    {
        return false;
    }
    if(!loadAlgorithms(doc_ptr, child_node_ptr, group_config.algorithm))
    {
        return false;
    }
    if(!loadDefaultAlgorithms(doc_ptr, child_node_ptr, group_config.default_algorithm))
    {
        return false;
    }
    return true;
}

bool GroupsConfig::loadAxesId(const xmlDocPtr doc_ptr, const xmlNodePtr child_node_ptr, std::vector<int32_t>& axis_id)
{
    int32_t id;
    xmlXPathObjectPtr node_set = xml_help_.getNodeObject(doc_ptr, child_node_ptr, BAD_CAST("./AxesId/AxisId"));
    for(int i = 0; i < node_set->nodesetval->nodeNr; i++)
    {
        if(!xml_help_.getDataFromNode<int32_t>(node_set->nodesetval->nodeTab[i], id))
        {
            return false;
        }
        axis_id.push_back(id);
    }
    return true;
}

bool GroupsConfig::loadAlgorithms(const xmlDocPtr doc_ptr, const xmlNodePtr child_node_ptr, std::vector<std::string>& algorithm)
{
    std::string algorithm_str;
    xmlXPathObjectPtr node_set = xml_help_.getNodeObject(doc_ptr, child_node_ptr, BAD_CAST("./Algorithms/Algorithm"));
    for(int i = 0; i < node_set->nodesetval->nodeNr; i++)
    {
        if(!xml_help_.getDataFromNode<std::string>(node_set->nodesetval->nodeTab[i], algorithm_str))
        {
            return false;
        }
        algorithm.push_back(algorithm_str);
    }
    return true;
}

bool GroupsConfig::loadDefaultAlgorithms(const xmlDocPtr doc_ptr, const xmlNodePtr child_node_ptr, std::vector<std::string>& default_algorithm)
{
    std::string algorithm_str;
    xmlXPathObjectPtr node_set = xml_help_.getNodeObject(doc_ptr, child_node_ptr, BAD_CAST("./DefaultAlgorithms/Algorithm"));
    for(int i = 0; i < node_set->nodesetval->nodeNr; i++)
    {
        if(!xml_help_.getDataFromNode<std::string>(node_set->nodesetval->nodeTab[i], algorithm_str))
        {
            return false;
        }
        default_algorithm.push_back(algorithm_str);
    }
    return true;
}


