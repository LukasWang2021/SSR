#include "forces_config.h"


using namespace system_model_space;
using namespace base_space;
using namespace std;

ForcesConfig::ForcesConfig(std::string file_path):
    file_path_(file_path), is_valid_(false)
{

}

ForcesConfig::~ForcesConfig()
{

}

bool ForcesConfig::load()
{
    xmlDocPtr doc_ptr;
    doc_ptr = xmlParseFile(file_path_.c_str());
    if(doc_ptr == NULL)
    {
        return false;
    } 

    force_config_.clear();
    xmlXPathObjectPtr node_set = xml_help_.getNodeObject(doc_ptr, BAD_CAST("/ForcesConfig/ForceConfig"));
    for(int i = 0; i < node_set->nodesetval->nodeNr; i++)
    {
        ForceConfig_t force_config;
        if(!loadForceConfig(doc_ptr, node_set->nodesetval->nodeTab[i], force_config))
        {
            return false;
        }
        force_config_.push_back(force_config);
    }
    xmlFree(node_set);
    xmlFreeDoc(doc_ptr);
    is_valid_ = true;
    return true;
}

bool ForcesConfig::isValid()
{
    return is_valid_;
}

std::vector<ForceConfig_t>& ForcesConfig::getRef()
{
    return force_config_;
}

bool ForcesConfig::loadForceConfig(const xmlDocPtr doc_ptr, const xmlNodePtr child_node_ptr, ForceConfig_t& force_config)
{
	if(!xml_help_.getDataFromNode<int32_t>(doc_ptr, child_node_ptr, force_config.force_id, BAD_CAST("./ForceId"))
        || !xml_help_.getDataFromNode<std::string>(doc_ptr, child_node_ptr, force_config.root_dir, BAD_CAST("./RootDir"))
        || !xml_help_.getDataFromNode<std::string>(doc_ptr, child_node_ptr, force_config.parameter, BAD_CAST("./Parameter")))
    {
        return false;
    }
   
    return true;
}

