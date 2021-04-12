#include "system_model_manager_config.h"
#include "common_file_path.h"

using namespace system_model_space;
using namespace base_space;


SystemModelManagerConfig::SystemModelManagerConfig():
    system_model_root_dir_("")
{
    file_path_ = std::string(COMPONENT_PARAM_FILE_DIR) + "system_model_manager.yaml";
}

SystemModelManagerConfig::~SystemModelManagerConfig()
{

}

bool SystemModelManagerConfig::load()
{ 
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("system_model_root_dir", system_model_root_dir_))
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool SystemModelManagerConfig::save()
{
    if(!yaml_help_.setParam("system_model_root_dir", system_model_root_dir_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    }

}

