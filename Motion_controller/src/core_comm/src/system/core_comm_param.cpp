#include "system/core_comm_param.h"
#include "common_file_path.h"
#include <string>

using namespace base_space;
using namespace core_comm_space;

CoreCommParam::CoreCommParam():
    file_path_(COMPONENT_PARAM_FILE_DIR)
{
    file_path_.append("core_comm.yaml");
}

CoreCommParam::~CoreCommParam()
{

}

bool CoreCommParam::loadParam()
{
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("device_path", device_path_)
        || !yaml_help_.getParam("config_file", config_file_)
        || !yaml_help_.getParam("cpu_id", cpu_id_))
    {
        return false;
    }
    else    
    {
        return true;
    }
}

bool CoreCommParam::loadParamSlave()
{
    std::string file_path = file_path_;
    file_path.append(".slave");
    if (!yaml_help_.loadParamFile(file_path.c_str())
        || !yaml_help_.getParam("device_path", device_path_)
        || !yaml_help_.getParam("cpu_id", cpu_id_))
    {
        return false;
    }
    else    
    {
        return true;
    }
}

bool CoreCommParam::saveParam()
{
    if(!yaml_help_.setParam("device_path", device_path_) 
        || !yaml_help_.setParam("config_file", config_file_)
        || !yaml_help_.setParam("cpu_id", cpu_id_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    }
}

