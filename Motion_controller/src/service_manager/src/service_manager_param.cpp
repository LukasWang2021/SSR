#include "service_manager_param.h"
#include "common_file_path.h"
#include <string>

using namespace fst_service_manager;

ServiceManagerParam::ServiceManagerParam():
    log_level_(3),  // default is Error Level
    cycle_time_(10000),
    file_path_(COMPONENT_PARAM_FILE_DIR)
{
    file_path_ += "service_manager.yaml";
}

ServiceManagerParam::~ServiceManagerParam()
{

}

bool ServiceManagerParam::loadParam()
{
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("cycle_time", cycle_time_)
        || !yaml_help_.getParam("max_barecore_timeout_count", max_barecore_timeout_count_)
        || !yaml_help_.getParam("heartbeat_with_barecore_count", heartbeat_with_barecore_count_)
        || !yaml_help_.getParam("max_send_resp_count", max_send_resp_count_))
    {
        return false;
    }
    else    
    {
        return true;
    } 
}

bool ServiceManagerParam::saveParam()
{
    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("cycle_time", cycle_time_)
        || !yaml_help_.setParam("max_barecore_timeout_count", max_barecore_timeout_count_)
        || !yaml_help_.setParam("heartbeat_with_barecore_count", heartbeat_with_barecore_count_)
        || !yaml_help_.setParam("max_send_resp_count", max_send_resp_count_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    } 
}

