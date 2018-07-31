#include "controller_param.h"
#include "common_file_path.h"
#include <string>

using namespace fst_ctrl;

ControllerParam::ControllerParam():
    file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(3),  // default is Error Level
    routine_cycle_time_(0),
    rt_cycle_time_(0),
    heartbeat_cycle_time_(0)
{
    file_path_ += "controller.yaml";
}

ControllerParam::~ControllerParam()
{

}

bool ControllerParam::loadParam()
{    
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("routine_cycle_time", routine_cycle_time_)
        || !yaml_help_.getParam("rt_cycle_time", rt_cycle_time_)
        || !yaml_help_.getParam("heartbeat_cycle_time", heartbeat_cycle_time_))
    {
        return false;
    }
    else    
    {
        return true;
    }
}

bool ControllerParam::saveParam()
{
    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("routine_cycle_time", routine_cycle_time_)
        || !yaml_help_.setParam("rt_cycle_time", rt_cycle_time_)
        || !yaml_help_.setParam("heartbeat_cycle_time", heartbeat_cycle_time_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    }
}

