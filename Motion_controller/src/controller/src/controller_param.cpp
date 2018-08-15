#include "controller_param.h"
#include "common_file_path.h"
#include "common_log.h"
#include <string>

using namespace fst_ctrl;

ControllerParam::ControllerParam():
    file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(fst_log::MSG_LEVEL_ERROR),  // default is Error Level
    routine_cycle_time_(100000),
    rt_cycle_time_(0),
    heartbeat_cycle_time_(0),
    routine_thread_priority_(50),
    reset_max_time_(5000000),
    enable_controller_heartbeat_(false)
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
        || !yaml_help_.getParam("heartbeat_cycle_time", heartbeat_cycle_time_)
        || !yaml_help_.getParam("routine_thread_priority", routine_thread_priority_)
        || !yaml_help_.getParam("reset_max_time", reset_max_time_)
        || !yaml_help_.getParam("enable_controller_heartbeat", enable_controller_heartbeat_))
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
        || !yaml_help_.setParam("routine_thread_priority", routine_thread_priority_)
        || !yaml_help_.setParam("reset_max_time", reset_max_time_)
        || !yaml_help_.setParam("enable_controller_heartbeat", enable_controller_heartbeat_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    }
}

