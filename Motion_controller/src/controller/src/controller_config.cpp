#include "controller_config.h"
#include "common_file_path.h"
#include "log_manager_datatype.h"

using namespace user_space;
using namespace std;

ControllerConfig::ControllerConfig():
    log_level_(LOG_INFO),   
    routine_cycle_time_(10000),
    planner_cycle_time_(10000),
    priority_cycle_time_(10000),
    realtime_cycle_time_(4000),
    routine_thread_priority_(50),
    planner_thread_priority_(60),
    priority_thread_priority_(70),
    realtime_thread_priority_(80),
    dio_exist_(false),
    safety_exist_(false),
    file_path_(COMPONENT_PARAM_FILE_DIR)
{
    file_path_ += string("controller.yaml");
}

ControllerConfig::~ControllerConfig()
{
}

bool ControllerConfig::load()
{    
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("routine_cycle_time", routine_cycle_time_)
        || !yaml_help_.getParam("planner_cycle_time", planner_cycle_time_)
        || !yaml_help_.getParam("priority_cycle_time", priority_cycle_time_)
        || !yaml_help_.getParam("realtime_cycle_time", realtime_cycle_time_)
        || !yaml_help_.getParam("rpc_cycle_time", rpc_cycle_time_)
        || !yaml_help_.getParam("rpc_cycle_time", online_traj_cycle_time_)
        || !yaml_help_.getParam("routine_thread_priority", routine_thread_priority_)
        || !yaml_help_.getParam("planner_thread_priority", planner_thread_priority_)
        || !yaml_help_.getParam("priority_thread_priority", priority_thread_priority_)
        || !yaml_help_.getParam("realtime_thread_priority", realtime_thread_priority_)
        || !yaml_help_.getParam("rpc_thread_priority", rpc_thread_priority_)
        || !yaml_help_.getParam("online_traj_thread_priority", rpc_thread_priority_)
        || !yaml_help_.getParam("dio_exist", dio_exist_)
        || !yaml_help_.getParam("safety_exist", safety_exist_))
    {
        return false;
    }
    else    
    {
        return true;
    }
}

bool ControllerConfig::save()
{
    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("routine_cycle_time", routine_cycle_time_)
        || !yaml_help_.setParam("planner_cycle_time", planner_cycle_time_)
        || !yaml_help_.setParam("priority_cycle_time", priority_cycle_time_)
        || !yaml_help_.setParam("realtime_cycle_time", realtime_cycle_time_)
        || !yaml_help_.setParam("routine_thread_priority", routine_thread_priority_)
        || !yaml_help_.setParam("planner_thread_priority", planner_thread_priority_)
        || !yaml_help_.setParam("priority_thread_priority", priority_thread_priority_)
        || !yaml_help_.setParam("realtime_thread_priority", realtime_thread_priority_)
        || !yaml_help_.setParam("dio_exist", dio_exist_)
        || !yaml_help_.setParam("safety_exist", safety_exist_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    }
}

