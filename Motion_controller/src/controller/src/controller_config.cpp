#include "controller_config.h"
#include "common_file_path.h"
#include "log_manager_datatype.h"

using namespace user_space;
using namespace std;

ControllerConfig::ControllerConfig():
    log_level_(LOG_INFO),   
    routine_cycle_time_(100000),
    realtime_cycle_time_(4000),
    routine_thread_priority_(50),
    realtime_thread_priority_(80),
    dio_exist_(false),
    aio_exist_(false),
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
        || !yaml_help_.getParam("realtime_cycle_time", realtime_cycle_time_)
        || !yaml_help_.getParam("routine_thread_priority", routine_thread_priority_)
        || !yaml_help_.getParam("realtime_thread_priority", realtime_thread_priority_)
        || !yaml_help_.getParam("dio_exist", dio_exist_)
        || !yaml_help_.getParam("aio_exist", aio_exist_))
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
        || !yaml_help_.setParam("realtime_cycle_time", realtime_cycle_time_)
        || !yaml_help_.setParam("routine_thread_priority", routine_thread_priority_)
        || !yaml_help_.setParam("realtime_thread_priority", realtime_thread_priority_)
        || !yaml_help_.setParam("dio_exist", dio_exist_)
        || !yaml_help_.setParam("aio_exist", aio_exist_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    }
}

