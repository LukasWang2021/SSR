#include "motion_control_param.h"
#include "common_file_path.h"
#include "common_log.h"
#include <string>

namespace fst_mc
{

MotionControlParam::MotionControlParam()
{
    file_path_ = COMPONENT_PARAM_FILE_DIR;
    file_path_ = file_path_ + "motion_control.yaml";

    rt_cycle_time_ = 5;
    non_rt_cycle_time_ = 1000;
    cycle_per_publish_ = 1000;
    enable_ros_publish_ = false;

    log_level_ = fst_log::MSG_LEVEL_ERROR;
}

MotionControlParam::~MotionControlParam()
{

}

bool MotionControlParam::loadParam()
{    
    if (yaml_help_.loadParamFile(file_path_.c_str()))
    {
        if (!yaml_help_.getParam("simulator.enable", enable_ros_publish_))              return false;
        if (!yaml_help_.getParam("simulator.cycle_per_publish", cycle_per_publish_))    return false;
        if (!yaml_help_.getParam("realtime_task.cycle_time", rt_cycle_time_))           return false;
        if (!yaml_help_.getParam("non_realtime_task.cycle_time", non_rt_cycle_time_))   return false;
        if (!yaml_help_.getParam("log_level", log_level_))                              return false;
        return true;
    }
    else
    {
        return false;
    }
}

bool MotionControlParam::saveParam()
{
    if (!yaml_help_.setParam("simulator.enable", enable_ros_publish_))              return false;
    if (!yaml_help_.setParam("simulator.cycle_per_publish", cycle_per_publish_))    return false;
    if (!yaml_help_.setParam("non_realtime_task.cycle_time", non_rt_cycle_time_))   return false;
    if (!yaml_help_.setParam("log_level", log_level_))                              return false;

    return yaml_help_.dumpParamFile(file_path_.c_str());
}

}
