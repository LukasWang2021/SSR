#ifndef MOTION_CONTROL_PARAM_H
#define MOTION_CONTROL_PARAM_H

#include "yaml_help.h"

namespace group_space
{

class MotionControlParam
{
public:
    MotionControlParam();
    ~MotionControlParam();

    bool loadParam();
    bool saveParam();

    // param to load & save
    bool    enable_ros_publish_;
    int     cycle_per_publish_;
    int     common_cycle_time_;
    int     planner_cycle_time_;
    int     priority_cycle_time_;
    int     realtime_cycle_time_;
    int     log_level_;
    std::string model_name_;

private:
    base_space::YamlHelp yaml_help_;
    std::string file_path_;
};

}


#endif

