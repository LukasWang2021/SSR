#ifndef MOTION_CONTROL_PARAM_H
#define MOTION_CONTROL_PARAM_H


#include "parameter_manager/parameter_manager_param_group.h"

namespace fst_mc
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
    int     priority_cycle_time_;
    int     realtime_cycle_time_;
    int     log_level_;
    std::string model_name_;

private:
    fst_parameter::ParamGroup yaml_help_;
    std::string file_path_;
};

}


#endif

