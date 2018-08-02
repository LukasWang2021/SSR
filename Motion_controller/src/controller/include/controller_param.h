#ifndef CONTROLLER_PARAM_H
#define CONTROLLER_PARAM_H


#include "parameter_manager/parameter_manager_param_group.h"

namespace fst_ctrl
{
class ControllerParam
{
public:
    ControllerParam();
    ~ControllerParam();

    bool loadParam();
    bool saveParam();

    // param to load & save
    int log_level_;
    int routine_cycle_time_;    // us
    int rt_cycle_time_;         // us
    int heartbeat_cycle_time_;  // us
    int routine_thread_priority_;
    int reset_max_time_;        // us
private:
    fst_parameter::ParamGroup yaml_help_;
    std::string file_path_;
};

}


#endif

