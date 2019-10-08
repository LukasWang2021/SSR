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
    int robot_state_timeout_; //us
    bool enable_controller_heartbeat_;
    int heartbeat_thread_priority_;
    bool enable_log_service_;
    bool enable_virtual_core1_;
    int virtual_core1_thread_priority_;
    int max_reg_publish_number_;
    int max_io_publish_number_;
    int max_continuous_manual_move_timeout_;    //us
    int max_unknown_user_op_mode_timeout_;  //us
    double max_limited_global_vel_ratio_;  // 0~1
    double max_limited_global_acc_ratio_;  // 0~1
    int loop_count_;
    bool enable_set_vel_in_auto_;
private:
    fst_parameter::ParamGroup yaml_help_;
    std::string file_path_;
};

}


#endif

