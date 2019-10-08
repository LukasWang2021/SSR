#include "controller_param.h"
#include "common_file_path.h"
#include "common_log.h"
#include <string>

using namespace fst_ctrl;

ControllerParam::ControllerParam():
    file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(fst_log::MSG_LEVEL_ERROR),  // default is Error Level
    routine_cycle_time_(100000),
    heartbeat_cycle_time_(0),
    routine_thread_priority_(50),
    reset_max_time_(5000000),
    robot_state_timeout_(1000000),
    enable_controller_heartbeat_(false),
    heartbeat_thread_priority_(50),
    enable_log_service_(false),
    enable_virtual_core1_(false),
    virtual_core1_thread_priority_(50),
    max_reg_publish_number_(0),
    max_io_publish_number_(0),
    max_continuous_manual_move_timeout_(0),
    max_unknown_user_op_mode_timeout_(0),
    max_limited_global_vel_ratio_(0),
    max_limited_global_acc_ratio_(0),
    loop_count_(0),
    enable_set_vel_in_auto_(false)
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
        || !yaml_help_.getParam("heartbeat_cycle_time", heartbeat_cycle_time_)
        || !yaml_help_.getParam("routine_thread_priority", routine_thread_priority_)
        || !yaml_help_.getParam("reset_max_time", reset_max_time_)
        || !yaml_help_.getParam("robot_state_timeout", robot_state_timeout_)
        || !yaml_help_.getParam("enable_controller_heartbeat", enable_controller_heartbeat_)
        || !yaml_help_.getParam("heartbeat_thread_priority", heartbeat_thread_priority_)
        || !yaml_help_.getParam("enable_log_service", enable_log_service_)
        || !yaml_help_.getParam("enable_virtual_core1", enable_virtual_core1_)
        || !yaml_help_.getParam("virtual_core1_thread_priority", virtual_core1_thread_priority_)
        || !yaml_help_.getParam("max_reg_publish_number", max_reg_publish_number_)
        || !yaml_help_.getParam("max_io_publish_number", max_io_publish_number_)
        || !yaml_help_.getParam("max_continuous_manual_move_timeout", max_continuous_manual_move_timeout_)
        || !yaml_help_.getParam("max_unknown_user_op_mode_timeout", max_unknown_user_op_mode_timeout_)
        || !yaml_help_.getParam("max_limited_global_vel_ratio", max_limited_global_vel_ratio_)
        || !yaml_help_.getParam("max_limited_global_acc_ratio", max_limited_global_acc_ratio_)
        || !yaml_help_.getParam("loop_count", loop_count_)
        || !yaml_help_.getParam("enable_set_vel_in_auto", enable_set_vel_in_auto_))
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
        || !yaml_help_.setParam("heartbeat_cycle_time", heartbeat_cycle_time_)
        || !yaml_help_.setParam("routine_thread_priority", routine_thread_priority_)
        || !yaml_help_.setParam("reset_max_time", reset_max_time_)
        || !yaml_help_.setParam("robot_state_timeout", robot_state_timeout_)
        || !yaml_help_.setParam("enable_controller_heartbeat", enable_controller_heartbeat_)
        || !yaml_help_.setParam("heartbeat_thread_priority", heartbeat_thread_priority_)
        || !yaml_help_.setParam("enable_log_service", enable_log_service_)
        || !yaml_help_.setParam("enable_virtual_core1", enable_virtual_core1_)
        || !yaml_help_.setParam("virtual_core1_thread_priority", virtual_core1_thread_priority_)
        || !yaml_help_.setParam("max_reg_publish_number", max_reg_publish_number_)
        || !yaml_help_.setParam("max_io_publish_number", max_io_publish_number_)
        || !yaml_help_.setParam("max_continuous_manual_move_timeout", max_continuous_manual_move_timeout_)
        || !yaml_help_.setParam("max_unknown_user_op_mode_timeout", max_unknown_user_op_mode_timeout_)
        || !yaml_help_.setParam("max_limited_global_vel_ratio", max_limited_global_vel_ratio_)
        || !yaml_help_.setParam("max_limited_global_acc_ratio", max_limited_global_acc_ratio_)
        || !yaml_help_.setParam("loop_count", loop_count_)
        || !yaml_help_.setParam("enable_set_vel_in_auto", enable_set_vel_in_auto_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    }
}

