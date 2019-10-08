#include "dynamic_alg_rtm_param.h"
#include "common_file_path.h"
#include <string>

using namespace basic_alg;

DynamicAlgRTMParam::DynamicAlgRTMParam():
    file_path_(DYNAMICS_DIR),
    log_level_(3)  // default is Error Level
{
}

DynamicAlgRTMParam::~DynamicAlgRTMParam()
{

}

bool DynamicAlgRTMParam::loadParam()
{
    type_file_path_ = DYNAMICS_DIR;
    type_file_path_ += "dynamic_alg_rtm_type.yaml";
    if (!yaml_type_.loadParamFile(type_file_path_.c_str())
        || !yaml_type_.getParam("type_file_name", type_file_name_))
    {
        return false;
    }
    else
    {
        file_path_ += type_file_name_;
    }

    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("number_of_links", number_of_links_)
        || !yaml_help_.getParam("current_payload_id", current_payload_id_)
        || !yaml_help_.getParam("acc_scale_factor", acc_scale_factor_)
        || !yaml_help_.getParam("motor_power", motor_power_)
        || !yaml_help_.getParam("motor_torque", motor_torque_)
        || !yaml_help_.getParam("gear_ratio", gear_ratio_)
        || !yaml_help_.getParam("max_torque", max_torque_)

        || !yaml_help_.getParam("link_0/ZZR", ZZR1)
        || !yaml_help_.getParam("link_0/FS", FS1)
        || !yaml_help_.getParam("link_0/FV", FV1)

        || !yaml_help_.getParam("link_1/XXR", XXR2)
        || !yaml_help_.getParam("link_1/XY", XY2)
        || !yaml_help_.getParam("link_1/XZR", XZR2)
        || !yaml_help_.getParam("link_1/YZ", YZ2)
        || !yaml_help_.getParam("link_1/ZZR", ZZR2)
        || !yaml_help_.getParam("link_1/MXR", MXR2)
        || !yaml_help_.getParam("link_1/MY", MY2)
        || !yaml_help_.getParam("link_1/FS", FS2)
        || !yaml_help_.getParam("link_1/FV", FV2)
        
        || !yaml_help_.getParam("link_2/XXR", XXR3)
        || !yaml_help_.getParam("link_2/XYR", XYR3)
        || !yaml_help_.getParam("link_2/XZ", XZ3)
        || !yaml_help_.getParam("link_2/YZ", YZ3)
        || !yaml_help_.getParam("link_2/ZZR", ZZR3)
        || !yaml_help_.getParam("link_2/MXR", MXR3)
        || !yaml_help_.getParam("link_2/MYR", MYR3)
        || !yaml_help_.getParam("link_2/Im", Im3)
        || !yaml_help_.getParam("link_2/FS", FS3)
        || !yaml_help_.getParam("link_2/FV", FV3)
        
        || !yaml_help_.getParam("link_3/XXR", XXR4)
        || !yaml_help_.getParam("link_3/XY", XY4)
        || !yaml_help_.getParam("link_3/XZ", XZ4)
        || !yaml_help_.getParam("link_3/YZ", YZ4)
        || !yaml_help_.getParam("link_3/ZZR", ZZR4)
        || !yaml_help_.getParam("link_3/MX", MX4)
        || !yaml_help_.getParam("link_3/MYR", MYR4)
        || !yaml_help_.getParam("link_3/Im", Im4)
        || !yaml_help_.getParam("link_3/FS", FS4)
        || !yaml_help_.getParam("link_3/FV", FV4)
        
        || !yaml_help_.getParam("link_4/XXR", XXR5)
        || !yaml_help_.getParam("link_4/XY", XY5)
        || !yaml_help_.getParam("link_4/XZ", XZ5)
        || !yaml_help_.getParam("link_4/YZ", YZ5)
        || !yaml_help_.getParam("link_4/ZZR", ZZR5)
        || !yaml_help_.getParam("link_4/MX", MX5)
        || !yaml_help_.getParam("link_4/MYR", MYR5)
        || !yaml_help_.getParam("link_4/Im", Im5)
        || !yaml_help_.getParam("link_4/FS", FS5)
        || !yaml_help_.getParam("link_4/FV", FV5)
        
        || !yaml_help_.getParam("link_5/XXR", XXR6)
        || !yaml_help_.getParam("link_5/XY", XY6)
        || !yaml_help_.getParam("link_5/XZ", XZ6)
        || !yaml_help_.getParam("link_5/YZ", YZ6)
        || !yaml_help_.getParam("link_5/ZZ", ZZ6)
        || !yaml_help_.getParam("link_5/MX", MX6)
        || !yaml_help_.getParam("link_5/MY", MY6)
        || !yaml_help_.getParam("link_5/Im", Im6)
        || !yaml_help_.getParam("link_5/FS", FS6)
        || !yaml_help_.getParam("link_5/FV", FV6))
    {
        return false;
    }
    else
    {
        return true;
    } 
}

bool DynamicAlgRTMParam::saveParam()
{
    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("number_of_links", number_of_links_)
        || !yaml_help_.setParam("current_payload_id", current_payload_id_)
        || !yaml_help_.setParam("acc_scale_factor", acc_scale_factor_)
        || !yaml_help_.setParam("motor_power", motor_power_)
        || !yaml_help_.setParam("motor_torque", motor_torque_)
        || !yaml_help_.setParam("gear_ratio", gear_ratio_)
        || !yaml_help_.setParam("max_torque", max_torque_)

        || !yaml_help_.setParam("link_0/ZZR", ZZR1)
        || !yaml_help_.setParam("link_0/FS", FS1)
        || !yaml_help_.setParam("link_0/FV", FV1)

        || !yaml_help_.setParam("link_1/XXR", XXR2)
        || !yaml_help_.setParam("link_1/XY", XY2)
        || !yaml_help_.setParam("link_1/XZR", XZR2)
        || !yaml_help_.setParam("link_1/YZ", YZ2)
        || !yaml_help_.setParam("link_1/ZZR", ZZR2)
        || !yaml_help_.setParam("link_1/MXR", MXR2)
        || !yaml_help_.setParam("link_1/MY", MY2)
        || !yaml_help_.setParam("link_1/FS", FS2)
        || !yaml_help_.setParam("link_1/FV", FV2)
        
        || !yaml_help_.setParam("link_2/XXR", XXR3)
        || !yaml_help_.setParam("link_2/XYR", XYR3)
        || !yaml_help_.setParam("link_2/XZ", XZ3)
        || !yaml_help_.setParam("link_2/YZ", YZ3)
        || !yaml_help_.setParam("link_2/ZZR", ZZR3)
        || !yaml_help_.setParam("link_2/MXR", MXR3)
        || !yaml_help_.setParam("link_2/MYR", MYR3)
        || !yaml_help_.setParam("link_2/Im", Im3)
        || !yaml_help_.setParam("link_2/FS", FS3)
        || !yaml_help_.setParam("link_2/FV", FV3)
        
        || !yaml_help_.setParam("link_3/XXR", XXR4)
        || !yaml_help_.setParam("link_3/XY", XY4)
        || !yaml_help_.setParam("link_3/XZ", XZ4)
        || !yaml_help_.setParam("link_3/YZ", YZ4)
        || !yaml_help_.setParam("link_3/ZZR", ZZR4)
        || !yaml_help_.setParam("link_3/MX", MX4)
        || !yaml_help_.setParam("link_3/MYR", MYR4)
        || !yaml_help_.setParam("link_3/Im", Im4)
        || !yaml_help_.setParam("link_3/FS", FS4)
        || !yaml_help_.setParam("link_3/FV", FV4)
        
        || !yaml_help_.setParam("link_4/XXR", XXR5)
        || !yaml_help_.setParam("link_4/XY", XY5)
        || !yaml_help_.setParam("link_4/XZ", XZ5)
        || !yaml_help_.setParam("link_4/YZ", YZ5)
        || !yaml_help_.setParam("link_4/ZZR", ZZR5)
        || !yaml_help_.setParam("link_4/MX", MX5)
        || !yaml_help_.setParam("link_4/MYR", MYR5)
        || !yaml_help_.setParam("link_4/Im", Im5)
        || !yaml_help_.setParam("link_4/FS", FS5)
        || !yaml_help_.setParam("link_4/FV", FV5)
        
        || !yaml_help_.setParam("link_5/XXR", XXR6)
        || !yaml_help_.setParam("link_5/XY", XY6)
        || !yaml_help_.setParam("link_5/XZ", XZ6)
        || !yaml_help_.setParam("link_5/YZ", YZ6)
        || !yaml_help_.setParam("link_5/ZZ", ZZ6)
        || !yaml_help_.setParam("link_5/MX", MX6)
        || !yaml_help_.setParam("link_5/MYR", MY6)
        || !yaml_help_.setParam("link_5/Im", Im6)
        || !yaml_help_.setParam("link_5/FS", FS6)
        || !yaml_help_.setParam("link_5/FV", FV6)

        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    } 
}

