#ifndef DYNAMIC_ALG_RTM_PARAM_H
#define DYNAMIC_ALG_RTM_PARAM_H


#include "parameter_manager/parameter_manager_param_group.h"

namespace basic_alg
{
class DynamicAlgRTMParam
{
public:
    DynamicAlgRTMParam();
    ~DynamicAlgRTMParam();

    bool loadParam();
    bool saveParam();

    // param to load & save
    int log_level_;
    int number_of_links_;
    int current_payload_id_;
    double acc_scale_factor_;
    std::vector<int> motor_power_;
    std::vector<double> motor_torque_;
    std::vector<double> gear_ratio_;
    std::vector<double> max_torque_;
    double ZZR1, FS1, FV1, XXR2, XY2, XZR2, YZ2, ZZR2, MXR2, MY2, FS2, FV2, XXR3, XYR3, XZ3, YZ3, ZZR3, MXR3, MYR3, Im3, FS3, FV3,
           XXR4, XY4, XZ4, YZ4, ZZR4, MX4, MYR4, Im4, FS4, FV4, XXR5, XY5, XZ5, YZ5, ZZR5, MX5, MYR5, Im5, FS5, FV5,
           XXR6, XY6, XZ6, YZ6, ZZ6, MX6, MY6, Im6, FS6, FV6;//puma, 52 parameters.

private:
    fst_parameter::ParamGroup yaml_type_;
    std::string type_file_path_;
    std::string type_file_name_;
    
    fst_parameter::ParamGroup yaml_help_;
    std::string file_path_;
};

}


#endif

