#ifndef DYNAMIC_ALG_RTM_PARAM_H
#define DYNAMIC_ALG_RTM_PARAM_H

#include "yaml_help.h"

#define DYNAMIC_PARAM_0KG_FILE "dynamic_alg_rtm_0kg.yaml"
#define DYNAMIC_PARAM_1KG_FILE "dynamic_alg_rtm_1kg.yaml"
#define DYNAMIC_PARAM_3p5KG_FILE "dynamic_alg_rtm_3.5kg.yaml"
#define DYNAMIC_PARAM_7KG_FILE "dynamic_alg_rtm_7kg.yaml"

namespace basic_alg
{

typedef enum
{
    PARAM_0KG   = 0,
    PARAM_1KG   = 1,
    PARAM_3p5KG = 2,
    PARAM_7KG   = 3,
    PARAM_MAX   = 4,
}DynParam;

class DynamicAlgRTMParam
{
public:
    DynamicAlgRTMParam();
    ~DynamicAlgRTMParam();

    bool loadParam();
    bool saveParam();

    // param to load & save
    std::vector<int> motor_power_[PARAM_MAX];
    std::vector<double> motor_torque_[PARAM_MAX];
    std::vector<double> gear_ratio_[PARAM_MAX];
    double ZZR1[PARAM_MAX], FS1[PARAM_MAX], FV1[PARAM_MAX], 
           XXR2[PARAM_MAX], XY2[PARAM_MAX], XZR2[PARAM_MAX], YZ2[PARAM_MAX], ZZR2[PARAM_MAX], MXR2[PARAM_MAX], MY2[PARAM_MAX], FS2[PARAM_MAX], FV2[PARAM_MAX], 
           XXR3[PARAM_MAX], XYR3[PARAM_MAX], XZ3[PARAM_MAX], YZ3[PARAM_MAX], ZZR3[PARAM_MAX], MXR3[PARAM_MAX], MYR3[PARAM_MAX], Im3[PARAM_MAX], FS3[PARAM_MAX], FV3[PARAM_MAX],
           XXR4[PARAM_MAX], XY4[PARAM_MAX], XZ4[PARAM_MAX], YZ4[PARAM_MAX], ZZR4[PARAM_MAX], MX4[PARAM_MAX], MYR4[PARAM_MAX], Im4[PARAM_MAX], FS4[PARAM_MAX], FV4[PARAM_MAX], 
           XXR5[PARAM_MAX], XY5[PARAM_MAX], XZ5[PARAM_MAX], YZ5[PARAM_MAX], ZZR5[PARAM_MAX], MX5[PARAM_MAX], MYR5[PARAM_MAX], Im5[PARAM_MAX], FS5[PARAM_MAX], FV5[PARAM_MAX],
           XXR6[PARAM_MAX], XY6[PARAM_MAX], XZ6[PARAM_MAX], YZ6[PARAM_MAX], ZZ6[PARAM_MAX], MX6[PARAM_MAX], MY6[PARAM_MAX], Im6[PARAM_MAX], FS6[PARAM_MAX], FV6[PARAM_MAX];//puma, 52 parameters.

private:
    base_space::YamlHelp yaml_help_;
    std::string file_path_;
    std::string file_path_0kg_param_;
    std::string file_path_1kg_param_;
    std::string file_path_3p5kg_param_;
    std::string file_path_7kg_param_;
};

}


#endif

