#ifndef FST_SAFETY_DEVICE_PARAM_H
#define FST_SAFETY_DEVICE_PARAM_H


#include "parameter_manager/parameter_manager_param_group.h"

namespace fst_hal
{
class FstSafetyDeviceParam
{
public:
    FstSafetyDeviceParam();
    ~FstSafetyDeviceParam();

    bool loadParam();
    bool saveParam();

    // param to load & save
    int log_level_;
    int cycle_time_;    // thread cycle time, ms
    bool is_virtual_;

private:
    fst_parameter::ParamGroup yaml_help_;
    std::string file_path_;
};

}


#endif

