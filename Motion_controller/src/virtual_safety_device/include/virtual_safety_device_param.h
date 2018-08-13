#ifndef VIRTUAL_SAFETY_DEVICE_PARAM_H
#define VIRTUAL_SAFETY_DEVICE_PARAM_H


#include "parameter_manager/parameter_manager_param_group.h"

namespace fst_hal
{
class VirtualSafetyDeviceParam
{
public:
    VirtualSafetyDeviceParam();
    ~VirtualSafetyDeviceParam();

    bool loadParam();
    bool saveParam();

    // param to load & save
    int log_level_;

private:
    fst_parameter::ParamGroup yaml_help_;
    std::string file_path_;
};

}


#endif

