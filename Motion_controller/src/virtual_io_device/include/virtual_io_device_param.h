#ifndef VIRTUAL_IO_DEVICE_PARAM_H
#define VIRTUAL_IO_DEVICE_PARAM_H


#include "parameter_manager/parameter_manager_param_group.h"

namespace fst_hal
{
class VirtualIoDeviceParam
{
public:
    VirtualIoDeviceParam();
    ~VirtualIoDeviceParam();

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

