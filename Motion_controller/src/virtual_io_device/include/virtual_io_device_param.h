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
    std::string device_type_;
    std::string comm_type_;
    uint32_t max_DI_number_;
    uint32_t max_DO_number_;

private:
    fst_parameter::ParamGroup yaml_help_;
    std::string file_path_;
};

}


#endif

