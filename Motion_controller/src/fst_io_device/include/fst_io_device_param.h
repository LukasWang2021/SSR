#ifndef FST_IO_DEVICE_PARAM_H
#define FST_IO_DEVICE_PARAM_H


#include "parameter_manager/parameter_manager_param_group.h"

namespace fst_hal
{
class FstIoDeviceParam
{
public:
    FstIoDeviceParam();
    ~FstIoDeviceParam();

    bool loadParam();
    bool saveParam();

    // param to load & save
    int log_level_;
    int cycle_time_;    // thread cycle time, ms
    int input_byte_size_;
    int output_byte_size_;
    bool is_virtual_;

private:
    fst_parameter::ParamGroup yaml_help_;
    std::string file_path_;
};

}


#endif

