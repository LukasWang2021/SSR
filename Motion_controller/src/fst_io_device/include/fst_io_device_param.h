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
    std::string device_type_;
    std::string comm_type_;
    uint32_t max_DI_number_;
    uint32_t max_DO_number_;
    uint32_t max_RI_number_;
    uint32_t max_RO_number_;
    bool is_virtual_;
    int comm_tolerance_;
  

private:
    fst_parameter::ParamGroup yaml_help_;
    std::string file_path_;
};

}


#endif

