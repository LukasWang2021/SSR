#ifndef DEVICE_VERSION_H
#define DEVICE_VERSION_H


#include "parameter_manager/parameter_manager_param_group.h"
#include "io_manager.h"
#include "fst_safety_device.h"

namespace fst_ctrl
{
class DeviceVersion
{
public:
    DeviceVersion();
    ~DeviceVersion();

    void init(fst_log::Logger* log_ptr,fst_hal::IoManager* io_manager_ptr, fst_hal::FstSafetyDevice* safety_device_ptr);
    
    ErrorCode getServoParamVersion(std::string *name, std::string *version);

    const int SERVO_PARAM_VER_ADDR = 0;
    const int SERVO_PARAM_SUM_ADDR = 1;

private:
    fst_parameter::ParamGroup yaml_help_;
    fst_log::Logger* log_ptr_;
    std::string file_path_;
    int servo_param_version_;
    int servo_param_sum_;
    
    fst_hal::IoManager* io_manager_ptr_;
    fst_hal::FstSafetyDevice* safety_device_ptr_;
};

}


#endif

