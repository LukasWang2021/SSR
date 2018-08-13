#ifndef VIRTUAL_SAFETY_DEVICE_H
#define VIRTUAL_SAFETY_DEVICE_H


#include "base_device.h"
#include "virtual_safety_device_param.h"
#include "common_log.h"

namespace fst_hal
{
class VirtualSafetyDevice : public BaseDevice
{
public:
    VirtualSafetyDevice(int address);
    ~VirtualSafetyDevice();

    virtual bool init();

private:
    VirtualSafetyDeviceParam* param_ptr_;
    fst_log::Logger* log_ptr_;  
};

}

#endif


