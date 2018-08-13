#ifndef VIRTUAL_IO_DEVICE_H
#define VIRTUAL_IO_DEVICE_H


#include "base_device.h"
#include "virtual_io_device_param.h"
#include "common_log.h"

namespace fst_hal
{
class VirtualIoDevice : public BaseDevice
{
public:
    VirtualIoDevice(int address);
    ~VirtualIoDevice();

    virtual bool init();

private:
    VirtualIoDeviceParam* param_ptr_;
    fst_log::Logger* log_ptr_;
};

}

#endif


