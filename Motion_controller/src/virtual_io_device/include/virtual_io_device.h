#ifndef VIRTUAL_IO_DEVICE_H
#define VIRTUAL_IO_DEVICE_H


#include "base_device.h"
#include "virtual_io_device_param.h"
#include "common_log.h"
#include "fst_io_device.h"

namespace fst_hal
{
class VirtualIoDevice : public BaseDevice
{
public:
    VirtualIoDevice(int address);
    ~VirtualIoDevice();

    virtual bool init();

    IODeviceInfo getDeviceInfo(void);

    ErrorCode getDiValue(uint8_t port_offset, uint8_t &value);
    ErrorCode getDoValue(uint8_t port_offset, uint8_t &value);

    ErrorCode setDiValue(uint8_t port_offset, uint8_t value);
    ErrorCode setDoValue(uint8_t port_offset, uint8_t value);

private:
    VirtualIoDeviceParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    int address_;
    IODeviceInfo dev_info_;
    uint8_t virtual_DI_[256];
    uint8_t virtual_DO_[256];

};

}

#endif


