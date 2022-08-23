#ifndef BASE_DEVICE_H
#define BASE_DEVICE_H
#include "common_error_code.h"
#include "common_datatype.h"
#include <string>

namespace hal_space
{
typedef enum
{
    DEVICE_TYPE_INVALID = 0,
    DEVICE_TYPE_DIO = 1,
    DEVICE_TYPE_AIO = 2,
    DEVICE_TYPE_SAFETY = 3,
    DEVICE_TYPE_FOC = 4
}DeviceType;

class BaseDevice
{
public:
    BaseDevice(DeviceType type);
    virtual ~BaseDevice();

    virtual bool init(bool is_real) = 0;
    virtual ErrorCode updateStatus() = 0;
    virtual Device_t* openDevice(std::string device_path, uint32_t base_address, size_t byte_size);
    virtual void closeDevice(Device_t* device);

    DeviceType getDeviceType();

private:
    BaseDevice();
    DeviceType type_;
};

}

#endif

