#ifndef BASE_DEVICE_H
#define BASE_DEVICE_H
#include "common_error_code.h"

namespace hal_space
{
typedef enum
{
    DEVICE_TYPE_INVALID = 0,
    DEVICE_TYPE_DIO = 1,
    DEVICE_TYPE_AIO = 2,
    DEVICE_TYPE_SAFETY = 3,
}DeviceType;

class BaseDevice
{
public:
    BaseDevice(DeviceType type);
    virtual ~BaseDevice();

    virtual bool init(bool is_real) = 0;
    virtual ErrorCode updateStatus() = 0;

    DeviceType getDeviceType();

private:
    BaseDevice();
    DeviceType type_;
};

}

#endif

