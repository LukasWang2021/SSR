#ifndef BASE_DEVICE_H
#define BASE_DEVICE_H

namespace fst_hal
{
typedef enum
{
    DEVICE_TYPE_INVALID = 0,
    DEVICE_TYPE_FST_AXIS = 1,
    DEVICE_TYPE_FST_IO = 2,
    DEVICE_TYPE_FST_SAFETY = 3,
    DEVICE_TYPE_FST_ANYBUS = 4,
    DEVICE_TYPE_VIRTUAL_AXIS = 5,
    DEVICE_TYPE_VIRTUAL_IO = 6,
    DEVICE_TYPE_VIRTUAL_SAFETY = 7,
    DEVICE_TYPE_NORMAL = 8,
    DEVICE_TYPE_MODBUS = 9,
}DeviceType;

class BaseDevice
{
public:
    BaseDevice(int address, DeviceType type);
    ~BaseDevice();

    virtual bool init() = 0;

    int getAddress();
    DeviceType getDeviceType();
    void setValid(bool is_valid);
    bool isValid();

private:
    BaseDevice();
    
    int address_;
    DeviceType type_;
    bool is_valid_;     // to represent if the device can work properly
};

}

#endif

