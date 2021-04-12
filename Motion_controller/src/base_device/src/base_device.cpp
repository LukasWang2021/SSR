#include "base_device.h"
#include "stdio.h"

using namespace hal_space;

BaseDevice::BaseDevice(DeviceType type)
{
    type_ = type;
}

BaseDevice::~BaseDevice()
{}

DeviceType BaseDevice::getDeviceType()
{
    return type_;
}

BaseDevice::BaseDevice():
    type_(DEVICE_TYPE_INVALID)
{

}

