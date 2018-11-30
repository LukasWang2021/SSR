#include "base_device.h"

using namespace fst_hal;

BaseDevice::BaseDevice(int address, DeviceType type):
    is_valid_(false)
{
    address_ = address;
    type_ = type;
}

BaseDevice::~BaseDevice()
{

}

int BaseDevice::getAddress()
{
    return address_;
}

DeviceType BaseDevice::getDeviceType()
{
    return type_;
}

void BaseDevice::setValid(bool is_valid)
{
    is_valid_ = is_valid;
}

bool BaseDevice::isValid()
{
    return is_valid_;
}

BaseDevice::BaseDevice():
    address_(0), type_(DEVICE_TYPE_INVALID), is_valid_(false)
{

}

