#include "fst_safety_device.h"


using namespace fst_hal;


FstSafetyDevice::FstSafetyDevice(int address):
    BaseDevice(address, fst_hal::DEVICE_TYPE_FST_SAFETY),
    log_ptr_(NULL),
    param_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new FstSafetyDeviceParam();
}

FstSafetyDevice::~FstSafetyDevice()
{

}

bool FstSafetyDevice::init()
{
    return true;
}

FstSafetyDevice::FstSafetyDevice():
    BaseDevice(0, fst_hal::DEVICE_TYPE_INVALID)
{

}

void FstSafetyDevice::updateThreadFunc()
{

}

