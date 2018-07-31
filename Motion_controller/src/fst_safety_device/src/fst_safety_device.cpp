#include "fst_safety_device.h"


using namespace fst_hal;


FstSafetyDevice::FstSafetyDevice(int address):
    BaseDevice(address, fst_hal::DEVICE_TYPE_FST_SAFETY)
{

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

