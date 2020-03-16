#include "virtual_safety_device.h"


using namespace fst_hal;


VirtualSafetyDevice::VirtualSafetyDevice(int address):
    BaseDevice(address, fst_hal::DEVICE_TYPE_VIRTUAL_SAFETY),
    param_ptr_(NULL),
    log_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new VirtualSafetyDeviceParam();
}

VirtualSafetyDevice::~VirtualSafetyDevice()
{

}

bool VirtualSafetyDevice::init()
{
    if(!param_ptr_->loadParam())
    {
        FST_ERROR("Failed to load VirtualSafetyDevice component parameters");
        return false;        
    }
    return true;
}

