#include "virtual_io_device.h"


using namespace fst_hal;


VirtualIoDevice::VirtualIoDevice(int address):
    BaseDevice(address, fst_hal::DEVICE_TYPE_VIRTUAL_IO),
    log_ptr_(NULL),
    param_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new VirtualIoDeviceParam();
}

VirtualIoDevice::~VirtualIoDevice()
{

}

bool VirtualIoDevice::init()
{
    if(!param_ptr_->loadParam())
    {
        FST_ERROR("Failed to load VirtualIoDevice component parameters");
        return false;        
    }

    return true;
}


