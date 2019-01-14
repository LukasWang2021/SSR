#include "virtual_axis_device.h"


using namespace fst_hal;


VirtualAxisDevice::VirtualAxisDevice(int address):
    BaseDevice(address, fst_hal::DEVICE_TYPE_VIRTUAL_AXIS),
    status_(VIRTUAL_AXIS_DEVICE_INIT), target_position_(0),
    feedback_position_(0),
    log_ptr_(NULL),
    param_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new VirtualAxisDeviceParam();
    FST_LOG_INIT("VirtualAxisDevice");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
}

VirtualAxisDevice::~VirtualAxisDevice()
{
    if(log_ptr_ != NULL)
    {
        delete log_ptr_;
        log_ptr_ = NULL;
    }
    if(param_ptr_ != NULL)
    {
        delete param_ptr_;
        param_ptr_ = NULL;
    }
}

bool VirtualAxisDevice::init()
{
    if(!param_ptr_->loadParam())
    {
        FST_ERROR("Failed to load VirtualAxisDevice component parameters");
        return false;        
    }
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
    
    target_position_ = 0;
    feedback_position_ = 0;
    status_ = VIRTUAL_AXIS_DEVICE_DISABLE;
    return true;
}

bool VirtualAxisDevice::setEnable()
{
    if(status_ != VIRTUAL_AXIS_DEVICE_INIT)
    {
        status_ = VIRTUAL_AXIS_DEVICE_ENABLE;
        return true;
    }
    else
    {
        return false;
    }
}

bool VirtualAxisDevice::setDisable()
{
    if(status_ != VIRTUAL_AXIS_DEVICE_INIT)
    {
        status_ = VIRTUAL_AXIS_DEVICE_DISABLE;
        return true;
    }
    else
    {
        return false;
    }
}

void VirtualAxisDevice::setZeroPosition()
{
    target_position_ = 0;
    feedback_position_ = 0;
}

void VirtualAxisDevice::setTargetPosition(double position)
{
    target_position_ = position;
    feedback_position_ = position;
}

double VirtualAxisDevice::getTargetPosition()
{
    return target_position_;
}

double VirtualAxisDevice::getFeedBack()
{
    return feedback_position_;
}

VirtualAxisDeviceStatus VirtualAxisDevice::getStatus()
{
    return status_;
}

