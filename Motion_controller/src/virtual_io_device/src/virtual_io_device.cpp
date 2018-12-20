#include "virtual_io_device.h"
#include <string.h>


using namespace fst_hal;


VirtualIoDevice::VirtualIoDevice(int address):
    BaseDevice(address, fst_hal::DEVICE_TYPE_VIRTUAL_IO),
    log_ptr_(NULL),
    param_ptr_(NULL),
    address_(address)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new VirtualIoDeviceParam();
}

VirtualIoDevice::~VirtualIoDevice()
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

bool VirtualIoDevice::init()
{
    if(!param_ptr_->loadParam())
    {
        FST_ERROR("Failed to load VirtualIoDevice component parameters");
        return false;        
    }

    dev_info_.address = address_;
    dev_info_.dev_type = DEVICE_TYPE_VIRTUAL_IO;
    dev_info_.device_type = param_ptr_->device_type_; 
    dev_info_.comm_type = param_ptr_->comm_type_;    
    dev_info_.DI_num = param_ptr_->max_DI_number_;
    dev_info_.DO_num = param_ptr_->max_DO_number_;
    dev_info_.RI_num = 0;
    dev_info_.RO_num = 0;
    dev_info_.is_valid = false;
    memset(&virtual_DI_, 0, sizeof(virtual_DI_));
    memset(&virtual_DO_, 0, sizeof(virtual_DO_));

    FST_INFO("virtual_io_device: address=%d, device_type=%s, comm_type=%s", 
        dev_info_.address, dev_info_.device_type.c_str(), dev_info_.comm_type.c_str());
        
    setValid(true);
    return true;
}

IODeviceInfo VirtualIoDevice::getDeviceInfo(void)
{
    dev_info_.is_valid = isValid();
    return dev_info_;
}

ErrorCode VirtualIoDevice::getDiValue(uint8_t port_offset, uint8_t &value)
{
    int frame = (port_offset - 1) / 8;
    int shift = (port_offset - 1) % 8;
    value = (virtual_DI_[frame] >> shift) & 0x01;
    return SUCCESS;
}

ErrorCode VirtualIoDevice::getDoValue(uint8_t port_offset, uint8_t &value)
{
    int frame = (port_offset - 1) / 8;
    int shift = (port_offset - 1) % 8;
    value = (virtual_DO_[frame] >> shift) & 0x01;
    return SUCCESS;
}

ErrorCode VirtualIoDevice::setDiValue(uint8_t port_offset, uint8_t value)
{
    int frame = (port_offset - 1) / 8;
    int shift = (port_offset - 1) % 8;
    
    if (value == 0) 
    {
        virtual_DI_[frame] &= ~(0x01 << shift);
    }
    else 
    {
        virtual_DI_[frame] |= 0x01 << shift;
    }

    return SUCCESS;
}
    
ErrorCode VirtualIoDevice::setDoValue(uint8_t port_offset, uint8_t value)
{
    int frame = (port_offset - 1) / 8;
    int shift = (port_offset - 1) % 8;
    
    if (value == 0) 
    {
        virtual_DO_[frame] &= ~(0x01 << shift);
    }
    else 
    {
        virtual_DO_[frame] |= 0x01 << shift;
    }

    return SUCCESS;
}
