#include "device_manager.h"
#include "fst_io_device.h"
#include "fst_safety_device.h"
#include "fst_axis_device.h"
#include "virtual_axis_device.h"


using namespace fst_hal;


DeviceManager::DeviceManager():
    log_ptr_(NULL),
    param_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new DeviceManagerParam();
    FST_LOG_INIT("DeviceManager");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
}

DeviceManager::~DeviceManager()
{

}

bool DeviceManager::init()
{
    if(!param_ptr_->loadParam())
    {
        FST_ERROR("Failed to load device manager component parameters");
        return false;
    } 
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_); 

    device_xml_ptr_ = new DeviceXml(log_ptr_, param_ptr_);
    if(device_xml_ptr_ == NULL
        || !device_xml_ptr_->loadDeviceConfig())
    {
        FST_ERROR("Failed to load device config file");
        return false;
    }

    std::vector<DeviceConfig>::iterator it;
    BaseDevice* device_ptr;
    for(it = device_xml_ptr_->device_config_list_.begin(); it != device_xml_ptr_->device_config_list_.end(); ++it)
    {
        switch(it->device_type)
        {
            case DEVICE_TYPE_FST_AXIS: return false;
            case DEVICE_TYPE_FST_IO: return false;
            case DEVICE_TYPE_FST_SAFETY: return false;
            case DEVICE_TYPE_FST_ANYBUS: return false;
            case DEVICE_TYPE_VIRTUAL_AXIS: device_ptr = new VirtualAxisDevice(it->address);
            case DEVICE_TYPE_VIRTUAL_IO: return false;
            case DEVICE_TYPE_VIRTUAL_SAFETY: return false;
            case DEVICE_TYPE_NORMAL: return false;
            default: return false;
        }

        if(device_ptr == NULL
            || !device_ptr->init())
        {
            return false;
        }
        device_map_[it->device_index] = device_ptr;
    }
    
    return true;
}

BaseDevice* DeviceManager::getDevicePtrByDeviceIndex(int device_index)
{
    std::map<int, BaseDevice*>::iterator it;
    it = device_map_.find(device_index);
    if(it != device_map_.end())
    {
        return it->second;
    }
    else
    {
        return NULL;
    }
}

std::vector<DeviceInfo> DeviceManager::getDeviceList()
{
    DeviceInfo info;
    std::vector<DeviceInfo> device_list;
    std::map<int, BaseDevice*>::iterator it;
    for(it = device_map_.begin(); it != device_map_.end(); ++it)
    {
        info.index = it->first;
        info.address = it->second->getAddress();
        info.type = it->second->getDeviceType();
        info.is_valid = it->second->isValid();
        device_list.push_back(info);
    }
    return device_list;
}

bool DeviceManager::addDevice(int device_index, BaseDevice* device_ptr)
{
    std::map<int, BaseDevice*>::iterator it;
    it = device_map_.find(device_index);
    if(it != device_map_.end())
    {
        return false;
    }

    if(device_ptr == NULL)
    {
        return false;
    }
    
    for(it = device_map_.begin(); it != device_map_.end(); ++it)
    {
        if(it->second == device_ptr)
        {
            return false;
        }
    }

    device_map_[device_index] = device_ptr;
    return true;
}


