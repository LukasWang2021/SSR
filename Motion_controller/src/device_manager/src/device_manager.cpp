#include "device_manager.h"
#include "fst_io_device.h"
#include "fst_safety_device.h"
#include "fst_axis_device.h"
#include "virtual_axis_device.h"
#include "modbus_manager.h"
#include "error_code.h"


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
    std::map<int, BaseDevice*>::iterator it;
    static int count_delete = 0;//temporary delete IOdevice once.to do delete.
    for(it = device_map_.begin(); it != device_map_.end(); ++it)
    {
        if(it->second != NULL)
        {
            switch(it->second->getDeviceType())
            {
                case DEVICE_TYPE_FST_AXIS:       break;
                case DEVICE_TYPE_FST_IO:
                    if(count_delete == 0) {
                        delete (FstIoDevice *) it->second;
                    }
                    count_delete++;//to do delete
                    break;
                case DEVICE_TYPE_FST_SAFETY:     delete (FstSafetyDevice*)it->second; break;
                case DEVICE_TYPE_FST_ANYBUS:     break;
                case DEVICE_TYPE_VIRTUAL_AXIS:   delete (VirtualAxisDevice*)it->second; break;
                case DEVICE_TYPE_VIRTUAL_IO:     break;
                case DEVICE_TYPE_VIRTUAL_SAFETY: break;
                case DEVICE_TYPE_NORMAL:         break;
                case DEVICE_TYPE_MODBUS:         delete (ModbusManager*)it->second; break;
            }
            it->second = NULL;
        }
    }
    device_map_.clear();

    if(device_xml_ptr_ != NULL)
    {
        delete device_xml_ptr_;
        device_xml_ptr_ = NULL;
    }
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

ErrorCode DeviceManager::init()
{
    if(!param_ptr_->loadParam())
    {
        FST_ERROR("Failed to load device manager component parameters");
        return DEVICE_MANAGER_LOAD_PARAM_FAILED;
    }
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_); 

    device_xml_ptr_ = new DeviceXml(log_ptr_, param_ptr_);
    if(device_xml_ptr_ == NULL
        || !device_xml_ptr_->loadDeviceConfig())
    {
        FST_ERROR("Failed to load device config file");
        return DEVICE_MANAGER_LOAD_DEVICE_CONFIG_FAILED;
    }

    std::vector<DeviceConfig>::iterator it;
    BaseDevice* device_ptr;
    static int count_new = 0;//temporary new IOdevice once.to do delete.
    for(it = device_xml_ptr_->device_config_list_.begin(); it != device_xml_ptr_->device_config_list_.end(); ++it)
    {
        printf("init device .address =%d\n", it->address);
        switch(it->device_type)
        {
            case DEVICE_TYPE_FST_AXIS:       return DEVICE_MANAGER_INVALID_DEVICE_TYPE;
            case DEVICE_TYPE_FST_IO:
                if(count_new == 0) { //temporary used.
                    device_ptr = new FstIoDevice(it->address);
                }
                count_new++;
				break;
            case DEVICE_TYPE_FST_SAFETY:
                device_ptr = new FstSafetyDevice(it->address);break;
            case DEVICE_TYPE_FST_ANYBUS:     return DEVICE_MANAGER_INVALID_DEVICE_TYPE;
            case DEVICE_TYPE_VIRTUAL_AXIS:
				device_ptr = new VirtualAxisDevice(it->address); break;
            case DEVICE_TYPE_VIRTUAL_IO:     return DEVICE_MANAGER_INVALID_DEVICE_TYPE;
            case DEVICE_TYPE_VIRTUAL_SAFETY: return DEVICE_MANAGER_INVALID_DEVICE_TYPE;
            case DEVICE_TYPE_NORMAL:         return DEVICE_MANAGER_INVALID_DEVICE_TYPE;
            case DEVICE_TYPE_MODBUS:
                device_ptr = new ModbusManager(it->address); break; // to do, use it->address
            default:                         return DEVICE_MANAGER_INVALID_DEVICE_TYPE;
        }

        if(device_ptr == NULL
            || !device_ptr->init())
        {
            return DEVICE_MANAGER_INIT_DEVICE_FAILED;
        }

        if (it->device_type == DEVICE_TYPE_MODBUS)
        {
            ErrorCode error_code;
            error_code = setModbusDetail(device_ptr, it->detail.modbus);
            if (error_code != SUCCESS) return error_code;
        }

        device_map_[it->device_index] = device_ptr;
    }

    return SUCCESS;
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

    //temporary used
    std::vector<DeviceConfig>::iterator it;
    for(it = device_xml_ptr_->device_config_list_.begin(); it != device_xml_ptr_->device_config_list_.end(); ++it)
    {
        info.index = it->device_index;
        info.address = it->address;
        info.type = it->device_type;
        info.is_valid = true;
        device_list.push_back(info);
    }//end temporary

    /* should be used in the future.
    std::map<int, BaseDevice*>::iterator it;
    for(it = device_map_.begin(); it != device_map_.end(); ++it)
    {
        info.index = it->first;
        info.address = it->second->getAddress();
        info.type = it->second->getDeviceType();
        info.is_valid = it->second->isValid();
        device_list.push_back(info);
    }
     */
    return device_list;
}

ErrorCode DeviceManager::addDevice(int device_index, BaseDevice* device_ptr)
{
    std::map<int, BaseDevice*>::iterator it;
    it = device_map_.find(device_index);
    if(it != device_map_.end())
    {
        return DEVICE_MANAGER_DEVICE_ALREADY_EXIST;
    }

    if(device_ptr == NULL)
    {
        return DEVICE_MANAGER_INVALID_ARG;
    }
    
    for(it = device_map_.begin(); it != device_map_.end(); ++it)
    {
        if(it->second == device_ptr)
        {
            return DEVICE_MANAGER_INVALID_ARG;
        }
    }

    device_map_[device_index] = device_ptr;
    return SUCCESS;
}

ErrorCode DeviceManager::setModbusDetail(BaseDevice* device_ptr, FstFstModbusConfigDetail detail)
{
    return SUCCESS;
}
