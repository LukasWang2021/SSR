/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       io_manager.h
Author:     Feng.Wu 
Create:     14-Dec-2018
Modify:     14-Dec-2018
Summary:    dealing with IO module
**********************************************/
#ifndef IO_MANAGER_CPP_
#define IO_MANAGER_CPP_

#include "io_manager.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "error_monitor.h"
#include "protoc.h"

using namespace fst_hal;
using namespace fst_base;


IoManager::IoManager():
    param_ptr_(NULL),
    log_ptr_(NULL),
    cycle_time_(10000),
    is_running_(false),
    device_manager_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new IoManagerParam();
    FST_LOG_INIT("io_manager");
    
}

IoManager::~IoManager()
{
    is_running_ = false;
    thread_ptr_.join();
    
    if(log_ptr_ != NULL){
        delete log_ptr_;
        log_ptr_ = NULL;
    }
    if(param_ptr_ != NULL){
        delete param_ptr_;
        param_ptr_ = NULL;
    }
}

/*
IoManager* IoManager::getInstance(fst_hal::DeviceManager* device_manager)
{
    static IoManager io_manager(device_manager);
    return &io_manager;
}
*/

ErrorCode IoManager::init(fst_hal::DeviceManager* device_manager_ptr)
{
    if(!param_ptr_->loadParam()){
        FST_ERROR("Failed to load io_manager component parameters");
        //ErrorMonitor::instance()->add(IO_MANAGERG_LOAD_PARAM_FAILED);
    }else{
        FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
        FST_INFO("Success to load io_manager component parameters");
    }

    cycle_time_ = param_ptr_->cycle_time_;

    device_manager_ptr_ = device_manager_ptr;
    device_list_ = device_manager_ptr_->getDeviceList();

    // thread only start one time.
    if (is_running_ == true)
        return SUCCESS;
    if(!thread_ptr_.run(&ioManagerRoutineThreadFunc, this, 20))
    {
        FST_ERROR("Failed to open io_manager thread");
        ErrorMonitor::instance()->add(IO_INIT_FAIL);
        return IO_INIT_FAIL;
    }
    is_running_ = true;
    
    return SUCCESS;
}

std::map<int, int> IoManager::getIoBoardVersion(void)
{
    std::map<int, int> version_map;
    int version = 0;
    for(unsigned int i = 0; i < device_list_.size(); ++i)
    {
        BaseDevice* device_ptr = device_manager_ptr_->getDevicePtrByDeviceIndex(device_list_[i].index);
        switch(device_list_[i].type)
        {
            case DEVICE_TYPE_FST_IO:
            {
                if (device_ptr == NULL) break;
                FstIoDevice* io_device_ptr = static_cast<FstIoDevice*>(device_ptr);
                io_device_ptr->getIoBoardVersion(version);
                version_map.insert(std::pair<int, int>(device_list_[i].address, version));
                break;
            }
            default: break;
        }
    }
    return version_map;
}

void IoManager::ioManagerThreadFunc()
{
    updateIoDevicesData();
    usleep(cycle_time_);
}

bool IoManager::isRunning()
{
    return is_running_;
}

//get di, do, ri, ro
ErrorCode IoManager::getBitValue(PhysicsID phy_id, uint8_t &value)
{
    switch(phy_id.info.port_type)
    {
        case MessageType_IoType_DI: return getDiValue(phy_id, value);
        case MessageType_IoType_DO: return getDoValue(phy_id, value); 
        case MessageType_IoType_RI: return getRiValue(phy_id, value); 
        case MessageType_IoType_RO: return getRoValue(phy_id, value);
        default: return IO_INVALID_PARAM_ID;
    }
}

//set di, do, ri, ro
ErrorCode IoManager::setBitValue(PhysicsID phy_id, uint8_t value)
{
    switch(phy_id.info.port_type)
    {
        case MessageType_IoType_DI: return setDiValue(phy_id, value);
        case MessageType_IoType_DO: return setDoValue(phy_id, value); 
        case MessageType_IoType_RI: return setRiValue(phy_id, value); 
        case MessageType_IoType_RO: return setRoValue(phy_id, value);
        default: return IO_INVALID_PARAM_ID;
    }
}

//get all io devices info, including io_board and modbus.
std::vector<fst_hal::IODeviceInfo> IoManager::getIODeviceInfoList(void)
{
    std::vector<fst_hal::IODeviceInfo> info_list;
    fst_hal::IODeviceInfo info;
    
    for(unsigned int i = 0; i < device_list_.size(); ++i)
    {
        BaseDevice* device_ptr = device_manager_ptr_->getDevicePtrByDeviceIndex(device_list_[i].index);
        switch(device_list_[i].type)
        {
            case DEVICE_TYPE_FST_IO:
            {
                FstIoDevice* io_device_ptr = static_cast<FstIoDevice*>(device_ptr);
                info = io_device_ptr->getDeviceInfo();
                info_list.push_back(info);
                break;
            }
            case DEVICE_TYPE_MODBUS:
            {
                ModbusManager* modbus_manager_ptr = static_cast<ModbusManager*>(device_ptr);
                ErrorCode error_code = getModbusDeviceInfo(info, modbus_manager_ptr);
                info_list.push_back(info);
                break;
            }
            case DEVICE_TYPE_VIRTUAL_IO:
            {
                VirtualIoDevice* virtual_io_ptr = static_cast<VirtualIoDevice*>(device_ptr);
                info = virtual_io_ptr->getDeviceInfo();
                info_list.push_back(info);
                break;
            }
            default: break;
        }
    }

    return info_list;
}

// get io board info
ErrorCode IoManager::getIODeviceInfo(uint8_t address, fst_hal::IODeviceInfo &info)
{
    for(unsigned int i = 0; i < device_list_.size(); ++i)
    { 
        if (address == device_list_[i].address && device_list_[i].type == DEVICE_TYPE_FST_IO)
        {
            BaseDevice* device_ptr = device_manager_ptr_->getDevicePtrByDeviceIndex(device_list_[i].index);
            FstIoDevice* io_device_ptr = static_cast<FstIoDevice*>(device_ptr);
            info = io_device_ptr->getDeviceInfo();
            return SUCCESS;
        }
    }
    return IO_INVALID_PARAM_ID;
}

//get io board port values
ErrorCode IoManager::getDevicePortValues(uint8_t address, fst_hal::IODevicePortValues &values)
{
    for(unsigned int i = 0; i < device_list_.size(); ++i)
    {
        if (address == device_list_[i].address && device_list_[i].type == DEVICE_TYPE_FST_IO)
        {
            BaseDevice* device_ptr = device_manager_ptr_->getDevicePtrByDeviceIndex(device_list_[i].index);
            FstIoDevice* io_device_ptr = static_cast<FstIoDevice*>(device_ptr);
            values = io_device_ptr->getDeviceValues();
            return SUCCESS;
        }
    }
    return IO_INVALID_PARAM_ID;
}

//private function

//getDi
ErrorCode IoManager::getDiValue(PhysicsID phy_id, uint8_t &value)
{
    //get device_ptr
    BaseDevice* device_ptr = getDevicePtr(phy_id);
    if (device_ptr == NULL)
    {
        FST_ERROR("IOManager::getDiValue(): Invalid physics id - 0x%lx", phy_id);
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PARAM_ID);
        return IO_INVALID_PARAM_ID;
    }

    //call specified device
    switch(phy_id.info.dev_type)
    {
        case DEVICE_TYPE_FST_IO:
        {
            FstIoDevice* io_device_ptr = static_cast<FstIoDevice*>(device_ptr);
            return io_device_ptr->getDiValue(phy_id.info.port, value);
        }
        case DEVICE_TYPE_MODBUS:
        {
            ModbusManager* modbus_manager_ptr = static_cast<ModbusManager*>(device_ptr);
            return getDiValueFromModbusServer(phy_id.info.port, value, modbus_manager_ptr);
        }
        case DEVICE_TYPE_VIRTUAL_IO:
        {
            VirtualIoDevice* virtual_io_ptr = static_cast<VirtualIoDevice*>(device_ptr);
            return virtual_io_ptr->getDiValue(phy_id.info.port, value);
        }
        default:
        {
            FST_ERROR("IOManager::getDiValue(): Invalid physics id - 0x%lx", phy_id);
            return IO_INVALID_PARAM_ID;
        }
    }
}

//getDo
ErrorCode IoManager::getDoValue(PhysicsID phy_id, uint8_t &value)
{
    //get device_ptr
    BaseDevice* device_ptr = getDevicePtr(phy_id);
    if (device_ptr == NULL)
    {
        FST_ERROR("IOManager::getDoValue(): Invalid physics id - 0x%lx", phy_id);
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PARAM_ID);
        return IO_INVALID_PARAM_ID;
    }

    //call specified device
    switch(phy_id.info.dev_type)
    {
        case DEVICE_TYPE_FST_IO:
        {
            FstIoDevice* io_device_ptr = static_cast<FstIoDevice*>(device_ptr);
            return io_device_ptr->getDoValue(phy_id.info.port, value);
        }
        case DEVICE_TYPE_MODBUS:
        {
            ModbusManager* modbus_manager_ptr = static_cast<ModbusManager*>(device_ptr);
            return getDoValueFromModbusServer(phy_id.info.port, value, modbus_manager_ptr);
        }
        case DEVICE_TYPE_VIRTUAL_IO:
        {
            VirtualIoDevice* virtual_io_ptr = static_cast<VirtualIoDevice*>(device_ptr);
            return virtual_io_ptr->getDoValue(phy_id.info.port, value);
        }
        default:
        {
            FST_ERROR("IOManager::getDoValue(): Invalid physics id - 0x%lx", phy_id);
            return IO_INVALID_PARAM_ID;
        }
    }

}

//getRi
ErrorCode IoManager::getRiValue(PhysicsID phy_id, uint8_t &value)
{
    //get device_ptr
    BaseDevice* device_ptr = getDevicePtr(phy_id);
    if (device_ptr == NULL)
    {
        FST_ERROR("IOManager::getRiValue(): Invalid physics id - 0x%lx", phy_id);
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PARAM_ID);
        return IO_INVALID_PARAM_ID;
    }
    
    //call specified device
    switch(phy_id.info.dev_type)
    {
        case DEVICE_TYPE_FST_IO:
        {
            FstIoDevice* io_device_ptr = static_cast<FstIoDevice*>(device_ptr);
            return io_device_ptr->getRiValue(phy_id.info.port, value);
        }
        default:
        {
            FST_ERROR("IOManager::getRiValue(): Invalid physics id - 0x%lx", phy_id);
            return IO_INVALID_PARAM_ID;
        }
    }
    
}

//getRo
ErrorCode IoManager::getRoValue(PhysicsID phy_id, uint8_t &value)
{
    //get device_ptr
    BaseDevice* device_ptr = getDevicePtr(phy_id);
    if (device_ptr == NULL)
    {
        FST_ERROR("IOManager::getRoValue(): Invalid physics id - 0x%lx", phy_id);
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PARAM_ID);
        return IO_INVALID_PARAM_ID;
    }
    
    //call specified device
    switch(phy_id.info.dev_type)
    {
        case DEVICE_TYPE_FST_IO:
        {
            FstIoDevice* io_device_ptr = static_cast<FstIoDevice*>(device_ptr);
            return io_device_ptr->getRoValue(phy_id.info.port, value);
        }
        default:
        {
            FST_ERROR("IOManager::getRoValue(): Invalid physics id - 0x%lx", phy_id);
            return IO_INVALID_PARAM_ID;
        }
    }
    
}

//setDi
ErrorCode IoManager::setDiValue(PhysicsID phy_id, uint8_t value)
{
    //get device_ptr
    BaseDevice* device_ptr = getDevicePtr(phy_id);
    if (device_ptr == NULL)
    {
        FST_ERROR("IOManager::setDiValue(): Invalid physics id - 0x%lx", phy_id);
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PARAM_ID);
        return IO_INVALID_PARAM_ID;
    }
    
    //call specified device
    switch(phy_id.info.dev_type)
    {
        case DEVICE_TYPE_VIRTUAL_IO:
        {
            /* code for virtual io */
            VirtualIoDevice* virtual_io_ptr = static_cast<VirtualIoDevice*>(device_ptr);
            return virtual_io_ptr->setDiValue(phy_id.info.port, value);
        }
        default:
        {
            FST_ERROR("IOManager::setDiValue(): Invalid physics id - 0x%lx", phy_id);
            return IO_INVALID_PARAM_ID;
        }
    }

}

//setDo
ErrorCode IoManager::setDoValue(PhysicsID phy_id, uint8_t value)
{
    //get device_ptr
    BaseDevice* device_ptr = getDevicePtr(phy_id);
    if (device_ptr == NULL)
    {
        FST_ERROR("IOManager::setDoValue(): Invalid physics id - 0x%lx", phy_id);
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PARAM_ID);
        return IO_INVALID_PARAM_ID;
    }
    
    //call specified device
    switch(phy_id.info.dev_type)
    {
        case DEVICE_TYPE_FST_IO:
        {
            FstIoDevice* io_device_ptr = static_cast<FstIoDevice*>(device_ptr);
            return io_device_ptr->setDoValue(phy_id.info.port, value);
        }
        case DEVICE_TYPE_MODBUS:
        {
            // code for modbus 
            ModbusManager* modbus_manager_ptr = static_cast<ModbusManager*>(device_ptr);
            return setDoValueToModbusServer(phy_id.info.port, value, modbus_manager_ptr);
        }
        case DEVICE_TYPE_VIRTUAL_IO:
        {
            /* code for virtual io */
            VirtualIoDevice* virtual_io_ptr = static_cast<VirtualIoDevice*>(device_ptr);
            return virtual_io_ptr->setDoValue(phy_id.info.port, value);
        }
        default:
        {
            FST_ERROR("IOManager::setDoValue(): Invalid physics id - 0x%lx", phy_id);
            return IO_INVALID_PARAM_ID;
        }
    }

}

//setRi
ErrorCode IoManager::setRiValue(PhysicsID phy_id, uint8_t value)
{
    //get device_ptr
    BaseDevice* device_ptr = getDevicePtr(phy_id);
    if (device_ptr == NULL)
    {
        FST_ERROR("IOManager::setRiValue(): Invalid physics id - %d", phy_id);
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PARAM_ID);
        return IO_INVALID_PARAM_ID;
    }
    
    //call specified device
    switch(phy_id.info.dev_type)
    {
        case DEVICE_TYPE_FST_IO:

        default:
        {
            FST_ERROR("IOManager::setRiValue(): Invalid physics id - 0x%lx", phy_id);
            return IO_INVALID_PARAM_ID;
        }
    }

}

//setRo
ErrorCode IoManager::setRoValue(PhysicsID phy_id, uint8_t value)
{
    //get device_ptr
    BaseDevice* device_ptr = getDevicePtr(phy_id);
    if (device_ptr == NULL)
    {
        FST_ERROR("IOManager::setRoValue(): Invalid physics id - %d", phy_id);
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PARAM_ID);
        return IO_INVALID_PARAM_ID;
    }
    
    //call specified device
    switch(phy_id.info.dev_type)
    {
        case DEVICE_TYPE_FST_IO:
        {
            FstIoDevice* io_device_ptr = static_cast<FstIoDevice*>(device_ptr);
            return io_device_ptr->setRoValue(phy_id.info.port, value);
        }
        default:
        {
            FST_ERROR("IOManager::setRoValue(): Invalid physics id - 0x%lx", phy_id);
            return IO_INVALID_PARAM_ID;
        }
    }
}

//get device ptr
BaseDevice* IoManager::getDevicePtr(PhysicsID phy_id)
{
    for(unsigned int i = 0; i < device_list_.size(); ++i)
    {
        if (phy_id.info.address == device_list_[i].address && phy_id.info.dev_type == device_list_[i].type)
            return device_manager_ptr_->getDevicePtrByDeviceIndex(device_list_[i].index);
    }
    return NULL;
}

// update the io board data.
ErrorCode IoManager::updateIoDevicesData(void)
{
    ErrorCode ret = SUCCESS;
    static ErrorCode pre_ret = SUCCESS;
    for(unsigned int i = 0; i < device_list_.size(); ++i)
    {
        switch(device_list_[i].type)
        {
            case DEVICE_TYPE_FST_IO:
            {
                BaseDevice* device_ptr = device_manager_ptr_->getDevicePtrByDeviceIndex(device_list_[i].index);
                if(device_ptr == NULL) break; //if devices instance are deconstructed, thread stop calling.
                FstIoDevice* io_device_ptr = static_cast<FstIoDevice*>(device_ptr);
                ret = io_device_ptr->updateDeviceData();
                if (ret != SUCCESS)
                {
                    //only upload error one time.
                    if (pre_ret != ret) 
                    {     
                        //FST_ERROR("Failed to get io data");                  
                        ErrorMonitor::instance()->add(ret);
                    }
                }
                pre_ret = ret;
                break;
            }
            case DEVICE_TYPE_MODBUS: break;
            default: break;
        }
    }

    return ret;
}


ErrorCode IoManager::getModbusDeviceInfo(fst_hal::IODeviceInfo &info, ModbusManager* modbus_manager)
{
    if (modbus_manager == NULL) return MODBUS_INVALID;
    info.comm_type = "TCP";
    info.address = modbus_manager->getAddress();
    info.device_type = "ModbusServer";

    if (modbus_manager->getStartMode() != MODBUS_SERVER)
    {
        info.is_valid = false;
        info.DO_num = 0;
        info.DI_num = 0;
        return SUCCESS;;
    }

    info.is_valid = modbus_manager->isModbusValid();

    ModbusRegAddrInfo addr_info;
    addr_info.max_nb = 0;
    addr_info.addr = 0;

    ErrorCode ret = modbus_manager->getServerConfigCoilInfo(addr_info);
    if (ret != SUCCESS)
    {
        info.DO_num = 0;
        info.DI_num = 0;
        return ret;
    }

    info.DO_num = addr_info.max_nb;

    addr_info.max_nb = 0;
    addr_info.addr = 0;
    ret = modbus_manager->getServerConfigDiscrepteInputInfo(addr_info);
    if (ret != SUCCESS)
    {
        info.DI_num = 0;
        return ret;
    } 

    info.DI_num = addr_info.max_nb;
    return SUCCESS;
}

ErrorCode IoManager::getDiValueFromModbusServer(uint8_t port, uint8_t &value, ModbusManager* modbus_manager)
{
    if (modbus_manager == NULL) return MODBUS_INVALID;

    if (!modbus_manager->isValid()
        || modbus_manager->getStartMode() != MODBUS_SERVER)
    {
        return MODBUS_START_MODE_ERROR;
    }

    int server_id = 0;
    int addr = static_cast<int>(port);
    return modbus_manager->readDiscreteInputs(server_id, addr, 1, &value);
}

ErrorCode IoManager::getDoValueFromModbusServer(uint8_t port, uint8_t &value, ModbusManager* modbus_manager)
{
    if (modbus_manager == NULL) return MODBUS_INVALID;

    if (!modbus_manager->isValid()
        || modbus_manager->getStartMode() != MODBUS_SERVER)
    {
        return MODBUS_START_MODE_ERROR;
    }

    int server_id = 0;
    int addr = static_cast<int>(port);
    return modbus_manager->readCoils(server_id, addr, 1, &value);
}


ErrorCode IoManager::setDoValueToModbusServer(uint8_t port, uint8_t &value, ModbusManager* modbus_manager)
{
    if (modbus_manager == NULL) return MODBUS_INVALID;

    if (!modbus_manager->isValid()
        || modbus_manager->getStartMode() != MODBUS_SERVER)
    {
        return MODBUS_START_MODE_ERROR;
    }

    int server_id = 0;
    int addr = static_cast<int>(port);
    return modbus_manager->writeCoils(server_id, addr, 1, &value);
}

// thread function
void ioManagerRoutineThreadFunc(void* arg)
{
    std::cout<<"io_manager routine thread running"<<std::endl;
    IoManager* io_manager = static_cast<IoManager*>(arg);
    while(io_manager->isRunning())
    {
        io_manager->ioManagerThreadFunc();
    }
    std::cout<<"io_manager routine thread exit"<<std::endl;
}






#endif //IO_MANAGER_IO_MANAGER_CPP_
