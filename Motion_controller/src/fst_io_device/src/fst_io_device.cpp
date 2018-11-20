/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       fst_io_device.cpp
Author:     Feng.Wu
Create:     14-Oct-2018
Modify:     25-Oct-2018
Summary:    dealing with IO module
**********************************************/

#include "fst_io_device.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "error_monitor.h"

using namespace std;
using namespace fst_hal;
using namespace fst_base;

FstIoDevice::FstIoDevice(int address):
    BaseDevice(address, fst_hal::DEVICE_TYPE_FST_IO),
    log_ptr_(NULL),
    param_ptr_(NULL),
    io_num_(0)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new FstIoDeviceParam();
    FST_LOG_INIT("fst_io_device");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
}

FstIoDevice::~FstIoDevice()
{
    if(log_ptr_ != NULL){
        delete log_ptr_;
        log_ptr_ = NULL;
    }
    if(param_ptr_ != NULL){
        delete param_ptr_;
        param_ptr_ = NULL;
    }
}

bool FstIoDevice::init()
{
    if(!param_ptr_->loadParam()){
        FST_ERROR("Failed to load fst_io_device component parameters");
        ErrorMonitor::instance()->add(IO_DEVICE_LOAD_PARAM_FAILED);
        return false;
    }else{
        FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
        FST_INFO("Success to load fst_io_device component parameters");
    }

    IOManager::instance(log_ptr_, param_ptr_)->init();

    return true;
}

//------------------------------------------------------------
// Function:    getIODevices
// Summary: get the info of all the device.
// In:      None
// Out:     None
// Return:  the vector of all io devices.
//------------------------------------------------------------
vector<fst_hal::IODeviceInfo> FstIoDevice::getIODeviceList()
{
    vector<fst_hal::IODeviceInfo> vector_dev ;
    fst_hal::IODeviceInfo dev_info;

    io_num_ = IOManager::instance(log_ptr_, param_ptr_)->getDevicesNum();
    for (int i = 0; i < io_num_; i++)
    {
        IOManager::instance(log_ptr_, param_ptr_)->getDevInfoByIndex(i, dev_info);
        vector_dev.push_back(dev_info);
    }
    return vector_dev;
}

ErrorCode FstIoDevice::getDevInfoByID(uint8_t address, IODeviceInfo &info)
{
    return IOManager::instance(log_ptr_, param_ptr_)->getDevInfoByID(address, info);
}

//------------------------------------------------------------
// Function:    getDeviceInfo
// Summary: get the information of each device.
// In:      index -> address -> the address is from io board.
// Out:     info  -> the information of each device.
// Return:  ErrorCode   -> error codes.
//------------------------------------------------------------
ErrorCode FstIoDevice::getDevicePortValues(uint8_t address, fst_hal::IODevicePortValues &values)
{
    return IOManager::instance(log_ptr_, param_ptr_)->getModuleValues(address, values);
}

//------------------------------------------------------------
// Function:    setDIOByBit
// Summary: Set the output to the specified port.
// In:      physics_id -> the specified physical port.
//          value      -> 1 = ON, 0 = OFF.
// Out:     None.
// Return:  ErrorCode   -> error codes.
//------------------------------------------------------------
ErrorCode FstIoDevice::setDIOByBit(uint32_t physics_id, uint8_t value)
{
    return  IOManager::instance(log_ptr_, param_ptr_)->setModuleValue(physics_id, value);
}

//------------------------------------------------------------
// Function:    getDIOByBit
// Summary: get the value of the specified port.
// In:      physics_id -> the specified physical port.
// Out:     value      -> 1 = ON, 0 = OFF.
// Return:  ErrorCode   -> error codes.
//------------------------------------------------------------
ErrorCode FstIoDevice::getDIOByBit(uint32_t physics_id, uint8_t &value)
{
    return  IOManager::instance(log_ptr_, param_ptr_)->getModuleValue(physics_id, value);
}


//------------------------------------------------------------
// Function:    getDevicesNum
// Summary: get the number of devices
// In:      None
// Out:     None
// Return:  int -> the total number of io devices.
//------------------------------------------------------------
int FstIoDevice::getIODevNum(void)
{
    io_num_ = IOManager::instance(log_ptr_, param_ptr_)->getDevicesNum();
    return io_num_;
}

//------------------------------------------------------------
// Function:    refreshIODevNum
// Summary: refresh the IO device slots.
// In:      None
// Out:     None
// Return:  int -> the total number of io devices.
//------------------------------------------------------------
int FstIoDevice::refreshIODevNum(void)
{
    io_num_ = IOManager::instance(log_ptr_, param_ptr_)->refreshDevicesNum();
    return io_num_;
}

