/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       io_interface.cpp
Author:     Feng.Wu WeiWang
Create:     12-Jun-2017
Modify:     25-Oct-2018
Summary:    dealing with IO module
**********************************************/

#include <unistd.h>
#include "io_interface.h"
#include "error_code.h"
#include <boost/algorithm/string.hpp>
using namespace std;
using namespace fst_hal;

IOInterface::IOInterface(fst_log::Logger * logger, fst_hal::FstIoDeviceParam* param)
{
    log_ptr_ = logger;
    param_ptr_ = param;
    ErrorCode result = initial();
    if (result != SUCCESS) {
    //  setWarning(result);
        FST_ERROR("IOInterface::initial failed :%llx", result);
    } else {
        FST_INFO("IOInterface::initial OK ");
    }

    is_virtual_ = true;
}

IOInterface::~IOInterface()
{
   if (io_manager_ != NULL)
       delete io_manager_;
   if (dev_info_ != NULL)
       delete [] dev_info_;
}

IOInterface* IOInterface::instance(fst_log::Logger* logger, fst_hal::FstIoDeviceParam* param)
{
    static IOInterface io_interface(logger, param);
    return &io_interface;
}
/*
ErrorCode IOInterface::initial()
{
    ErrorCode ret = 0;
    io_manager_ = new fst_hal::IOManager(log_ptr_, param_ptr_);
    is_virtual_ = param_ptr_->is_virtual_;
    ret = io_manager_->init(is_virtual_);

    if (ret != 0)
        return ret;

    //-----------------get num of devices.----------------------//
    io_num_.store(io_manager_->refreshDevicesNum());
    FST_INFO("IOInterface::initial io_num_:%d",io_num_.load());
    dev_info_ = new fst_hal::IODeviceInfo[io_num_];//delete soon
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    getDevicesNum
// Summary: get the number of devices without freshening.
// In:      None
// Out:     None
// Return:  int -> the total number of io devices.
//------------------------------------------------------------
int IOInterface::getIODevNum()
{
    return io_num_;
}

//------------------------------------------------------------
// Function:    refreshIODevNum
// Summary: refresh the IO device slots.
// In:      None
// Out:     None
// Return:  int -> the total number of io devices.
//------------------------------------------------------------
int IOInterface::refreshIODevNum()
{
    io_num_.store(io_manager_->refreshDevicesNum());
    return io_num_;
}

//------------------------------------------------------------
// Function:    getIODevices
// Summary: get the info of all the device.
// In:      None
// Out:     None
// Return:  the vector of all io devices.
//------------------------------------------------------------
vector<fst_hal::IODeviceInfo> IOInterface::getIODeviceList()
{
	vector<fst_hal::IODeviceInfo> vectorIODeviceInfo ;
    fst_hal::IODeviceInfo dev_info;
	
    for (int i = 0; i < io_num_; i++)
    {
        io_manager_->getDevInfoByIndex(i, dev_info);
		vectorIODeviceInfo.push_back(dev_info);
    }
	return vectorIODeviceInfo ;
}

//------------------------------------------------------------
// Function:    getDeviceInfo
// Summary: get the information of each device.
// In:      index -> the sequence number of the device.
// Out:     info  -> the information of each device.
// Return:  ErrorCode   -> error codes.
//------------------------------------------------------------
ErrorCode IOInterface::getDevicePortValues(uint8_t address, fst_hal::IODevicePortValues &values)
{
    return io_manager_->getModuleValues(address, values);
}

//------------------------------------------------------------
// Function:    setDIOByBit
// Summary: Set the output to the specified port.
// In:      physics_id -> the specified physical port.
//          value      -> 1 = ON, 0 = OFF.
// Out:     None.
// Return:  ErrorCode   -> error codes.
//------------------------------------------------------------
ErrorCode IOInterface::setDIOByBit(uint32_t physics_id, uint8_t value)
{
    return  io_manager_->setModuleValue(physics_id, value);
}

//------------------------------------------------------------
// Function:    getDIOByBit
// Summary: get the value of the specified port.
// In:      physics_id -> the specified physical port.
// Out:     value      -> 1 = ON, 0 = OFF.
// Return:  ErrorCode   -> error codes.
//------------------------------------------------------------
ErrorCode IOInterface::getDIOByBit(uint32_t physics_id, uint8_t &value)
{
    return  io_manager_->getModuleValue(physics_id, value);
}

 */
