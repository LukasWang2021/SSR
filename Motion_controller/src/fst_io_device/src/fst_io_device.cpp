/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       fst_io_device.cpp
Author:     Feng.Wu
Create:     14-Dec-2018
Modify:     14-Dec-2018
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

bool FstIoDevice::is_mem_init_ = false;
int FstIoDevice::io_dev_count_ = 0;

FstIoDevice::FstIoDevice(int address):
    BaseDevice(address, fst_hal::DEVICE_TYPE_FST_IO),
    log_ptr_(NULL),
    param_ptr_(NULL),
    error_code_(0),
    pre_code_(0),
    address_(address)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new FstIoDeviceParam();
    FST_LOG_INIT("fst_io_device");
    address_ = address_ & 0x0F;//address is 0-15.
}

FstIoDevice::~FstIoDevice()
{
    if(is_mem_init_ == true)
    {
        io_dev_count_--;
        if (io_dev_count_ == 0)
        {
            ioClose();
            is_mem_init_ = false;
        }
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

bool FstIoDevice::init()
{
    if(!param_ptr_->loadParam())
    {
        FST_ERROR("Failed to load fst_io_device component parameters");
        ErrorMonitor::instance()->add(LOAD_IO_CONFIG_FAIL);
        return false;
    }else
    {
        FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
        FST_INFO("Success to load fst_io_device component parameters");
    }
    
    // init the attributes
    dev_info_.address = address_;
    dev_info_.dev_type = DEVICE_TYPE_FST_IO;
    dev_info_.device_type = param_ptr_->device_type_; //"IMB00401"
    dev_info_.comm_type = param_ptr_->comm_type_;     //"RS485"
    dev_info_.DI_num = param_ptr_->max_DI_number_;
    dev_info_.DO_num = param_ptr_->max_DO_number_;
    dev_info_.RI_num = param_ptr_->max_RI_number_;
    dev_info_.RO_num = param_ptr_->max_RO_number_;
    dev_info_.is_valid = false;
    FST_INFO("fst_io_device: address=%d, device_type=%s, comm_type=%s", 
        dev_info_.address, dev_info_.device_type.c_str(), dev_info_.comm_type.c_str());

    is_virtual_ = param_ptr_->is_virtual_;
    comm_tolerance_ = param_ptr_->comm_tolerance_;
    memset(&output_, 0, sizeof(output_));
    setValid(true);

    //map between address and id, id is used to find the sharemem address according to protocol.
    addr_map_[address_] = io_dev_count_;
    io_dev_count_++;
    if(io_dev_count_ > IO_BOARD_NUM_MAX)
    {
        FST_ERROR("exceed the max io_board number.");
        return false;
    }

    // init the sharemem only one time for serveral io_boards
    if(is_mem_init_ == true)
    {
        ioWriteId(addr_map_[address_], address_);
        return true;
    }

    if (is_virtual_ == false)
    {
        int result = ioInit();
        if (result == 0)
        {
            is_mem_init_ = true;
            ioWriteId(addr_map_[address_], address_);
            FST_INFO("Use real IO");
        }else
        {
            FST_ERROR("init FstIoDevice failed");
            fst_base::ErrorMonitor::instance()->add(IO_INIT_FAIL);
            setValid(false);
            return false;
        } 
    }else
    {
        FST_INFO("Use virtual IO");
    }
    
    return true;
}

void FstIoDevice::getIoBoardVersion(int &version)
{
    if (is_virtual_ == false)
    {
        ioBoardVersionFromMem(addr_map_[address_], &version);
    }
    else
    {
        version = 0;
    }
}

IODeviceInfo FstIoDevice::getDeviceInfo(void)
{
    dev_info_.is_valid = isValid();
    return dev_info_;
}

IODevicePortValues FstIoDevice::getDeviceValues(void)
{
    data_mutex_.lock();
    IODevicePortValues values = dev_values_;
    data_mutex_.unlock();
    return values;
}

ErrorCode FstIoDevice::getDiValue(uint32_t port_offset, uint8_t &value)
{
    if (port_offset > param_ptr_->max_DI_number_ || port_offset == 0)
    {
        FST_ERROR("FstIoDevice::getDiValue(): invalid port seq for DI - %d", port_offset);
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PORT_SEQ);
        return IO_INVALID_PORT_SEQ;
    }
    
    int frame = (port_offset - 1) / 8;
    int shift = (port_offset - 1) % 8;
    data_mutex_.lock();
    value = (dev_values_.DI[frame] >> shift) & 0x01;
    data_mutex_.unlock();

    return SUCCESS;
}

ErrorCode FstIoDevice::getDoValue(uint32_t port_offset, uint8_t &value)
{
    if (port_offset > param_ptr_->max_DO_number_ || port_offset == 0)
    {
        FST_ERROR("FstIoDevice::getDoValue(): invalid port seq for DO - %d", port_offset);
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PORT_SEQ);
        return IO_INVALID_PORT_SEQ;
    }
    
    int frame = (port_offset - 1) / 8;
    int shift = (port_offset - 1) % 8;
    data_mutex_.lock();
    value = (dev_values_.DO[frame] >> shift) & 0x01;
    data_mutex_.unlock();

    return SUCCESS;
}

ErrorCode FstIoDevice::getRiValue(uint32_t port_offset, uint8_t &value)
{
    if (port_offset > param_ptr_->max_RI_number_ || port_offset == 0)
    {
        FST_ERROR("FstIoDevice::getRiValue(): invalid port seq for RI - %d", port_offset);
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PORT_SEQ);
        return IO_INVALID_PORT_SEQ;
    }
    
    int frame = (port_offset - 1) / 8;
    int shift = (port_offset - 1) % 8;
    data_mutex_.lock();
    value = (dev_values_.RI[frame] >> shift) & 0x01;
    data_mutex_.unlock();

    return SUCCESS;
}

ErrorCode FstIoDevice::getRoValue(uint32_t port_offset, uint8_t &value)
{
    if (port_offset > param_ptr_->max_RO_number_ || port_offset == 0)
    {
        FST_ERROR("FstIoDevice::getRoValue(): invalid port seq for RO - %d", port_offset);
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PORT_SEQ);
        return IO_INVALID_PORT_SEQ;
    }
    
    int frame = (port_offset - 1) / 8;
    int shift = (port_offset - 1) % 8;
    data_mutex_.lock();
    value = (dev_values_.RO[frame] >> shift) & 0x01;
    data_mutex_.unlock();

    return SUCCESS;
}


ErrorCode FstIoDevice::getUiValue(uint32_t port_offset, uint8_t &value)
{
    return getDiValue(port_offset, value);
}

ErrorCode FstIoDevice::getUoValue(uint32_t port_offset, uint8_t &value)
{
    return getDoValue(port_offset, value);
}


ErrorCode FstIoDevice::setDoValue(uint32_t port_offset, uint8_t value)
{
    if (error_code_ == IO_DEVICE_UNFOUND)
    {
        return error_code_;
    }

    if((port_offset > param_ptr_->max_DO_number_) || port_offset == 0) 
    {
        FST_ERROR("FstIoDevice::setDoValue(): invalid port seq for DO - %d", port_offset);
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PORT_SEQ);
        return IO_INVALID_PORT_SEQ;
    }

    int frame = (port_offset - 1) / 8;
    int shift = (port_offset - 1) % 8;

    data_mutex_.lock();
    if (value == 0)
        output_[frame] &= ~(0x01 << shift);
    else
        output_[frame] |= 0x01 << shift;
    data_mutex_.unlock();

    return SUCCESS;
}

ErrorCode FstIoDevice::setRoValue(uint32_t port_offset, uint8_t value)
{
    if (error_code_ == IO_DEVICE_UNFOUND)
    {
        return error_code_;
    }

    if((port_offset > param_ptr_->max_RO_number_) || port_offset == 0) 
    {
        FST_ERROR("FstIoDevice::setRoValue(): invalid port seq for RO - %d", port_offset);
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PORT_SEQ);
        return IO_INVALID_PORT_SEQ;
    }

    int frame = (port_offset - 1) / 8;
    int shift = (port_offset - 1) % 8;

    data_mutex_.lock();
    if (value == 0)
        output_[frame + 4] &= ~(0x01 << shift);
    else
        output_[frame + 4] |= 0x01 << shift;
    data_mutex_.unlock();

    return SUCCESS;
}


ErrorCode FstIoDevice::setUoValue(uint32_t port_offset, uint8_t value)
{
    return setDoValue(port_offset, value);
}


ErrorCode FstIoDevice::updateDeviceData(void)
{
    IODeviceData data;
    initIODeviceData(data);
    data_mutex_.lock();
    memcpy(data.output, output_, sizeof(data.output));
    data_mutex_.unlock();

    error_code_ = getDeviceDataFromMem(data);
    if(error_code_ == SUCCESS)
    {
        data_mutex_.lock();
        memcpy(&dev_values_.DI, &data.input, 4);    // DI contains 4 bytes
        memcpy(&dev_values_.DO, &data.output, 4);
        memcpy(&dev_values_.RI, &data.input[4], 1); // RI contains 1 bytes
        memcpy(&dev_values_.RO, &data.output[4], 1);
        data_mutex_.unlock();
        setValid(true);
    }
    else if (error_code_ == GET_IO_FAIL)
    {
        setValid(false);
    }
    else if (error_code_ == IO_DEVICE_UNFOUND)
    {
        setValid(false);
    }
    
    //only upload error one time.
    if ((pre_code_ != error_code_) && (error_code_ != SUCCESS))
    {
        pre_code_ = error_code_;
        return error_code_;
    }
    pre_code_ = error_code_;

    return SUCCESS;
}


//private

void FstIoDevice::initIODeviceData(IODeviceData &data)
{
    data.offset = addr_map_[address_];
    data.model = 0;
    data.enable = 1;
    data.verify = 1;
    for (int i = 0; i< IO_DATAFRAME_MAX; ++i) {
        data.output[i] = 0;
        data.input[i] = 0;
    }
}

ErrorCode FstIoDevice::getDeviceDataFromMem(IODeviceData &data)
{
    if (is_virtual_ == true) {
        data.input[0] = 0x0;
        data.input[1] = 0x0;// simulation input as binary 10101010
        data.input[2] = 0x0;
        data.input[3] = 0x0;
        data.input[4] = 0x0;
        return SUCCESS;
    }

    // fetch data from memory.
    if (!readWriteMem(data))
        return GET_IO_FAIL;

    // check enable.
    if (data.enable == 0)
    {
        return IO_DEVICE_UNFOUND;
    }    
    if (data.verify == 0)
    {
        //FST_WARN("id = %d, IODeviceData verify failed, err = 0x%llx", address_, IO_VERIFY_FALSE);
        return IO_VERIFY_FALSE;
    }
    return SUCCESS; 
}

bool FstIoDevice::readWriteMem(IODeviceData &data)
{
    //new protocol from 201905
    if (ioWriteDownload(&data) != 0) 
    {
        return false;
    }
    if (ioReadUpload(&data) != 0) 
    {
        return false;
    }
    return true;
}
