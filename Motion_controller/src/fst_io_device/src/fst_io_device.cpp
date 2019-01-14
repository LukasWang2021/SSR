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

FstIoDevice::FstIoDevice(int address):
    BaseDevice(address, fst_hal::DEVICE_TYPE_FST_IO),
    log_ptr_(NULL),
    param_ptr_(NULL),
    address_(address)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new FstIoDeviceParam();
    FST_LOG_INIT("fst_io_device");
}

FstIoDevice::~FstIoDevice()
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

    // init the sharemem only one time for serveral io_boards
    if(is_mem_init_ == true)
        return true;

    if (is_virtual_ == false)
    {
        int result = ioInit();
        if (result == 0)
        {
            is_mem_init_ = true;
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

ErrorCode FstIoDevice::getDiValue(uint8_t port_offset, uint8_t &value)
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

ErrorCode FstIoDevice::getDoValue(uint8_t port_offset, uint8_t &value)
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

ErrorCode FstIoDevice::getRiValue(uint8_t port_offset, uint8_t &value)
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

ErrorCode FstIoDevice::getRoValue(uint8_t port_offset, uint8_t &value)
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


ErrorCode FstIoDevice::setDoValue(uint8_t port_offset, uint8_t value)
{
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

ErrorCode FstIoDevice::setRoValue(uint8_t port_offset, uint8_t value)
{
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


ErrorCode FstIoDevice::updateDeviceData(void)
{
    IODeviceData data;
    initIODeviceData(data);
    data_mutex_.lock();
    memcpy(data.output, output_, sizeof(data.output));
    data_mutex_.unlock();

    ErrorCode ret = getDeviceDataFromMem(data);
    if(ret == SUCCESS)
    {
        data_mutex_.lock();
        memcpy(&dev_values_.DI, &data.input, 4);    // DI contains 4 bytes
        memcpy(&dev_values_.DO, &data.output, 4);
        memcpy(&dev_values_.RI, &data.input[4], 1); // RI contains 1 bytes
        memcpy(&dev_values_.RO, &data.output[4], 1);
        data_mutex_.unlock();
    }
    else if (ret == GET_IO_FAIL)
    {
        setValid(false);
    }
    else if (ret == IO_DEVICE_UNFOUND)
    {
        setValid(false);
    }

    return ret;
}


//private

void FstIoDevice::initIODeviceData(IODeviceData &data)
{
    data.id = address_;
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
        data.input[0] = 0xAA;// simulation input as binary 10101010
        data.input[1] = 0xAA;
        data.input[2] = 0xAA;
        data.input[3] = 0xAA;
        data.input[4] = 0x2A;
        return SUCCESS;
    }

    uint8_t id = data.id;
    // fetch data from memory.
    if (!readWriteMem(data))
        return GET_IO_FAIL;

    // check enable and id.
    if (data.enable == 0x0 || id != data.id)
    {
        //printf("IODeviceData ID=%x, enable=%x, verify=%x, model=%x, input[]=%x-%x-%x-%x-%x,output=%x-%x-%x-%x-%x\n",
        //      data.id, data.enable, data.verify, data.model,
        //      data.input[4],data.input[3],data.input[2],data.input[1],data.input[0],
        //     data.output[4],data.output[3],data.output[2],data.output[1],data.output[0]);
        return IO_DEVICE_UNFOUND;
    }    
    if (data.verify == 0)
    {
        //printf("IODeviceData verify=%x\n", data.verify);
        return IO_VERIFY_FALSE;
    }
    return SUCCESS; 
}

bool FstIoDevice::readWriteMem(IODeviceData &data)
{
    static uint8_t seq = 1; // the flags to read and write FPGA.
    static int read_counter = 0; // the counter for the error times of reading FPGA.
    static int write_counter = 0; // the counter for the error times of writing FPGA.

    if (seq >= 15)
        seq = 1;

    uint8_t id = data.id;
    //step1: write download data(output).
    if (ioWriteDownload(&data) != 0) {
        //std::cout<<"ioWriteDownload() failed."<<std::endl;
        return false;
    }

    uint8_t idseq = id;
    idseq = idseq << 4;
    idseq += seq;

    //step2: write ID&seq.
    if (ioSetIdSeq(idseq) != 0) {
        //std::cout<<"ioSetIdSeq() failed."<<std::endl;
        return false;
    }

    // wait for FPGA reply.
    usleep(200);

    //step3:get upload and download seq.
    uint8_t  read_seq;
    if (ioGetSeq(&read_seq) != 0) {
        //std::cout<<"ioGetSeq() failed."<<std::endl;
        return false;
    }

    uint8_t upload_seq = read_seq & 0x0F;
    uint8_t download_seq = (read_seq >> 4) & 0x0F;
    //printf("seq=%x, upload_seq=%x, download_seq=%x\n",seq,upload_seq,download_seq);

    //step4:read upload data(input)
    if (upload_seq == seq) {
        if (ioReadUpload(&data) != 0) {
            //std::cout<<"ioReadUpload() failed."<<std::endl;
            return false;
        }
        read_counter = 0;
    } else {
        ++read_counter;
    }

    if (download_seq == seq)
        write_counter = 0;
    else
        ++write_counter;
        
    if (read_counter > comm_tolerance_ || write_counter > comm_tolerance_) {
        //std::cout<<"read write io with FPGA failed too many."<<std::endl;
        return false;
    }

    ++seq;
    return true;
}