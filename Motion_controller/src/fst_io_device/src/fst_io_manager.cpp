/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       io_manager.cpp
Author:     Feng.Wu 
Create:     14-Feb-2017
Modify:     18-Oct-2018
Summary:    dealing with IO module
**********************************************/
#ifndef IO_MANAGER_IO_MANAGER_CPP_
#define IO_MANAGER_IO_MANAGER_CPP_

#include "fst_io_manager.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "error_monitor.h"

namespace fst_hal
{

//------------------------------------------------------------
// Function:  IOManager
// Summary: The constructor of class
// In:      None
// Out:     None
// Return:  None 
//------------------------------------------------------------
IOManager::IOManager(fst_log::Logger* logger, fst_hal::FstIoDeviceParam* param)
{
    log_ptr_ = logger;
    param_ptr_ = param;
    cycle_time_ = 10000;
    is_virtual_ = false;
    error_ = SUCCESS;
    thread_error_ = SUCCESS;
    thread_status_ = INIT_STATUS;
}

//------------------------------------------------------------
// Function:  ~IOManager
// Summary: The destructor of class
// In:      None
// Out:     None
// Return:  None 
//------------------------------------------------------------
IOManager::~IOManager()
{
    io_thread_.interrupt();
    io_thread_.join();
}

//------------------------------------------------------------
// Function:  instance
// Summary: static instance
// In:
// Out:     None
// Return:  None
//------------------------------------------------------------
IOManager* IOManager::instance(fst_log::Logger* logger, fst_hal::FstIoDeviceParam* param)
{
    static IOManager io_manager(logger, param);
    return &io_manager;
}

//------------------------------------------------------------
// Function:    init
// Summary: Initialize 
// In:      0 -> true init, 1 -> fake init.
// Out:     None
// Return:  ErrorCode -> error codes.
//------------------------------------------------------------
ErrorCode IOManager::init(void)
{
    cycle_time_ = param_ptr_->cycle_time_;
    is_virtual_ = param_ptr_->is_virtual_;
    max_dev_num_ = param_ptr_->max_dev_number_;
    max_DI_num_ = param_ptr_->max_DI_number_;
    max_DO_num_ = param_ptr_->max_DO_number_;
    max_RI_num_ = param_ptr_->max_RI_number_;
    max_RO_num_ = param_ptr_->max_RO_number_;
    virtual_board_address_ = param_ptr_->virtual_board_address_;
    virtual_DI_number_ = param_ptr_->virtual_DI_number_;
    virtual_DO_number_ = param_ptr_->virtual_DI_number_;

    int result = 0;
    if (is_virtual_ == false){
        result = ioInit();
        if (result != 0){
            FST_ERROR("init FstSafetyDevice failed");
            fst_base::ErrorMonitor::instance()->add(IO_INIT_FAIL);
            return IO_INIT_FAIL;
        }
        FST_INFO("Use real IO");
    } else{
        is_virtual_ = true;
        FST_INFO("Use virtual IO");
    }

    startThread();

    bool ret = waitThreadFirstData();
    if(ret == false){
        FST_ERROR("init FstSafetyDevice failed: Overtime");
        fst_base::ErrorMonitor::instance()->add(IO_INIT_FAIL);
        return IO_INIT_FAIL;
    }

    return SUCCESS;
}

//------------------------------------------------------------
// Function:    refreshDevicesNum
// Summary: refresh, and get the number of devices
// In:      None
// Out:     None
// Return:  int -> the total number of io devices.
//------------------------------------------------------------
int IOManager::refreshDevicesNum(void)
{
    if (thread_status_ == RUNNING_STATUS)
        thread_status_ = INIT_STATUS;

    bool ret = waitThreadFirstData();
    if(ret == false){
        FST_ERROR("init FstSafetyDevice failed: Overtime");
        fst_base::ErrorMonitor::instance()->add(IO_INIT_FAIL);
    }

    boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
    return vector_dev_.size();
}

//------------------------------------------------------------
// Function:    getDevicesNum
// Summary: get the number of devices
// In:      None
// Out:     None
// Return:  int -> the total number of io devices.
//------------------------------------------------------------
int IOManager::getDevicesNum(void)
{
    boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
    return vector_dev_.size();
}

//------------------------------------------------------------
// Function:    getDevInfoByIndex
// Summary: get the information of each device.
// In:      index -> the sequence number of the device.
// Out:     info  -> the information of each device.
// Return:  ErrorCode   -> error codes.
//------------------------------------------------------------
ErrorCode IOManager::getDevInfoByIndex(unsigned int index, IODeviceInfo &info)
{
    std::vector<IODeviceUnit> io;
    {
        boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
        io = vector_dev_;
    }

    if (index >= io.size() || index < 0) {
        FST_ERROR("IOManager::getDevInfoByIndex():Invalid index to get the device info.");
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_DEV_INDEX);
        return IO_INVALID_DEV_INDEX;
    }

    info = io[index].info;
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    getDevInfoByID
// Summary: get the information of each device by id
// In:      in    -> the address id from device_config.xml.
// Out:     info  -> the information of each device.
// Return:  ErrorCode   -> error codes.
//------------------------------------------------------------
ErrorCode IOManager::getDevInfoByID(uint8_t address, IODeviceInfo &info)
{
    std::vector<IODeviceUnit> io;
    {
        boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
        io = vector_dev_;
    }

    int index = searchParamId(address, io);
    if (index == -1) {
        FST_ERROR("IOManager::getDevInfoByID(): Invalid address id - %d", address);
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PARAM_ID);
        return IO_INVALID_PARAM_ID;
    }

    info = io[index].info;
    return SUCCESS;
}



//------------------------------------------------------------
// Function:    getModuleValues
// Summary: get the information of each device by id
// In:      id      -> the address id from device_config.xml.
// Out:     values  -> the information of each device.
// Return:  ErrorCode   -> error codes.
// ------------------------------------------------------------
ErrorCode IOManager::getModuleValues(uint8_t address, IODevicePortValues &values)
{
    std::vector<IODeviceUnit> io;
    {
        boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
        io = vector_dev_;
    }

    int index = searchParamId(address, io);
    if (index == -1) {
        FST_ERROR("IOManager::getModuleValues(): Invalid address id - %d", address);
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PARAM_ID);
        return IO_INVALID_PARAM_ID;
    }

    values = io[index].port_values;
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    getModuleValue
// Summary: get the status value of one port on one device.
// In:      physics_id -> the id to the physics port.
// Out:     port_value -> the port status.
// Return:  ErrorCode        -> error codes.
//------------------------------------------------------------
ErrorCode IOManager::getModuleValue(uint32_t physics_id, uint8_t &port_value)
{
    std::vector<IODeviceUnit> io;
    {
        boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
        io = vector_dev_;
    }

    PhysicsID id;
    id.number = physics_id;

    int index = searchParamId(id.info.address, io);
    if (index == -1) {
        FST_ERROR("IOManager::getModuleValue(): Invalid physics id - %d", physics_id);
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PARAM_ID);
        return IO_INVALID_PARAM_ID;
    }

    int frame = (id.info.port - 1) / 8;
    int shift = (id.info.port - 1) % 8;

    if (id.info.port_type == IO_TYPE_DI) {
        if((id.info.port > io[index].info.DI_num) || (id.info.port == 0)) {
            FST_ERROR("IOManager::getModuleValue(): invalid port seq for DI - %d", id.info.port);
            fst_base::ErrorMonitor::instance()->add(IO_INVALID_PORT_SEQ);
            return IO_INVALID_PORT_SEQ;
        }

        //del port_value = io[index].data.input[frame] >> shift & 1;
        port_value = (io[index].port_values.DI[frame] >> shift) & 0x01;
        printf("get di, frame=%d, shift=%d, value=%d\n",frame, shift, port_value);
    } else if (id.info.port_type == IO_TYPE_DO) {
        if(id.info.port > io[index].info.DO_num || id.info.port == 0) {
            FST_ERROR("IOManager::getModuleValue(): invalid port seq for DO - %d", id.info.port);
            fst_base::ErrorMonitor::instance()->add(IO_INVALID_PORT_SEQ);
            return IO_INVALID_PORT_SEQ;
        }

        //del port_value = io[index].data.output[frame] >> shift & 1;
        port_value = (io[index].port_values.DO[frame] >> shift) & 0x01;
    } else if (id.info.port_type == IO_TYPE_RI){
        if(id.info.port > io[index].info.RI_num || id.info.port == 0) {
            FST_ERROR("IOManager::getModuleValue(): invalid port seq for RI - %d", id.info.port);
            fst_base::ErrorMonitor::instance()->add(IO_INVALID_PORT_SEQ);
            return IO_INVALID_PORT_SEQ;
        }

        //del port_value = io[index].data.input[frame + 4] >> shift & 1;
        port_value = (io[index].port_values.RI[frame] >> shift) & 0x01;
    }else if (id.info.port_type == IO_TYPE_RO){
        if(id.info.port > io[index].info.RO_num || id.info.port == 0) {
            FST_ERROR("IOManager::getModuleValue(): invalid port seq for DO - %d", id.info.port);
            fst_base::ErrorMonitor::instance()->add(IO_INVALID_PORT_SEQ);
            return IO_INVALID_PORT_SEQ;
        }

        //del port_value = io[index].data.output[frame + 4] >> shift & 1;
        port_value = (io[index].port_values.RO[frame] >> shift) & 0x01;
    }else {
        return IO_INVALID_PARAM_ID;
    }


    //for virtual board
    if(id.info.address == param_ptr_->virtual_board_address_)
    {
        return virtualGetModuleValue(physics_id, port_value, index, io);
    }


    return SUCCESS;
}

//------------------------------------------------------------
// Function:    getModuleValues
// Summary: get the status values of all ports on one device
// In:      id  -> the parameter id according to the parameter path.
//          len -> the size of the memory to store port values.
// Out:     ptr -> the address of the memoryto store port values.
//          num -> the total number of the ports.
// Return:  ErrorCode -> error codes.
//------------------------------------------------------------
/*
ErrorCode IOManager::getModuleValues(uint32_t physics_id, int len, unsigned char *ptr, int &num)
{
    if (thread_status_ != RUNNING_STATUS){
        fst_base::ErrorMonitor::instance()->add(IO_THREAD_INIT_STATUS);
        return IO_THREAD_INIT_STATUS;
    }

    std::vector<IODeviceUnit> io;
    {
        boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
        io = vector_dev_;
    }
    PhysicsID id;
    id.number = physics_id;
    int index = searchParamId(id.info.address, io);
    if (index == -1)
    {
        std::cout<<"Error in IOManager::getModuleValues(): Invalid id."<<std::endl;
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PARAM_ID);
        return IO_INVALID_PARAM_ID;
    }
    
    size_t input_num = (io[index].info.DI_num)/8;
    if ((io[index].info.DI_num)%8 != 0)
        input_num++;
    size_t output_num = (io[index].info.DO_num)/8;
    if ((io[index].info.DO_num)%8 != 0)
        output_num++;
    num = input_num + output_num;

    if (len < num)
    {
        std::cout<<"Error in IOManager::getModuleValues(): Invalid port len."<<std::endl;
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PORT_LEN);
        return IO_INVALID_PORT_LEN;
    }

    memcpy(ptr, &(io[index].data.input), input_num);
    ptr = ptr + input_num;
    memcpy(ptr, &(io[index].data.output), output_num);
    ptr = ptr - input_num;

    return SUCCESS;
}
*/

//------------------------------------------------------------
// Function:    setModuleValue
// Summary: set the status value of one port on one device.
// In:      physics_id -> the id to find the physics port.
// Out:     None.
// Return:  ErrorCode        -> error codes.
//------------------------------------------------------------
ErrorCode IOManager::setModuleValue(uint32_t physics_id, uint8_t port_value)
{
    std::vector<IODeviceUnit> io;
    {
        boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
        io = vector_dev_;
    }

    PhysicsID id;
    id.number = physics_id;
    //printf("the id:%d, port_seq:%d, port_value:%d\n", id.info.address, id.info.port, port_value);
    int index = searchParamId(id.info.address, io);
    if (index == -1)
    {
        //std::cout<<"Error in IOManager::setModuleValue(): Invalid id."<<std::endl;
        FST_ERROR("IOManager::setModuleValue(): Invalid physics id - %d", physics_id);
        fst_base::ErrorMonitor::instance()->add(IO_INVALID_PARAM_ID);
        return IO_INVALID_PARAM_ID;
    }

    //for virtual board
    if(id.info.address == param_ptr_->virtual_board_address_)
    {
        return virtualSetModuleValue(physics_id, port_value, index, io);
    }


    int frame = (id.info.port - 1) / 8;
    int shift = (id.info.port - 1) % 8;
    if (id.info.port_type == IO_TYPE_DO) {
        if((id.info.port > io[index].info.DO_num) || (id.info.port == 0)) {
            FST_ERROR("IOManager::setModuleValue(): invalid port seq for DO - %d", id.info.port);
            fst_base::ErrorMonitor::instance()->add(IO_INVALID_PORT_SEQ);
            return IO_INVALID_PORT_SEQ;
        }

    } else if (id.info.port_type == IO_TYPE_RO) {
        if((id.info.port > io[index].info.RO_num) || (id.info.port == 0)) {
            FST_ERROR("IOManager::setModuleValue(): invalid port seq for RO - %d", id.info.port);
            fst_base::ErrorMonitor::instance()->add(IO_INVALID_PORT_SEQ);
            return IO_INVALID_PORT_SEQ;
        }
        frame = frame + DO_FRAME_MAX;
    }

    {
        boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
        if (port_value == 0) {
            vector_dev_[index].output[frame] &= ~(0x01 << shift);
        }
        else
            vector_dev_[index].output[frame] |= 0x01 << shift;
    }

    return SUCCESS;
}

//------------------------------------------------------------
// Function:    getThreadError
// Summary: get the error status of the io thread.
// In:      None.
// Out:     None.
// Return:  ErrorCode -> error codes.
//------------------------------------------------------------
ErrorCode IOManager::getIOError(void)
{
    if (error_.load() != 0){
        return error_.load();
    }
    return SUCCESS;
}


bool IOManager::waitThreadFirstData(void)
{
    // wait until the thread is in runnning status.
    boost::posix_time::ptime time_begin, time_end;
    boost::posix_time::millisec_posix_time_system_config::time_duration_type time_elapse;
    time_begin = boost::posix_time::microsec_clock::universal_time();

    while (thread_status_ == INIT_STATUS) {
        usleep(cycle_time_);
        time_end = boost::posix_time::microsec_clock::universal_time();
        time_elapse = time_end - time_begin;
        int ticks = time_elapse.ticks();
        if (ticks > 100*cycle_time_)
            return false;
    }
    return true;
}

//------------------------------------------------------------
// Function:    searchParamId
// Summary: find the index of table according to the parameter id.
// In:      id  -> the parameter id according to the parameter path.
//          io  -> the table to store all the io data.
// Out:     None.
// Return:  int -> the index of the table.
//          -1  -> can't find.
//------------------------------------------------------------
int IOManager::searchParamId(unsigned int id, std::vector<IODeviceUnit> &io)
{
    int len = io.size();
    int index = -1;
    for (int i = 0; i < len; ++i) {
        if (id == io[i].info.id) {
            index = i;
            break;
        }
    }
    return index;
}

//------------------------------------------------------------
// Function:    startThread
// Summary: start a thread.
// In:      None.
// Out:     None.
// Return:  None.
//------------------------------------------------------------
void IOManager::startThread(void)
{
    io_thread_ = boost::thread(boost::bind(&IOManager::runThread, this));
    //io_thread.timed_join(boost::posix_time::milliseconds(100)); // check the thread running or not.
}

//------------------------------------------------------------
// Function:    runThread
// Summary: main function of io thread.
// In:      None.
// Out:     None.
// Return:  None.
//------------------------------------------------------------
void IOManager::runThread(void)
{
    try
    {
        while (true)
        {
            updateThreadFunc();
            // set interruption point.
            boost::this_thread::sleep(boost::posix_time::microseconds(cycle_time_));
        }
    }
    catch (boost::thread_interrupted &)
    {
        FST_INFO("~Stop IO Thread Safely.~");
    }
}

void IOManager::updateThreadFunc()
{
    if (thread_status_ == INIT_STATUS)
    {
        initDevicesData();
        thread_status_ = RUNNING_STATUS;
    }
    updateDevicesData();
}

//------------------------------------------------------------
// Function:    initDeviceData
// Summary: read io and load configuration file when the thread is starting up.
// In:      None.
// Out:     None.
// Return:  None.
//------------------------------------------------------------
void IOManager::initDevicesData(void)
{
    //clean vector_dev_ vector before init io data.
    {
        boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
        std::vector <IODeviceUnit>().swap(vector_dev_);
        error_ = SUCCESS;
        thread_error_ = SUCCESS;
    }

    // Get the devices data from shared memory.
    IODeviceUnit unit;
    IODeviceData data;
    ErrorCode ret = SUCCESS;
    for (int id = 0; id < max_dev_num_; ++id)
    {
        initIODeviceUnit(unit);
        initIODeviceData(data);
        data.id = id;
        //printf("|== Thread init data before data.id= %d\n", data.id);
        ret = getDeviceDataFromMem(data);
        //printf("|== Thread init data after  data.id=%d, enable=%d, err=%lx\n\n",data.id, data.enable, ret);

        if (ret == SUCCESS) {
            // fill the unit.info.
            unit.info.id = data.id; //recode in future because the source of id need to be defined.
            unit.info.dev_type = DEVICE_TYPE_FST_IO;
            unit.info.device_type = "RF-P8A";
            unit.info.comm_type = "RS485";
            unit.info.DI_num = max_DI_num_;
            unit.info.DO_num = max_DO_num_;
            unit.info.RI_num = max_RI_num_;
            unit.info.RO_num = max_RO_num_;
            unit.info.is_valid = true;

            {   //fill the unit.port_values.
                boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
                unit.port_values.id = data.id;
                memcpy(&unit.port_values.DI, &data.input, DI_FRAME_MAX);
                memcpy(&unit.port_values.DO, &data.output, DO_FRAME_MAX);
                memcpy(&unit.port_values.RI, &data.input[DI_FRAME_MAX], RI_FRAME_MAX);
                memcpy(&unit.port_values.RO, &data.output[DO_FRAME_MAX], RO_FRAME_MAX);
                vector_dev_.push_back(unit);
            }
        } else if (ret == GET_IO_FAIL) {
            thread_error_ = GET_IO_FAIL;
        }
    }

    //add virtual board.
    addVirtualBoard();

    int num = vector_dev_.size();
    for(int i = 0; i < num; ++i)
        printf("board num=%d, address=%d\n", num, vector_dev_[i].info.id);

    setThreadError();
}

//------------------------------------------------------------
// Function:    updateDeviceData
// Summary: update the io data.
// In:      None.
// Out:     None.
// Return:  ErrorCode -> error codes.
//------------------------------------------------------------
ErrorCode IOManager::updateDevicesData(void)
{
    int num = 0;
    {
        boost::mutex::scoped_lock lock(mutex_);  //------lock mutex-----//
        num = vector_dev_.size();
    }

    // Get the devices data from shared memory.
    IODeviceData data;
    ErrorCode ret = SUCCESS;
    for (int index = 0; index < num; ++index)
    {
        initIODeviceData(data);

        // update begins with id and output data filled.
        {
            boost::mutex::scoped_lock lock(mutex_);  //------lock mutex-----//
            data.id = vector_dev_[index].port_values.id;
            memcpy(data.output, vector_dev_[index].output, sizeof(data.output));
        }

        ret = getDeviceDataFromMem(data);

        switch (ret)
        {
            case SUCCESS:
                {   //fill the unit.port_values.
                    boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
                    memcpy(&vector_dev_[index].port_values.DI, &data.input, DI_FRAME_MAX);
                    memcpy(&vector_dev_[index].port_values.DO, &data.output, DO_FRAME_MAX);
                    memcpy(&vector_dev_[index].port_values.RI, &data.input[DI_FRAME_MAX], RI_FRAME_MAX);
                    memcpy(&vector_dev_[index].port_values.RO, &data.output[DO_FRAME_MAX], RO_FRAME_MAX);
                }
                break;
            case GET_IO_FAIL:
                //std::cout<<"ERROR in IOManager::updataDevicesData():fail to fetch data from memory."<<std::endl;
                thread_error_ = GET_IO_FAIL;
                break;
            case IO_DEVICE_CHANGED:
                //std::cout<<"ERROR in IOManager::updataDevicesData():IO device changed on "<<int(data.id)<<std::endl;
                thread_error_ = IO_DEVICE_CHANGED;
                vector_dev_[index].info.is_valid = false;
                break;
            case IO_VERIFY_FALSE:
            default:
                break;
        }
    }

    setThreadError();
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    setThreadError
// Summary: set the thread_error_.
// In:      None.
// Out:     None.
// Return:  None.
//------------------------------------------------------------
void IOManager::setThreadError(void)
{
    static ErrorCode pre_error = SUCCESS;
    if (thread_error_ != SUCCESS || pre_error != thread_error_){
        error_.store(thread_error_);
        fst_base::ErrorMonitor::instance()->add(error_);
        pre_error = thread_error_;
    }
}

//------------------------------------------------------------
// Function:    initIODeviceUnit
// Summary: initialize the variable of IODeviceUnit structure.
// In:      None.
// Out:     io -> variable.
// Return:  None.
//------------------------------------------------------------
void IOManager::initIODeviceUnit(IODeviceUnit &io)
{
    for (int i = 0; i< IO_DATAFRAME_MAX; ++i)
        io.output[i] = 0;
    io.info.id = 0;
    io.info.dev_type = DEVICE_TYPE_FST_IO;
    io.info.device_type = "";
    io.info.comm_type = "";
    io.info.DI_num = 0;
    io.info.DO_num = 0;
    io.info.RI_num = 0;
    io.info.RO_num = 0;
    io.info.is_valid = true;
}

//------------------------------------------------------------
// Function:    initIODeviceData
// Summary: initialize the variable of IODeviceData structure.
// In:      None.
// Out:     io -> variable.
// Return:  None.
//------------------------------------------------------------
void IOManager::initIODeviceData(IODeviceData &data)
{
    data.model = 0;
    data.enable = 1;
    data.verify = 1;
    for (int i = 0; i< IO_DATAFRAME_MAX; ++i) {
        data.output[i] = 0;
        data.input[i] = 0;
    }
}

//------------------------------------------------------------
// Function:    getDeviceDataFromMem
// Summary: get io data by RS485 and check data.
// In:      None.
// Out:     data -> io data from RS485.
// Return:  ErrorCode  -> error codes.
//------------------------------------------------------------
ErrorCode IOManager::getDeviceDataFromMem(IODeviceData &data)
{
    if (is_virtual_ == true) {
        data.input[0] = 0xA0;// simulation input as binary 10101010
        data.input[1] = 0xA1;
        data.input[2] = 0xFF;
        data.input[3] = 0xFF;
        data.input[4] = 0xFF;
        return SUCCESS;
    }
    //for virtual board
    if(data.id == param_ptr_->virtual_board_address_)
    {
        return SUCCESS;
    }

    uint8_t id = data.id;
    // fetch data from memory.
    if (!readWriteMem(data))
        return GET_IO_FAIL;

    // check enable and id.
    if (data.enable == 0x0 || id != data.id)
    {
        //printf("after----IODeviceData ID=%x, enable=%x, verify=%x, model=%x, input[]=%x-%x-%x-%x-%x,output=%x-%x-%x-%x-%x\n",
        //      data.id, data.enable, data.verify, data.model,
        //      data.input[4],data.input[3],data.input[2],data.input[1],data.input[0],
        //      data.output[4],data.output[3],data.output[2],data.output[1],data.output[0]);
        return IO_DEVICE_CHANGED;
    }    
    if (data.verify == 0)
    {
        //printf("after----IODeviceData verify=%x\n", data.verify);
        return IO_VERIFY_FALSE;
    }
    return SUCCESS;   
}

//------------------------------------------------------------
// Function:    readWriteMem
// Summary: interact with FPGA to get data.
// In:      None.
// Out:     data  -> io data from RS485.
// Return:  true  -> get data successfully.
//          false -> get data failed.
//------------------------------------------------------------
bool IOManager::readWriteMem(IODeviceData &data)
{
    static uint8_t seq = 1; // the flags to read and write FPGA.
    static int read_counter = 0; // the counter for the error times of reading FPGA.
    static int write_counter = 0; // the counter for the error times of writing FPGA.

    if (seq >= 15)
        seq = 1;

    uint8_t id = data.id;
    //write download data.
    if (ioWriteDownload(&data) != 0) {
        std::cout<<"ioWriteDownload() failed."<<std::endl;
        return false;
    }

    uint8_t idseq = id;
    idseq = idseq << 4;
    idseq += seq;

    //write ID&seq.
    if (ioSetIdSeq(idseq) != 0) {
        std::cout<<"ioSetIdSeq() failed."<<std::endl;
        return false;
    }

    // wait for FPGA reply.
    boost::this_thread::sleep(boost::posix_time::microseconds(200));

    //get upload and download seq.
    uint8_t  read_seq;
    if (ioGetSeq(&read_seq) != 0) {
        std::cout<<"ioGetSeq() failed."<<std::endl;
        return false;
    }

    uint8_t upload_seq = read_seq & 0x0F;
    uint8_t download_seq = (read_seq >> 4) & 0x0F;
    //printf("seq=%x, upload_seq=%x, download_seq=%x\n",seq,upload_seq,download_seq);
    if (upload_seq == seq) {
        if (ioReadUpload(&data) != 0) {
            std::cout<<"ioReadUpload() failed."<<std::endl;
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
    if (read_counter > FAULT_TOL || write_counter > FAULT_TOL) {
        std::cout<<"read write io driver failed too many."<<std::endl;
        return false;
    }

    ++seq;
    return true;
}

} //namespace fst_io_manager

#endif //IO_MANAGER_IO_MANAGER_CPP_
