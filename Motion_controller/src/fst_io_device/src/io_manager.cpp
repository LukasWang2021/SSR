/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       io_manager.cpp
Author:     Feng.Wu 
Create:     14-Feb-2017
Modify:     08-Jun-2017
Summary:    dealing with IO module
**********************************************/
#ifndef IO_MANAGER_IO_MANAGER_CPP_
#define IO_MANAGER_IO_MANAGER_CPP_

#include "io_manager.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace fst_io_manager
{

//------------------------------------------------------------
// Function:  IOManager
// Summary: The constructor of class
// In:      None
// Out:     None
// Return:  None 
//------------------------------------------------------------
IOManager::IOManager()
{
    seq_ = 1;
    read_counter_ = 0;
    write_counter_ = 0;
    thread_error_ = THREAD_SUCCESS;
    last_error_ = THREAD_SUCCESS;
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
    io_thread.interrupt();
    io_thread.join();
}

//------------------------------------------------------------
// Function:    init
// Summary: Initialize 
// In:      None
// Out:     None
// Return:  U64 -> error codes.
//------------------------------------------------------------
U64 IOManager::init(int fake)
{
    error_map_[THREAD_LOAD_IO_CONFIG_FAIL] = LOAD_IO_CONFIG_FAIL;
    error_map_[THREAD_GET_IO_FAIL] = GET_IO_FAIL;
    error_map_[THREAD_IO_DEVICE_CHANGED] = IO_DEVICE_CHANGED;

    int result = 0;
    switch (fake)
    {
        case 0:
            result = ioInit(0);
            break;
        case 1:
        default:
            result = ioInit(1);
            break;
    }
    
    if (result != 0)
        return IO_INIT_FAIL;
    startThread();
    return SUCCESS;

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
    if (thread_status_ == RUNNING_STATUS)
        thread_status_ = INIT_STATUS;

    // wait until the thread is in runnning status.
    boost::posix_time::ptime time_begin, time_end;
    boost::posix_time::millisec_posix_time_system_config::time_duration_type time_elapse;
    time_begin = boost::posix_time::microsec_clock::universal_time();

    while (thread_status_ == INIT_STATUS)
    {
        usleep(2*1000);
        time_end = boost::posix_time::microsec_clock::universal_time();
        time_elapse = time_end - time_begin;
        int ticks = time_elapse.ticks();
        if (ticks > 500*1000)
        {
            std::cout<<"Error in getDevicesNum():Overtime:"<<ticks<<"us"<<std::endl;
            return -1;
        }
    }

    boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
    int num = io_r_.size();
    return num;
}

//------------------------------------------------------------
// Function:    getDeviceInfo
// Summary: get the information of each device.
// In:      index -> the sequence number of the device.
// Out:     info  -> the information of each device.
// Return:  U64   -> error codes.
//------------------------------------------------------------
U64 IOManager::getDeviceInfo(unsigned int index, IODeviceInfo &info)
{
    if (thread_status_ != RUNNING_STATUS)
        return IO_THREAD_INIT_STATUS;

    std::vector<IOTable> io;
    {
        boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
        io = io_r_;
    }

    if (index >= io.size() || index < 0)
    {
        std::cout<<"Error in IOManager::getDeviceInfo():Invalid index to get the device info."<<std::endl;
        return IO_INVALID_DEV_INDEX;
    }        
    info = io[index].info;
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    getModuleValue
// Summary: get the status value of one port on one device.
// In:      id         -> the parameter id according to the parameter path.
//          port_type  -> IO_INPUT or IO_output.
//          port_seq   -> the sequence number of the ports.
// Out:     port_value -> the port status.
// Return:  U64        -> error codes.
//------------------------------------------------------------
U64 IOManager::getModuleValue(unsigned int id, int port_type, unsigned int port_seq, unsigned char &port_value)
{
/*    boost::posix_time::ptime time_now, time_now1;
    boost::posix_time::millisec_posix_time_system_config::time_duration_type time_elapse;
    time_now = boost::posix_time::microsec_clock::universal_time();
*/
    if (thread_status_ != RUNNING_STATUS)
        return IO_THREAD_INIT_STATUS;

    std::vector<IOTable> io;
    {
        boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
        io = io_r_;
    }

    int index = searchParamId(id, io);
    if (index == -1)
    {
        std::cout<<"Error in IOManager::getModuleValue(): Invalid id."<<std::endl;
        return IO_INVALID_PARAM_ID;
    }

    if (port_type == IO_INPUT)
    {
        if((port_seq > io[index].info.input) || (port_seq == 0))
        {
            std::cout<<"Error in IOManager::getModuleValue():invalid port seq for input."<<std::endl;
            return IO_INVALID_PORT_SEQ;
        }
        int frame = (port_seq - 1) / 8;
        int shift = (port_seq - 1) % 8;
        port_value = io[index].data.input[frame] >> shift & 1;
    }
    else
    {
        if(port_seq > io[index].info.output || port_seq == 0)
        {
            std::cout<<"Error in IOManager::getModuleValue():invalid port seq for output."<<std::endl;
            return IO_INVALID_PORT_SEQ;
        }
        int frame = (port_seq - 1) / 8;
        int shift = (port_seq - 1) % 8;
        port_value = io[index].data.output[frame] >> shift & 1;
    } 
/*
    time_now1 = boost::posix_time::microsec_clock::universal_time();
    time_elapse = time_now1 - time_now;
    std::cout<<"time elapse in getModuleValue():"<<time_elapse.ticks()<<std::endl;
*/
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    getModuleValues
// Summary: get the status values of all ports on one device
// In:      id  -> the parameter id according to the parameter path.
//          len -> the size of the memory to store port values.
// Out:     ptr -> the address of the memoryto store port values.
//          num -> the total number of the ports.
// Return:  U64 -> error codes.
//------------------------------------------------------------
U64 IOManager::getModuleValues(unsigned int id, int len, unsigned char *ptr, int &num)
{
    if (thread_status_ != RUNNING_STATUS)
        return IO_THREAD_INIT_STATUS;

    std::vector<IOTable> io;
    {
        boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
        io = io_r_;
    }

    int index = searchParamId(id, io);
    if (index == -1)
    {
        std::cout<<"Error in IOManager::getModuleValues(): Invalid id."<<std::endl;
        return IO_INVALID_PARAM_ID;
    }
/*    num = io[index].info.input + io[index].info.output;
*/
    size_t input_num = (io[index].info.input)/8;
    if ((io[index].info.input)%8 != 0)
        input_num++;
    size_t output_num = (io[index].info.output)/8;
    if ((io[index].info.output)%8 != 0)
        output_num++;
    num = input_num + output_num;

    if (len < num)
    {
        std::cout<<"Error in IOManager::getModuleValues(): Invalid port len."<<std::endl;
        return IO_INVALID_PORT_LEN;
    }

    memcpy(ptr, &(io[index].data.input), input_num);
    ptr = ptr + input_num;
    memcpy(ptr, &(io[index].data.output), output_num);
    ptr = ptr - input_num;
/*
    int frame, shift;
    for (unsigned int i = 0; i < io[index].info.input; ++i)
    {
        frame = i / 8;
        shift = i % 8;
        *ptr = io[index].data.input[frame] >> shift & 1;
        ++ptr;
    }

    for (unsigned int i = 0; i < io[index].info.output; ++i)
    {
        frame = i / 8;
        shift = i % 8;
        *ptr = io[index].data.output[frame] >> shift & 1;
        ++ptr;
    }
    ptr = ptr - num;
*/
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    setModuleValue
// Summary: set the status value of one port on one device.
// In:      id         -> the parameter id according to the parameter path.
//          port_seq   -> the sequence number of the port.
//          port_value -> the status value of the port.
// Out:     None.
// Return:  U64        -> error codes.
//------------------------------------------------------------
U64 IOManager::setModuleValue(unsigned int id, unsigned int port_seq, unsigned char port_value)
{      
    if (thread_status_ != RUNNING_STATUS)
        return IO_THREAD_INIT_STATUS;
    //printf("the id:%d, port_seq:%d, port_value:%d\n", id, port_seq, port_value);

    std::vector<IOTable> io;
    {
        boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
        io = io_r_;
    }

    int index = searchParamId(id, io);
    if (index == -1)
    {
        std::cout<<"Error in IOManager::setModuleValue(): Invalid id."<<std::endl;
        return IO_INVALID_PARAM_ID;
    }

    if (port_seq > io[index].info.output || port_seq == 0)
    {
        std::cout<<"Error in IOManager::setModuleValue():invalid port seq for output."<<std::endl;
        return IO_INVALID_PORT_SEQ;
    }

    int frame = (port_seq - 1) / 8;
    int shift = (port_seq - 1) % 8;

    {
        boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
        if (port_value == 0)
            io_r_[index].output[frame] &= ~(1 << shift);
        else
            io_r_[index].output[frame] |= 1 << shift;
    }

    printf("io_r:%x,%x,%x,%x,%x\n", io_r_[index].output[0], io_r_[index].output[1], io_r_[index].output[2], io_r_[index].output[3],io_r_[index].output[4]);

    return SUCCESS;
}

//------------------------------------------------------------
// Function:    getThreadError
// Summary: get the error status of the io thread.
// In:      None.
// Out:     None.
// Return:  U64 -> error codes.
//------------------------------------------------------------
U64 IOManager::getIOError(void)
{
//    if (thread_error_ != 0)
//        std::cout<<"Error in getThreadError(): error code = "<<thread_error_<<std::endl;
    return error_map_[thread_error_];
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
int IOManager::searchParamId(unsigned int id, std::vector<IOTable> &io)
{
    int len = io.size();
    int index = -1;
    for (int i = 0; i < len; ++i)
    {
        if (id == io[i].info.id)
        {
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
    io_thread = boost::thread(boost::bind(&IOManager::runThread, this));
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
            if (thread_status_ == INIT_STATUS)
            {
                initDevicesData();
                thread_status_ = RUNNING_STATUS;
                setThreadError();
            }
            updateDevicesData();
            setThreadError();

            // set interruption point.
            boost::this_thread::sleep(boost::posix_time::microseconds(LOOP_CYCLE));
        }
    }
    catch (boost::thread_interrupted &)
    {
        std::cout<<"~Stop IO Thread Safely.~"<<std::endl;
    }
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
    //clean io_r_ vector before init io data.    
    {
        boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
        std::vector <IOTable>().swap(io_r_);
        thread_error_ = THREAD_SUCCESS;
        last_error_ = THREAD_SUCCESS;
    }

    IOTable io;
    // Get the devices from 485 connection.
    U64 result = 0;
    for (int id = 0; id < RS_DEV_NUM; ++id)
    {
        initIOTableVar(io);
        io.data.id = id;
        //std::cout<<"|=== before data.id="<<int(io.data.id)<<". result="<<result<<std::endl;
        result = getDeviceDataBy485(io.data);
        //std::cout<<"|==== data.id="<<int(io.data.id)<<". data.enable="<<int(io.data.enable)<<". result="<<result<<std::endl<<std::endl;


        if (result == SUCCESS)
        {
            // implement the info for tp.
            io.info.communication_type = "RS485";
            io.info.device_number = id;            
            io.info.path = getPathOfDevice("RS485", id);
            io.info.id = id * MULTIPLIER + ID_DIFF;             
            implementConfigFile(io);
            {
                boost::mutex::scoped_lock lock(mutex_); //------locak mutex-----//
                io_r_.push_back(io);
            }
        }
        else if (result == GET_IO_FAIL)
        {
            last_error_ = THREAD_GET_IO_FAIL;
        }
    }
}

//------------------------------------------------------------
// Function:    updateDeviceData
// Summary: refreshen the io data.
// In:      None.
// Out:     None.
// Return:  U64 -> error codes.
//------------------------------------------------------------
U64 IOManager::updateDevicesData(void)
{
    IOTable io;
    initIOTableVar(io);

    int size = 0;
    {
        boost::mutex::scoped_lock lock(mutex_);  //------locak mutex-----//
        size= io_r_.size();
    }
    U64 result = 0;
    for (int index = 0; index < size; ++index)
    {
        // update begins with id and output data filled.
        {
            boost::mutex::scoped_lock lock(mutex_);  //------lock mutex-----//
            io.data.id = io_r_[index].data.id;
            memcpy(io.data.output, io_r_[index].output, sizeof(io.data.output));
        }

        result = getDeviceDataBy485(io.data);

        switch (result)
        {
            case SUCCESS:
                {
                    boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
                    io_r_[index].data = io.data;
                }
                break;
            case GET_IO_FAIL:
//                std::cout<<"ERROR in IOManager::updataDevicesData():fail to call IO driver."<<std::endl;
                last_error_ = THREAD_GET_IO_FAIL;
                break;
            case IO_DEVICE_CHANGED:
//                std::cout<<"ERROR in IOManager::updataDevicesData():IO device changed on "<<int(io.data.id)<<std::endl;
                last_error_ = THREAD_IO_DEVICE_CHANGED;
                break;
            case IO_VERIFY_FALSE:
            default:
                break;
        }
    }
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
    if (last_error_ != THREAD_SUCCESS)
        thread_error_ = last_error_;
}

//------------------------------------------------------------
// Function:    initIOTableVar
// Summary: initialize the variable of IOTable structure.
// In:      None.
// Out:     io -> variable.
// Return:  None.
//------------------------------------------------------------
void IOManager::initIOTableVar(IOTable &io)
{
    io.data.model = 0;
    io.data.enable = 1;
    io.data.verify = 1;
    for (int i = 0; i< IO_DATAFRAME_MAX; ++i)
    {
        io.data.output[i] = 0;
        io.data.input[i] = 0;
        io.output[i] = 0;
    }
}

//------------------------------------------------------------
// Function:    implementConfigFile
// Summary: try to load configuration files to fill the complete information.
// In:      None.
// Out:     io -> fill io.info.
// Return:  None.
//------------------------------------------------------------
void IOManager::implementConfigFile(IOTable &io)
{
    bool load_flag = true;
    int model = io.data.model;
    // load yaml file if never load before.
    if (input_map_.find(model) == input_map_.end())
    {
        if (!loadConfigFile(model))
        {
            std::cout<<"Error in IOManager::implementConfigFile():failed to load io configuration file."<<std::endl;
            last_error_ = THREAD_LOAD_IO_CONFIG_FAIL;
            load_flag = false;
        }
    }
    // fill the info with config.yaml file data.
    if (load_flag == true)
    { 
        io.info.device_type = type_map_[model]; 
        io.info.input = input_map_[model];
        io.info.output = output_map_[model];
    }
    else
    {
        io.info.device_type = UNKNOWN; 
        io.info.input = 0; 
        io.info.output = 0; 
    }
}

//------------------------------------------------------------
// Function:    loadConfigFile
// Summary: try to load configuration files to fill the complete information.
// In:      model -> the model type of the device.
// Out:     None.
// Return:  true  -> load file successfully.
//          false -> load file failed.
//------------------------------------------------------------
bool IOManager::loadConfigFile(int model)
{
    // load configuration yaml file.
    std::ostringstream oss;
    oss<<"share/configuration/machine/io_"<<model<<".yaml";
    std::string ss = oss.str();      
    if (!param_.loadParamFile(ss))
    {
        std::cout<<"Error in IOManager::loadConfigFile():failed to load -> "<<ss<<std::endl;
        return false;
    }

    // load parameters in yaml file.
    std::string name;
    int input, output, type;
    if (!param_.getParam("io/name", name))
    {
        std::cout<<"Error in IOManager::loadConfigFile():failed to read "<<ss<<"/io/name."<<std::endl;
        return false;
    }
    if (!param_.getParam("io/input", input))
    {
        std::cout<<"Error in IOManager::loadConfigFile():failed to read "<<ss<<"/io/input."<<std::endl;
        return false;
    }
    if (!param_.getParam("io/output", output))
    {
        std::cout<<"Error in IOManager::loadConfigFile():failed to read "<<ss<<"/io/output."<<std::endl;
        return false;
    }
    if (!param_.getParam("io/type", type))
    {
        std::cout<<"Error in IOManager::loadConfigFile():failed to read "<<ss<<"/io/type."<<std::endl;
        return false;
    }
   
    name_map_[model] = name;            // store a <model,name> map.
    input_map_[model] = input;          // store a <model,input> map.
    output_map_[model] = output;        // store a <model,output> map.
    type_map_[model] = (DeviceType)type;// store a <model,deviceType> map.

    return true;
}

//------------------------------------------------------------
// Function:    getPathOfDevice
// Summary: create the parameter path.
// In:      type          -> the communication type.
//          device_number -> the id in device.
// Out:     None.
// Return:  std::string   -> the parameter path.
//------------------------------------------------------------
std::string IOManager::getPathOfDevice(std::string type, int device_number)
{
    std::ostringstream oss;
    oss<<"root/IO/"<<type<<"/"<<device_number;
    std::string path = oss.str(); 
    return path;
}

//------------------------------------------------------------
// Function:    getDeviceDataBy485
// Summary: get io data by RS485 and check data.
// In:      None.
// Out:     data -> io data from RS485.
// Return:  U64  -> error codes.
//------------------------------------------------------------
U64 IOManager::getDeviceDataBy485(IODeviceData &data)
{
    uint8_t id = data.id;
    // call io driver.    
    if (!readWriteBy485Driver(data))
        return GET_IO_FAIL;
    // check enable and id.
    if (data.enable == 0x0 || id != data.id)
    {
//        std::cout<<"data.enable="<<int(data.enable)<<". id="<<int(id)<<". data.id="<<int(data.id)<<std::endl;
        return IO_DEVICE_CHANGED;
    }    
    if (data.verify == 0)
    {
//        std::cout<<"data.verify="<<int(data.verify)<<std::endl;
        return IO_VERIFY_FALSE;
    }
    return SUCCESS;   
}

//------------------------------------------------------------
// Function:    readWriteBy485Driver
// Summary: interact with FPGA to get data.
// In:      None.
// Out:     data  -> io data from RS485.
// Return:  true  -> get data successfully.
//          false -> get data failed.
//------------------------------------------------------------
bool IOManager::readWriteBy485Driver(IODeviceData &data)
{
    if (seq_ >= 15)
        seq_ = 1;

    uint8_t id = data.id;
    //write download data.
    if (ioWriteDownload(&data) != 0)
    {
        std::cout<<"ioWriteDownload() failed."<<std::endl;
        return false;
    }

    uint8_t idseq = id;
    idseq = idseq << 4;
    idseq += seq_;

    //write ID&seq.
    if (ioSetIdSeq(idseq) != 0)
    {
        std::cout<<"ioSetIdSeq() failed."<<std::endl;
        return false;
    }

    // wait for FPGA reply.
    boost::this_thread::sleep(boost::posix_time::microseconds(200));

    //get upload and download seq.
    uint8_t  read_seq;
    if (ioGetSeq(&read_seq) != 0)
    {
        std::cout<<"ioGetSeq() failed."<<std::endl;
        return false;
    }

    uint8_t upload_seq = read_seq & 0x0F;
    uint8_t download_seq = (read_seq >> 4) & 0x0F;
    if (upload_seq == seq_)
    {
        if (ioReadUpload(&data) != 0)
        {
            std::cout<<"ioReadUpload() failed."<<std::endl;
            return false;
        }
        read_counter_ = 0;
    }
    else
    {
        ++read_counter_;
    }

    if (download_seq == seq_)
        write_counter_ = 0;
    else
        ++write_counter_;
    if (read_counter_ > FAULT_TOL || write_counter_ > FAULT_TOL)
    {
        std::cout<<"read write io driver failed too many."<<std::endl;
        return false;
    }

    ++seq_;  

    return true;
}


/*
//simulate driver
int IOManager::ioSetIdSeq(uint8_t idseq)
{
    return 0;
}
int IOManager::ioGetSeq(uint8_t *seq)
{
    *seq = 0x88;
    return 0;
}
int IOManager::ioWriteDownload(struct IODeviceData *idd)
{
    return 0;
}
int IOManager::ioReadUpload(struct IODeviceData *idd)
{
    int id = idd->id;
    if (id == 0)
    {
        idd->id = 0x00;
        idd->enable = 0x01;
        idd->verify = 0x01;
        idd->model = 0x0;
        for (int i = 0; i < IO_DATAFRAME_MAX; ++i)
        {
            idd->input[i] = 0x0F; 
           // idd->output[i] = 0x0F;
        }
    }
    if (id == 1)
    {
        idd->id = 0x01;
        idd->enable = 0x01;
        idd->verify = 0x01;
        idd->model = 0x01;
        for (int i = 0; i < IO_DATAFRAME_MAX; ++i)
        {
            idd->input[i] = 0x0F; 
           // idd->output[i] = 0x0F;
        }
    }
    if (id == 2)
    {
        idd->id = 0x02;
        idd->enable = 0x01;
        idd->verify = 0x01;
        idd->model = 0x01;
        for (int i = 0; i < IO_DATAFRAME_MAX; ++i)
        {
            idd->input[i] = 0x0F;
            //if (idd->output[i] == 0)
            //    idd->output[i] = 0x0F;
        }
    }
    if (id == 3)
    {
        idd->id = 0x03;
        idd->enable = 0x0;
        idd->verify = 0x01;
        idd->model = 0x0;
        for (int i = 0; i < IO_DATAFRAME_MAX; ++i)
        {
            idd->input[i] = 0x0F; 
            //idd->output[i] = 0x0F;
        }
    }
    return 0;
}
*/


} //namespace fst_io_manager

#endif //IO_MANAGER_IO_MANAGER_CPP_
