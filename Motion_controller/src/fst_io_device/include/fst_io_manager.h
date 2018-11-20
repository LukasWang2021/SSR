/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       io_manager.h
Author:     Feng.Wu 
Create:     14-Feb-2017
Modify:     08-Jun-2017
Summary:    dealing with IO module
**********************************************/

#ifndef IO_MANAGER_IO_MANAGER_H_
#define IO_MANAGER_IO_MANAGER_H_

#include <vector>
#include <string>
#include <map>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <parameter_manager/parameter_manager_param_group.h>
#include "common_log.h"
#include "fst_io_device_param.h"
#include "base_datatype.h"
#include "base_device.h"
#include "error_code.h"
#include "fst_io_mem.h"

#define DI_FRAME_MAX 4
#define DO_FRAME_MAX 4
#define RI_FRAME_MAX 1
#define RO_FRAME_MAX 1

namespace fst_hal
{

typedef union PhysicsID
{
    uint32_t number;
    struct{
        char port:8;
        char port_type:8;
        char address:8;
        char dev_type:8;
    }info;
}PhysicsID;

// This is output info.
struct IODeviceInfo
{
    uint32_t id;        //address
    uint8_t dev_type;  //?DEVICE_TYPE_FST_IO
    std::string device_type;
    std::string comm_type;
    uint32_t DI_num;
    uint32_t DO_num;
    uint32_t RI_num;
    uint32_t RO_num;
    bool is_valid;
};

struct IODevicePortValues
{
    uint32_t id;        //address
    uint8_t DI[DI_FRAME_MAX];
    uint8_t DO[DO_FRAME_MAX];
    uint8_t RI[RI_FRAME_MAX];
    uint8_t RO[RO_FRAME_MAX];
    uint8_t virtual_DI[256];//temporary for virtual
    uint8_t virtual_DO[256];//temporary for virtual
};


/*
// This is the data to driver.
struct IODeviceData
{
    uint8_t id;
    uint8_t enable;
    uint8_t verify;
    uint8_t model;
    unsigned char input[IO_DATAFRAME_MAX]; 
    unsigned char output[IO_DATAFRAME_MAX];
};
*/

// unit data for upper.
struct IODeviceUnit
{
    //IODeviceData data;
    IODeviceInfo info;
    IODevicePortValues port_values;
    unsigned char output[IO_DATAFRAME_MAX]; // store the output request from tp.
};

typedef enum
{
    INIT_STATUS = 1,
    RUNNING_STATUS = 2,
}ThreadStatus;

typedef enum
{
    IO_TYPE_DI = 0,
    IO_TYPE_DO = 1,
    IO_TYPE_RI = 2,
    IO_TYPE_RO = 3,
    IO_TYPE_UI = 4,
    IO_TYPE_UO = 5,
}IOType;

class IOManager
{
public:
    //------------------------------------------------------------
    // Function:  IOManager
    // Summary: The constructor of class
    // In:      None
    // Out:     None
    // Return:  None 
    //------------------------------------------------------------
    IOManager(fst_log::Logger* logger, fst_hal::FstIoDeviceParam* param);

    //------------------------------------------------------------
    // Function:  ~IOManager
    // Summary: The destructor of class
    // In:      None
    // Out:     None
    // Return:  None 
    //------------------------------------------------------------
    ~IOManager();

    //------------------------------------------------------------
    // Function:  instance
    // Summary: static instance
    // In:
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    static IOManager *instance(fst_log::Logger *logger, fst_hal::FstIoDeviceParam *param);

    //------------------------------------------------------------
    // Function:    init
    // Summary: Initialize
    // In:      0 -> true init, 1 -> fake init.
    // Out:     None
    // Return:  0 -> success.
    //          ERROR_CODE -> failed.
    //------------------------------------------------------------
    ErrorCode init(void);

    //------------------------------------------------------------
    // Function:    refreshDevicesNum
    // Summary: refresh, and get the number of devices
    // In:      None
    // Out:     None
    // Return:  int -> the total number of io devices.
    //------------------------------------------------------------
    int refreshDevicesNum(void);

    //------------------------------------------------------------
    // Function:    getDevicesNum
    // Summary: get the number of devices
    // In:      None
    // Out:     None
    // Return:  int -> the total number of io devices.
    //------------------------------------------------------------
    int getDevicesNum(void);

    //------------------------------------------------------------
    // Function:    getDevInfoByIndex
    // Summary: get the information of each device.
    // In:      index -> the sequence number of the device.
    // Out:     info  -> the information of each device.
    // Return:  ErrorCode   -> error codes.
    //------------------------------------------------------------
    ErrorCode getDevInfoByIndex(unsigned int index, IODeviceInfo &info);

    //------------------------------------------------------------
    // Function:    getDevInfoByID
    // Summary: get the information of each device by id
    // In:      in    -> the address id from device_config.xml.
    // Out:     info  -> the information of each device.
    // Return:  ErrorCode   -> error codes.
    //------------------------------------------------------------
    ErrorCode getDevInfoByID(uint8_t address, IODeviceInfo &info);

    //------------------------------------------------------------
    // Function:    getModuleValues
    // Summary: get the information of each device by id
    // In:      id      -> the address id from device_config.xml.
    // Out:     values  -> the information of each device.
    // Return:  ErrorCode   -> error codes.
    //------------------------------------------------------------
    ErrorCode getModuleValues(uint8_t address, IODevicePortValues &values);

    //------------------------------------------------------------
    // Function:    getModuleValue
    // Summary: get the status value of one port on one device.
    // In:      id         -> the parameter id according to the parameter path.
    //          port_type  -> IO_INPUT or IO_output.
    //          port_seq   -> the sequence number of the ports.
    // Out:     port_value -> the port status.
    // Return:  ErrorCode        -> error codes.
    //------------------------------------------------------------
    ErrorCode getModuleValue(uint32_t physics_id, uint8_t &port_value);
    
    //------------------------------------------------------------
    // Function:    getModuleValues
    // Summary: get the status values of all ports on one device
    // In:      id  -> the parameter id according to the parameter path.
    //          len -> the size of the memory to store port values.
    // Out:     ptr -> the address of the memoryto store port values.
    //          num -> the total number of the ports.
    // Return:  ErrorCode -> error codes.
    //------------------------------------------------------------
    //ErrorCode getModuleValues(uint32_t physics_id, int len, unsigned char *ptr, int &num); // need redefine.

    //------------------------------------------------------------
    // Function:    setModuleValue
    // Summary: set the status value of one port on one device.
    // In:      id         -> the parameter id according to the parameter path.
    //          port_seq   -> the sequence number of the port.
    //          port_value -> the status value of the port.
    // Out:     None.
    // Return:  ErrorCode        -> error codes.
    //------------------------------------------------------------
    ErrorCode setModuleValue(uint32_t physics_id, uint8_t port_value);
    
    //------------------------------------------------------------
    // Function:    getThreadError
    // Summary: get the error status of the io thread.
    // In:      None.
    // Out:     None.
    // Return:  ErrorCode -> error codes.
    //------------------------------------------------------------
    ErrorCode getIOError(void);

    // the faulty tolerance times.
    static const int FAULT_TOL = 20;

private:

    IOManager();

    int cycle_time_;
    bool is_virtual_;

    // the total number of device.
    uint8_t max_dev_num_;
    uint32_t max_DI_num_;
    uint32_t max_DO_num_;
    uint32_t max_RI_num_;
    uint32_t max_RO_num_;
    uint8_t virtual_board_address_;
    uint32_t virtual_DI_number_;
    uint32_t virtual_DO_number_;

    std::atomic<ErrorCode> error_;

    ErrorCode thread_error_;

    // the status of the io thread.
    ThreadStatus thread_status_;

    // The buffer to store all the reading io data. Lock it when used.
    std::vector<IODeviceUnit> vector_dev_;

    // mutex lock
    boost::mutex mutex_;

    // the thread object.
    boost::thread io_thread_;

    fst_log::Logger* log_ptr_;

    fst_hal::FstIoDeviceParam* param_ptr_;

    //------------------------------------------------------------
    // Function:    waitThreadFirstData
    // Summary: wait the thread startup, and get the first data.
    // In:      None.
    // Out:     None.
    // Return:  false -> overtime
    //          true  -> startup success.
    //------------------------------------------------------------
    bool waitThreadFirstData(void);

    //------------------------------------------------------------
    // Function:    searchParamId
    // Summary: find the index of table according to the parameter id.
    // In:      id  -> the parameter id according to the parameter path.
    //          io  -> the table to store all the io data.
    // Out:     None.
    // Return:  int -> the index of the table.
    //          -1  -> can't find.
    //------------------------------------------------------------
    int searchParamId(unsigned int id, std::vector<IODeviceUnit> &io);

    //------------------------------------------------------------
    // Function:    startThread
    // Summary: start a thread.
    // In:      None.
    // Out:     None.
    // Return:  None.
    //------------------------------------------------------------
    void startThread(void);

    //------------------------------------------------------------
    // Function:    runThread
    // Summary: main function of io thread.
    // In:      None.
    // Out:     None.
    // Return:  None.
    //------------------------------------------------------------
    void runThread(void);

    void updateThreadFunc(void);

    //------------------------------------------------------------
    // Function:    initDeviceData
    // Summary: read io and load configuration file when the thread is starting up.
    // In:      None.
    // Out:     None.
    // Return:  None.
    //------------------------------------------------------------
    void initDevicesData(void);

    //------------------------------------------------------------
    // Function:    updateDeviceData
    // Summary: refreshen the io data.
    // In:      None.
    // Out:     None.
    // Return:  ErrorCode -> error codes.
    //------------------------------------------------------------
    ErrorCode updateDevicesData(void);

    //------------------------------------------------------------
    // Function:    setThreadError
    // Summary: set the thread_error_.
    // In:      None.
    // Out:     None.
    // Return:  None.
    //------------------------------------------------------------
    void setThreadError(void);

    //------------------------------------------------------------
    // Function:    initIODeviceUnitVar
    // Summary: initialize the variable of IODeviceUnit structure.
    // In:      None.
    // Out:     io -> variable.
    // Return:  None.
    //------------------------------------------------------------
    void initIODeviceUnit(IODeviceUnit &io);

    //------------------------------------------------------------
    // Function:    initIODeviceData
    // Summary: initialize the variable of IODeviceData structure.
    // In:      None.
    // Out:     io -> variable.
    // Return:  None.
    //------------------------------------------------------------
    void initIODeviceData(IODeviceData &data);

    //------------------------------------------------------------
    // Function:    getDeviceDataFromMem
    // Summary: get io data by RS485 and check data.
    // In:      None.
    // Out:     data -> io data from RS485.
    // Return:  ErrorCode  -> error codes.
    //------------------------------------------------------------
    ErrorCode getDeviceDataFromMem(IODeviceData &data);

    //------------------------------------------------------------
    // Function:    readWriteMem
    // Summary: interact with FPGA to get data.
    // In:      None.
    // Out:     data  -> io data from RS485.
    // Return:  true  -> get data successfully.
    //          false -> get data failed.
    //------------------------------------------------------------
    bool readWriteMem(IODeviceData &data);

    bool addVirtualBoard(void);
    ErrorCode virtualGetModuleValue(uint32_t physics_id, uint8_t &port_value, int index, std::vector<IODeviceUnit> &io);
    ErrorCode virtualSetModuleValue(uint32_t physics_id, uint8_t port_value, int index, std::vector<IODeviceUnit> &io);

};
} //namespace fst_io_manager

#endif //IO_MANAGER_IO_MANAGER_H_
