/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       fst_io_device.h
Author:     Feng.Wu
Create:     14-Oct-2018
Modify:     25-Oct-2018
Summary:    dealing with IO module
**********************************************/

#ifndef FST_IO_DEVICE_H
#define FST_IO_DEVICE_H

#include <vector>
#include <mutex>
#include <string>
#include <map>

#include "base_device.h"
#include "common_log.h"
#include "error_code.h"
#include "fst_io_mem.h"
#include "fst_io_device_param.h"

namespace fst_hal
{

// This is output info.
struct IODeviceInfo
{
    uint32_t address;  
    uint8_t dev_type;  //DEVICE_TYPE_FST_IO
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
    uint8_t DI[4];
    uint8_t DO[4];
    uint8_t RI[1];
    uint8_t RO[1];
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
	
class FstIoDevice: public BaseDevice
{
public:
    FstIoDevice(int address);
    ~FstIoDevice();

    virtual bool init();

    void getIoBoardVersion(int &version);

    IODeviceInfo getDeviceInfo(void);
    IODevicePortValues getDeviceValues(void);

    ErrorCode getDiValue(uint32_t port_offset, uint8_t &value);
    ErrorCode getDoValue(uint32_t port_offset, uint8_t &value);
    ErrorCode getRiValue(uint32_t port_offset, uint8_t &value);
    ErrorCode getRoValue(uint32_t port_offset, uint8_t &value);
    ErrorCode getUiValue(uint32_t port_offset, uint8_t &value);
    ErrorCode getUoValue(uint32_t port_offset, uint8_t &value);

    ErrorCode setDoValue(uint32_t port_offset, uint8_t value);
    ErrorCode setRoValue(uint32_t port_offset, uint8_t value);
    ErrorCode setUoValue(uint32_t port_offset, uint8_t value);

    ErrorCode updateDeviceData(void);
    
    // only open sharemem onece.
    static bool is_mem_init_;
    
    static int io_dev_count_;

private:
    FstIoDeviceParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    std::mutex data_mutex_;
    int address_;
    bool is_virtual_;
    int comm_tolerance_;
    IODeviceInfo dev_info_;
    IODevicePortValues dev_values_;
    uint8_t output_[IO_DATAFRAME_MAX];
    std::map<int, int> addr_map_;
    ErrorCode error_code_;
    ErrorCode pre_code_;

    FstIoDevice();

    void initIODeviceData(IODeviceData &data);

    ErrorCode freshenDeviceDataFromMem(IODeviceData &data);

    bool readWriteMem(IODeviceData &data);

};

}

#endif

