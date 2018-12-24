/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       io_manager.h
Author:     Feng.Wu 
Create:     14-Dec-2018
Modify:     14-Dec-2018
Summary:    dealing with IO module
**********************************************/

#ifndef IO_MANAGER_H_
#define IO_MANAGER_H_

#include <vector>
//#include <string>
//#include <map>
//#include <boost/thread/thread.hpp>
//#include <boost/thread/mutex.hpp>
//#include <parameter_manager/parameter_manager_param_group.h>


#include "common_log.h"
#include "io_manager_param.h"
#include "base_datatype.h"
#include "base_device.h"
#include "error_code.h"
#include "device_manager.h"

#include "fst_io_device.h"
#include "virtual_io_device.h"
#include "thread_help.h"
#include "modbus_manager.h"


namespace fst_hal
{

typedef union PhysicsID
{
    uint32_t number;
    struct{
        uint8_t port:8;
        uint8_t port_type:8;
        uint8_t address:8;
        uint8_t dev_type:8;
    }info;
}PhysicsID;


class IoManager
{
public:
  
    IoManager(fst_hal::DeviceManager* device_manager);

    ~IoManager();

    static IoManager *getInstance(fst_hal::DeviceManager* device_manager);

    ErrorCode init(void);

    // thread to fresh data
    void ioManagerThreadFunc();
    bool isRunning();

    ErrorCode getBitValue(PhysicsID phy_id, uint8_t &value);
    ErrorCode setBitValue(PhysicsID phy_id, uint8_t value);

    std::vector<fst_hal::IODeviceInfo> getIODeviceInfoList(void); //get all io devices info, including io_board and modbus.

    ErrorCode getIODeviceInfo(uint8_t address, fst_hal::IODeviceInfo &info);//get io_board info 

    ErrorCode getDevicePortValues(uint8_t address, fst_hal::IODevicePortValues &values);// get io_board values

private:
    IoManager();

    ErrorCode getDiValue(PhysicsID phy_id, uint8_t &value);
    ErrorCode getDoValue(PhysicsID phy_id, uint8_t &value);
    ErrorCode getRiValue(PhysicsID phy_id, uint8_t &value);
    ErrorCode getRoValue(PhysicsID phy_id, uint8_t &value);

    ErrorCode setDiValue(PhysicsID phy_id, uint8_t value);
    ErrorCode setDoValue(PhysicsID phy_id, uint8_t value);
    ErrorCode setRiValue(PhysicsID phy_id, uint8_t value);
    ErrorCode setRoValue(PhysicsID phy_id, uint8_t value);

    BaseDevice* getDevicePtr(PhysicsID phy_id);
    ErrorCode updateIoDevicesData(void);
    
    int cycle_time_;
    bool is_running_;

    IoManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    fst_hal::DeviceManager* device_manager_ptr_;
    std::vector<fst_hal::DeviceInfo> device_list_;
    fst_base::ThreadHelp thread_ptr_;

};
} //namespace

void ioManagerRoutineThreadFunc(void* arg);

#endif //IO_MANAGER_H_
