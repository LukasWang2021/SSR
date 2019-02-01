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
  
    IoManager();

    ~IoManager();

    ErrorCode init(fst_hal::DeviceManager* device_manager_ptr);

    std::map<int, int> getIoBoardVersion(void);

    // thread to fresh data
    void ioManagerThreadFunc();
    bool isRunning();

    ErrorCode getBitValue(PhysicsID phy_id, uint8_t &value);
    ErrorCode setBitValue(PhysicsID phy_id, uint8_t value);

    //get all io devices info, including io_board and modbus.
    std::vector<fst_hal::IODeviceInfo> getIODeviceInfoList(void); 
    
    //get io_board info
    ErrorCode getIODeviceInfo(uint8_t address, fst_hal::IODeviceInfo &info);

    // get modbus info
    ErrorCode getModbusDeviceInfo( fst_hal::IODeviceInfo &info, ModbusManager* modbus_manager);

    //get io_board values
    ErrorCode getDevicePortValues(uint8_t address, fst_hal::IODevicePortValues &values);

private:

    ErrorCode getDiValue(PhysicsID phy_id, uint8_t &value);
    ErrorCode getDoValue(PhysicsID phy_id, uint8_t &value);
    ErrorCode getRiValue(PhysicsID phy_id, uint8_t &value);
    ErrorCode getRoValue(PhysicsID phy_id, uint8_t &value);
    ErrorCode getUiValue(PhysicsID phy_id, uint8_t &value);
    ErrorCode getUoValue(PhysicsID phy_id, uint8_t &value);

    ErrorCode setDiValue(PhysicsID phy_id, uint8_t value);
    ErrorCode setDoValue(PhysicsID phy_id, uint8_t value);
    ErrorCode setRiValue(PhysicsID phy_id, uint8_t value);
    ErrorCode setRoValue(PhysicsID phy_id, uint8_t value);
    ErrorCode setUiValue(PhysicsID phy_id, uint8_t value);
    ErrorCode setUoValue(PhysicsID phy_id, uint8_t value);

    BaseDevice* getDevicePtr(PhysicsID phy_id);
    ErrorCode updateIoDevicesData(void);

    ErrorCode getDiValueFromModbusServer(uint8_t port, uint8_t &value, ModbusManager* modbus_manager);
    ErrorCode getDoValueFromModbusServer(uint8_t port, uint8_t &value, ModbusManager* modbus_manager);
    ErrorCode setDoValueToModbusServer(uint8_t port, uint8_t &value, ModbusManager* modbus_manager);

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
