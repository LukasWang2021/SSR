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
#include <mutex>
#include "common_log.h"
#include "io_manager_param.h"
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
    uint64_t number;
    struct{
        uint32_t port:32;      //port offset
        uint8_t port_type:8; //such as DI DO RI RO
        uint8_t address:8;   //physical id switches
        uint8_t dev_type:8;  //such as io_board, modbus
        uint8_t reserve:8;
    }info;
}PhysicsID;

typedef struct{
    PhysicsID id;
    int time;               //the unit is ms.
}IOPulseInfo;


class IoManager
{
public:
  
    IoManager();

    ~IoManager();

    ErrorCode init(fst_hal::DeviceManager* device_manager_ptr);

    //------------------------------------------------------------
    // Function:  getIoBoardVersion
    // Summary: get the version of io board.
    // In:      None
    // Out:     None
    // Return:  map<int, int> -> <address, version>
    //------------------------------------------------------------
    std::map<int, int> getIoBoardVersion(void);

    //------------------------------------------------------------
    // Function:  routineThreadFunc
    // Summary: thread to update the data of io board.
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void routineThreadFunc();

    //------------------------------------------------------------
    // Function:  isRunning
    // Summary: to indicate the thread to run or not.
    // In:      None
    // Out:     None
    // Return:  true -> enable thread to run
    //          false -> stop thread running
    //------------------------------------------------------------
    bool isRunning();

    //------------------------------------------------------------
    // Function:  getBitValue
    // Summary: get the value.
    // In:      phy_id -> internal id.
    // Out:     value -> high or low (1 or 0)
    // Return:  ErrorCode
    //------------------------------------------------------------
    ErrorCode getBitValue(PhysicsID phy_id, uint8_t &value);

    //------------------------------------------------------------
    // Function:  setBitValue
    // Summary: set the value.
    // In:      phy_id -> internal id.
    //          value -> high or low (1 or 0)
    // Out:     None
    // Return:  ErrorCode
    //------------------------------------------------------------
    ErrorCode setBitValue(PhysicsID phy_id, uint8_t value);

    //------------------------------------------------------------
    // Function:  setBitPulse
    // Summary: set the value.
    // In:      phy_id -> internal id.
    //          time -> unit is second.
    // Out:     None
    // Return:  ErrorCode
    //------------------------------------------------------------
    ErrorCode setBitPulse(PhysicsID phy_id, double time);

    //------------------------------------------------------------
    // Function:  getIODeviceInfoList
    // Summary: get all io devices info, including io_board and modbus.
    // In:      None
    // Out:     None
    // Return:  vector
    //------------------------------------------------------------
    std::vector<fst_hal::IODeviceInfo> getIODeviceInfoList(void); 
       
    //------------------------------------------------------------
    // Function:  getIODeviceInfo
    // Summary: get io_board info according to given address.
    // In:      address ->address in io board.
    // Out:     info
    // Return:  ErrorCode
    //------------------------------------------------------------
    ErrorCode getIODeviceInfo(uint8_t address, fst_hal::IODeviceInfo &info);

    //------------------------------------------------------------
    // Function:  getModbusDeviceInfo
    // Summary: get modbus info.
    // In:      *modbus_manager
    // Out:     info
    // Return:  ErrorCode
    //------------------------------------------------------------
    ErrorCode getModbusDeviceInfo( fst_hal::IODeviceInfo &info, ModbusManager* modbus_manager);

    //------------------------------------------------------------
    // Function:  getIODeviceInfo
    // Summary: get all the value of io_board for publishing.
    // In:      address ->address in io board.
    // Out:     values
    // Return:  ErrorCode
    //------------------------------------------------------------
    ErrorCode getDevicePortValues(uint8_t address, fst_hal::IODevicePortValues &values);

private:

    //------------------------------------------------------------
    // Function:  get**Value
    // Summary: get the value.
    // In:      phy_id -> internal id.
    // Out:     value -> high or low (1 or 0)
    // Return:  ErrorCode
    //------------------------------------------------------------
    ErrorCode getDiValue(PhysicsID phy_id, uint8_t &value);
    ErrorCode getDoValue(PhysicsID phy_id, uint8_t &value);
    ErrorCode getRiValue(PhysicsID phy_id, uint8_t &value);
    ErrorCode getRoValue(PhysicsID phy_id, uint8_t &value);
    ErrorCode getUiValue(PhysicsID phy_id, uint8_t &value);
    ErrorCode getUoValue(PhysicsID phy_id, uint8_t &value);

    //------------------------------------------------------------
    // Function:  set**Value
    // Summary: set the value.
    // In:      phy_id -> internal id.
    //          value -> high or low (1 or 0)
    // Out:     None
    // Return:  ErrorCode
    //------------------------------------------------------------
    ErrorCode setDiValue(PhysicsID phy_id, uint8_t value);
    ErrorCode setDoValue(PhysicsID phy_id, uint8_t value);
    ErrorCode setRiValue(PhysicsID phy_id, uint8_t value);
    ErrorCode setRoValue(PhysicsID phy_id, uint8_t value);
    ErrorCode setUiValue(PhysicsID phy_id, uint8_t value);
    ErrorCode setUoValue(PhysicsID phy_id, uint8_t value);

    ErrorCode setDoPulse(PhysicsID phy_id, double time);
    ErrorCode setRoPulse(PhysicsID phy_id, double time);

    BaseDevice* getDevicePtr(PhysicsID phy_id);
    ErrorCode updateIoDevicesData(void);
    void handlePulse(void);

    ErrorCode getDiValueFromModbusServer(uint32_t port, uint8_t &value, ModbusManager* modbus_manager);
    ErrorCode getDoValueFromModbusServer(uint32_t port, uint8_t &value, ModbusManager* modbus_manager);
    ErrorCode setDoValueToModbusServer(uint32_t port, uint8_t &value, ModbusManager* modbus_manager);

    ErrorCode getUiValueFromModbusServer(uint32_t port, uint8_t &value, ModbusManager* modbus_manager);
    ErrorCode getUoValueFromModbusServer(uint32_t port, uint8_t &value, ModbusManager* modbus_manager);
    ErrorCode setUoValueToModbusServer(uint32_t port, uint8_t &value, ModbusManager* modbus_manager);


    int cycle_time_;
    bool is_running_;
    std::vector<IOPulseInfo> pulse_vec_;
    std::mutex pulse_mutex_;//to lock pulse_vec_

    IoManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    fst_hal::DeviceManager* device_manager_ptr_;
    std::vector<fst_hal::DeviceInfo> device_list_;
    fst_base::ThreadHelp routine_thread_;

};
} //namespace

void ioManagerRoutineThreadFunc(void* arg);

#endif //IO_MANAGER_H_
