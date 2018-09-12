#ifndef MODBUS_MANAGER
#define MODBUS_MANAGER

#include <string>
#include <mutex>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include "common_log.h"
#include "parameter_manager/parameter_manager_param_group.h"

#include "tcp_client.h"
#include "tcp_server.h"
#include "modbus_manager_param.h"

using namespace std;
using namespace fst_modbus;

namespace fst_modbus
{
enum StartMode{ START_MODE_CLIENT = 0, START_MODE_SERVER = 1,};
enum DeviceType{ IO = 1,};
enum DeviceNum{ IONum = 1, };
enum PortType{ DO = 1, DI = 2, AO = 3, AI = 4 };
enum RegisterType{ DEFAULT = 0, COIL = 1, DISCREPTE_INPUT = 2, 
    HOLDING_REGISTER = 3, INPUT_REGISTER = 4,};
enum RegisterOneOpNum{ STATUS_ONE_OP_NUM = 1968, REGISTER_ONE_OP_NUM = 121, };

typedef struct
{
    DeviceType device_type;
    int device_index;
    int device_addr;
}DeviceInfo;

typedef struct 
{
    DeviceInfo device_info;
    PortType port_type;
    RegisterType modbus_reg_type;
    int modbus_reg_addr;
}DeviceModbusInfo;

class ModbusManager
{
public:
    ModbusManager();
    ~ModbusManager();

    ErrorCode init(int mode, bool is_fake = true, bool is_debug = true);
    int getDeviceNum() { return device_number_; }
    string getConfigIP() { return config_ip_; }
    int getConfigPort() { return config_port_; }
    int getStartMode() { return start_mode_; }

    void clearModbusMap();
    bool pushToModbusMap(DeviceModbusInfo &element);
    DeviceModbusInfo eraseFromModbusMap(DeviceInfo &device_info);
    int getRegStartAddrFromModbusMap(DeviceInfo &device_info); // search from mapping
    RegisterType getRegTypeFromModbusMap(DeviceInfo &device_info); // search from mapping
    ErrorCode getRegInfoOfDeviceFromModbusMap(DeviceType device_type, int &num); // search from mapping

    RegisterInfo getModbusRegInfo();
    ErrorCode setByteTimeout(timeval& timeout);
    ErrorCode setResponseTimeout(timeval& timeout);
    ErrorCode setDeviceStatus(DeviceInfo &device_info, int addr_nb, uint8_t* dest);
    ErrorCode getDeviceStatus(DeviceInfo &device_info, int addr_nb, uint8_t* dest);
    ErrorCode setDeviceRegisters(DeviceInfo &device_info, int addr_nb, uint16_t* dest);
    ErrorCode getDeviceRegisters(DeviceInfo &device_info, int addr_nb, uint16_t* dest);

private:
    ModbusManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    fst_modbus::ModbusTCPClient * tcp_client_;
    fst_modbus::ModbusTCPServer * tcp_server_;
    fst_ip::LocalIP local_ip_;
    StartMode start_mode_;

    // component params
    string config_ip_;
    int config_port_;
    int connection_number_;
    int device_number_;
    int cycle_time_;
    bool is_debug_;
    timeval bytes_timeout_;
    timeval response_timeout_;

    std::mutex modbus_map_mutex_;
    std::vector<DeviceModbusInfo> modbus_map_;

    // the thread object.
    boost::thread modbus_thread;
    void startThread(void);
    void runThread(void);

    bool checkModbusRegsNum(DeviceModbusInfo &device_modbus_info);
};
}

#endif


