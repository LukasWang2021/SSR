#ifndef _MODBUS_TCP_MAP_HPP
#define _MODBUS_TCP_MAP_HPP

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <mutex>
#include <map>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include "common_log.h"
#include "parameter_manager/parameter_manager_param_group.h"

#include "modbus_manager_param.h"
#include "tcp_client.h"

using namespace std;

namespace fst_modbus
{
enum ModbusRegType{ DEFAULT = 0, COIL = 1, DISCREPTE_INPUT = 2, HOLDING_REGISTER = 3, INPUT_REGISTER = 4,};

typedef struct
{
    ModbusRegType reg_type;
    int addr;
}ModbusRegInfo;

typedef enum
{
    DEVICE_TYPE_INVALID = 0,
    DEVICE_TYPE_FST_AXIS = 1,
    DEVICE_TYPE_FST_IO = 2,
    DEVICE_TYPE_FST_SAFETY = 3,
    DEVICE_TYPE_FST_ANYBUS = 4,
    DEVICE_TYPE_VIRTUAL_AXIS = 5,
    DEVICE_TYPE_VIRTUAL_IO = 6,
    DEVICE_TYPE_VIRTUAL_SAFETY = 7,
    DEVICE_TYPE_NORMAL = 8,
}DeviceType;

typedef struct
{
    int index;
    int address;
    DeviceType type;
    bool is_valid;
}DeviceInfo;

typedef struct
{
    DeviceInfo device_info;
    ModbusRegInfo modbus_reg_info;
}ModbusMapElement;

class ModbusMap
{
public:
    ModbusMap();
     ~ModbusMap();
    void init();
    void addElement(int id, ModbusMapElement& element);
    ErrorCode deleteElement(int id);
    ErrorCode updateElement(int id, ModbusMapElement& element);
    ErrorCode getElementInfoById(int id, ModbusMapElement& element);
    ErrorCode getDeviceInfoById(int id, DeviceInfo& info);
    ErrorCode getModbusRegInfoById(int id, ModbusRegInfo& info);

private:
    ModbusManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;

    std::mutex modbus_map_mutex_;
    std::map<int, ModbusMapElement> modbus_map_;
};
}

#endif
