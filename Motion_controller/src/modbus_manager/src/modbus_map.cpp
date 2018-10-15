#include "modbus_map.h"
#include "tcp_client.h"
#include "error_monitor.h"
#include "error_code.h"
#include "serverAlarmApi.h"

using namespace std;
using namespace fst_modbus;

ModbusMap::ModbusMap():
    log_ptr_(NULL), param_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ModbusManagerParam();
    FST_LOG_INIT("ModbusMap");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
}

ModbusMap::~ModbusMap()
{}

void ModbusMap::init()
{
    modbus_map_mutex_.lock();
    modbus_map_.clear();
    modbus_map_mutex_.unlock();
}

void ModbusMap::addElement(int id, ModbusMapElement& element)
{
    modbus_map_mutex_.lock();
    modbus_map_.insert(pair<int, ModbusMapElement>(id, element));
    modbus_map_mutex_.unlock();
}

ErrorCode ModbusMap::deleteElement(int id)
{
    map<int, ModbusMapElement>::iterator iter;
    modbus_map_mutex_.lock();

    iter = modbus_map_.find(id);
    if(iter == modbus_map_.end())
    {
        modbus_map_mutex_.unlock();
        return MODBUS_MAP_ID_ERROR;
    }

    modbus_map_.erase(iter);
    modbus_map_mutex_.unlock();
    return SUCCESS;
}

ErrorCode ModbusMap::updateElement(int id, ModbusMapElement& element)
{
    map<int, ModbusMapElement>::iterator iter;
    modbus_map_mutex_.lock();

    iter = modbus_map_.find(id);
    if(iter == modbus_map_.end())
    {
        modbus_map_mutex_.unlock();
        return MODBUS_MAP_ID_ERROR;
    }

    iter->second.device_info.index = element.device_info.index;
    iter->second.device_info.address = element.device_info.address;
    iter->second.device_info.type = element.device_info.type;
    iter->second.device_info.is_valid = element.device_info.is_valid;
    iter->second.modbus_reg_info.reg_type = element.modbus_reg_info.reg_type;
    iter->second.modbus_reg_info.addr = element.modbus_reg_info.addr;

    modbus_map_mutex_.unlock();
    return SUCCESS;
}

ErrorCode ModbusMap::getElementInfoById(int id, ModbusMapElement& element)
{
    map<int, ModbusMapElement>::iterator iter;
    modbus_map_mutex_.lock();

    iter = modbus_map_.find(id);
    if(iter == modbus_map_.end())
    {
        modbus_map_mutex_.unlock();
        return MODBUS_MAP_ID_ERROR;
    }

    element.device_info.index = iter->second.device_info.index;
    element.device_info.address = iter->second.device_info.address;
    element.device_info.type = iter->second.device_info.type;
    element.device_info.is_valid = iter->second.device_info.is_valid;
    element.modbus_reg_info.reg_type = iter->second.modbus_reg_info.reg_type;
    element.modbus_reg_info.addr = iter->second.modbus_reg_info.addr;

    modbus_map_mutex_.unlock();
    return SUCCESS;
}

ErrorCode ModbusMap::getDeviceInfoById(int id, DeviceInfo& info)
{
    map<int, ModbusMapElement>::iterator iter;
    modbus_map_mutex_.lock();

    iter = modbus_map_.find(id);
    if(iter == modbus_map_.end())
    {
        modbus_map_mutex_.unlock();
        return MODBUS_MAP_ID_ERROR;
    }

    info.index = iter->second.device_info.index;
    info.address = iter->second.device_info.address;
    info.type = iter->second.device_info.type;
    info.is_valid = iter->second.device_info.is_valid;

    modbus_map_mutex_.unlock();
    return SUCCESS;
}

ErrorCode ModbusMap::getModbusRegInfoById(int id, ModbusRegInfo& info)
{
    map<int, ModbusMapElement>::iterator iter;
    modbus_map_mutex_.lock();

    iter = modbus_map_.find(id);
    if(iter == modbus_map_.end())
    {
        modbus_map_mutex_.unlock();
        return MODBUS_MAP_ID_ERROR;
    }

    info.reg_type = iter->second.modbus_reg_info.reg_type;
    info.addr = iter->second.modbus_reg_info.addr;

    modbus_map_mutex_.unlock();
    return SUCCESS;
}
