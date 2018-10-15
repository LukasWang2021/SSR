#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "error_monitor.h"
#include "error_code.h"
#include "common_log.h"

#include "modbus_manager.h"

using namespace std;
using namespace fst_modbus;

ModbusManager* ModbusManager::instance_ = NULL;

ModbusManager::ModbusManager():
    log_ptr_(NULL), param_ptr_(NULL),
    server_ip_(""), server_port_(-1)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ModbusManagerParam();
    FST_LOG_INIT("ModbusManager");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
}

ModbusManager::~ModbusManager()
{
    if (client_ != NULL)
    {
        delete client_;
        client_ = NULL;
    }
    if (server_ != NULL)
    {
        delete server_;
        server_ = NULL;
    }
    if (map_ != NULL)
    {
        delete map_;
        map_ = NULL;
    }
    if (log_ptr_ != NULL)
    {
        delete log_ptr_;
        log_ptr_ = NULL;
    }
    if (param_ptr_ != NULL)
    {
        delete param_ptr_;
        param_ptr_ = NULL;
    }
}

ModbusManager* ModbusManager::getInstance()
{
    if(instance_ == NULL)
    {
        instance_ = new ModbusManager();
    }
    return instance_;
}

ErrorCode ModbusManager::init()
{
    if(!param_ptr_->loadParam())
    {
        return MODBUS_MANAGER_INIT_FAILED;
    }

    server_ip_ = param_ptr_->server_ip_;
    server_port_ = param_ptr_->server_port_;

    return SUCCESS;
}

ErrorCode ModbusManager::initClient()
{
    client_ = new ModbusTCPClient(server_ip_, server_port_);
    return client_->init();
}

void ModbusManager::initMap()
{
    map_ = new ModbusMap();
    map_->init();
}

ErrorCode ModbusManager::initServer()
{
    server_ = new ModbusTCPServer(server_port_);
    return server_->init();
}

ErrorCode ModbusManager::setValueById(int id, void* value)
{
    ModbusRegInfo modbus_reg_info;
    ErrorCode error_code = map_->getModbusRegInfoById(id, modbus_reg_info);

    if(SUCCESS != error_code)
    {
        return error_code;
    }

    switch(modbus_reg_info.reg_type)
    {
        case COIL:
        {
            ModbusStatus status;
            status.nb = 1;
            status.dest = (uint8_t*)value;
            status.addr = modbus_reg_info.addr;
            error_code = client_->writeCoils(status);
         }
            break;
        case HOLDING_REGISTER:
        {
            ModbusRegs regs;
            regs.nb = 1;
            regs.dest = (uint16_t*)value;
            regs.addr = modbus_reg_info.addr;
            error_code = client_->writeHoldingRegs(regs);
        }
            break;
        default:
            error_code =  MODBUS_REG_TYPE_ERROR;
    };

    return error_code;
}

ErrorCode ModbusManager::getValueById(int id, void* value)
{
    ModbusRegInfo modbus_reg_info;
    ErrorCode error_code = map_->getModbusRegInfoById(id, modbus_reg_info);
    if(SUCCESS != error_code)
    {
        return error_code; 
    }

    switch(modbus_reg_info.reg_type)
    {
        case COIL:
        {
            ModbusStatus status;
            status.addr = modbus_reg_info.addr;
            FST_ERROR("Read coil : addr = %d", status.addr);
            status.nb = 1;
            status.dest = (uint8_t*)value;
            error_code = client_->readCoils(status);
        }
            break;
        case DISCREPTE_INPUT:
        {
            ModbusStatus status;
            status.addr = modbus_reg_info.addr;
            status.nb = 1;
            status.dest = (uint8_t*)value;
            error_code = client_->readDiscreteInputs(status);
        }
            break;
        case HOLDING_REGISTER:
        {
            ModbusRegs regs;
            regs.addr = modbus_reg_info.addr;
            regs.nb = 1;
            regs.dest = (uint16_t*)value;
            error_code = client_->readHoldingRegs(regs);
        }
            break;
        case INPUT_REGISTER:
        {
            ModbusRegs regs;
            regs.addr = modbus_reg_info.addr;
            regs.nb = 1;
            regs.dest = (uint16_t*)value;
            error_code = client_->readInputRegs(regs);
        }
            break;
        default:
            error_code = MODBUS_REG_TYPE_ERROR;
    };

    return error_code;
}