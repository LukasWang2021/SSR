#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "error_monitor.h"
#include "error_code.h"
#include "common_log.h"
#include "net_connection.h"

#include "modbus_manager.h"

using namespace std;
using namespace fst_hal;

ErrorCode ModbusManager::openServer()
{
    if (start_mode_ != MODBUS_SERVER)
       return MODBUS_START_MODE_ERROR;

    if (server_ != NULL && server_->isRunning())
    {
        return SUCCESS;
    }

    ErrorCode error_code = server_->openServer();
    if (error_code != SUCCESS)
    {
        server_->closeServer();
        return error_code;
    }

    return SUCCESS;
}

ErrorCode ModbusManager::closeServer()
{
    if (start_mode_ != MODBUS_SERVER)
       return MODBUS_START_MODE_ERROR;

    if (server_ != NULL && server_->isRunning())
    {
        server_->closeServer();
    }

    return SUCCESS;
}

bool ModbusManager::isServerRunning()
{
    return server_->isRunning();
}

ErrorCode ModbusManager::setServerEnableStatus(bool &status)
{
    if (start_mode_ != MODBUS_SERVER)
       return MODBUS_START_MODE_ERROR;

    return server_->setEnableStatus(status);
}

ErrorCode ModbusManager::getServerEnableStatus(bool &status)
{
    if (start_mode_ != MODBUS_SERVER) 
       return MODBUS_START_MODE_ERROR;

    status = server_->getEnableStatus();
    return SUCCESS;
}

ErrorCode ModbusManager::setServerStartInfo(ModbusServerStartInfo &start_info)
{
    if (start_mode_ != MODBUS_SERVER) 
       return MODBUS_START_MODE_ERROR;

    return server_->setStartInfo(start_info);
}

ErrorCode ModbusManager::getServerStartInfo(ModbusServerStartInfo &start_info)
{
    if (start_mode_ != MODBUS_SERVER) 
       return MODBUS_START_MODE_ERROR;

    start_info =  server_->getStartInfo();
    return SUCCESS;
}

ErrorCode ModbusManager::setServerRegInfo(ModbusServerRegInfo &config_reg_info)
{
    if (start_mode_ != MODBUS_SERVER) 
       return MODBUS_START_MODE_ERROR;

    return server_->setRegInfo(config_reg_info);
}

ErrorCode ModbusManager::getServerRegInfo(ModbusServerRegInfo &config_reg_info)
{
    if (start_mode_ != MODBUS_SERVER) 
       return MODBUS_START_MODE_ERROR;

    config_reg_info = server_->getRegInfo();
    return SUCCESS;
}

ErrorCode ModbusManager::getServerConfigCoilInfo(ModbusRegAddrInfo &info)
{
    if (start_mode_ != MODBUS_SERVER)
       return MODBUS_START_MODE_ERROR;

    info = server_->getConfigCoilInfo();
    return SUCCESS;
}

ErrorCode ModbusManager::getServerConfigDiscrepteInputInfo(ModbusRegAddrInfo &info)
{
    if (start_mode_ != MODBUS_SERVER)  
       return MODBUS_START_MODE_ERROR;

    info = server_->getConfigDiscrepteInputInfo();
    return SUCCESS;
}

ErrorCode ModbusManager::getServerConfigHoldingRegInfo(ModbusRegAddrInfo &info)
{
    if (start_mode_ != MODBUS_SERVER) 
       return MODBUS_START_MODE_ERROR;

    info = server_->getConfigHoldingRegInfo();
    return SUCCESS;
}

ErrorCode ModbusManager::getServerConfigParams(ModbusServerConfigParams &params)
{
    if (start_mode_ != MODBUS_SERVER) 
       return MODBUS_START_MODE_ERROR;

    params = server_->getConfigParams();
    return SUCCESS;
}

ErrorCode ModbusManager::getServerConfigInputRegInfo(ModbusRegAddrInfo &info)
{
    if (start_mode_ != MODBUS_SERVER)  
       return MODBUS_START_MODE_ERROR;

    info = server_->getConfigInputRegInfo();
    return SUCCESS;
}

ErrorCode ModbusManager::writeCoilsToServer(int addr, int nb, uint8_t *dest)
{
    if (start_mode_ != MODBUS_SERVER)
       return MODBUS_START_MODE_ERROR;

    if (!server_->isRunning())
        return MODBUS_SERVER_BE_NOT_OPENED;

    return server_->writeCoils(addr, nb, dest);
}

ErrorCode ModbusManager::readCoilsFromServer(int addr, int nb, uint8_t *dest)
{
    if (start_mode_ != MODBUS_SERVER) 
       return MODBUS_START_MODE_ERROR;

    if (!server_->isRunning())
        return MODBUS_SERVER_BE_NOT_OPENED;

    return server_->readCoils(addr, nb, dest);
}

ErrorCode ModbusManager::readDiscreteInputsFromServer(int addr, int nb, uint8_t *dest)
{
    if (start_mode_ != MODBUS_SERVER)  
        return MODBUS_START_MODE_ERROR;

    if (!server_->isRunning())
        return MODBUS_SERVER_BE_NOT_OPENED;

    return server_->readDiscreteInputs(addr, nb, dest);
}

ErrorCode ModbusManager::writeHoldingRegsToServer(int addr, int nb, uint16_t *dest)
{
    if (start_mode_ != MODBUS_SERVER) 
       return MODBUS_START_MODE_ERROR;

    if (!server_->isRunning())
        return MODBUS_SERVER_BE_NOT_OPENED;

    return server_->writeHoldingRegs(addr, nb, dest);
}

ErrorCode ModbusManager::readHoldingRegsFromServer(int addr, int nb, uint16_t *dest)
{
    if (start_mode_ != MODBUS_SERVER) 
       return MODBUS_START_MODE_ERROR;

    if (!server_->isRunning())
        return MODBUS_SERVER_BE_NOT_OPENED;

    return server_->readHoldingRegs(addr, nb, dest);
}

ErrorCode ModbusManager::readInputRegsFromServer(int addr, int nb, uint16_t *dest)
{
    if (start_mode_ != MODBUS_SERVER)
       return MODBUS_START_MODE_ERROR;

    if (!server_->isRunning())
        return MODBUS_SERVER_BE_NOT_OPENED;

    return server_->readInputRegs(addr, nb, dest);
}

ErrorCode ModbusManager::writeInputRegsToServer(int addr, int nb, uint16_t *dest)
{
    if (start_mode_ != MODBUS_SERVER) 
       return MODBUS_START_MODE_ERROR;

    if (!server_->isRunning())
        return MODBUS_SERVER_BE_NOT_OPENED;

    return server_->writeInputRegs(addr, nb, dest);
}