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
    if (start_mode_ != SERVER) 
       return MODBUS_SERVER_BE_NOT_OPENED;

    ErrorCode error_code = server_->openServer();
    if (error_code != SUCCESS)
    {
        server_->closeServer();
        return MODBUS_SERVER_INIT_FAILED;
    }

    return SUCCESS;
}

ErrorCode ModbusManager::closeServer()
{
    if (start_mode_ != SERVER) 
       return MODBUS_SERVER_BE_NOT_OPENED;

    server_->closeServer();
    return SUCCESS;
}

ErrorCode ModbusManager::setConnectStatusToServer(bool &status)
{
    if (start_mode_ != SERVER)
       return MODBUS_SERVER_BE_NOT_OPENED;

    return server_->setConnectStatus(status);
}

ErrorCode ModbusManager::getConnectStatusFromServer(bool &status)
{
    if (start_mode_ != SERVER) 
       return MODBUS_SERVER_BE_NOT_OPENED;

    status = server_->getConnectStatus();
    printf("server connect status = %d\n", status);
    return SUCCESS;
}

ErrorCode ModbusManager::setConfigToServer(ModbusServerConfig &config)
{
    if (start_mode_ != SERVER) 
       return MODBUS_SERVER_BE_NOT_OPENED;

    return server_->setConfig(config);
}

ErrorCode ModbusManager::getConfigFromServer(ModbusServerConfig &config)
{
    if (start_mode_ != SERVER) 
       return MODBUS_SERVER_BE_NOT_OPENED;

    config =  server_->getConfig();
    return SUCCESS;
}

ErrorCode ModbusManager::getCoilInfoFromServer(ModbusRegAddrInfo &info)
{
    if (start_mode_ != SERVER) 
       return MODBUS_SERVER_BE_NOT_OPENED;

    info = server_->getCoilInfo();
    return SUCCESS;
}

ErrorCode ModbusManager::getDiscrepteInputInfoFromServer(ModbusRegAddrInfo &info)
{
    if (start_mode_ != SERVER)  
       return MODBUS_SERVER_BE_NOT_OPENED;

    info = server_->getDiscrepteInputInfo();
    return SUCCESS;
}

ErrorCode ModbusManager::getHoldingRegInfoFromServer(ModbusRegAddrInfo &info)
{
    if (start_mode_ != SERVER) 
       return MODBUS_SERVER_BE_NOT_OPENED;

    info = server_->getHoldingRegInfo();
    return SUCCESS;
}

ErrorCode ModbusManager::getInputRegInfoFromServer(ModbusRegAddrInfo &info)
{
    if (start_mode_ != SERVER)  
       return MODBUS_SERVER_BE_NOT_OPENED;

    info = server_->getInputRegInfo();
    return SUCCESS;
}

ErrorCode ModbusManager::getStartInfoFromServer(ModbusServerStartInfo &info)
{
    if (start_mode_ != SERVER) 
       return MODBUS_SERVER_BE_NOT_OPENED;

    info = server_->getStartInfo();
    return SUCCESS;
}

ErrorCode ModbusManager::writeCoilsToServer(int addr, int nb, uint8_t *dest)
{
    if (start_mode_ != SERVER || !server_->isRunning())
       return MODBUS_SERVER_BE_NOT_OPENED;

    return server_->writeCoils(addr, nb, dest);
}

ErrorCode ModbusManager::readCoilsFromServer(int addr, int nb, uint8_t *dest)
{
    if (start_mode_ != SERVER || !server_->isRunning()) 
       return MODBUS_SERVER_BE_NOT_OPENED;

    return server_->readCoils(addr, nb, dest);
}

ErrorCode ModbusManager::readDiscreteInputsFromServer(int addr, int nb, uint8_t *dest)
{
    if (start_mode_ != SERVER || !server_->isRunning())  
        return MODBUS_SERVER_BE_NOT_OPENED;

    return server_->readDiscreteInputs(addr, nb, dest);
}

ErrorCode ModbusManager::writeHoldingRegsToServer(int addr, int nb, uint16_t *dest)
{
    if (start_mode_ != SERVER || !server_->isRunning()) 
       return MODBUS_SERVER_BE_NOT_OPENED;

    return server_->writeHoldingRegs(addr, nb, dest);
}

ErrorCode ModbusManager::readHoldingRegsFromServer(int addr, int nb, uint16_t *dest)
{
    if (start_mode_ != SERVER || !server_->isRunning()) 
       return MODBUS_SERVER_BE_NOT_OPENED;

    return server_->readHoldingRegs(addr, nb, dest);
}

ErrorCode ModbusManager::readInputRegsFromServer(int addr, int nb, uint16_t *dest)
{
    if (start_mode_ != SERVER || !server_->isRunning())
       return MODBUS_SERVER_BE_NOT_OPENED;

    return server_->readInputRegs(addr, nb, dest);
}


bool ModbusManager::isServerRunning()
{
    return server_->isRunning();
}


ErrorCode ModbusManager::getServerRegInfoFromServer(ModbusServerRegInfo &info)
{
    if (start_mode_ != SERVER) 
       return MODBUS_SERVER_BE_NOT_OPENED;

    info.coil = server_->getCoilInfo();
    info.discrepte_input = server_->getDiscrepteInputInfo();
    info.holding_reg = server_->getInputRegInfo();
    info.input_reg = server_->getHoldingRegInfo();
    return SUCCESS;
}