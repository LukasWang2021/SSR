#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "error_monitor.h"
#include "error_code.h"
#include "common_log.h"
#include "net_connection.h"

#include "modbus_manager.h"

using namespace std;
using namespace fst_hal;

ErrorCode ModbusManager::addClient(int client_id)
{
    if (start_mode_ != MODBUS_TCP_CLIENT)
       return MODBUS_START_MODE_ERROR;

    if (client_ != NULL)
    {
        return MODBUS_CLIENT_EXISTED;
    }

    client_ = new ModbusTCPClient(param_ptr_->client_file_path_, client_id);
    return client_->initParam();
}

ErrorCode ModbusManager::deleteClient(int client_id)
{    
    if (start_mode_ != MODBUS_TCP_CLIENT)
       return MODBUS_START_MODE_ERROR;

    if (client_ == NULL || client_id != client_->getId())
    {
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if (client_->isRunning())
    {
        return MODBUS_CLIENT_IS_RUNNING;
    }

    delete client_;
    client_ = NULL;

    return SUCCESS;
}

ErrorCode ModbusManager::openClient(int client_id)
{
    if (start_mode_ != MODBUS_TCP_CLIENT)
       return MODBUS_START_MODE_ERROR;

    if (client_ == NULL || client_id != client_->getId())
    {
        return MODBUS_CLIENT_INVALID_ARG;
    }

    bool status = false;
    if (SUCCESS != client_->getConnectStatus(status)
        || !status)
    {
        return MODBUS_CLIENT_CONNECT_FAILED;
    }

    return client_->openClient();
}


ErrorCode ModbusManager::closeClient(int client_id)
{
    if (start_mode_ != MODBUS_TCP_CLIENT)
       return MODBUS_START_MODE_ERROR;

    if (client_ == NULL || client_id != client_->getId())
    {
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if (!client_->isRunning())
    {
        return MODBUS_CLIENT_BE_NOT_OPENED;
    }

    client_->closeClient();
    return SUCCESS;
}

ErrorCode ModbusManager::setConnectStatusToClient(int client_id, bool status)
{
    if (start_mode_ != MODBUS_TCP_CLIENT)
       return MODBUS_START_MODE_ERROR;

    if (client_ == NULL || client_id != client_->getId())
    {
        return  MODBUS_CLIENT_INVALID_ARG;
    }

    return client_->setConnectStatus(status);
}

ErrorCode ModbusManager::getConnectStatusFromClient(int client_id, bool status)
{
    if (start_mode_ != MODBUS_TCP_CLIENT)
       return MODBUS_START_MODE_ERROR;

    if (client_ == NULL || client_id != client_->getId())
    {
        return  MODBUS_CLIENT_INVALID_ARG;
    }

    return client_->getConnectStatus(status);
}

ErrorCode ModbusManager::setConfigToClient(int client_id, ModbusClientConfig config)
{
    if (start_mode_ != MODBUS_TCP_CLIENT)
       return MODBUS_START_MODE_ERROR;

    if (client_ == NULL || client_id != client_->getId())
    {
        return  MODBUS_CLIENT_INVALID_ARG;
    }

    return client_->setConfig(config);
}

ErrorCode ModbusManager::getConfigFromClient(int client_id, ModbusClientConfig config)
{
    if (start_mode_ != MODBUS_TCP_CLIENT)
       return MODBUS_START_MODE_ERROR;

    if (client_ == NULL || client_id != client_->getId())
    {
        return  MODBUS_CLIENT_INVALID_ARG;
    }

    config = client_->getConfig();
    return SUCCESS;
}

ErrorCode ModbusManager::writeCoilsByClient(int client_id, int addr, int nb, uint8_t *dest)
{
    if (start_mode_ != MODBUS_TCP_CLIENT)
       return MODBUS_START_MODE_ERROR;

    if (client_ == NULL || client_id != client_->getId())
    {
        return MODBUS_CLIENT_INVALID_ARG;
    }

    bool status = false;
    if (SUCCESS != client_->getConnectStatus(status)
        || !status)
    {
        return  MODBUS_CLIENT_CONNECT_FAILED;
    }

    return client_->writeCoils(addr, nb, dest);
}

ErrorCode ModbusManager::readCoilsByClient(int client_id, int addr, int nb, uint8_t *dest)
{
    if (start_mode_ != MODBUS_TCP_CLIENT)
       return MODBUS_START_MODE_ERROR;

    if (client_ == NULL || client_id != client_->getId())
    {
        return MODBUS_CLIENT_INVALID_ARG;
    }

    bool status = false;
    if (SUCCESS != client_->getConnectStatus(status)
        ||!status)
    {
        return MODBUS_CLIENT_CONNECT_FAILED;
    }

    return client_->readCoils(addr, nb, dest);
}

ErrorCode ModbusManager::readDiscreteInputsByClient(int client_id, int addr, int nb, uint8_t *dest)
{
    if (start_mode_ != MODBUS_TCP_CLIENT)
       return MODBUS_START_MODE_ERROR;

    if (client_ == NULL || client_id != client_->getId())
    {
        return  MODBUS_CLIENT_INVALID_ARG;
    }

    bool status = false;
    if (SUCCESS != client_->getConnectStatus(status)
        || !status)
    {
        return MODBUS_CLIENT_CONNECT_FAILED;
    }

    return client_->readDiscreteInputs(addr, nb, dest);
}

ErrorCode ModbusManager::writeHoldingRegsByClient(int client_id, int addr, int nb, uint16_t *dest)
{
    if (start_mode_ != MODBUS_TCP_CLIENT)
       return MODBUS_START_MODE_ERROR;

    if (client_ == NULL || client_id != client_->getId())
    {
        return MODBUS_CLIENT_INVALID_ARG;
    }

    bool status = false;
    if (SUCCESS != client_->getConnectStatus(status)
        || !status)
    {
        return  MODBUS_CLIENT_CONNECT_FAILED;
    }

    return client_->writeHoldingRegs(addr, nb, dest);
}

ErrorCode ModbusManager::readHoldingRegsByClient(int client_id, int addr, int nb, uint16_t *dest)
{
    if (start_mode_ != MODBUS_TCP_CLIENT)
       return MODBUS_START_MODE_ERROR;

    if (client_ == NULL || client_id != client_->getId())
    {
        return  MODBUS_CLIENT_INVALID_ARG;
    }

    bool status = false;
    if (SUCCESS != client_->getConnectStatus(status) || !status)
    {
        return  MODBUS_CLIENT_CONNECT_FAILED;
    }

    return client_->readHoldingRegs(addr, nb, dest);
}

ErrorCode ModbusManager::writeAndReadHoldingRegsByClient(int client_id, int write_addr, int write_nb, const uint16_t *write_dest,
    int read_addr, int read_nb, uint16_t *read_dest)
{
    if (start_mode_ != MODBUS_TCP_CLIENT)
       return MODBUS_START_MODE_ERROR;

    if (client_ == NULL || client_id != client_->getId())
    {
        return MODBUS_CLIENT_INVALID_ARG;
    }

    bool status = false;
    if (SUCCESS != client_->getConnectStatus(status) || !status)
    {
        return MODBUS_CLIENT_CONNECT_FAILED;
    }

    return client_->writeAndReadHoldingRegs(write_addr, write_nb, write_dest, read_addr, read_nb, read_dest);
}

ErrorCode ModbusManager::readInputRegsByClient(int client_id, int addr, int nb, uint16_t *dest)
{
    if (start_mode_ != MODBUS_TCP_CLIENT)
       return MODBUS_START_MODE_ERROR;

    if (client_ == NULL || client_id != client_->getId())
    {
        return  MODBUS_CLIENT_INVALID_ARG;
    }

    bool status = false;
    if (SUCCESS != client_->getConnectStatus(status) || !status)
    {
        return MODBUS_CLIENT_CONNECT_FAILED;
    }

    return client_->readInputRegs(addr, nb, dest);
}

bool ModbusManager::isClientRunning(int client_id)
{
    if (client_id != client_->getId())
    {
        return false;
    }

    return client_->isRunning();
}

void ModbusManager::getClientIdAndName(int id,  string name)
{
    id = client_->getId();
    name = client_->getName();
}