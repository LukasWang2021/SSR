#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "error_monitor.h"
#include "error_code.h"
#include "common_log.h"
#include "net_connection.h"

#include "modbus_manager.h"

using namespace std;
using namespace fst_hal;

ErrorCode ModbusManager::addClient(ModbusClientStartInfo &start_info)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->addClient(start_info);
}

ErrorCode ModbusManager::deleteClient(int client_id)
{    
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->deleteClient(client_id);
}

ErrorCode ModbusManager::connectClient(int client_id)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->connectClient(client_id);
}

ErrorCode ModbusManager::closeClient(int client_id)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->closeClient(client_id);
}

ErrorCode ModbusManager::setClientEnableStatus(int client_id, bool &status)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->setEnableStatus(client_id, status);
}

ErrorCode ModbusManager::getClientEnableStatus(int client_id, bool &status)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->getEnableStatus(client_id, status);
}

ErrorCode ModbusManager::setClientRegInfo(int client_id, ModbusClientRegInfo &reg_info)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->setRegInfo(client_id, reg_info);
}

ErrorCode ModbusManager::getClientRegInfo(int client_id, ModbusClientRegInfo &reg_info)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->getRegInfo(client_id, reg_info);
}

ErrorCode ModbusManager::writeCoilsByClient(int client_id, int addr, int nb, uint8_t *dest)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->writeCoils(client_id, addr, nb, dest);
}

ErrorCode ModbusManager::readCoilsByClient(int client_id, int addr, int nb, uint8_t *dest)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->readCoils(client_id, addr, nb, dest);
}

ErrorCode ModbusManager::readDiscreteInputsByClient(int client_id, int addr, int nb, uint8_t *dest)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->readDiscreteInputs(client_id, addr, nb, dest);
}

ErrorCode ModbusManager::writeHoldingRegsByClient(int client_id, int addr, int nb, uint16_t *dest)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->writeHoldingRegs(client_id, addr, nb, dest);
}

ErrorCode ModbusManager::readHoldingRegsByClient(int client_id, int addr, int nb, uint16_t *dest)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->readHoldingRegs(client_id, addr, nb, dest);
}

ErrorCode ModbusManager::writeAndReadHoldingRegsByClient(int client_id, int write_addr, int write_nb, const uint16_t *write_dest,
    int read_addr, int read_nb, uint16_t *read_dest)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->writeAndReadHoldingRegs(client_id, write_addr, write_nb, write_dest, read_addr, read_nb, read_dest);
}

ErrorCode ModbusManager::readInputRegsByClient(int client_id, int addr, int nb, uint16_t *dest)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->readInputRegs(client_id, addr, nb, dest);
}

ErrorCode ModbusManager::getConnectedClientIdList(vector<int> &id_list)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    id_list = client_manager_ptr_->getConnectedClientIdList();
    return SUCCESS;
}

ErrorCode ModbusManager::getClientIdList(vector<int> &id_list)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    id_list = client_manager_ptr_->getClientIdList();
    return SUCCESS;
}

ErrorCode ModbusManager::getClientStartInfo(int client_id, ModbusClientStartInfo &start_info)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->getStartInfo(client_id, start_info);
}


ErrorCode ModbusManager::getClientConfigParamsList(vector<ModbusClientConfigParams> &client_config_params_list)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    client_config_params_list = client_manager_ptr_->getConfigParamsList();
    return SUCCESS;
}


ErrorCode ModbusManager::getClientCtrlState(int client_id, int &ctrl_state)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->getCtrlState(client_id, ctrl_state);
}

ErrorCode ModbusManager::updateClientStartInfo(ModbusClientStartInfo &start_info)
{
    if (start_mode_ != MODBUS_CLIENT)
       return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->updateStartInfo(start_info);
}

ErrorCode ModbusManager::isConnected(int client_id, bool &is_connected)
{
    if (start_mode_ != MODBUS_CLIENT)
        return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->isConnected(client_id, is_connected);
}

ErrorCode ModbusManager::getClientConfigParams(int client_id, ModbusClientConfigParams &client_config_params)
{
    if (start_mode_ != MODBUS_CLIENT)
        return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->getConfigParams(client_id, client_config_params);
}

ErrorCode ModbusManager::getClientScanRate(int client_id, int &scan_rate)
{
    if (start_mode_ != MODBUS_CLIENT)
        return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->getClientScanRate(client_id, scan_rate);
}

ErrorCode ModbusManager::replaceClient(int &replaced_id, ModbusClientStartInfo &start_info)
{
    if (start_mode_ != MODBUS_CLIENT)
        return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->replaceClient(replaced_id, start_info);
}

ErrorCode ModbusManager::scanClientDataArea(int &client_id)
{
    if (start_mode_ != MODBUS_CLIENT)
        return MODBUS_START_MODE_ERROR;

    return client_manager_ptr_->scanDataArea(client_id);
}