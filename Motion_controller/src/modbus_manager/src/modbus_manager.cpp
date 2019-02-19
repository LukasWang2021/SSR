#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "error_monitor.h"
#include "error_code.h"
#include "common_log.h"
#include "net_connection.h"

#include "modbus_manager.h"

using namespace std;
using namespace fst_hal;

ModbusManager::ModbusManager(int address):
    BaseDevice(address, fst_hal::DEVICE_TYPE_MODBUS), address_(address),
    log_ptr_(NULL), param_ptr_(NULL),
    client_manager_ptr_(NULL), server_(NULL),
    start_mode_(MODBUS_START_MODE_INVALID)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ModbusManagerParam();
    FST_LOG_INIT("ModbusManager");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
}

ModbusManager::ModbusManager():
    BaseDevice(0, fst_hal::DEVICE_TYPE_MODBUS), address_(0),
    log_ptr_(NULL), param_ptr_(NULL),
    client_manager_ptr_(NULL), server_(NULL),
    start_mode_(MODBUS_START_MODE_INVALID)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ModbusManagerParam();
    FST_LOG_INIT("ModbusManager");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
}

bool ModbusManager::init()
{
    if(!param_ptr_->loadParam())
        return false;
    
    start_mode_ = param_ptr_->start_mode_;

    if (server_ == NULL)
        server_ = new ModbusServer(param_ptr_->server_file_path_, param_ptr_->server_config_file_path_);

    if (server_->initParam() != SUCCESS)
        return false;

    if (client_manager_ptr_ == NULL)
        client_manager_ptr_ = new ModbusClientManager(
            param_ptr_->client_file_path_, param_ptr_->client_config_file_path_);

    if (client_manager_ptr_->initParam() != SUCCESS)
        return false;

    // if (start_mode_ == MODBUS_CLIENT)
    // {
        // if (client_manager_ptr_->initCLientListByParams() != SUCCESS) return false;
    // }

    return true;
}

ModbusManager::~ModbusManager()
{
    if (client_manager_ptr_ != NULL)
    {
        delete client_manager_ptr_;
        client_manager_ptr_ = NULL;
    }

    if (server_ != NULL)
    {
        server_->closeServer();
        delete server_;
        server_ = NULL;
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

ErrorCode ModbusManager::setStartMode(int start_mode)
{
    if (start_mode == start_mode_) return SUCCESS;

    if (start_mode == MODBUS_SERVER) //client transform to server
    {
        if (!client_manager_ptr_->isAllClientClosed())
        {
            return MODBUS_CLIENT_NOT_ALL_CLOSED;
        }

        param_ptr_->start_mode_ = start_mode;
        if (!param_ptr_->saveStartMode()) return MODBUS_MANAGER_SAVE_PARAM_FAILED;

        start_mode_ = param_ptr_->start_mode_;
        return SUCCESS;
    }

    if (start_mode == MODBUS_CLIENT) // server transform to client
    {
        if (server_->isRunning())
        {
            return MODBUS_SERVER_IS_RUNNING;
        }

        param_ptr_->start_mode_ = start_mode;
        if (!param_ptr_->saveStartMode()) return MODBUS_MANAGER_SAVE_PARAM_FAILED;

        start_mode_ = param_ptr_->start_mode_;
        return SUCCESS;
    }

    return MODBUS_START_MODE_ERROR;
}


int ModbusManager::getStartMode()
{
    return start_mode_;
}

ErrorCode ModbusManager::writeCoils(int id, int addr, int nb, uint8_t *dest)
{
    if (start_mode_ == MODBUS_SERVER)
    {
        return writeCoilsToServer(addr, nb, dest);
    }

    if (start_mode_ == MODBUS_CLIENT)
    {
        return writeCoilsByClient(id, addr, nb, dest);
    }

    return MODBUS_START_MODE_ERROR;
}

ErrorCode ModbusManager::readCoils(int id, int addr, int nb, uint8_t *dest)
{
    if (start_mode_ == MODBUS_SERVER)
    {
        return readCoilsFromServer(addr, nb, dest);
    }

    if (start_mode_ == MODBUS_CLIENT)
    {
        return readCoilsByClient(id, addr, nb, dest);
    }

    return MODBUS_START_MODE_ERROR;
}

ErrorCode ModbusManager::readDiscreteInputs(int id, int addr, int nb, uint8_t *dest)
{
    if (start_mode_ == MODBUS_SERVER)
    {
        return readDiscreteInputsFromServer(addr, nb, dest);
    }

    if (start_mode_ == MODBUS_CLIENT)
    {
        return readDiscreteInputsByClient(id, addr, nb, dest);
    }

    return MODBUS_START_MODE_ERROR;
}

ErrorCode ModbusManager::writeHoldingRegs(int id, int addr, int nb, uint16_t *dest)
{
    if (start_mode_ == MODBUS_SERVER)
    {
        return writeHoldingRegsToServer(addr, nb, dest);
    }

    if (start_mode_ == MODBUS_CLIENT)
    {
        return writeHoldingRegsByClient(id, addr, nb, dest);
    }

    return MODBUS_START_MODE_ERROR;
}

ErrorCode ModbusManager::readHoldingRegs(int id, int addr, int nb, uint16_t *dest)
{
    if (start_mode_ == MODBUS_SERVER)
    {
        return readHoldingRegsFromServer(addr, nb, dest);
    }

    if (start_mode_ == MODBUS_CLIENT)
    {
        return readHoldingRegsByClient(id, addr, nb, dest);
    }

    return MODBUS_START_MODE_ERROR;
}

ErrorCode ModbusManager::writeInputRegs(int id, int addr, int nb, uint16_t *dest)
{
    if (start_mode_ == MODBUS_SERVER)
    {
        return writeInputRegsToServer(addr, nb, dest);
    }

    return MODBUS_START_MODE_ERROR;
}

ErrorCode ModbusManager::readInputRegs(int id, int addr, int nb, uint16_t *dest)
{
    if (start_mode_ == MODBUS_SERVER)
    {
        return readInputRegsFromServer(addr, nb, dest);
    }

    if (start_mode_ == MODBUS_CLIENT)
    {
        return readInputRegsByClient(id, addr, nb, dest);
    }

    return MODBUS_START_MODE_ERROR;
}

bool ModbusManager::isModbusValid()
{
    if (start_mode_ == MODBUS_SERVER && isServerRunning())
    {
        setValid(true);
        return true;
    }

    if (start_mode_ == MODBUS_CLIENT && !client_manager_ptr_->isAllClientClosed())
    {
        setValid(true);
        return true;
    }

    setValid(false);
    return false;
}


