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
    BaseDevice(address, fst_hal::DEVICE_TYPE_MODBUS),address_(address),
    log_ptr_(NULL), param_ptr_(NULL),
    client_(NULL), server_(NULL),
    server_ip_(""), server_port_(-1),
    start_mode_(0), client_scan_rate_(0)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ModbusManagerParam();
    FST_LOG_INIT("ModbusManager");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
}

bool ModbusManager::init()
{
    if(!param_ptr_->loadParam())
        return false; //load param error

    server_ = new ModbusTCPServer(param_ptr_->server_file_path_);

    if (server_->initParam() != SUCCESS)
        return false;
    return true;
}

ModbusManager::~ModbusManager()
{
    if (client_ != NULL)
    {
        client_->closeClient();
        delete client_;
        client_ = NULL;
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
    if (start_mode == start_mode_)
    {
        return MODBUS_SAME_START_MODE;
    }

    if (start_mode == SERVER)
    {
        start_mode_ = start_mode; // SERVER
        if (server_ != NULL && server_->isRunning())
        {
            server_->closeServer();
        }
    }
    else if (start_mode == CLIENT)
    {
        start_mode_ = start_mode; // CLIENT
        if (client_ != NULL && client_->isRunning())
        {
            client_->closeClient();
            delete client_;
            client_ = NULL;
        }
    }

    return SUCCESS;
}

int ModbusManager::getStartMode()
{
    return start_mode_;
}

ErrorCode ModbusManager::setScanRate(int scan_rate)
{
    param_ptr_->client_scan_rate_ = scan_rate;

    if (!param_ptr_->saveClientScanRate())
    {
        return MODBUS_MANAGER_SAVE_PARAM_FAILED;
    }

    client_scan_rate_ = scan_rate;
    return SUCCESS;
}

int ModbusManager::getScanRate()
{
    return client_scan_rate_;
}

ErrorCode ModbusManager::writeCoils(int id, int addr, int nb, uint8_t *dest)
{
    if (start_mode_ == SERVER)
    {
        return writeCoilsToServer(addr, nb, dest);
    }

    if (start_mode_ == CLIENT)
    {
        return writeCoilsByClient(id, addr, nb, dest);
    }

    return MODBUS_START_MODE_ERROR;
}

ErrorCode ModbusManager::readCoils(int id, int addr, int nb, uint8_t *dest)
{
    if (start_mode_ == SERVER)
    {
        return readCoilsFromServer(addr, nb, dest);
    }

    if (start_mode_ == CLIENT)
    {
        return readCoilsByClient(id, addr, nb, dest);
    }

    return MODBUS_START_MODE_ERROR;
}

ErrorCode ModbusManager::readDiscreteInputs(int id, int addr, int nb, uint8_t *dest)
{
    if (start_mode_ == SERVER)
    {
        return readDiscreteInputsFromServer(addr, nb, dest);
    }

    if (start_mode_ == CLIENT)
    {
        return readDiscreteInputsByClient(id, addr, nb, dest);
    }

    return MODBUS_START_MODE_ERROR;
}

ErrorCode ModbusManager::writeHoldingRegs(int id, int addr, int nb, uint16_t *dest)
{
    if (start_mode_ == SERVER)
    {
        return writeHoldingRegsToServer(addr, nb, dest);
    }

    if (start_mode_ == CLIENT)
    {
        return writeHoldingRegsByClient(id, addr, nb, dest);
    }

    return MODBUS_START_MODE_ERROR;
}

ErrorCode ModbusManager::readHoldingRegs(int id, int addr, int nb, uint16_t *dest)
{
    if (start_mode_ == SERVER)
    {
        return readHoldingRegsFromServer(addr, nb, dest);
    }

    if (start_mode_ == CLIENT)
    {
        return readHoldingRegsByClient(id, addr, nb, dest);
    }

    return MODBUS_START_MODE_ERROR;
}

ErrorCode ModbusManager::readInputRegs(int id, int addr, int nb, uint16_t *dest)
{
    if (start_mode_ == SERVER)
    {
        return readInputRegsFromServer(addr, nb, dest);
    }

    if (start_mode_ == CLIENT)
    {
        return readInputRegsByClient(id, addr, nb, dest);
    }

    return MODBUS_START_MODE_ERROR;
}

bool ModbusManager::isValid()
{
    if (start_mode_ == SERVER && isServerRunning())
    {
        return true;
    }

    if (start_mode_ == CLIENT && isClientRunning(client_->getId()))
    {
        return true;
    }

    return false;
}

#if 0
void modbusTcpAllClientRoutineThreadFunc(void* arg)
{
    ModbusManager* modbus_manager = static_cast<ModbusManager*>(arg);
    while(modbus_manager->isRunningClient(getClientId()))
    {
        modbus_manager->modbusTcpAllClientThreadFunc();
    }
}
#endif