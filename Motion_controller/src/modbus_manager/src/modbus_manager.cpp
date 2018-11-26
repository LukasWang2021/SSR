#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "error_monitor.h"
#include "error_code.h"
#include "common_log.h"

#include "modbus_manager.h"

using namespace std;
using namespace fst_hal;

ModbusManager* ModbusManager::instance_ = NULL;

ModbusManager::ModbusManager():
    log_ptr_(NULL), param_ptr_(NULL),
    client_(NULL), server_(NULL),
    server_ip_(""), server_port_(-1),
    start_mode_(0), client_param_ptr_(NULL)
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
    if (client_param_ptr_ != NULL)
    {
        delete client_param_ptr_;
        client_param_ptr_ = NULL;
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

ErrorCode ModbusManager::init(int start_mode)
{
    if (start_mode != SERVER && start_mode != CLIENT)
    {
        return MODBUS_MANAGER_INIT_FAILED;
    }

    if(!param_ptr_->loadParam())
    {
        return MODBUS_MANAGER_LOAD_PARAM_FAILED;
    }

    start_mode_ = start_mode;

    client_ = new ModbusTCPClient(param_ptr_->client_file_path_);
    return client_->initParam();
}

ErrorCode ModbusManager::initModbus()
{
    ErrorCode error = SUCCESS;

    if (start_mode_ == SERVER)
    {
        server_ = new ModbusTCPServer(param_ptr_->server_file_path_);

        error = server_->init();
        if (error != SUCCESS) return error;

        error = server_->open();
        if (error != SUCCESS) return error;

        server_ip_ = server_->getIp();
        server_port_ = server_->getPort();

        error = client_->setIp(server_ip_);
        if (error != SUCCESS) return error;

        error = client_->setPort(server_port_);
        if (error != SUCCESS) return error;
    }

    error = client_->init();
    if (error != SUCCESS) return error;

    return SUCCESS;
}

bool ModbusManager::isServerRunning()
{
    return server_->isRunning();
}

ErrorCode ModbusManager::getServerInfo(ServerInfo& info)
{
    if (start_mode_ != SERVER) 
       return MODBUS_SERVER_NOT_EXISTED;

    info = server_->getInfo();
    return SUCCESS;
}

ErrorCode ModbusManager::getServerCoilInfo(ModbusRegAddrInfo& info)
{
    if (start_mode_ != SERVER) 
        return MODBUS_SERVER_NOT_EXISTED;

    info = server_->getCoilInfo();
    return SUCCESS;
}

ErrorCode ModbusManager::getServerDiscrepteInputInfo(ModbusRegAddrInfo& info)
{
    if (start_mode_ != SERVER) 
        return MODBUS_SERVER_NOT_EXISTED;

    info = server_->getDiscrepteInputInfo();
    return SUCCESS;
}

ErrorCode ModbusManager::getServerHoldingRegInfo(ModbusRegAddrInfo& info)
{
    if (start_mode_ != SERVER) 
        return MODBUS_SERVER_NOT_EXISTED;

    info = server_->getHoldingRegInfo();
    return SUCCESS;
}

ErrorCode ModbusManager::getServerInputRegInfo(ModbusRegAddrInfo& info)
{
    if (start_mode_ != SERVER) 
        return MODBUS_SERVER_NOT_EXISTED;

    info = server_->getInputRegInfo();
    return SUCCESS;
}

ErrorCode ModbusManager::getServerIp(string& ip)
{
    if (start_mode_ != SERVER) 
        return MODBUS_SERVER_NOT_EXISTED;

    ip = server_->getIp();
    return SUCCESS;
}

ErrorCode ModbusManager::getServerPort(int& port)
{
    if (start_mode_ != SERVER) 
        return MODBUS_SERVER_NOT_EXISTED;

    port = server_->getPort();
    return SUCCESS;
}

ErrorCode ModbusManager::setClientIp(string ip)
{
    return client_->setIp(ip);
}

ErrorCode ModbusManager::setClientPort(int port)
{
    return client_->setPort(port);
}

ErrorCode ModbusManager::setResponseTimeout(timeval timeout)
{
    return client_->setResponseTimeout(timeout);
}

ErrorCode ModbusManager::setBytesTimeout(timeval timeout)
{
    return client_->setBytesTimeout(timeout);
}

ErrorCode ModbusManager::getResponseTimeout(timeval& timeout)
{
    return client_->getResponseTimeout(timeout);
}

ErrorCode ModbusManager::getBytesTimeout(timeval& timeout)
{
    return client_->getBytesTimeout(timeout);
}

ErrorCode ModbusManager::writeCoils(int addr, int nb, uint8_t *dest)
{
    return client_->writeCoils(addr, nb, dest);
}

ErrorCode ModbusManager::readCoils(int addr, int nb, uint8_t *dest)
{
    return client_->readCoils(addr, nb, dest);
}

ErrorCode ModbusManager::readDiscreteInputs(int addr, int nb, uint8_t *dest)
{
    return client_->readDiscreteInputs(addr, nb, dest);
}

ErrorCode ModbusManager::writeHoldingRegs(int addr, int nb, uint16_t *dest)
{
    return client_->writeHoldingRegs(addr, nb, dest);
}

ErrorCode ModbusManager::readHoldingRegs(int addr, int nb, uint16_t *dest)
{
    return client_->readHoldingRegs(addr, nb, dest);
}

ErrorCode ModbusManager::writeAndReadHoldingRegs(
        int write_addr, int write_nb, const uint16_t *write_dest,
        int read_addr, int read_nb, uint16_t *read_dest)
{
    return client_->writeAndReadHoldingRegs(write_addr, write_nb, write_dest,
                                            read_addr, read_nb, read_dest);
}

ErrorCode ModbusManager::readInputRegs(int addr, int nb, uint16_t *dest)
{
    return client_->readInputRegs(addr, nb, dest);
}