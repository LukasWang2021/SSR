#include "tcp_client.h"

#include "error_monitor.h"
#include "error_code.h"
#include "serverAlarmApi.h"

using namespace fst_hal;

ModbusTCPClient::ModbusTCPClient(string file_path, int id):
    log_ptr_(NULL), param_ptr_(NULL), ctx_(NULL),
    is_debug_(false), socket_(17), id_(id), is_running_(false)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ModbusClientParam(file_path);

    FST_LOG_INIT("ModbusTcpClient");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);

    is_debug_ = false;
    comm_type_ = "TCP";
    is_enable_ = false;

    config_.name = "";
    config_.ip = "";
    config_.port = -1;
    config_.response_timeout_sec = 0;
    config_.response_timeout_usec = 0;
    config_.bytes_timeout_sec = 0;
    config_.bytes_timeout_usec = 0;
    config_.reg_info.coil.addr = 0;
    config_.reg_info.coil.max_nb = 0;
    config_.reg_info.discrepte_input.addr = 0;
    config_.reg_info.discrepte_input.max_nb = 0;
    config_.reg_info.holding_reg.addr = 0;
    config_.reg_info.holding_reg.max_nb = 0;
    config_.reg_info.input_reg.addr = 0;
    config_.reg_info.input_reg.max_nb = 0;
}

ModbusTCPClient::ModbusTCPClient():
    log_ptr_(NULL), param_ptr_(NULL), ctx_(NULL),
    is_debug_(false), socket_(17), id_(0), is_running_(false)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ModbusClientParam("null");

    FST_LOG_INIT("ModbusTcpClient");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);

    is_debug_ = false;
    comm_type_ = "TCP";
    is_enable_ = false;

    config_.name = "";
    config_.ip = "";
    config_.port = -1;
    config_.response_timeout_sec = 0;
    config_.response_timeout_usec = 0;
    config_.bytes_timeout_sec = 0;
    config_.bytes_timeout_usec = 0;
    config_.reg_info.coil.addr = 0;
    config_.reg_info.coil.max_nb = 0;
    config_.reg_info.discrepte_input.addr = 0;
    config_.reg_info.discrepte_input.max_nb = 0;
    config_.reg_info.holding_reg.addr = 0;
    config_.reg_info.holding_reg.max_nb = 0;
    config_.reg_info.input_reg.addr = 0;
    config_.reg_info.input_reg.max_nb = 0;
}

ModbusTCPClient::~ModbusTCPClient()
{
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

    modbus_close(ctx_);
    if (ctx_ != NULL)
    {
        modbus_free(ctx_);
        ctx_ = NULL;
    }
}

ErrorCode ModbusTCPClient::initParam()
{
    if (!param_ptr_->loadParam())
    {
        return MODBUS_CLIENT_LOAD_PARAM_FAILED;
    }

    param_ptr_->id_ = id_;

    if (!param_ptr_->saveId())
    {
        return MODBUS_CLIENT_SAVE_PARAM_FAILED;
    }

    return SUCCESS;
}

ErrorCode ModbusTCPClient::setConnectStatus(bool status)
{
    if (!status)
    {
        is_enable_ = status;
        return SUCCESS;
    }

    if (!checkConnectStatus())
    {
        return MODBUS_CLIENT_CONNECT_FAILED;
    }

    param_ptr_->is_enable_ = status;

    if (!param_ptr_->saveConnectStatus())
    {
        return MODBUS_CLIENT_SAVE_PARAM_FAILED;
    }

    is_enable_ = status;

    return SUCCESS;
}

ErrorCode ModbusTCPClient::getConnectStatus(bool status)
{
    if (checkConnectStatus())
    {
        status = is_enable_;
        return MODBUS_CLIENT_CONNECT_FAILED;
    }

    is_enable_ = false;
    param_ptr_->is_enable_ = false;

    if (!param_ptr_->saveConnectStatus())
    {
        return MODBUS_CLIENT_SAVE_PARAM_FAILED;
    }

    status = is_enable_;
    return SUCCESS;
}

ErrorCode ModbusTCPClient::setConfig(ModbusClientConfig config)
{
    if (config.port <= 0
        || config.name.length() <= 0
        || config.ip.length() <= 0
        || config.reg_info.coil.max_nb <= 0 
        || config.reg_info.coil.addr < 0
        || config.reg_info.discrepte_input.max_nb <= 0
        || config.reg_info.discrepte_input.addr < 0
        || config.reg_info.input_reg.max_nb <= 0
        || config.reg_info.input_reg.addr < 0
        || config.reg_info.holding_reg.max_nb <= 0
        || config.reg_info.holding_reg.addr < 0)
    {
        return MODBUS_CLIENT_INVALID_ARG;
    }

    param_ptr_->id_ = id_;
    param_ptr_->config_ = config;

    if (!param_ptr_->saveConfig())
    {
        return MODBUS_CLIENT_SAVE_PARAM_FAILED;
    }

    config_ = config;
    return SUCCESS;
}

ModbusClientConfig ModbusTCPClient::getConfig()
{
    return config_;
}

bool ModbusTCPClient::checkConnectStatus()
{
    return true;
}

int ModbusTCPClient::getId()
{
    return id_;
}

ErrorCode ModbusTCPClient::openClient()
{
    if (!checkConnectStatus())
    {
        return MODBUS_CLIENT_CONNECT_FAILED;
    }

    if (!param_ptr_->loadParam())
        return MODBUS_CLIENT_LOAD_PARAM_FAILED;

    config_ = param_ptr_->config_;

    ctx_ = modbus_new_tcp(config_.ip.c_str(), config_.port);
    if(ctx_ == NULL)
        return MODBUS_CLIENT_INIT_FAILED;

    if (modbus_set_error_recovery(ctx_, MODBUS_ERROR_RECOVERY_LINK) < 0)
    {
        FST_ERROR("Failed to set error recovery : %s", modbus_strerror(errno));
        return MODBUS_CLIENT_INIT_FAILED;
    }

    if (modbus_set_response_timeout(ctx_, config_.response_timeout_sec, config_.response_timeout_usec) < 0)
    {
        FST_ERROR("Falied set response timeout : %s", modbus_strerror(errno));
        return MODBUS_CLIENT_INIT_FAILED;
    }

    if (modbus_set_byte_timeout(ctx_, config_.bytes_timeout_sec, config_.bytes_timeout_usec) < 0)
    {
        FST_ERROR("Failed to set bytes timeout : %s", modbus_strerror(errno));
        return MODBUS_CLIENT_INIT_FAILED;
    }

    if (modbus_connect(ctx_) < 0)
    {
        FST_ERROR("Failed to connect socket : %s, socket = %d",
            modbus_strerror(errno),  modbus_get_socket(ctx_));
        return MODBUS_CLIENT_INIT_FAILED;
    }

    if (modbus_set_debug(ctx_, is_debug_) < 0)
    {
        FST_ERROR("Failed to set debug : %s", modbus_strerror(errno));
        return MODBUS_CLIENT_INIT_FAILED;
    }

    is_running_ = true;
    return SUCCESS;
}

void ModbusTCPClient::closeClient()
{
    modbus_close(ctx_);
    modbus_free(ctx_);
    is_running_ = false;
    if (ctx_ != NULL)  ctx_ = NULL;
}

bool ModbusTCPClient::isRunning()
{
    return is_running_;
}

ErrorCode ModbusTCPClient::readDiscreteInputs(int addr, int nb, uint8_t *dest)
{
    if (!checkConnectStatus())
    {
        return MODBUS_CLIENT_CONNECT_FAILED;
    }

    if (STATUS_ONE_OP_NUM < nb)
        return MODBUS_CLIENT_INVALID_ARG;

    if (addr < config_.reg_info.discrepte_input.addr
        || (config_.reg_info.discrepte_input.addr + config_.reg_info.discrepte_input.max_nb - 1) < (addr + nb -1))
        return MODBUS_CLIENT_INVALID_ARG;

    if (nb != modbus_read_input_bits(ctx_, addr, nb, dest))
        return MODBUS_CLIENT_READ_FAILED;

    return SUCCESS;
}

ErrorCode ModbusTCPClient::readInputRegs(int addr, int nb, uint16_t *dest)
{
    if (!checkConnectStatus())
    {
        return MODBUS_CLIENT_CONNECT_FAILED;
    }

    if (STATUS_ONE_OP_NUM < nb)
        return MODBUS_CLIENT_INVALID_ARG;

    if (addr < config_.reg_info.input_reg.addr
        || (config_.reg_info.input_reg.addr + config_.reg_info.input_reg.max_nb - 1) < (addr + nb -1))
        return MODBUS_CLIENT_INVALID_ARG;

    if(nb != modbus_read_input_registers(ctx_, addr, nb, dest))
        return MODBUS_CLIENT_READ_FAILED;

    return SUCCESS;
}

ErrorCode ModbusTCPClient::readCoils(int addr, int nb, uint8_t *dest)
{
    if (!checkConnectStatus())
    {
        return MODBUS_CLIENT_CONNECT_FAILED;
    }

    if (STATUS_ONE_OP_NUM < nb)
        return MODBUS_CLIENT_INVALID_ARG;

    if (addr < config_.reg_info.coil.addr
        || (config_.reg_info.coil.addr + config_.reg_info.coil.max_nb - 1) < (addr + nb -1))
        return MODBUS_CLIENT_INVALID_ARG;

    if (nb != modbus_read_bits(ctx_, addr, nb, dest))
         return MODBUS_CLIENT_READ_FAILED;

    return SUCCESS;
}

ErrorCode ModbusTCPClient::writeCoils(int addr, int nb, uint8_t *dest)
{
    if (!checkConnectStatus())
    {
        return MODBUS_CLIENT_CONNECT_FAILED;
    }

    if (STATUS_ONE_OP_NUM < nb)
        return MODBUS_CLIENT_INVALID_ARG;

    if (addr < config_.reg_info.coil.addr
        || (config_.reg_info.coil.addr + config_.reg_info.coil.max_nb - 1) < (addr + nb -1))
        return MODBUS_CLIENT_INVALID_ARG;

    if(nb != modbus_write_bits(ctx_, addr, nb, dest))
        return MODBUS_CLIENT_WRITE_FAILED;

    return SUCCESS;
}


ErrorCode ModbusTCPClient::readHoldingRegs(int addr, int nb, uint16_t *dest)
{
    if (!checkConnectStatus())
    {
        return MODBUS_CLIENT_CONNECT_FAILED;
    }

    if (STATUS_ONE_OP_NUM < nb)
        return MODBUS_CLIENT_INVALID_ARG;

    if (addr < config_.reg_info.holding_reg.addr
        || (config_.reg_info.holding_reg.addr + config_.reg_info.holding_reg.max_nb - 1) < (addr + nb -1))
        return MODBUS_CLIENT_INVALID_ARG;

    if (nb != modbus_read_registers(ctx_, addr, nb, dest))
        return MODBUS_CLIENT_READ_FAILED;

    return SUCCESS;
}

ErrorCode ModbusTCPClient::writeHoldingRegs(int addr, int nb, uint16_t *dest)
{
    if (!checkConnectStatus())
    {
        return MODBUS_CLIENT_CONNECT_FAILED;
    }

    if (STATUS_ONE_OP_NUM < nb)
        return MODBUS_CLIENT_INVALID_ARG;

    if (addr < config_.reg_info.holding_reg.addr
        || (config_.reg_info.holding_reg.addr + config_.reg_info.holding_reg.max_nb - 1) < (addr + nb -1))
        return MODBUS_CLIENT_INVALID_ARG;

    if(nb != modbus_write_registers(ctx_, addr, nb, dest))
        return MODBUS_CLIENT_WRITE_FAILED;

    return SUCCESS;
}

ErrorCode ModbusTCPClient::writeAndReadHoldingRegs(
        int write_addr, int write_nb, const uint16_t *write_dest,
        int read_addr, int read_nb, uint16_t *read_dest)
{
    if (!checkConnectStatus())
    {
        return MODBUS_CLIENT_CONNECT_FAILED;
    }

    if (REGISTER_ONE_OP_NUM < write_nb
    || REGISTER_ONE_OP_NUM < read_nb)
        return MODBUS_CLIENT_INVALID_ARG;

    if (write_addr < config_.reg_info.holding_reg.addr
        || (config_.reg_info.holding_reg.addr + config_.reg_info.holding_reg.max_nb - 1) < (write_addr + write_nb -1)
        || read_addr < config_.reg_info.holding_reg.addr
        || (config_.reg_info.holding_reg.addr + config_.reg_info.holding_reg.max_nb - 1) < (read_addr + read_nb -1))
        return MODBUS_CLIENT_INVALID_ARG;

    if(read_nb != modbus_write_and_read_registers(ctx_,
        write_addr, write_nb, write_dest, read_addr, read_nb, read_dest))
       return MODBUS_CLIENT_WRITE_FAILED;

    return SUCCESS;
}

string ModbusTCPClient::getName()
{
    return config_.name;
}
