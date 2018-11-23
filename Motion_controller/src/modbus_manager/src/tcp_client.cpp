#include "tcp_client.h"

#include "error_monitor.h"
#include "error_code.h"
#include "serverAlarmApi.h"

using namespace fst_hal;

ModbusTCPClient::ModbusTCPClient(string file_path):
    log_ptr_(NULL), param_ptr_(NULL), ctx_(NULL),
    is_debug_(true), socket_(17)
{
    ip_ = "";
    port_ = -1;
    comm_type_ = "TCP";
    response_timeout_.tv_sec = 0;
    response_timeout_.tv_usec = 0;
    bytes_timeout_.tv_sec = 0;
    bytes_timeout_.tv_usec = 0;

    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ModbusClientParam(file_path);

    FST_LOG_INIT("ModbusTcpClient");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
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

ErrorCode ModbusTCPClient::setResponseTimeout(timeval timeout)
{
    param_ptr_->response_timeout_.tv_sec = timeout.tv_sec;
    param_ptr_->response_timeout_.tv_usec = timeout.tv_usec;

    if (!param_ptr_->saveResponseTimeoutParam())
        return MODBUS_MANAGER_LOAD_PARAM_FAILED;

    response_timeout_.tv_sec = timeout.tv_sec;
    response_timeout_.tv_usec = timeout.tv_usec;

    return SUCCESS;
}

ErrorCode ModbusTCPClient::setBytesTimeout(timeval timeout)
{
    param_ptr_->bytes_timeout_.tv_sec = timeout.tv_sec;
    param_ptr_->bytes_timeout_.tv_usec = timeout.tv_usec;

    if (!param_ptr_->saveBytesTimeoutParam())
        return MODBUS_MANAGER_LOAD_PARAM_FAILED; // SAVE_FALIED

    bytes_timeout_.tv_sec = timeout.tv_sec;
    bytes_timeout_.tv_usec = timeout.tv_usec;
    return SUCCESS;
}


ErrorCode ModbusTCPClient::setIp(string ip)
{
    param_ptr_->ip_ = ip;

    if (!param_ptr_->saveIp())
        return MODBUS_MANAGER_LOAD_PARAM_FAILED; // SAVE_FALIED

    ip_ = ip;

    return SUCCESS;
}

ErrorCode ModbusTCPClient::setPort(int port)
{
    param_ptr_->port_ = port;

    if (!param_ptr_->savePort())
        return MODBUS_MANAGER_LOAD_PARAM_FAILED; // SAVE_FALIED

    port_ = port;
    return SUCCESS;
}

string ModbusTCPClient::getIp()
{ 
    return ip_;
}

int ModbusTCPClient::getPort()
{
    return port_;
}

ErrorCode ModbusTCPClient::initParam()
{
    if (!param_ptr_->loadParam()) 
        return MODBUS_CLIENT_LOAD_PARAM_FAILED;

    return SUCCESS;
}

ErrorCode ModbusTCPClient::init()
{
    if (!param_ptr_->loadParam())
        return MODBUS_CLIENT_LOAD_PARAM_FAILED;

    this->ip_ = param_ptr_->ip_;
    this->port_ = param_ptr_->port_;
    this->comm_type_ = param_ptr_->comm_type_;
    this->is_debug_ = param_ptr_->is_debug_;
    this->bytes_timeout_.tv_sec = param_ptr_->bytes_timeout_.tv_sec;
    this->bytes_timeout_.tv_usec = param_ptr_->bytes_timeout_.tv_usec;
    this->response_timeout_.tv_sec = param_ptr_->response_timeout_.tv_sec;
    this->response_timeout_.tv_usec = param_ptr_->response_timeout_.tv_usec;

    ctx_ = modbus_new_tcp(ip_.c_str(), port_);
    if(ctx_ == NULL)
        return MODBUS_CLIENT_INIT_FAILED;

    if (modbus_set_error_recovery(ctx_, MODBUS_ERROR_RECOVERY_LINK) < 0)
    {
        FST_ERROR("Failed to set error recovery : %s", modbus_strerror(errno));
        return MODBUS_CLIENT_INIT_FAILED;
    }

    if (modbus_set_response_timeout(ctx_, response_timeout_.tv_sec, response_timeout_.tv_usec) < 0)
    {
        FST_ERROR("Falied set response timeout : %s", modbus_strerror(errno));
        return MODBUS_CLIENT_INIT_FAILED;
    }

    if (modbus_set_byte_timeout(ctx_, bytes_timeout_.tv_sec, bytes_timeout_.tv_usec) < 0)
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
    return SUCCESS;
}

ErrorCode ModbusTCPClient::getResponseTimeout(timeval& timeout)
{
    uint32_t sec = 0;
    uint32_t usec = 0;

    if (modbus_get_response_timeout(ctx_, &sec, &usec) < 0)
        return MODBUS_CLIENT_GET_RESPONSE_TIMEOUT_FAILED;

    timeout.tv_sec = static_cast<time_t>(sec);
    timeout.tv_usec = static_cast<suseconds_t>(usec);

    return SUCCESS;
}

ErrorCode ModbusTCPClient::getBytesTimeout(timeval& timeout)
{
    uint32_t sec = 0;
    uint32_t usec = 0;

    if (modbus_get_byte_timeout(ctx_,&sec, &usec) < 0)
        return MODBUS_CLIENT_GET_BYTES_TIMEOUT_FAILED;

    timeout.tv_sec = static_cast<time_t>(sec);
    timeout.tv_usec = static_cast<suseconds_t>(usec);

    return SUCCESS;
}

ErrorCode ModbusTCPClient::readDiscreteInputs(int addr, int nb, uint8_t *dest)
{
    if (REGISTER_ONE_OP_NUM < nb)
        return MODBUS_CLIENT_OPERATION_NUM_TOO_LARGE;

    if (nb != modbus_read_input_bits(ctx_, addr, nb, dest))
        return MODBUS_CLIENT_READ_FAILED;

    return SUCCESS;
}

ErrorCode ModbusTCPClient::readInputRegs(int addr, int nb, uint16_t *dest)
{
    if (REGISTER_ONE_OP_NUM < nb)
        return MODBUS_CLIENT_READ_FAILED;

    if(nb != modbus_read_input_registers(ctx_, addr, nb, dest))
        return MODBUS_CLIENT_READ_FAILED;

    return SUCCESS;
}

ErrorCode ModbusTCPClient::readCoils(int addr, int nb, uint8_t *dest)
{
    if (STATUS_ONE_OP_NUM < nb)
        return MODBUS_CLIENT_OPERATION_NUM_TOO_LARGE;

    if (nb != modbus_read_bits(ctx_, addr, nb, dest))
         return MODBUS_CLIENT_READ_FAILED ;

    return SUCCESS;
}

ErrorCode ModbusTCPClient::writeCoils(int addr, int nb, uint8_t *dest)
{
    if (STATUS_ONE_OP_NUM < nb)
        return MODBUS_CLIENT_OPERATION_NUM_TOO_LARGE;

    if(nb != modbus_write_bits(ctx_, addr, nb, dest))
        return MODBUS_CLIENT_WRITE_FAILED;

    return SUCCESS;
}


ErrorCode ModbusTCPClient::readHoldingRegs(int addr, int nb, uint16_t *dest)
{
    if (REGISTER_ONE_OP_NUM < nb)
        return MODBUS_CLIENT_OPERATION_NUM_TOO_LARGE;

    if (nb != modbus_read_registers(ctx_, addr, nb, dest))
        return MODBUS_CLIENT_READ_FAILED;

    return SUCCESS;
}

ErrorCode ModbusTCPClient::writeHoldingRegs(int addr, int nb, uint16_t *dest)
{
    if (REGISTER_ONE_OP_NUM < nb)
        return MODBUS_CLIENT_OPERATION_NUM_TOO_LARGE;

    if(nb != modbus_write_registers(ctx_, addr, nb, dest))
        return MODBUS_CLIENT_WRITE_FAILED;

    return SUCCESS;
}

ErrorCode ModbusTCPClient::writeAndReadHoldingRegs(
        int write_addr, int write_nb, const uint16_t *write_dest,
        int read_addr, int read_nb, uint16_t *read_dest)
{
    if (REGISTER_ONE_OP_NUM < write_nb
    || REGISTER_ONE_OP_NUM < read_nb)
        return MODBUS_CLIENT_OPERATION_NUM_TOO_LARGE;

    if(read_nb != modbus_write_and_read_registers(ctx_,
        write_addr, write_nb, write_dest, read_addr, read_nb, read_dest))
       return MODBUS_CLIENT_WRITE_FAILED;

    return SUCCESS;
}
