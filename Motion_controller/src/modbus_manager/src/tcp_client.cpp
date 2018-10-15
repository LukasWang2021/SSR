#include "tcp_client.h"

#include "error_monitor.h"
#include "error_code.h"
#include "serverAlarmApi.h"

using namespace fst_modbus;

ModbusTCPClient::ModbusTCPClient(string ip, int port):
    log_ptr_(NULL), param_ptr_(NULL),
    tcp_client_file_path_(COMPONENT_PARAM_FILE_DIR),
    ip_(ip), port_(port), is_debug_(true), socket_(17)
{
    ctx_ = modbus_new_tcp(ip_.c_str(), port_);
    response_timeout_.tv_sec = 0;
    response_timeout_.tv_usec = 0;
    bytes_timeout_.tv_sec = 0;
    bytes_timeout_.tv_usec = 0;

    tcp_client_file_path_ += "tcp_client.yaml";
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ModbusManagerParam();
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

void ModbusTCPClient::setDebug(bool flag)
{
    is_debug_ = flag;
}

void ModbusTCPClient::setSocket(int s)
{
    socket_ = s;
}
void ModbusTCPClient::setResponseTimeout(timeval& timeout)
{
    response_timeout_.tv_sec = timeout.tv_sec;
    response_timeout_.tv_usec = timeout.tv_usec;
}

void ModbusTCPClient::setBytesTimeout(timeval& timeout)
{
    bytes_timeout_.tv_sec = timeout.tv_sec;
    bytes_timeout_.tv_usec = timeout.tv_usec;
}

bool ModbusTCPClient::loadComponentParams()
{
    int bytes_sec = 0;
    int bytes_usec = 0;
    int response_sec = 0;
    int response_usec = 0;

    if (!tcp_client_yaml_help_.loadParamFile(tcp_client_file_path_.c_str())
        || !tcp_client_yaml_help_.getParam("is_debug", is_debug_)
        || !tcp_client_yaml_help_.getParam("socket", socket_)
        || !tcp_client_yaml_help_.getParam("response_timeout/tv_sec", response_sec)
        || !tcp_client_yaml_help_.getParam("response_timeout/tv_usec", response_usec)
        || !tcp_client_yaml_help_.getParam("bytes_timeout/tv_sec", bytes_sec)
        || !tcp_client_yaml_help_.getParam("bytes_timeout/tv_usec", bytes_usec))
    {
        cout << " Failed load tcp_client.yaml " << endl;
        return false;
    }

    bytes_timeout_.tv_sec = static_cast<time_t>(bytes_sec);
    bytes_timeout_.tv_usec = static_cast<suseconds_t>(bytes_usec);
    response_timeout_.tv_sec = static_cast<time_t>(bytes_sec);
    response_timeout_.tv_usec = static_cast<suseconds_t>(bytes_usec);

    return true;
}

bool ModbusTCPClient::saveComponentParams()
{
    int bytes_sec = static_cast<int>(bytes_timeout_.tv_sec);
    int bytes_usec = static_cast<int>(bytes_timeout_.tv_usec);
    int response_sec = static_cast<int>(response_timeout_.tv_sec);
    int response_usec = static_cast<int>(response_timeout_.tv_usec);

    FST_ERROR("tcp client save file path = %s", tcp_client_file_path_.c_str());
    if (!tcp_client_yaml_help_.setParam("is_debug", is_debug_)
        || !tcp_client_yaml_help_.setParam("socket", socket_)
        || !tcp_client_yaml_help_.setParam("response_timeout/bytes_timeout", response_sec)
        || !tcp_client_yaml_help_.setParam("response_timeout/tv_usec", response_usec)
        || !tcp_client_yaml_help_.setParam("bytes_timeout/tv_sec", bytes_sec)
        || !tcp_client_yaml_help_.setParam("bytes_timeout/tv_usec", bytes_usec)
        || !tcp_client_yaml_help_.dumpParamFile(tcp_client_file_path_.c_str()))
    {
        cout << " Failed save tcp_client.yaml " << endl;
        return false;
    }

    return true;
}

ErrorCode ModbusTCPClient::init()
{
#if 0
    if (!saveComponentParams())
    {
        return MODBUS_CLIENT_INIT_FAILED; 
    }
#endif

    if (!loadComponentParams())
    {
        return MODBUS_CLIENT_INIT_FAILED; 
    }
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

int ModbusTCPClient::getSocket()
{
    return socket_;
}

ErrorCode ModbusTCPClient::getResponseTimeout(timeval& timeout)
{
    uint32_t sec = 0;
    uint32_t usec = 0;
    if (modbus_get_response_timeout(ctx_, &sec, &usec) < 0)
    {
        // get error
        FST_ERROR("Modbus : get response timeout failed: %s", modbus_strerror(errno));
        return MODBUS_CLIENT_GET_RESPONSE_TIMEOUT_FAILED;
    }

    timeout.tv_sec = static_cast<time_t>(sec);
    timeout.tv_usec = static_cast<suseconds_t>(usec);

    return SUCCESS;
}

ErrorCode ModbusTCPClient::getByteTimeout(timeval& timeout)
{
    uint32_t sec = 0;
    uint32_t usec = 0;
    if (modbus_get_byte_timeout(ctx_,&sec, &usec) < 0)
    {
        FST_ERROR("Modbus : get bytes timeout failed: %s", modbus_strerror(errno));
        return MODBUS_CLIENT_GET_BYTES_TIMEOUT_FAILED;
    }

    timeout.tv_sec = static_cast<time_t>(sec);
    timeout.tv_usec = static_cast<suseconds_t>(usec);

    return SUCCESS;
}

ErrorCode ModbusTCPClient::readDiscreteInputs(ModbusStatus& status)
{
    if (REGISTER_ONE_OP_NUM < status.nb)
    {
        return MODBUS_CLIENT_OPERATION_NUM_TOO_LARGE;
    }

    if (status.nb != modbus_read_input_bits(ctx_, status.addr, status.nb, status.dest))
    {
        return MODBUS_CLIENT_READ_FAILED;
    }

    return SUCCESS;
}

ErrorCode ModbusTCPClient::readInputRegs(ModbusRegs& regs)
{
    if (REGISTER_ONE_OP_NUM < regs.nb)
    {
        return MODBUS_CLIENT_READ_FAILED;
    }

    if(regs.nb != modbus_read_input_registers(ctx_, regs.addr, regs.nb, regs.dest))
    {
        return MODBUS_CLIENT_READ_FAILED;
    }
    return SUCCESS;
}

ErrorCode ModbusTCPClient::readCoils(ModbusStatus& status)
{
    if (STATUS_ONE_OP_NUM < status.nb)
    {
        return MODBUS_CLIENT_OPERATION_NUM_TOO_LARGE;
    }
    if (status.nb != modbus_read_bits(ctx_, status.addr, status.nb, status.dest))
    {
        FST_ERROR("Modbus : fa iled read coils: %s : 0x%x", modbus_strerror(errno), errno);
        return MODBUS_CLIENT_READ_FAILED ;
    }

    return SUCCESS;
}

ErrorCode ModbusTCPClient::writeCoils(ModbusStatus& status)
{
    if (STATUS_ONE_OP_NUM < status.nb)
    {
        return MODBUS_CLIENT_OPERATION_NUM_TOO_LARGE;
    }

    if(status.nb != modbus_write_bits(ctx_, status.addr, status.nb, status.dest))
    {
        return MODBUS_CLIENT_WRITE_FAILED;
    }

    return SUCCESS;
}

ErrorCode ModbusTCPClient::readHoldingRegs(ModbusRegs& regs)
{
    if (REGISTER_ONE_OP_NUM < regs.nb)
    {
        return MODBUS_CLIENT_OPERATION_NUM_TOO_LARGE;
    }
    if (regs.nb != modbus_read_registers(ctx_, regs.addr, regs.nb, regs.dest))
    {
        FST_ERROR("Modbus : failed read holding register: %s", modbus_strerror(errno));
        return MODBUS_CLIENT_READ_FAILED;
    }

    return SUCCESS;
}

ErrorCode ModbusTCPClient::writeHoldingRegs(ModbusRegs& regs)
{
    if (REGISTER_ONE_OP_NUM < regs.nb)
    {
        return MODBUS_CLIENT_OPERATION_NUM_TOO_LARGE;
    }
    if(regs.nb != modbus_write_registers(ctx_, regs.addr, regs.nb, regs.dest))
    {
        FST_ERROR("Modbus : failed write holding register: %s", modbus_strerror(errno));
        return MODBUS_CLIENT_WRITE_FAILED;
    }

    return SUCCESS;
}

ErrorCode ModbusTCPClient::writeSingleCoil(int coil_addr, uint8_t status)
{
    if (1 != modbus_write_bit(ctx_, coil_addr, status))
    {
        FST_ERROR("Modbus : failed write single coil: %s", modbus_strerror(errno));
        return MODBUS_CLIENT_WRITE_FAILED;
    }
    return SUCCESS;
}

ErrorCode ModbusTCPClient::writeSingleHoldingReg(int reg_addr, uint16_t value)
{
    if (1 != modbus_write_register(ctx_, reg_addr, value))
    {
        FST_ERROR("Modbus : failed write single holding register: %s", modbus_strerror(errno));
        return MODBUS_CLIENT_WRITE_FAILED;
    }
    return SUCCESS;
}

ErrorCode ModbusTCPClient::writeAndReadHoldingRegs(ModbusRegs& write_regs, ModbusRegs& read_regs)
{
    read_regs.addr = write_regs.addr;
    read_regs.nb = write_regs.nb;

    if (REGISTER_ONE_OP_NUM < write_regs.nb)
    {
        FST_ERROR("Failed to write and read holding register, operation number too large");
        return MODBUS_CLIENT_OPERATION_NUM_TOO_LARGE;
    }
    if(read_regs.nb != modbus_write_and_read_registers(ctx_,
        write_regs.addr, write_regs.nb, write_regs.dest,
        read_regs.addr, read_regs.nb, read_regs.dest))
    {
        FST_ERROR("Modbus : failed write and read holding register: %s", modbus_strerror(errno));
        return MODBUS_CLIENT_WRITE_FAILED;
    }

    return SUCCESS;
}

ErrorCode ModbusTCPClient::writeAndReadSingleCoil(int addr,  uint8_t& write_status, uint8_t& read_status)
{
    if(1 != writeSingleCoil(addr, write_status))
    {
        FST_ERROR("Modbus Client : error write single coil : %s", modbus_strerror(errno));
        return MODBUS_CLIENT_WRITE_FAILED;
    }

    ModbusStatus modbus_status;
    modbus_status.addr = addr;
    modbus_status.nb = 1;
    modbus_status.dest = &read_status;

    if(1 != readCoils(modbus_status)
        || write_status != read_status)
    {
        FST_ERROR("Modbus Client : Failed read single coil :%s",  modbus_strerror(errno));
        return MODBUS_CLIENT_READ_FAILED;
    }

    return SUCCESS;
}

ErrorCode ModbusTCPClient::writeAndReadSingleHoldingReg(int addr, uint16_t& write_reg, uint16_t& read_reg)
{
    ErrorCode error_code;
    if(1 != writeSingleHoldingReg(addr, write_reg))
    {
        FST_ERROR("Modbus Client : error write single holding reg : %s", modbus_strerror(errno));
        return MODBUS_CLIENT_WRITE_FAILED;
    }

    ModbusRegs modbus_regs;
    modbus_regs.addr = addr;
    modbus_regs.nb = 1;
    modbus_regs.dest = &read_reg;

    if(1 != readHoldingRegs(modbus_regs)
        || write_reg != read_reg)
    {
        FST_ERROR("Modbus Client : Failed read single holding reg :%s", modbus_strerror(errno));
        return MODBUS_CLIENT_READ_FAILED;
    }

    return SUCCESS;
}

ErrorCode ModbusTCPClient::writeAndReadCoils(ModbusStatus& write_status, ModbusStatus& read_status)
{
    int nb = write_status.nb;
    if (STATUS_ONE_OP_NUM < nb)
    {
        FST_ERROR("Failed to write coils, operation number too large");
        return MODBUS_CLIENT_OPERATION_NUM_TOO_LARGE;
    }
    if (nb != writeCoils(write_status))
    {
        FST_ERROR("Modbus Client : Failed write coils: %s", modbus_strerror(errno));
        return MODBUS_CLIENT_WRITE_FAILED;
    }

    read_status.addr = write_status.addr;
    read_status.nb = write_status.nb;

    if (nb != readCoils(read_status)
        || write_status.dest != read_status.dest)
    {
        FST_ERROR("Modbus Client : Failed read coils: %s", modbus_strerror(errno));
        return MODBUS_CLIENT_READ_FAILED;
    }

    return SUCCESS;
}
