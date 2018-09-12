#include "tcp_client.h"

#include "error_monitor.h"
#include "error_code.h"
#include "serverAlarmApi.h"

using namespace fst_modbus;

ModbusTCPClient::ModbusTCPClient(string ip, int port):
    log_ptr_(NULL), param_ptr_(NULL)
{
    nb_fail_ = 0;
    ctx_ = modbus_new_tcp(ip.c_str(), port);

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

bool ModbusTCPClient::init()
{
    if (modbus_set_socket(ctx_, 17) < 0)
    {
        FST_ERROR("Modbus : Client set socket failed : %s, socket = %d",
            modbus_strerror(errno),  modbus_get_socket(ctx_));
        return false;
    }

    if (modbus_connect(ctx_) < 0)
    {
        FST_ERROR("Modbus : Client connect failed : %s, socket = %d",
            modbus_strerror(errno),  modbus_get_socket(ctx_));
        return false;
    }

    if (!setDebug(false))
    {
        FST_ERROR("Modbus : Client debug init failed : %s", modbus_strerror(errno));
        return false;
    }

    return true;
}

int ModbusTCPClient::getSocket()
{
    return modbus_get_socket(ctx_);
}

bool ModbusTCPClient::setSocket(int socket)
{
    if ((socket < SOCKET_ID_MIN) || (SOCKET_ID_MAX < socket))
    {
        FST_ERROR("Modbus : set socket failed, value is too large");
        return false;
    }

    if ((modbus_set_socket(ctx_, socket) < 0)
        || (socket != modbus_get_socket(ctx_)))
    {
        FST_ERROR("Modbus : set socket failed: %s, socket = %d", modbus_strerror(errno), socket);
        return false;
    }

    return true;
}



bool ModbusTCPClient::getResponseTimeout(timeval& timeout)
{
    uint32_t sec = static_cast<uint32_t>(timeout.tv_sec);
    uint32_t usec = static_cast<uint32_t>(timeout.tv_usec);

    if (modbus_get_response_timeout(ctx_, &sec, &usec) < 0)
    {
        // get error
        FST_ERROR("Modbus : get response timeout failed: %s", modbus_strerror(errno));
        return false;
    }

    return true;
}


bool ModbusTCPClient::setResponseTimeout(timeval& timeout)
{
    uint32_t sec = static_cast<uint32_t>(timeout.tv_sec);
    uint32_t usec = static_cast<uint32_t>(timeout.tv_usec);

    if (modbus_set_response_timeout(ctx_, sec, usec) < 0)
    {
        // get error
        FST_ERROR("Modbus : set response timeout failed: %s", modbus_strerror(errno));
        return false;
    }

    return true;
}

bool ModbusTCPClient::getByteTimeout(timeval& timeout)
{
    uint32_t sec = static_cast<uint32_t>(timeout.tv_sec);
    uint32_t usec = static_cast<uint32_t>(timeout.tv_usec);

    if (modbus_get_byte_timeout(ctx_,&sec, &usec) < 0)
    {
        // get error
        FST_ERROR("Modbus : get bytes timeout failed: %s", modbus_strerror(errno));
        return false;
    }

    return true;
}

bool ModbusTCPClient::setByteTimeout(timeval& timeout)
{
    uint32_t sec = static_cast<uint32_t>(timeout.tv_sec);
    uint32_t usec = static_cast<uint32_t>(timeout.tv_usec);

    if (modbus_set_byte_timeout(ctx_, sec, usec) < 0)
    {
        // get error
        FST_ERROR("Modbus : set bytes timeout failed: %s", modbus_strerror(errno));
        return false;
    }

    return true;
}

bool ModbusTCPClient::setDebug(bool flag)
{
    if (modbus_set_debug(ctx_, flag) < 0)
    {
        // get error
        FST_ERROR("Modbus : set debug failed: %s", modbus_strerror(errno));
        return false;
    }

    return true;
}

bool ModbusTCPClient::readDiscreteInputs(ModbusStatus& status)
{
    if (status.nb != modbus_read_input_bits(ctx_, status.addr, status.nb, status.dest))
    {
        FST_ERROR("Modbus : failed read discrete inputs: %s", modbus_strerror(errno));
        nb_fail_++;
        return false;
    }

    return true;
}

bool ModbusTCPClient::readInputRegs(ModbusRegisters& regs)
{
    if(regs.nb != modbus_read_input_registers(ctx_, regs.addr, regs.nb, regs.dest))
    {
        FST_ERROR("Modbus : failed read inputs register: %s", modbus_strerror(errno));
        nb_fail_++;
        return false;
    }
    return true;
}

bool ModbusTCPClient::readCoils(ModbusStatus& status)
{
    if (status.nb != modbus_read_bits(ctx_, status.addr, status.nb, status.dest))
    {
        FST_ERROR("Modbus : failed read coils: %s", modbus_strerror(errno));
        nb_fail_++;
        return false;
    }

    return true;
}

bool ModbusTCPClient::writeCoils(ModbusStatus& status)
{
    if(status.nb != modbus_write_bits(ctx_, status.addr, status.nb, status.dest))
    {
        FST_ERROR("Modbus : failed write Coils: %s", modbus_strerror(errno));
        nb_fail_++;
        return false;
    }

    return true;
}

bool ModbusTCPClient::readHoldingRegs(ModbusRegisters& regs)
{
    if (regs.nb != modbus_read_registers(ctx_, regs.addr, regs.nb, regs.dest))
    {
        FST_ERROR("Modbus : failed read holding register: %s", modbus_strerror(errno));
        nb_fail_++;
        return false;
    }

    return true;
}

bool ModbusTCPClient::writeHoldingRegs(ModbusRegisters& regs)
{
    if(regs.nb != modbus_write_registers(ctx_, regs.addr, regs.nb, regs.dest))
    {
        FST_ERROR("Modbus : failed write holding register: %s", modbus_strerror(errno));
        nb_fail_++;
        return false;
    }

    return true;
}

bool ModbusTCPClient::writeSingleCoil(int coil_addr, uint8_t status)
{
    if (1 != modbus_write_bit(ctx_, coil_addr, status))
    {
        FST_ERROR("Modbus : failed write single coil: %s", modbus_strerror(errno));
        nb_fail_++;
        return false;
    }
    return true;
}

bool ModbusTCPClient::writeSingleHoldingReg(int reg_addr, uint16_t value)
{
    if (1 != modbus_write_register(ctx_, reg_addr, value))
    {
        FST_ERROR("Modbus : failed write single holding register: %s", modbus_strerror(errno));
        nb_fail_++;
        return false;
    }
    return true;
}

bool ModbusTCPClient::writeAndReadHoldingRegs(ModbusRegisters& write_regs, ModbusRegisters& read_regs)
{
    read_regs.addr = write_regs.addr;
    read_regs.nb = write_regs.nb;

    if(read_regs.nb !=  modbus_write_and_read_registers(ctx_, write_regs.addr, write_regs.nb, write_regs.dest,
        read_regs.addr, read_regs.nb, read_regs.dest))
    {
        FST_ERROR("Modbus : failed write and read holding register: %s", modbus_strerror(errno));
        nb_fail_++;
        return false;
    }

    return true;
}


bool ModbusTCPClient::setErrorRecovery(modbus_error_recovery_mode error_recovery)
{
    if (modbus_set_error_recovery(ctx_, error_recovery) < 0)
    {
        FST_ERROR("Modbus : failed set error recovery mode : %s", modbus_strerror(errno));
        return false;
    }

    return true;
}


bool ModbusTCPClient::writeAndReadSingleCoil(int addr,  uint8_t& write_status, uint8_t& read_status)
{
    if(1 != writeSingleCoil(addr, write_status))
    {
        FST_ERROR("Modbus Client : error write single coil : %s", modbus_strerror(errno));
        return false;
    }

    ModbusStatus modbus_status;
    modbus_status.addr = addr;
    modbus_status.nb = 1;
    modbus_status.dest = &read_status;

    if(1 != readCoils(modbus_status)
        || write_status != read_status)
    {
        FST_ERROR("Modbus Client : Failed read single coil :%s",  modbus_strerror(errno));
        return false;
    }

    return true;
}

bool ModbusTCPClient::writeAndReadSingleHoldingReg(int addr, uint16_t& write_reg, uint16_t& read_reg)
{
    if(1 != writeSingleHoldingReg(addr, write_reg))
    {
        FST_ERROR("Modbus Client : error write single holding reg : %s", modbus_strerror(errno));
        return false;
    }

    ModbusRegisters modbus_regs;
    modbus_regs.addr = addr;
    modbus_regs.nb = 1;
    modbus_regs.dest = &read_reg;

    if(1 != readHoldingRegs(modbus_regs)
        || write_reg != read_reg)
    {
        FST_ERROR("Modbus Client : Failed read single holding reg :%s", modbus_strerror(errno));
        return false;
    }

    return true;
}

bool ModbusTCPClient::writeAndReadCoils(ModbusStatus& write_status, ModbusStatus& read_status)
{
    int nb = write_status.nb;
    if (nb != writeCoils(write_status))
    {
        FST_ERROR("Modbus Client : Failed write coils: %s", modbus_strerror(errno));
        return false;
    }

    read_status.addr = write_status.addr;
    read_status.nb = write_status.nb;

    if (nb != readCoils(read_status)
        || write_status.dest != read_status.dest)
    {
        FST_ERROR("Modbus Client : Failed read coils: %s", modbus_strerror(errno));
        return false;
    }

    return true;
}


int ModbusTCPClient::getFailedOperationNumber()
{
    return nb_fail_;
}

bool ModbusTCPClient::setSlaveId(int slave_id)
{
    if (-1 == modbus_set_slave(ctx_, slave_id))
    {
        return false;
    }

    return true;
}