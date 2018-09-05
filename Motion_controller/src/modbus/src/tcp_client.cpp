#include "tcp_client.h"

#include "error_monitor.h"
#include "error_code.h"
#include "serverAlarmApi.h"

using namespace fst_modbus;

ModbusTCPClient::ModbusTCPClient(string strIP, int port):
    log_ptr_(NULL), param_ptr_(NULL)
{
    string ip = strIP ; // local_ip_.get();
    ctx_ = modbus_new_tcp(ip.c_str(), port);
    nb_fail_ = 0;

    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ModbusManagerParam();
    FST_LOG_INIT("Modbus");
    log_level_ = 1;
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
}

ModbusTCPClient::~ModbusTCPClient()
{
    modbus_close(ctx_);
    modbus_free(ctx_);
}

bool ModbusTCPClient::init()
{
    if (modbus_connect(ctx_) < 0)
    {
        FST_ERROR("Modbus : Client connect failed");
        return false;
    }

    if (!setDebug(false))
    {
        FST_ERROR("Modbus : Client debug init failed");
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
        FST_ERROR("Modbus : set socket failed, value error");
        return false;
    }

    if (modbus_set_socket(ctx_, socket) < 0)
    {
        FST_ERROR("Modbus : set socket failed");
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
        FST_ERROR("Modbus : get response timeout failed");
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
        FST_ERROR("Modbus : set response timeout failed");
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
        FST_ERROR("Modbus : get bytes timeout failed");
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
        FST_ERROR("Modbus : set bytes timeout failed");
        return false;
    }

    return true;
}

bool ModbusTCPClient::setDebug(bool flag)
{
    if (modbus_set_debug(ctx_, flag) < 0)
    {
        // get error
        FST_ERROR("Modbus : set debug failed");
        return false;
    }

    return true;
}

bool ModbusTCPClient::readDiscreteInputs(ModbusStatus& status)
{
    if (status.nb != modbus_read_input_bits(ctx_, status.addr, status.nb, status.dest))
    {
        return false;
        nb_fail_++;
    }

    return true;
}

bool ModbusTCPClient::readInputRegs(ModbusRegisters& regs)
{
    if(regs.nb != modbus_read_input_registers(ctx_, regs.addr, regs.nb, regs.dest))
    {
        return false;
        nb_fail_++;
    }
    return true;
}

bool ModbusTCPClient::readCoils(ModbusStatus& status)
{
    if (status.nb != modbus_read_bits(ctx_, status.addr, status.nb, status.dest))
    {
        return false;
        nb_fail_++;
    }

    return true;
}

bool ModbusTCPClient::writeCoils(ModbusStatus& status)
{
    if(status.nb != modbus_write_bits(ctx_, status.addr, status.nb, status.dest))
    {
        return false;
        nb_fail_++;
    }

    return true;
}

bool ModbusTCPClient::readHoldingRegs(ModbusRegisters& regs)
{
    if (regs.nb != modbus_read_registers(ctx_, regs.addr, regs.nb, regs.dest))
    {
        return false;
        nb_fail_++;
    }

    return true;
}

bool ModbusTCPClient::writeHoldingRegs(ModbusRegisters& regs)
{
    if(regs.nb != modbus_write_registers(ctx_, regs.addr, regs.nb, regs.dest))
    {
        return false;
        nb_fail_++;
    }

    return true;
}

bool ModbusTCPClient::writeSingleCoil(int coil_addr, uint8_t status)
{
    if (1 != modbus_write_bit(ctx_, coil_addr, status))
    {
        return false;
        nb_fail_++;
    }
    return true;
}

bool ModbusTCPClient::writeSingleHoldingReg(int reg_addr, uint16_t value)
{
    if (1 != modbus_write_register(ctx_, reg_addr, value))
    {
        return false;
        nb_fail_++;
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
        return false;
        nb_fail_++;
    }

    return true;
}


bool ModbusTCPClient::setErrorRecovery(int error_recovery)
{
    modbus_error_recovery_mode recovery = static_cast<modbus_error_recovery_mode>(error_recovery);

    if (modbus_set_error_recovery(ctx_, recovery) < 0)
    {
        return false;
    }

    return true;
}


bool ModbusTCPClient::writeAndReadSingleCoil(int addr,  uint8_t& write_status, uint8_t& read_status)
{
    if(1 != writeSingleCoil(addr, write_status))
    {
        FST_ERROR("Modbus Client : error write single coil. adddr = %d, value = %d", addr, write_status);
        return false;
    }

    ModbusStatus modbus_status;
    modbus_status.addr = addr;
    modbus_status.nb = 1;
    modbus_status.dest = &read_status;

    if(1 != readCoils(modbus_status)
        || write_status != read_status)
    {
        FST_ERROR("Modbus Client : Failed read single coil : addr = %d", addr);
        return false;
    }

    return true;
}

bool ModbusTCPClient::writeAndReadSingleHoldingReg(int addr, uint16_t& write_reg, uint16_t& read_reg)
{
    if(1 != writeSingleHoldingReg(addr, write_reg))
    {
        FST_ERROR("Modbus Client : error write single holding reg. adddr = %d, value = %d", addr, write_reg);
        return false;
    }

    ModbusRegisters modbus_regs;
    modbus_regs.addr = addr;
    modbus_regs.nb = 1;
    modbus_regs.dest = &read_reg;

    if(1 != readHoldingRegs(modbus_regs)
        || write_reg != read_reg)
    {
        FST_ERROR("Modbus Client : Failed read single holding reg : addr = %d", addr);
        return false;
    }

    return true;
}

bool ModbusTCPClient::writeAndReadCoils(ModbusStatus& write_status, ModbusStatus& read_status)
{
    int nb = write_status.nb;
    if (nb != writeCoils(write_status))
    {
        FST_ERROR("Modbus Client : Failed write coils: addr = %d, nb = %d", 
            write_status.addr, write_status.nb);
        return false;
    }

    read_status.addr = write_status.addr;
    read_status.nb = write_status.nb;

    if (nb != readCoils(read_status)
        || write_status.dest != read_status.dest)
    {
        FST_ERROR("Modbus Client : Failed read coils: addr = %d, nb = %d",
            read_status.addr, read_status.nb);
        return false;
    }

    return true;
}


int ModbusTCPClient::getFailedOperationNumber()
{
    return nb_fail_;
}