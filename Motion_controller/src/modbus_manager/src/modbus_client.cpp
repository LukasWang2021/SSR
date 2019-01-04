#include "modbus_client.h"

#include "error_monitor.h"
#include "error_code.h"
#include "serverAlarmApi.h"

using namespace fst_hal;

ModbusClient::ModbusClient(int id, bool is_debug, int log_level):
    is_debug_(is_debug), log_level_(log_level)
{
    log_ptr_ = new fst_log::Logger();
    FST_LOG_INIT("ModbusClient");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)log_level_);

    ctx_ = NULL;
    socket_ = 17;
    is_connected_ = false;
    ctrl_state_ = MODBUS_CLIENT_CTRL_DISABLED;
    is_config_param_valid_ = false;

    config_param_.is_enable = false;
    config_param_.start_info.id = id;
    config_param_.start_info.name = "";
    config_param_.start_info.ip = "";
    config_param_.start_info.port = -1;
    config_param_.start_info.scan_rate = 0;
    config_param_.start_info.response_timeout = 0;
    config_param_.reg_info.coil.addr = 0;
    config_param_.reg_info.coil.max_nb = 0;
    config_param_.reg_info.discrepte_input.addr = 0;
    config_param_.reg_info.discrepte_input.max_nb = 0;
    config_param_.reg_info.holding_reg.addr = 0;
    config_param_.reg_info.holding_reg.max_nb = 0;
    config_param_.reg_info.input_reg.addr = 0;
    config_param_.reg_info.input_reg.max_nb = 0;
}

ModbusClient::ModbusClient():
    is_debug_(false), log_level_(1)
{
    log_ptr_ = new fst_log::Logger();
    FST_LOG_INIT("ModbusClient");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)log_level_);

    ctx_ = NULL;
    socket_ = 17;
    is_connected_ = false;
    ctrl_state_ = MODBUS_CLIENT_CTRL_DISABLED;
    is_config_param_valid_ = false;

    config_param_.is_enable = false;
    config_param_.start_info.id = 0;
    config_param_.start_info.name = "";
    config_param_.start_info.ip = "";
    config_param_.start_info.port = -1;
    config_param_.start_info.scan_rate = 0;
    config_param_.start_info.response_timeout = 0;
    config_param_.reg_info.coil.addr = 0;
    config_param_.reg_info.coil.max_nb = 0;
    config_param_.reg_info.discrepte_input.addr = 0;
    config_param_.reg_info.discrepte_input.max_nb = 0;
    config_param_.reg_info.holding_reg.addr = 0;
    config_param_.reg_info.holding_reg.max_nb = 0;
    config_param_.reg_info.input_reg.addr = 0;
    config_param_.reg_info.input_reg.max_nb = 0;
}

ModbusClient::~ModbusClient()
{
    if (log_ptr_ != NULL)
    {
        delete log_ptr_;
        log_ptr_ = NULL;
    }

    if (ctx_ != NULL)
    {
        modbus_free(ctx_);
        ctx_ = NULL;
    }
}

ErrorCode ModbusClient::setEnableStatus(bool &status)
{
    if (is_connected_)
    {
        return MODBUS_CLIENT_CONNECTED;
    }

    if (status == config_param_.is_enable)
    {
        return SUCCESS;
    }

    if (!status) //status = false; is_enable = true
    {
        config_param_.is_enable = status;
        ctrl_state_ = MODBUS_CLIENT_CTRL_DISABLED;
        return SUCCESS;
    }

    // status = true; is_enable = false
    if (!checkConfigParamValid())
    {
        return MODBUS_CLIENT_ENABLE_FAILED;
    }

    config_param_.is_enable = status;
    ctrl_state_ = MODBUS_CLIENT_CTRL_ENABLED;
    return SUCCESS;
}

bool ModbusClient::getEnableStatus()
{
    return config_param_.is_enable;
}

bool ModbusClient::checkConfigParamValid()
{
    if (config_param_.start_info.name.length() == 0
        || config_param_.start_info.ip.length() == 0
        || config_param_.start_info.port < 0
        || config_param_.start_info.scan_rate < 0
        || config_param_.start_info.response_timeout < 0)
    {
        return false;
    }

    if (0 < config_param_.reg_info.coil.max_nb && config_param_.reg_info.coil.addr < 1)
    {
        return false;
    }

    if (0 < config_param_.reg_info.discrepte_input.max_nb && config_param_.reg_info.discrepte_input.addr < 1)
    {
        return false;
    }

    if (0 < config_param_.reg_info.input_reg.max_nb && config_param_.reg_info.input_reg.addr < 1)
    {
        return false;
    }

    if (0 < config_param_.reg_info.holding_reg.max_nb && config_param_.reg_info.holding_reg.addr < 1)
    {
        return false;
    }

    return true;
}

ErrorCode ModbusClient::setStartInfo(ModbusClientStartInfo &start_info)
{
    if (start_info.port <= 0
        || start_info.name.length() == 0
        || start_info.ip.length() == 0
        || start_info.scan_rate < 0
        || start_info.response_timeout < 0)
    {
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if (config_param_.is_enable)
    {
        return MODBUS_CLIENT_ENABLED;
    }

    config_param_.start_info.ip = start_info.ip;
    config_param_.start_info.port = start_info.port;
    config_param_.start_info.name = start_info.name;
    config_param_.start_info.scan_rate = start_info.scan_rate;
    config_param_.start_info.response_timeout = start_info.response_timeout;
    return SUCCESS;
}

ModbusClientStartInfo ModbusClient::getStartInfo()
{
    return config_param_.start_info;
}

ErrorCode ModbusClient::setRegInfo(ModbusClientRegInfo &reg_info)
{
    if (config_param_.is_enable)
    {
        return MODBUS_CLIENT_ENABLED;
    }

    if (reg_info.coil.addr < 0
        || reg_info.coil.max_nb < 0
        || reg_info.discrepte_input.addr < 0
        || reg_info.discrepte_input.max_nb < 0
        || reg_info.input_reg.addr < 0
        || reg_info.input_reg.max_nb < 0 
        || reg_info.holding_reg.addr < 0
        || reg_info.holding_reg.max_nb < 0)
    {
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if (0 < config_param_.reg_info.coil.max_nb && config_param_.reg_info.coil.addr < 1)
    {
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if (0 < config_param_.reg_info.discrepte_input.max_nb && config_param_.reg_info.discrepte_input.addr < 1)
    {
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if (0 < config_param_.reg_info.input_reg.max_nb && config_param_.reg_info.input_reg.addr < 1)
    {
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if (0 < config_param_.reg_info.holding_reg.max_nb && config_param_.reg_info.holding_reg.addr < 1)
    {
        return MODBUS_CLIENT_INVALID_ARG;
    }

    config_param_.reg_info = reg_info;
    return SUCCESS;
}

ModbusClientRegInfo ModbusClient::getRegInfo()
{
    ModbusClientRegInfo reg_info;
    reg_info = config_param_.reg_info;
    return reg_info;
}

int ModbusClient::getScanRate()
{
    return config_param_.start_info.scan_rate;
}

int ModbusClient::getId()
{
    return config_param_.start_info.id;
}

int ModbusClient::getCtrlState()
{
    return ctrl_state_;
}

ModbusClientConfigParams ModbusClient::getConfigParams()
{
    return config_param_;
}

ErrorCode ModbusClient::init()
{
    if (ctx_ == NULL)
        ctx_ = modbus_new_tcp(config_param_.start_info.ip.c_str(), config_param_.start_info.port);
    if(ctx_ == NULL) return MODBUS_CLIENT_INIT_FAILED;

    if (modbus_set_error_recovery(ctx_, MODBUS_ERROR_RECOVERY_LINK) < 0)
    {
        //FST_ERROR("Failed to set error recovery : %s", modbus_strerror(errno));
        return MODBUS_CLIENT_INIT_FAILED;
    }

    int response_timeout_sec = config_param_.start_info.response_timeout / 1000;
    int response_timeout_usec = (config_param_.start_info.response_timeout % 1000) * 1000;

    if (modbus_set_response_timeout(ctx_, response_timeout_sec, response_timeout_usec) < 0)
    {
        //FST_ERROR("Falied set response timeout : %s", modbus_strerror(errno));
        return MODBUS_CLIENT_INIT_FAILED;
    }

    if (modbus_set_byte_timeout(ctx_, response_timeout_sec, response_timeout_usec) < 0)
    {
        //FST_ERROR("Falied set response timeout : %s", modbus_strerror(errno));
        return MODBUS_CLIENT_INIT_FAILED;
    }

    if (modbus_set_debug(ctx_, is_debug_) < 0)
    {
        //FST_ERROR("Failed to set debug : %s", modbus_strerror(errno));
        return MODBUS_CLIENT_INIT_FAILED;
    }

    return SUCCESS;
}

ErrorCode ModbusClient::connect()
{
    if (!config_param_.is_enable) return MODBUS_CLIENT_DISABLED;
    if (is_connected_) return SUCCESS;

    ErrorCode error_code = init();
    if (error_code != SUCCESS) return error_code;

    if (modbus_connect(ctx_) < 0)
    {
         FST_ERROR("Failed to connect socket : %s, socket = %d",
            modbus_strerror(errno),  modbus_get_socket(ctx_));

        is_connected_ = false;
        return MODBUS_CLIENT_CONNECT_FAILED;
    }

    ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
    is_connected_ = true;

    return SUCCESS;
}

bool ModbusClient::isSocketValid()
{
    if (modbus_get_socket(ctx_) < 0) return false;
    return true;
}

bool ModbusClient::scanDataArea()
{
    int value_nb = 0;
    if(config_param_.reg_info.coil.max_nb < config_param_.reg_info.discrepte_input.max_nb)
    {
        value_nb = config_param_.reg_info.discrepte_input.max_nb;
    }
    else
    {
        value_nb = config_param_.reg_info.coil.max_nb;
    }

    uint8_t bit_value[value_nb];

    if (0 < config_param_.reg_info.coil.max_nb)
    {
        ErrorCode error_code = readCoils(config_param_.reg_info.coil.addr, 
            config_param_.reg_info.coil.max_nb, bit_value);
        if (error_code == MODBUS_CLIENT_OPERATION_FAILED) return false;
    }

    if (0 < config_param_.reg_info.discrepte_input.max_nb)
    {
        ErrorCode error_code = readDiscreteInputs(config_param_.reg_info.discrepte_input.addr, 
            config_param_.reg_info.discrepte_input.max_nb, bit_value);
        if (error_code == MODBUS_CLIENT_OPERATION_FAILED) return false;
    }

    if(config_param_.reg_info.holding_reg.max_nb < config_param_.reg_info.input_reg.max_nb)
    {
        value_nb = config_param_.reg_info.input_reg.max_nb;
    }
    else
    {
        value_nb = config_param_.reg_info.holding_reg.max_nb;
    }

    uint16_t reg_value[value_nb];
    int reg_read_times = config_param_.reg_info.holding_reg.max_nb / REGISTER_ONE_OP_NUM;
    int reg_read_left_nb = config_param_.reg_info.holding_reg.max_nb % REGISTER_ONE_OP_NUM;

    for (int i = 0; i != reg_read_times; ++i)
    {
        ErrorCode error_code = readHoldingRegs(config_param_.reg_info.holding_reg.addr, 
            REGISTER_ONE_OP_NUM, reg_value);
        if (error_code == MODBUS_CLIENT_OPERATION_FAILED) return false;
    }

    if (reg_read_left_nb != 0)
    {
        ErrorCode error_code = readHoldingRegs(config_param_.reg_info.holding_reg.addr, 
            reg_read_left_nb, reg_value);
        if (error_code == MODBUS_CLIENT_OPERATION_FAILED) return false;
    }

    reg_read_times = config_param_.reg_info.input_reg.max_nb / REGISTER_ONE_OP_NUM;
    reg_read_left_nb = config_param_.reg_info.input_reg.max_nb % REGISTER_ONE_OP_NUM;

    for (int i = 0; i != reg_read_times; ++i)
    {
        ErrorCode error_code = readInputRegs(config_param_.reg_info.input_reg.addr, 
            REGISTER_ONE_OP_NUM, reg_value);
        if (error_code == MODBUS_CLIENT_OPERATION_FAILED) return false;
    }

    if (reg_read_left_nb != 0)
    {
        ErrorCode error_code = readInputRegs(config_param_.reg_info.input_reg.addr,
            reg_read_left_nb, reg_value);
        if (error_code == MODBUS_CLIENT_OPERATION_FAILED) return false;
    }

    return true;
}

void ModbusClient::close()
{
    if (!is_connected_) return;
    modbus_close(ctx_);

    if (ctx_ != NULL)
    {
        modbus_free(ctx_);
        ctx_ = NULL;
    }

    ctrl_state_ = MODBUS_CLIENT_CTRL_ENABLED;
    is_connected_ = false;
}


bool ModbusClient::isConnected()
{
    return is_connected_;
}

ErrorCode ModbusClient::readDiscreteInputs(int addr, int nb, uint8_t *dest)
{
    if (!is_connected_)
    {
        return MODBUS_CLIENT_NOT_CONNECT;
    }

    if (STATUS_ONE_OP_NUM < nb || nb <= 0) 
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if (addr < config_param_.reg_info.discrepte_input.addr
        || (config_param_.reg_info.discrepte_input.addr + config_param_.reg_info.discrepte_input.max_nb - 1) < (addr + nb -1))
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if (nb != modbus_read_input_bits(ctx_, addr, nb, dest))
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_OPERATION_FAILED;
    }

    ctrl_state_ = MODBUS_CLIENT_CTRL_OPERATIONAL;
    return SUCCESS;
}

ErrorCode ModbusClient::readInputRegs(int addr, int nb, uint16_t *dest)
{
    if (!is_connected_)
    {
        return MODBUS_CLIENT_NOT_CONNECT;
    }

    if (STATUS_ONE_OP_NUM < nb || nb <= 0)
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if (addr < config_param_.reg_info.input_reg.addr
        || (config_param_.reg_info.input_reg.addr + config_param_.reg_info.input_reg.max_nb - 1) < (addr + nb -1))
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if(nb != modbus_read_input_registers(ctx_, addr, nb, dest))
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_OPERATION_FAILED;
    }

    ctrl_state_ = MODBUS_CLIENT_CTRL_OPERATIONAL;
    return SUCCESS;
}

ErrorCode ModbusClient::readCoils(int addr, int nb, uint8_t *dest)
{
    if (!is_connected_)
    {
        return MODBUS_CLIENT_NOT_CONNECT;
    }

    if (STATUS_ONE_OP_NUM < nb || nb <= 0)
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if (addr < config_param_.reg_info.coil.addr
        || (config_param_.reg_info.coil.addr + config_param_.reg_info.coil.max_nb - 1) < (addr + nb -1))
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if (nb != modbus_read_bits(ctx_, addr, nb, dest))
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_OPERATION_FAILED;
    }

    ctrl_state_ = MODBUS_CLIENT_CTRL_OPERATIONAL;
    return SUCCESS;
}

ErrorCode ModbusClient::writeCoils(int addr, int nb, uint8_t *dest)
{
    if (!is_connected_)
    {
        return MODBUS_CLIENT_NOT_CONNECT;
    }

    if (STATUS_ONE_OP_NUM < nb || nb <= 0)
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if (addr < config_param_.reg_info.coil.addr
        || (config_param_.reg_info.coil.addr + config_param_.reg_info.coil.max_nb - 1) < (addr + nb -1))
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if(nb != modbus_write_bits(ctx_, addr, nb, dest))
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_OPERATION_FAILED;
    }

    ctrl_state_ = MODBUS_CLIENT_CTRL_OPERATIONAL;
    return SUCCESS;
}


ErrorCode ModbusClient::readHoldingRegs(int addr, int nb, uint16_t *dest)
{
    if (!is_connected_)
    {
        return MODBUS_CLIENT_NOT_CONNECT;
    }

    if (STATUS_ONE_OP_NUM < nb || nb <= 0)
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if (addr < config_param_.reg_info.holding_reg.addr
        || (config_param_.reg_info.holding_reg.addr + config_param_.reg_info.holding_reg.max_nb - 1) < (addr + nb -1))
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if (nb != modbus_read_registers(ctx_, addr, nb, dest))
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_OPERATION_FAILED;
    }

    ctrl_state_ = MODBUS_CLIENT_CTRL_OPERATIONAL;
    return SUCCESS;
}

ErrorCode ModbusClient::writeHoldingRegs(int addr, int nb, uint16_t *dest)
{
    if (!is_connected_)
    {
        return MODBUS_CLIENT_NOT_CONNECT;
    }

    if (STATUS_ONE_OP_NUM < nb || nb <= 0)
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if (addr < config_param_.reg_info.holding_reg.addr
        || (config_param_.reg_info.holding_reg.addr + config_param_.reg_info.holding_reg.max_nb - 1) < (addr + nb -1))
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if(nb != modbus_write_registers(ctx_, addr, nb, dest))
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_OPERATION_FAILED;
    }

    ctrl_state_ = MODBUS_CLIENT_CTRL_OPERATIONAL;
    return SUCCESS;
}

ErrorCode ModbusClient::writeAndReadHoldingRegs(
        int write_addr, int write_nb, const uint16_t *write_dest,
        int read_addr, int read_nb, uint16_t *read_dest)
{
    if (!is_connected_)
    {
        return MODBUS_CLIENT_NOT_CONNECT;
    }

    if (REGISTER_ONE_OP_NUM < write_nb
    || REGISTER_ONE_OP_NUM < read_nb
    || write_nb <= 0
    || read_nb <= 0)
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if (write_addr < config_param_.reg_info.holding_reg.addr
        || (config_param_.reg_info.holding_reg.addr + config_param_.reg_info.holding_reg.max_nb - 1) < (write_addr + write_nb -1)
        || read_addr < config_param_.reg_info.holding_reg.addr
        || (config_param_.reg_info.holding_reg.addr + config_param_.reg_info.holding_reg.max_nb - 1) < (read_addr + read_nb -1))
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_INVALID_ARG;
    }

    if(read_nb != modbus_write_and_read_registers(ctx_,
        write_addr, write_nb, write_dest, read_addr, read_nb, read_dest))
    {
        ctrl_state_ = MODBUS_CLIENT_CTRL_CONNECTED;
        return MODBUS_CLIENT_OPERATION_FAILED;
    }

    ctrl_state_ = MODBUS_CLIENT_CTRL_OPERATIONAL;
    return SUCCESS;
}


#if 0
void modbusClientRoutineThreadFunc(void* arg)
{
    ModbusClient* modbus_client = static_cast<ModbusClient*>(arg);
    while(modbus_client->isConnected())
    {
        modbus_client->modbusTcpClientThreadFunc();
    }
}
#endif
