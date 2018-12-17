#include "tcp_server.h"

#include "error_monitor.h"
#include "error_code.h"
#include "serverAlarmApi.h"

using namespace fst_hal;

ModbusTCPServer::ModbusTCPServer(string file_path):
    ctx_(NULL), mb_mapping_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ModbusServerParam(file_path);
    FST_LOG_INIT("ModbusTcpServer");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);

    thread_priority_ = -1;
    server_socket_ = -1;
    FD_ZERO(&refset_);
    fdmax_ = 0;

    is_running_ = false;

    port_ = -1;
    comm_type_ = "TCP";
    is_debug_ = false;
    cycle_time_ = 0;
    connection_nb_ = 0;

    reg_info_.coil.addr = 0;
    reg_info_.coil.max_nb = 0;
    reg_info_.discrepte_input.addr = 0;
    reg_info_.discrepte_input.max_nb = 0;
    reg_info_.input_reg.addr = 0;
    reg_info_.input_reg.max_nb = 0;
    reg_info_.holding_reg.addr = 0;
    reg_info_.holding_reg.max_nb = 0;

    config_.response_delay = 0;
    config_.reg_info.coil.addr = 0;
    config_.reg_info.coil.max_nb = 0;
    config_.reg_info.coil.is_valid = 0;
    config_.reg_info.discrepte_input.addr = 0;
    config_.reg_info.discrepte_input.max_nb = 0;
    config_.reg_info.discrepte_input.is_valid = 0;
    config_.reg_info.input_reg.addr = 0;
    config_.reg_info.input_reg.max_nb = 0;
    config_.reg_info.input_reg.is_valid = 0;
    config_.reg_info.holding_reg.addr = 0;
    config_.reg_info.holding_reg.max_nb = 0;
    config_.reg_info.holding_reg.is_valid = 0;

    is_enable_ = false;
    valid_reg_type_ = MODBUS_SERVER_REG_TYPE_INVALID;
}

ModbusTCPServer::ModbusTCPServer():
    ctx_(NULL), mb_mapping_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ModbusServerParam("null");
    FST_LOG_INIT("ModbusTcpServer");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);

    thread_priority_ = -1;
    server_socket_ = -1;
    FD_ZERO(&refset_);
    fdmax_ = 0;

    is_running_ = false;

    port_ = -1;
    comm_type_ = "TCP";
    is_debug_ = false;
    cycle_time_ = 0;
    connection_nb_ = 0;

    reg_info_.coil.addr = 0;
    reg_info_.coil.max_nb = 0;
    reg_info_.discrepte_input.addr = 0;
    reg_info_.discrepte_input.max_nb = 0;
    reg_info_.input_reg.addr = 0;
    reg_info_.input_reg.max_nb = 0;
    reg_info_.holding_reg.addr = 0;
    reg_info_.holding_reg.max_nb = 0;

    config_.response_delay = 0;
    config_.reg_info.coil.addr = 0;
    config_.reg_info.coil.max_nb = 0;
    config_.reg_info.coil.is_valid = 0;
    config_.reg_info.discrepte_input.addr = 0;
    config_.reg_info.discrepte_input.max_nb = 0;
    config_.reg_info.discrepte_input.is_valid = 0;
    config_.reg_info.input_reg.addr = 0;
    config_.reg_info.input_reg.max_nb = 0;
    config_.reg_info.input_reg.is_valid = 0;
    config_.reg_info.holding_reg.addr = 0;
    config_.reg_info.holding_reg.max_nb = 0;
    config_.reg_info.holding_reg.is_valid = 0;

    is_enable_ = false;
    valid_reg_type_ = MODBUS_SERVER_REG_TYPE_INVALID;
}

ModbusTCPServer::~ModbusTCPServer()
{
    this->closeServer();

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

    if (ctx_ != NULL)
    {
        modbus_free(ctx_);
        ctx_ = NULL;
    }

    if(mb_mapping_ != NULL)
    {
        modbus_mapping_free(mb_mapping_);
        mb_mapping_ = NULL;
    }
}

ErrorCode ModbusTCPServer::setConnectStatus(bool status)
{
    if (is_running_)
    {
        return MODBUS_SERVER_IS_RUNNING;
    }

    if (status == is_enable_)
        return SUCCESS;

    if (!status)
    {
        param_ptr_->is_enable_ = status;

        if (!param_ptr_->saveConnectStatus())
            return MODBUS_SERVER_SAVE_PARAM_FALIED;
        
        is_enable_ = param_ptr_->is_enable_;
        return SUCCESS;
    }

    if (!checkConnect())
    {
        return MODBUS_SERVER_CONNECT_FALIED;
    }

    param_ptr_->is_enable_ = status;

    if (!param_ptr_->saveConnectStatus())
        return MODBUS_SERVER_SAVE_PARAM_FALIED;

    is_enable_ = param_ptr_->is_enable_;

    return SUCCESS;
}

int ModbusTCPServer::getResponseDelay()
{
    return config_.response_delay;
}

bool ModbusTCPServer::getConnectStatus()
{
    return is_enable_;
}

bool ModbusTCPServer::checkConnect()
{
    return true;
}

ErrorCode ModbusTCPServer::setConfig(ModbusServerConfig config)
{
    if (is_running_)
    {
        return MODBUS_SERVER_IS_RUNNING;
    }

    if ((config.reg_info.coil.is_valid && config.reg_info.discrepte_input.is_valid)
        || (config.reg_info.coil.is_valid && config.reg_info.input_reg.is_valid)
        || (config.reg_info.coil.is_valid && config.reg_info.holding_reg.is_valid)
        || (config.reg_info.discrepte_input.is_valid && config.reg_info.input_reg.is_valid)
        || (config.reg_info.discrepte_input.is_valid && config.reg_info.holding_reg.is_valid)
        || (config.reg_info.input_reg.is_valid && config.reg_info.holding_reg.is_valid)
        || config.response_delay < 0)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    if (config.reg_info.coil.max_nb < 0
        || config.reg_info.coil.addr < reg_info_.coil.addr
        || reg_info_.coil.addr + reg_info_.coil.max_nb - 1 
            < config.reg_info.coil.max_nb + config.reg_info.coil.addr - 1)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    if (config.reg_info.discrepte_input.max_nb < 0
        || config.reg_info.discrepte_input.addr < reg_info_.discrepte_input.addr
        || reg_info_.discrepte_input.addr + reg_info_.discrepte_input.max_nb - 1 
            < config.reg_info.discrepte_input.max_nb + config.reg_info.discrepte_input.addr - 1)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    if (config.reg_info.input_reg.max_nb < 0
        || config.reg_info.input_reg.addr < reg_info_.input_reg.addr
        || reg_info_.input_reg.addr + reg_info_.input_reg.max_nb - 1 
            < config.reg_info.input_reg.max_nb + config.reg_info.input_reg.addr - 1)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    if (config.reg_info.holding_reg.max_nb < 0
        || config.reg_info.holding_reg.addr < reg_info_.holding_reg.addr
        || reg_info_.holding_reg.addr + reg_info_.holding_reg.max_nb - 1 
            < config.reg_info.holding_reg.max_nb + config.reg_info.holding_reg.addr - 1)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    param_ptr_->config_ = config;

    if (!param_ptr_->saveConfig())
    {
        return MODBUS_SERVER_SAVE_PARAM_FALIED;
    }

    config_ = param_ptr_->config_;

    return SUCCESS;
}

ModbusServerConfig ModbusTCPServer::getConfig()
{
    ModbusServerConfig config;
    config = config_;
    return config;
}

bool ModbusTCPServer::isRunning()
{
    return is_running_;
}

ErrorCode ModbusTCPServer::initParam()
{
    if(! param_ptr_->loadParam())
    {
        return MODBUS_SERVER_LOAD_PARAM_FALIED;
    }

    this->port_ = param_ptr_->port_;
    this->comm_type_ = param_ptr_->comm_type_;
    this->is_debug_ = param_ptr_->is_debug_;
    this->cycle_time_ = param_ptr_->cycle_time_;
    this->connection_nb_ = param_ptr_->connection_nb_;
    this->reg_info_ = param_ptr_->reg_info_;
    this->config_ = param_ptr_->config_;
    this->is_enable_ = param_ptr_->is_enable_;
    this->thread_priority_ = param_ptr_->thread_priority_;

    return SUCCESS;
}

ModbusRegAddrInfo ModbusTCPServer::getCoilInfo()
{
    ModbusRegAddrInfo info;
    info.addr = config_.reg_info.coil.addr;
    info.max_nb = config_.reg_info.coil.max_nb;
    info.is_valid = config_.reg_info.coil.is_valid;
    return info;
}

ModbusRegAddrInfo ModbusTCPServer::getDiscrepteInputInfo()
{
    ModbusRegAddrInfo info;
    info.addr = config_.reg_info.discrepte_input.addr;
    info.max_nb = config_.reg_info.discrepte_input.max_nb;
    info.is_valid = config_.reg_info.discrepte_input.is_valid;
    return info;
}

ModbusRegAddrInfo ModbusTCPServer::getHoldingRegInfo()
{
    ModbusRegAddrInfo info;
    info.addr = config_.reg_info.holding_reg.addr;
    info.max_nb = config_.reg_info.holding_reg.max_nb;
    info.is_valid = config_.reg_info.holding_reg.is_valid;
    return info;
}

ModbusRegAddrInfo ModbusTCPServer::getInputRegInfo()
{
    ModbusRegAddrInfo info;
    info.addr = config_.reg_info.input_reg.addr;
    info.max_nb = config_.reg_info.input_reg.max_nb;
    info.is_valid = config_.reg_info.input_reg.is_valid;
    return info;
}


bool ModbusTCPServer::loadParam()
{
    if (!param_ptr_->loadConfig())
    {
        return false;
    }

    this->port_ = param_ptr_->port_;
    this->comm_type_ = param_ptr_->comm_type_;
    this->is_debug_ = param_ptr_->is_debug_;
    this->cycle_time_ = param_ptr_->cycle_time_;
    this->connection_nb_ = param_ptr_->connection_nb_;
    this->reg_info_ = param_ptr_->reg_info_;
    this->config_ = param_ptr_->config_;
    this->is_enable_ = param_ptr_->is_enable_;

    return true;
}

ErrorCode ModbusTCPServer::init()
{
    if (!param_ptr_->loadConfig())
    {
        return MODBUS_SERVER_LOAD_PARAM_FALIED;
    }

    if (!getConnectStatus())
    {
        return MODBUS_SERVER_CONNECT_FALIED;
    }

    string server_ip = local_ip_.get();
    ctx_ = modbus_new_tcp(server_ip.c_str(), port_);

    if (ctx_ == NULL)
        return MODBUS_SERVER_INIT_FAILED;

    if (modbus_set_debug(ctx_, is_debug_) < 0)
    {
        FST_ERROR("Failed set tcp server debug : %s", modbus_strerror(errno));
        return MODBUS_SERVER_INIT_FAILED;
    }

    server_socket_ = modbus_tcp_listen(ctx_, connection_nb_);
    if (server_socket_ < 0)
    {
        FST_ERROR("Failed to listen tcp :%s", modbus_strerror(errno));
        return MODBUS_SERVER_INIT_FAILED;
    }

    FD_ZERO(&refset_);
    /* Add the server socket */
    FD_SET(server_socket_, &refset_);
    fdmax_ = server_socket_;

    mb_mapping_ = modbus_mapping_new_start_address(
        reg_info_.coil.addr, reg_info_.coil.max_nb, 
        reg_info_.discrepte_input.addr, reg_info_.discrepte_input.max_nb,
        reg_info_.holding_reg.addr, reg_info_.holding_reg.max_nb,
        reg_info_.input_reg.addr, reg_info_.input_reg.max_nb);

    if(mb_mapping_ == NULL)
    {
        FST_ERROR("Modbus Manager : failed to new mapping for modbus : %s", modbus_strerror(errno));
        return MODBUS_SERVER_INIT_FAILED;
    }

    return SUCCESS;
}

ErrorCode ModbusTCPServer::openServer()
{
    if (!checkConnect())
    {
        if (is_enable_)
        {
            setConnectStatus(false);
        }

        return MODBUS_SERVER_CONNECT_FALIED;
    }

    ErrorCode error_code = init();
    if (error_code != SUCCESS) return error_code;

    is_running_ = true;

    if(!thread_ptr_.run(&modbusTcpServerRoutineThreadFunc, this, thread_priority_))
    {
        FST_ERROR("Failed to open ModbusTcpServer");
        return MODBUS_SERVER_OPEN_FAILED;
    }

    return SUCCESS;
}

void ModbusTCPServer::closeServer()
{
    is_running_ = false;
    thread_ptr_.join();
    close(server_socket_);
    modbus_close(ctx_);

    FD_ZERO(&refset_);
    fdmax_ = 0;
    server_socket_ = -1;
    connection_nb_ = 0;

    if (ctx_ != NULL)
    {
        modbus_free(ctx_);
        ctx_ = NULL;
    }

    if(mb_mapping_ != NULL)
    {
        modbus_mapping_free(mb_mapping_);
        mb_mapping_ = NULL;
    }
}

void ModbusTCPServer::modbusTcpServerThreadFunc()
{
    if (!checkConnect())
    {
        if (is_enable_)
        {
            closeServer();
            setConnectStatus(false);
        }

        is_running_ = false;
        return; 
    }

    fd_set rdset = refset_;

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100;
    if (select(fdmax_ + 1, &rdset, NULL, NULL, &timeout) == -1) 
    {
        FST_ERROR("Server select() Failed.");
        if (server_socket_ != -1)
        {
            close(server_socket_);
        }

        usleep(cycle_time_);
        return;
    }

    /* Run through the existing connections looking for data to be read */
    for (int master_socket = 0; master_socket <= fdmax_; master_socket++)
    {
        if (!FD_ISSET(master_socket, &rdset)) continue;

        if (master_socket == server_socket_)
        {
            /* A client is asking a new connection and Handle new connections */
            struct sockaddr_in clientaddr;
            memset(&clientaddr, 0, sizeof(clientaddr));

            socklen_t addrlen = sizeof(clientaddr);

            int newfd = accept(server_socket_, (struct sockaddr *)&clientaddr, &addrlen);
            if (newfd == -1) 
            {
                FST_ERROR("Server accept() error.");
                continue;
            }

            FD_SET(newfd, &refset_);

            /* Keep track of the maximum */
            if (fdmax_ < newfd) fdmax_ = newfd;

            FST_INFO("New connection from %s:%d on socket %d\n",
                inet_ntoa(clientaddr.sin_addr), clientaddr.sin_port, newfd);
            continue;
        }

        modbus_set_socket(ctx_, master_socket);

        int rc = modbus_receive(ctx_, query_);
        if ( rc <= 0)
        {
            /* This example server in ended on connection closing or any errors. */
            FST_INFO("Connection closed on socket %d\n", master_socket);
            close(master_socket);

            /* Remove from reference set */
            FD_CLR(master_socket, &refset_);

            if (master_socket == fdmax_) fdmax_--;
            continue;
        }
        usleep(config_.response_delay);
        modbus_reply(ctx_, query_, rc, mb_mapping_);
    }

    usleep(cycle_time_);
}

ErrorCode ModbusTCPServer::writeCoils(int addr, int nb, uint8_t *dest)
{
    if (!is_running_)
        return MODBUS_SERVER_BE_NOT_OPENED;

    if (!config_.reg_info.coil.is_valid)
        return MODBUS_SERVER_FUNCTION_INVALID;

    int last_addr = addr + nb -1;
    if (last_addr < addr 
        || addr < this->config_.reg_info.coil.addr
        || this->config_.reg_info.coil.max_nb  + this->config_.reg_info.coil.addr - 1 < last_addr)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    tab_bit_mutex_.lock();
    for (unsigned int i = addr; i < addr + nb; i++)
    {
        mb_mapping_->tab_bits[i] = (*dest) & 0x01;
        dest++;
    }
    tab_bit_mutex_.unlock();

    return SUCCESS;
}

ErrorCode ModbusTCPServer::readCoils(int addr, int nb, uint8_t *dest)
{
    if (!is_running_)
        return MODBUS_SERVER_BE_NOT_OPENED;

    if (!config_.reg_info.coil.is_valid)
        return MODBUS_SERVER_FUNCTION_INVALID;

    int last_addr = addr + nb -1;
    if (last_addr < addr 
        || addr < this->config_.reg_info.coil.addr
        || this->config_.reg_info.coil.max_nb  + this->config_.reg_info.coil.addr - 1 < last_addr)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    tab_bit_mutex_.lock();
    for (unsigned int i = addr; i < addr + nb; i++)
    {
        *dest = mb_mapping_->tab_bits[i];
        dest++;
    }
    tab_bit_mutex_.unlock();
    return SUCCESS;
}

ErrorCode ModbusTCPServer::readDiscreteInputs(int addr, int nb, uint8_t *dest)
{
    if (!is_running_)
        return MODBUS_SERVER_BE_NOT_OPENED;

    if (!config_.reg_info.discrepte_input.is_valid)
        return MODBUS_SERVER_FUNCTION_INVALID;

    int last_addr = addr + nb -1;
    if (last_addr < addr 
        || addr < config_.reg_info.discrepte_input.addr
        || config_.reg_info.discrepte_input.max_nb  + config_.reg_info.discrepte_input.addr - 1 < last_addr)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    tab_input_bit_mutex_.lock();
    for (unsigned int i = addr; i < addr + nb; i++)
    {
        *dest = mb_mapping_->tab_input_bits[i];
        dest++;
    }
    tab_input_bit_mutex_.unlock();

    return SUCCESS;
}

ErrorCode ModbusTCPServer::writeHoldingRegs(int addr, int nb, uint16_t *dest)
{
    if (!is_running_)
        return MODBUS_SERVER_BE_NOT_OPENED;

    if (!config_.reg_info.holding_reg.is_valid)
        return MODBUS_SERVER_FUNCTION_INVALID;

    int last_addr = addr + nb -1;
    if (last_addr < addr 
        || addr < config_.reg_info.holding_reg.addr
        || config_.reg_info.holding_reg.max_nb  + config_.reg_info.holding_reg.addr - 1 < last_addr)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    tab_reg_mutex_.lock();
    for (unsigned int i = addr; i < addr + nb; i++)
    {
        mb_mapping_->tab_registers[i] = (*dest) & 0xffff;
        dest++;
    }
    tab_reg_mutex_.unlock();

    return SUCCESS;
}

ErrorCode ModbusTCPServer::readHoldingRegs(int addr, int nb, uint16_t *dest)
{
    if (!is_running_)
        return MODBUS_SERVER_BE_NOT_OPENED;

    if (!config_.reg_info.holding_reg.is_valid)
        return MODBUS_SERVER_FUNCTION_INVALID;

    int last_addr = addr + nb -1;
    if (last_addr < addr 
        || addr < config_.reg_info.holding_reg.addr
        || config_.reg_info.holding_reg.max_nb  + config_.reg_info.holding_reg.addr - 1 < last_addr)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    tab_reg_mutex_.lock();
    for (unsigned int i = addr; i < addr + nb; i++)
    {
        *dest = mb_mapping_->tab_registers[i];
        dest++;
    }
    tab_reg_mutex_.unlock();

    return SUCCESS;
}

ErrorCode ModbusTCPServer::readInputRegs(int addr, int nb, uint16_t *dest)
{
    if (!is_running_)
        return MODBUS_SERVER_BE_NOT_OPENED;

    if (!config_.reg_info.input_reg.is_valid)
        return MODBUS_SERVER_FUNCTION_INVALID;

    int last_addr = addr + nb -1;
    if (last_addr < addr 
        || addr < config_.reg_info.input_reg.addr
        || config_.reg_info.input_reg.max_nb  + config_.reg_info.input_reg.addr - 1 < last_addr)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    tab_input_reg_mutex_.lock();
    for (unsigned int i = addr; i < addr + nb; i++)
    {
        *dest = mb_mapping_->tab_input_registers[i];
        dest++;
    }
    tab_input_reg_mutex_.lock();

    return SUCCESS;
}

ModbusServerStartInfo ModbusTCPServer::getStartInfo()
{
    ModbusServerStartInfo info;
    info.ip = local_ip_.get();
    info.port = port_;
    return info;
}


ErrorCode ModbusTCPServer::getValidRegInfo(int reg_type, ModbusRegAddrInfo info)
{
    if (config_.reg_info.coil.is_valid)
    {
        reg_type = MODBUS_SERVER_COIL;
        info.addr = config_.reg_info.coil.addr;
        info.max_nb = config_.reg_info.coil.max_nb;
        info.is_valid = config_.reg_info.coil.is_valid;
    }
    else if (config_.reg_info.discrepte_input.is_valid)
    {
        reg_type = MODBUS_SERVER_DISCREPTE_INPUT;
        info.addr = config_.reg_info.discrepte_input.addr;
        info.max_nb = config_.reg_info.discrepte_input.max_nb;
        info.is_valid = config_.reg_info.discrepte_input.is_valid;
    }
    else if (config_.reg_info.holding_reg.is_valid)
    {
        reg_type = MODBUS_SERVER_HOLDING_REG;
        info.addr = config_.reg_info.holding_reg.addr;
        info.max_nb = config_.reg_info.holding_reg.max_nb;
        info.is_valid = config_.reg_info.holding_reg.is_valid;
    }
    else if (config_.reg_info.input_reg.is_valid)
    {
        reg_type = MODBUS_SERVER_INPUT_REG;
        info.addr = config_.reg_info.input_reg.addr;
        info.max_nb = config_.reg_info.input_reg.max_nb;
        info.is_valid = config_.reg_info.input_reg.is_valid;
    }
    else
    {
        return MODBUS_SERVER_FUNCTION_INVALID;
    }

    return SUCCESS;
}


void modbusTcpServerRoutineThreadFunc(void* arg)
{
    ModbusTCPServer* modbus_tcp_server = static_cast<ModbusTCPServer*>(arg);
    while(modbus_tcp_server->isRunning())
    {
        modbus_tcp_server->modbusTcpServerThreadFunc();
    }
}

