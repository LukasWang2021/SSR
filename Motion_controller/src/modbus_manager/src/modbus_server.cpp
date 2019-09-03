#include "modbus_server.h"

#include "error_monitor.h"
#include "error_code.h"
#include "serverAlarmApi.h"

using namespace fst_hal;

ModbusServer::ModbusServer(string param_file_path, string config_param_file_path):
    ctx_(NULL), mb_mapping_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ModbusServerParam(param_file_path);
    config_param_ptr_ = new ModbusServerConfigParam(config_param_file_path);
    FST_LOG_INIT("ModbusServer");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);

    server_socket_ = -1;
    fdmax_ = 0;
    FD_ZERO(&refset_);

    port_ =-1;
    connection_nb_ = 0;
    is_debug_ = false;
    comm_type_ = "";
    reg_info_.coil.addr = 0;
    reg_info_.coil.max_nb = 0;
    reg_info_.discrepte_input.addr = 0;
    reg_info_.discrepte_input.max_nb = 0;
    reg_info_.holding_reg.addr = 0;
    reg_info_.holding_reg.max_nb = 0;
    reg_info_.input_reg.addr = 0;
    reg_info_.input_reg.max_nb = 0;

    is_enable_ = false;
    config_start_info_.ip = "";
    config_start_info_.name = "";
    config_start_info_.response_delay = 0;
    config_reg_info_.coil.addr = 0;
    config_reg_info_.coil.max_nb = 0;
    config_reg_info_.discrepte_input.addr = 0;
    config_reg_info_.discrepte_input.max_nb = 0;
    config_reg_info_.holding_reg.addr = 0;
    config_reg_info_.holding_reg.max_nb = 0;
    config_reg_info_.input_reg.addr = 0;
    config_reg_info_.input_reg.max_nb = 0;

    cycle_time_ = 0;
    is_running_ = false;
    thread_priority_ = 0;
}

ModbusServer::ModbusServer():
    ctx_(NULL), mb_mapping_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ModbusServerParam("");
    config_param_ptr_ = new ModbusServerConfigParam("");
    FST_LOG_INIT("ModbusServer");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);

    server_socket_ = -1;
    fdmax_ = 0;
    FD_ZERO(&refset_);

    port_ =-1;
    connection_nb_ = 0;
    is_debug_ = false;
    comm_type_ = "";
    reg_info_.coil.addr = 0;
    reg_info_.coil.max_nb = 0;
    reg_info_.discrepte_input.addr = 0;
    reg_info_.discrepte_input.max_nb = 0;
    reg_info_.holding_reg.addr = 0;
    reg_info_.holding_reg.max_nb = 0;
    reg_info_.input_reg.addr = 0;
    reg_info_.input_reg.max_nb = 0;

    is_enable_ = false;
    config_start_info_.ip = "";
    config_start_info_.name = "";
    config_start_info_.response_delay = 0;
    config_reg_info_.coil.addr = 0;
    config_reg_info_.coil.max_nb = 0;
    config_reg_info_.discrepte_input.addr = 0;
    config_reg_info_.discrepte_input.max_nb = 0;
    config_reg_info_.holding_reg.addr = 0;
    config_reg_info_.holding_reg.max_nb = 0;
    config_reg_info_.input_reg.addr = 0;
    config_reg_info_.input_reg.max_nb = 0;

    cycle_time_ = 0;
    is_running_ = false;
    thread_priority_ = 0;
}

ModbusServer::~ModbusServer()
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
    if (config_param_ptr_ != NULL)
    {
        delete config_param_ptr_;
        config_param_ptr_ = NULL;
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

ErrorCode ModbusServer::setEnableStatus(bool status)
{
    if (is_running_)
    {
        return MODBUS_SERVER_IS_RUNNING;
    }

    if (status == is_enable_)
        return SUCCESS;

    if (!status)
    {
        config_param_ptr_->is_enable_ = status;

        if (!config_param_ptr_->saveEnableStatus())
            return MODBUS_SERVER_SAVE_PARAM_FALIED;
        
        is_enable_ = config_param_ptr_->is_enable_;
        return SUCCESS;
    }

    config_param_ptr_->is_enable_ = status;

    if (!config_param_ptr_->saveEnableStatus())
        return MODBUS_SERVER_SAVE_PARAM_FALIED;

    is_enable_ = config_param_ptr_->is_enable_;

    return SUCCESS;
}

bool ModbusServer::getEnableStatus()
{
    return is_enable_;
}

ErrorCode ModbusServer::setRegInfo(ModbusServerRegInfo config_reg_info)
{
    if (is_running_)
    {
        return MODBUS_SERVER_IS_RUNNING;
    }

    if (is_enable_)
    {
        return MODBUS_SERVER_ENABLED;
    }

    if (config_reg_info.coil.max_nb <= 0 
        && config_reg_info.discrepte_input.max_nb <= 0
        && config_reg_info.holding_reg.max_nb <= 0
        && config_reg_info.input_reg.max_nb <= 0)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    if (0 < config_reg_info.coil.max_nb)
    {
        if (config_reg_info.coil.addr < reg_info_.coil.addr
            ||  reg_info_.coil.addr + reg_info_.coil.max_nb - 1
            < config_reg_info.coil.max_nb + config_reg_info.coil.addr - 1)
        {
            return MODBUS_SERVER_INVALID_ARG;
        }

        config_param_ptr_->reg_info_.coil.addr = config_reg_info.coil.addr;
        config_param_ptr_->reg_info_.coil.max_nb = config_reg_info.coil.max_nb;
    }

    if (0 < config_reg_info.discrepte_input.max_nb)
    {
        if (config_reg_info.discrepte_input.addr < reg_info_.discrepte_input.addr
            ||  reg_info_.discrepte_input.addr + reg_info_.discrepte_input.max_nb - 1
            < config_reg_info.discrepte_input.max_nb + config_reg_info.discrepte_input.addr - 1)
        {
            return MODBUS_SERVER_INVALID_ARG;
        }

        config_param_ptr_->reg_info_.discrepte_input.addr = config_reg_info.discrepte_input.addr;
        config_param_ptr_->reg_info_.discrepte_input.max_nb = config_reg_info.discrepte_input.max_nb;
    }

    if (0 < config_reg_info.holding_reg.max_nb)
    {
        if (config_reg_info.holding_reg.addr < reg_info_.holding_reg.addr
            ||  reg_info_.holding_reg.addr + reg_info_.holding_reg.max_nb - 1
            < config_reg_info.holding_reg.max_nb + config_reg_info.holding_reg.addr - 1)
        {
            return MODBUS_SERVER_INVALID_ARG;
        }

        config_param_ptr_->reg_info_.holding_reg.addr = config_reg_info.holding_reg.addr;
        config_param_ptr_->reg_info_.holding_reg.max_nb = config_reg_info.holding_reg.max_nb;
    }

    if (0 < config_reg_info.input_reg.max_nb)
    {
        if (config_reg_info.input_reg.addr < reg_info_.input_reg.addr
            ||  reg_info_.input_reg.addr + reg_info_.input_reg.max_nb - 1
            < config_reg_info.input_reg.max_nb + config_reg_info.input_reg.addr - 1)
        {
            return MODBUS_SERVER_INVALID_ARG;
        }

        config_param_ptr_->reg_info_.input_reg.addr = config_reg_info.input_reg.addr;
        config_param_ptr_->reg_info_.input_reg.max_nb = config_reg_info.input_reg.max_nb;
    }

    if (!config_param_ptr_->saveRegInfo())
    {
        return MODBUS_SERVER_SAVE_PARAM_FALIED;
    }

    config_reg_info_ = config_param_ptr_->reg_info_;
    return SUCCESS;
}

ErrorCode ModbusServer::setStartInfo(ModbusServerStartInfo start_info)
{
    if (is_running_)
    {
        return MODBUS_SERVER_IS_RUNNING;
    }

    if (is_enable_)
    {
        return MODBUS_SERVER_ENABLED;
    }

    if (start_info.name.length() == 0
        ||start_info.response_delay < 0)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    config_param_ptr_->start_info_.name = start_info.name;
    config_param_ptr_->start_info_.response_delay = start_info.response_delay;

    if (!config_param_ptr_->saveStartInfo())
    {
        return MODBUS_SERVER_SAVE_PARAM_FALIED;
    }

    config_start_info_.name = config_param_ptr_->start_info_.name;
    config_start_info_.response_delay = config_param_ptr_->start_info_.response_delay;
    return SUCCESS;
}

ModbusServerStartInfo ModbusServer::getStartInfo()
{
    return config_start_info_;
}

ModbusServerRegInfo ModbusServer::getRegInfo()
{
    return config_reg_info_;
}

bool ModbusServer::isRunning()
{
    return is_running_;
}

ErrorCode ModbusServer::initParam()
{
    if (!param_ptr_->loadParam())
    {
        return MODBUS_SERVER_LOAD_PARAM_FALIED;
    }

    if (!config_param_ptr_->loadParam())
    {
        return MODBUS_SERVER_LOAD_PARAM_FALIED;
    }

    this->cycle_time_ = param_ptr_->cycle_time_;
    this->thread_priority_ = param_ptr_->thread_priority_;
    this->comm_type_ = param_ptr_->comm_type_;
    this->port_ = param_ptr_->port_;
    this->connection_nb_ = param_ptr_->connection_nb_;
    this->is_debug_ = param_ptr_->is_debug_;
    this->reg_info_ = param_ptr_->reg_info_;

    is_enable_ = config_param_ptr_->is_enable_;

    if (config_param_ptr_->start_info_.ip.compare("local_host") == 0)
    {
        this->config_start_info_.ip  = local_ip_.get();
    }
    else 
    {
        this->config_start_info_.ip = config_param_ptr_->start_info_.ip;
    }

    this->config_start_info_.name = config_param_ptr_->start_info_.name;
    this->config_start_info_.response_delay = config_param_ptr_->start_info_.response_delay;
    this->config_reg_info_ = config_param_ptr_->reg_info_;
    this->is_enable_ = config_param_ptr_->is_enable_;

    return SUCCESS;
}

ModbusRegAddrInfo ModbusServer::getConfigCoilInfo()
{
    ModbusRegAddrInfo info;
    info.addr = config_reg_info_.coil.addr;
    info.max_nb = config_reg_info_.coil.max_nb;
    return info;
}

ModbusRegAddrInfo ModbusServer::getConfigDiscrepteInputInfo()
{
    ModbusRegAddrInfo info;
    info.addr = config_reg_info_.discrepte_input.addr;
    info.max_nb = config_reg_info_.discrepte_input.max_nb;
    return info;
}

ModbusRegAddrInfo ModbusServer::getConfigHoldingRegInfo()
{
    ModbusRegAddrInfo info;
    info.addr = config_reg_info_.holding_reg.addr;
    info.max_nb = config_reg_info_.holding_reg.max_nb;
    return info;
}

ModbusRegAddrInfo ModbusServer::getConfigInputRegInfo()
{
    ModbusRegAddrInfo info;
    info.addr = config_reg_info_.input_reg.addr;
    info.max_nb = config_reg_info_.input_reg.max_nb;
    return info;
}

ErrorCode ModbusServer::init()
{
    if (!config_param_ptr_->loadParam())
    {
        return MODBUS_SERVER_LOAD_PARAM_FALIED;
    }

    if (!is_enable_)
    {
        return MODBUS_SERVER_DISABLED;
    }

    if (ctx_ == NULL)
        ctx_ = modbus_new_tcp(config_start_info_.ip.c_str(), port_);

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

ErrorCode ModbusServer::openServer()
{
    if (!is_enable_)
    {
        return MODBUS_SERVER_DISABLED;
    }

    ErrorCode error_code = init();
    if (error_code != SUCCESS) return error_code;

    is_running_ = true;
    if(!thread_ptr_.run(&modbusServerRoutineThreadFunc, this, thread_priority_))
    {
        FST_ERROR("Failed to open ModbusTcpServer");
        return MODBUS_SERVER_OPEN_FAILED;
    }

    return SUCCESS;
}

void ModbusServer::closeServer()
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

void ModbusServer::modbusTcpServerThreadFunc()
{
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
        usleep(config_start_info_.response_delay * 1000);
        modbus_reply(ctx_, query_, rc, mb_mapping_);
    }

    usleep(cycle_time_);
}

ErrorCode ModbusServer::writeCoils(int addr, int nb, uint8_t *dest)
{
    if (NULL == dest
        || addr + nb -1 < addr
        || addr < config_reg_info_.coil.addr
        || config_reg_info_.coil.max_nb  + config_reg_info_.coil.addr - 1 < addr + nb -1)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    if (!is_running_)
        return MODBUS_SERVER_NOT_OPENED;

    tab_bit_mutex_.lock();
    for (unsigned int i = addr; i < addr + nb; i++)
    {
        mb_mapping_->tab_bits[i] = (*dest) & 0x01;
        dest++;
    }
    tab_bit_mutex_.unlock();

    return SUCCESS;
}

ErrorCode ModbusServer::readCoils(int addr, int nb, uint8_t *dest)
{
    if (NULL == dest
        || addr + nb -1 < addr 
        || addr < config_reg_info_.coil.addr
        || config_reg_info_.coil.max_nb  + config_reg_info_.coil.addr - 1 < addr + nb -1)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    if (!is_running_)
        return MODBUS_SERVER_NOT_OPENED;

    tab_bit_mutex_.lock();
    for (unsigned int i = addr; i < addr + nb; i++)
    {
        *dest = mb_mapping_->tab_bits[i];
        dest++;
    }
    tab_bit_mutex_.unlock();
    return SUCCESS;
}

ErrorCode ModbusServer::readDiscreteInputs(int addr, int nb, uint8_t *dest)
{
    if (NULL == dest
        || addr + nb -1 < addr
        || addr < config_reg_info_.discrepte_input.addr
        || config_reg_info_.discrepte_input.max_nb  + config_reg_info_.discrepte_input.addr - 1 < addr + nb -1)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    if (!is_running_)
        return MODBUS_SERVER_NOT_OPENED;

    tab_input_bit_mutex_.lock();
    for (unsigned int i = addr; i < addr + nb; i++)
    {
        *dest = mb_mapping_->tab_input_bits[i];
        dest++;
    }
    tab_input_bit_mutex_.unlock();

    return SUCCESS;
}

ErrorCode ModbusServer::writeHoldingRegs(int addr, int nb, uint16_t *dest)
{
    if (NULL == dest
        || addr + nb -1 < addr
        || addr < config_reg_info_.holding_reg.addr
        || config_reg_info_.holding_reg.max_nb  + config_reg_info_.holding_reg.addr - 1 < addr + nb -1)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    if (!is_running_)
        return MODBUS_SERVER_NOT_OPENED;

    tab_reg_mutex_.lock();
    for (unsigned int i = addr; i < addr + nb; i++)
    {
        mb_mapping_->tab_registers[i] = (*dest) & 0xffff;
        dest++;
    }
    tab_reg_mutex_.unlock();

    return SUCCESS;
}

ErrorCode ModbusServer::readHoldingRegs(int addr, int nb, uint16_t *dest)
{
    if (NULL == dest
        || addr + nb -1 < addr 
        || addr < config_reg_info_.holding_reg.addr
        || config_reg_info_.holding_reg.max_nb  + config_reg_info_.holding_reg.addr - 1 < addr + nb -1)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    if (!is_running_)
        return MODBUS_SERVER_NOT_OPENED;

    tab_reg_mutex_.lock();
    for (unsigned int i = addr; i < addr + nb; i++)
    {
        *dest = mb_mapping_->tab_registers[i];
        dest++;
    }
    tab_reg_mutex_.unlock();

    return SUCCESS;
}

ErrorCode ModbusServer::readInputRegs(int addr, int nb, uint16_t *dest)
{
    if (NULL == dest
        || addr + nb -1 < addr 
        || addr < config_reg_info_.input_reg.addr
        || config_reg_info_.input_reg.max_nb  + config_reg_info_.input_reg.addr - 1 < addr + nb -1)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    if (!is_running_)
        return MODBUS_SERVER_NOT_OPENED;

    tab_input_reg_mutex_.lock();
    for (unsigned int i = addr; i < addr + nb; i++)
    {
        *dest = mb_mapping_->tab_input_registers[i];
        dest++;
    }
    tab_input_reg_mutex_.unlock();

    return SUCCESS;
}


ErrorCode ModbusServer::writeInputRegs(int addr, int nb, uint16_t *dest)
{
    if (NULL == dest
        || addr + nb -1 < addr 
        || addr < config_reg_info_.input_reg.addr
        || config_reg_info_.input_reg.max_nb  + config_reg_info_.input_reg.addr - 1 < addr + nb -1)
    {
        return MODBUS_SERVER_INVALID_ARG;
    }

    if (!is_running_)
        return MODBUS_SERVER_NOT_OPENED;

    tab_input_reg_mutex_.lock();
    for (unsigned int i = addr; i < addr + nb; i++)
    {
        mb_mapping_->tab_input_registers[i] = (*dest) & 0xffff;
        dest++;
    }
    tab_input_reg_mutex_.unlock();

    return SUCCESS;
}

ModbusServerConfigParams ModbusServer::getConfigParams()
{
    ModbusServerConfigParams params;
    params.is_enable = is_enable_;
    params.start_info = config_start_info_;
    params.reg_info = config_reg_info_;
    return params;
}

void modbusServerRoutineThreadFunc(void* arg)
{
    ModbusServer* modbus_server = static_cast<ModbusServer*>(arg);
    while(modbus_server->isRunning())
    {
        modbus_server->modbusTcpServerThreadFunc();
    }
}
