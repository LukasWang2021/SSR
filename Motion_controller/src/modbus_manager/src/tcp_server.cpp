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

    port_ = -1;
    comm_type_ = "TCP";
    is_debug_ = false;
    cycle_time_ = 0;
    is_running_ = false;
    server_socket_ = -1;
    connection_nb_ = -1;

    server_info_.comm_type = comm_type_;
    server_info_.ip = local_ip_.get();
    server_info_.port = port_;
    server_info_.coil.addr = 0;
    server_info_.coil.max_nb = 0;
    server_info_.discrepte_input.addr = 0;
    server_info_.discrepte_input.max_nb = 0;
    server_info_.holding_reg.addr = 0;
    server_info_.holding_reg.max_nb = 0;
    server_info_.input_reg.addr = 0;
    server_info_.input_reg.max_nb = 0;

    FD_ZERO(&refset_);
    fdmax_ = 0;
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

ServerInfo ModbusTCPServer::getInfo()
{
    ServerInfo info;
    info.comm_type = comm_type_;
    info.ip = local_ip_.get();
    info.port = port_;
    info.coil.addr = server_info_.coil.addr;
    info.coil.max_nb = server_info_.coil.max_nb;
    info.discrepte_input.addr = server_info_.discrepte_input.addr;
    info.discrepte_input.max_nb = server_info_.discrepte_input.max_nb;
    info.holding_reg.addr = server_info_.holding_reg.addr;
    info.holding_reg.max_nb = server_info_.holding_reg.max_nb;
    info.input_reg.addr = server_info_.input_reg.addr;
    info.input_reg.max_nb = server_info_.input_reg.max_nb;
    info.is_valid = is_running_;
    return info;
}

bool ModbusTCPServer::isRunning()
{
    return is_running_;
}

bool ModbusTCPServer::initParam()
{
    return param_ptr_->loadParam();
}

ErrorCode ModbusTCPServer::init()
{
    if (!param_ptr_->loadParam())
    {
        return MODBUS_SERVER_INIT_FAILED;
    }

    this->port_ = param_ptr_->port_;
    this->comm_type_ = param_ptr_->comm_type_;
    this->cycle_time_ = param_ptr_->cycle_time_;
    this->connection_nb_ = param_ptr_->connection_nb_;
    this->is_debug_ = param_ptr_->is_debug_;
    this->server_info_.coil.addr = param_ptr_->coil_addr_;
    this->server_info_.coil.max_nb = param_ptr_->coil_max_nb_;
    this->server_info_.discrepte_input.addr = param_ptr_->discrepte_input_addr_;
    this->server_info_.discrepte_input.max_nb = param_ptr_->discrepte_input_max_nb_;
    this->server_info_.holding_reg.addr = param_ptr_->holding_register_addr_;
    this->server_info_.holding_reg.max_nb = param_ptr_->holding_register_max_nb_;
    this->server_info_.input_reg.addr = param_ptr_->input_register_addr_;
    this->server_info_.input_reg.max_nb = param_ptr_->input_register_max_nb_;

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
        server_info_.coil.addr, server_info_.coil.max_nb, 
        server_info_.discrepte_input.addr, server_info_.discrepte_input.max_nb, 
        server_info_.holding_reg.addr, server_info_.holding_reg.max_nb, 
        server_info_.input_reg.addr, server_info_.input_reg.max_nb);

    if(mb_mapping_ == NULL)
    {
        FST_ERROR("Modbus Manager : failed to new mapping for modbus : %s", modbus_strerror(errno));
        return MODBUS_SERVER_INIT_FAILED;
    }

    return SUCCESS;
}

ErrorCode ModbusTCPServer::open()
{
    is_running_ = true;

    if(!thread_ptr_.run(&modbusTcpServerRoutineThreadFunc, this, 50))
    {
        FST_ERROR("Failed to open ModbusTcpServer");
        return MODBUS_SERVER_INIT_FAILED; 
    }

    return SUCCESS;
}

void ModbusTCPServer::closeServer()
{
    is_running_ = false;
    thread_ptr_.join();

    modbus_close(ctx_);

    close(server_socket_);
    FD_ZERO(&refset_);
    fdmax_ = 0;
    server_socket_ = -1;
    connection_nb_ = -1;
}

void ModbusTCPServer::modbusTcpServerThreadFunc()
{
    fd_set rdset = refset_;

    if (select(fdmax_ + 1, &rdset, NULL, NULL, NULL) == -1) 
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

        modbus_reply(ctx_, query_, rc, mb_mapping_);
    }

    usleep(cycle_time_);
}

string ModbusTCPServer::getIp()
{
    return local_ip_.get();
}

int ModbusTCPServer::getPort()
{
    return port_;
}

ModbusRegAddrInfo ModbusTCPServer::getCoilInfo()
{
    ModbusRegAddrInfo info;
    info.addr = server_info_.coil.addr;
    info.max_nb = server_info_.coil.max_nb;
    return info;
}

ModbusRegAddrInfo ModbusTCPServer::getDiscrepteInputInfo()
{
    ModbusRegAddrInfo info;
    info.addr = server_info_.discrepte_input.addr;
    info.max_nb = server_info_.discrepte_input.max_nb;
    return info;
}

ModbusRegAddrInfo ModbusTCPServer::getHoldingRegInfo()
{
    ModbusRegAddrInfo info;
    info.addr = server_info_.holding_reg.addr;
    info.max_nb = server_info_.holding_reg.max_nb;
    return info;
}

ModbusRegAddrInfo ModbusTCPServer::getInputRegInfo()
{
    ModbusRegAddrInfo info;
    info.addr = server_info_.input_reg.addr;
    info.max_nb = server_info_.input_reg.max_nb;
    return info;
}

void modbusTcpServerRoutineThreadFunc(void* arg)
{
    ModbusTCPServer* modbus_tcp_server = static_cast<ModbusTCPServer*>(arg);
    while(modbus_tcp_server->isRunning())
    {
        modbus_tcp_server->modbusTcpServerThreadFunc();
    }
}
