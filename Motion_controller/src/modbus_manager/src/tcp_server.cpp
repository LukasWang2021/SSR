#include "tcp_server.h"

#include "error_monitor.h"
#include "error_code.h"
#include "serverAlarmApi.h"

using namespace fst_modbus;

ModbusTCPServer::ModbusTCPServer(int port):
    log_ptr_(NULL), param_ptr_(NULL),
    tcp_server_file_path_(COMPONENT_PARAM_FILE_DIR),
    port_(port), server_socket_(-1),
    connection_number_(1), is_debug_(true)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ModbusManagerParam();
    FST_LOG_INIT("ModbusTcpServer");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
    tcp_server_file_path_ += "tcp_server.yaml";

    cycle_time_ = 0;
    string ip = local_ip_.get();
    ctx_ = modbus_new_tcp(ip.c_str(), port_);

    fd_max_ = 0;
    FD_ZERO(&refset_);
    FD_ZERO(&rdset_);

    server_reg_info_.coil_addr = 0;
    server_reg_info_.coil_nb = 0;
    server_reg_info_.discrepte_input_addr = 0;
    server_reg_info_.discrepte_input_nb = 0;
    server_reg_info_.holding_register_addr= 0;
    server_reg_info_.holding_register_nb= 0;
    server_reg_info_.input_register_addr= 0;
    server_reg_info_.input_register_nb= 0;
}

ModbusTCPServer::~ModbusTCPServer()
{
    this->close();

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

void ModbusTCPServer::setDebug(bool flag)
{
    is_debug_ = flag;
}

void ModbusTCPServer::setConnectionNnumber(int nb)
{
    connection_number_ = nb;
}

ServerRegInfo ModbusTCPServer::getServerRegInfo()
{
    ServerRegInfo info;
    info.coil_addr = server_reg_info_.coil_addr;
    info.coil_nb = server_reg_info_.coil_nb;
    info.discrepte_input_addr = server_reg_info_.discrepte_input_addr;
    info.discrepte_input_nb = server_reg_info_.discrepte_input_nb;
    info.holding_register_addr = server_reg_info_.holding_register_addr;
    info.holding_register_nb = server_reg_info_.holding_register_nb;
    info.input_register_addr = server_reg_info_.input_register_addr;
    info.input_register_nb = server_reg_info_.input_register_nb;
    return info;
}

bool ModbusTCPServer::mapping_new_start_address(
        unsigned int start_colis, unsigned int nb_colis,
        unsigned int start_discrete_inputs, unsigned int nb_discrete_inputs,
        unsigned int start_holding_registers, unsigned int nb_holding_registers,
        unsigned int start_input_registers, unsigned int nb_input_registers)
{
    mb_mapping_ = modbus_mapping_new_start_address(start_colis, nb_colis, 
        start_discrete_inputs, nb_discrete_inputs,
        start_holding_registers, nb_holding_registers,
        start_input_registers, nb_input_registers);

    if (NULL == mb_mapping_)
    {
        return false;
    }

    return true;
}

void ModbusTCPServer::mapping_free()
{
    modbus_mapping_free(mb_mapping_);
}

bool ModbusTCPServer::loadComponentParams()
{
    if (!tcp_server_yaml_help_.loadParamFile(tcp_server_file_path_.c_str())
        || !tcp_server_yaml_help_.getParam("cycle_time", cycle_time_)
        || !tcp_server_yaml_help_.getParam("connection_nb", connection_number_)
        || !tcp_server_yaml_help_.getParam("is_debug", is_debug_)
        || !tcp_server_yaml_help_.getParam("reg_info/coil/addr", server_reg_info_.coil_addr)
        || !tcp_server_yaml_help_.getParam("reg_info/coil/max_nb", server_reg_info_.coil_nb)
        || !tcp_server_yaml_help_.getParam("reg_info/discrepte_input/addr", server_reg_info_.discrepte_input_addr)
        || !tcp_server_yaml_help_.getParam("reg_info/discrepte_input/max_nb", server_reg_info_.discrepte_input_nb)
        || !tcp_server_yaml_help_.getParam("reg_info/input_register/addr", server_reg_info_.holding_register_addr)
        || !tcp_server_yaml_help_.getParam("reg_info/input_register/max_nb", server_reg_info_.holding_register_nb)
        || !tcp_server_yaml_help_.getParam("reg_info/holding_register/addr", server_reg_info_.input_register_addr)
        || !tcp_server_yaml_help_.getParam("reg_info/holding_register/max_nb", server_reg_info_.input_register_nb))
    {
        cout << " Failed load tcp-server.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusTCPServer::saveComponentParams()
{
    FST_ERROR("tcp server save file path = %s", tcp_server_file_path_.c_str());
    if (!tcp_server_yaml_help_.setParam("connection_nb", connection_number_)
        || !tcp_server_yaml_help_.setParam("is_debug", is_debug_)
        || !tcp_server_yaml_help_.dumpParamFile(tcp_server_file_path_.c_str()))
    {
        return false;
    }

    return true;
}

bool ModbusTCPServer::isRunning()
{
    return is_running_;
}

ErrorCode ModbusTCPServer::init()
{
#if 0
    if (!saveComponentParams())
    {
        return MODBUS_SERVER_INIT_FAILED;
    }
#endif
    if (!loadComponentParams())
    {
        return MODBUS_SERVER_INIT_FAILED;
    }

    mb_mapping_ = modbus_mapping_new_start_address(
        server_reg_info_.coil_addr, server_reg_info_.coil_nb, 
        server_reg_info_.discrepte_input_addr, server_reg_info_.discrepte_input_nb, 
        server_reg_info_.holding_register_addr, server_reg_info_.holding_register_nb, 
        server_reg_info_.input_register_addr, server_reg_info_.input_register_nb);
    if(mb_mapping_ == NULL)
    {
        FST_ERROR("Modbus Manager : failed to new mapping for modbus : %s", modbus_strerror(errno));
        return MODBUS_SERVER_INIT_FAILED;
    }

    if (modbus_set_debug(ctx_, is_debug_) < 0)
    {
        FST_ERROR("Failed set tcp server debug : %s", modbus_strerror(errno));
        return MODBUS_SERVER_INIT_FAILED;
    }

    server_socket_ = modbus_tcp_listen(ctx_, connection_number_);
    if (server_socket_ < 0)
    {
        FST_ERROR("Failed to listen tcp :%s", modbus_strerror(errno));
        return MODBUS_SERVER_INIT_FAILED;
    }

    FST_INFO("Modbus tcp server init success");
    return SUCCESS;
}

ErrorCode ModbusTCPServer::open()
{
    is_running_ = true;
    ErrorCode error_code = SUCCESS;
    if (modbus_set_socket(ctx_, server_socket_) < 0)
    {
        FST_ERROR("Modbus : Client set socket failed : %s, socket = %d",
            modbus_strerror(errno),  modbus_get_socket(ctx_));
        return MODBUS_SERVER_INIT_FAILED;
    }
    if (modbus_tcp_accept(ctx_, &server_socket_) < 0)
    {
        FST_ERROR("Modbus :unable to accept from tcp : %s", modbus_strerror(errno));
        return MODBUS_SERVER_INIT_FAILED;
    }

    if(!thread_ptr_.run(&modbusTcpServerRoutineThreadFunc, this, 50))
    {
        FST_ERROR("Failed to open ModbusTcpServer");
        return MODBUS_SERVER_INIT_FAILED; 
    }

    return SUCCESS;
}

void ModbusTCPServer::close()
{
    is_running_ = false;
    modbus_close(ctx_);
    thread_ptr_.join();
}

void ModbusTCPServer::modbusTcpServerThreadFunc()
{
    if (0 <= modbus_receive(ctx_, query_))
    {
        modbus_reply(ctx_, query_, MODBUS_TCP_MAX_ADU_LENGTH, mb_mapping_);
    }
    else
    {
        modbus_close(ctx_);
        modbus_tcp_accept(ctx_, &server_socket_);
    }

    usleep(cycle_time_);
}

void modbusTcpServerRoutineThreadFunc(void* arg)
{
    ModbusTCPServer* modbus_tcp_server = static_cast<ModbusTCPServer*>(arg);
    while(modbus_tcp_server->isRunning())
    {
        modbus_tcp_server->modbusTcpServerThreadFunc();
    }
}