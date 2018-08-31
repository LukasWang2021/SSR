#include "tcp_server.h"

#include "error_monitor.h"
#include "error_code.h"
#include "serverAlarmApi.h"

using namespace fst_modbus;

ModbusTCPServer::ModbusTCPServer(int port):
    log_ptr_(NULL), param_ptr_(NULL)
{
    string ip = local_ip_.get();
    ctx_ = modbus_new_tcp(ip.c_str(), port);

    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ModbusManagerParam();
    FST_LOG_INIT("Modbus");

    log_level_ = 1;
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
}

ModbusTCPServer::~ModbusTCPServer()
{
    modbus_close(ctx_);
    modbus_free(ctx_);
}

bool ModbusTCPServer::init(int nb_connection)
{
    if (modbus_connect(ctx_) < 0)
    {
        FST_ERROR("Modbus : connect failed");
        return false;
    }

    if (!setDebug(false)) return false;

    server_socket_ = modbus_tcp_listen(ctx_, nb_connection);
    if (server_socket_ < 0)
    {
        FST_ERROR("Modbus :unable to listen tcp!");
        return false;
    }

    if (modbus_tcp_accept(ctx_, &server_socket_) < 0)
    {
        FST_ERROR("Modbus :unable to accept from tcp!");
        return false;
    }

    return true;
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

bool ModbusTCPServer::mapping_new(
    unsigned int nb_colis, unsigned int nb_discrete_inputs,
    unsigned int nb_holding_registers, unsigned int nb_input_registers)
{
    int number_coils = static_cast<int>(nb_colis);
    int number_discrete_inputs = static_cast<int>(nb_discrete_inputs);
    int number_holding_registers = static_cast<int>(nb_holding_registers);
    int number_input_registers = static_cast<int>(nb_input_registers);

    mb_mapping_ = modbus_mapping_new(number_coils, number_discrete_inputs,
        number_holding_registers, number_input_registers);

    if (NULL == mb_mapping_)
    {
        FST_ERROR("Failed mapping:%s", modbus_strerror(errno));
        return false;
    }

    return true;
}

void ModbusTCPServer::mapping_free()
{
    modbus_mapping_free(mb_mapping_);
}

int ModbusTCPServer::receive(uint8_t *req)
{
    return modbus_receive(ctx_, req);
}

int ModbusTCPServer::reply(const uint8_t *req, int req_length)
{
    return modbus_reply(ctx_, req, req_length, mb_mapping_);
}

bool ModbusTCPServer::setDebug(bool flag)
{
    if (modbus_set_debug(ctx_, flag) < 0)
    {
        // get error
        FST_ERROR("Modbus : set debug failed");
        return false;
    }

    return true;
}

int ModbusTCPServer::listen(int nb_connection)
{
    return modbus_tcp_listen(ctx_, nb_connection);
}

int ModbusTCPServer::accept()
{
    return modbus_tcp_accept(ctx_, &server_socket_);
}

void ModbusTCPServer::close()
{
    modbus_close(ctx_);
}