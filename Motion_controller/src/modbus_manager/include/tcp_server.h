#ifndef _MODBUS_TCP_SERVER_HPP
#define _MODBUS_TCP_SERVER_HPP

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "parameter_manager/parameter_manager_param_group.h"
#include "common_log.h"
#include "thread_help.h"

#include "modbus/modbus-private.h"
#include "modbus/modbus-tcp.h"
#include "modbus_manager_param.h"
#include "local_ip.h"

using namespace std;
using namespace fst_modbus;

namespace fst_modbus
{
struct ServerRegInfo
{
    int coil_addr;
    int coil_nb; //
    int discrepte_input_addr;
    int discrepte_input_nb;
    int holding_register_addr;
    int holding_register_nb;
    int input_register_addr;
    int input_register_nb;
};

class ModbusTCPServer
{
public:
    ModbusTCPServer(int port);
     ~ModbusTCPServer();

    void setDebug(bool flag);
    void setConnectionNnumber(int nb);
    ServerRegInfo getServerRegInfo();

    bool isRunning();
    void modbusTcpServerThreadFunc();

    ErrorCode init();
    ErrorCode open();
    void close();

    bool saveComponentParams();

private:
    modbus_t* ctx_;
    int port_;
    fst_ip::LocalIP local_ip_;

    int server_socket_;
    int connection_number_;
    bool is_debug_;

    int fd_max_;
    fd_set refset_;
    fd_set rdset_;

    modbus_mapping_t* mb_mapping_;
    uint8_t query_[MODBUS_TCP_MAX_ADU_LENGTH];
    ServerRegInfo server_reg_info_;

    ModbusManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    fst_parameter::ParamGroup tcp_server_yaml_help_;
    std::string tcp_server_file_path_;

    int cycle_time_;
    bool is_running_;
    fst_base::ThreadHelp thread_ptr_;

    bool loadComponentParams();

    bool mapping_new_start_address(
        unsigned int start_colis, unsigned int nb_colis,
        unsigned int start_discrete_inputs, unsigned int nb_discrete_inputs,
        unsigned int start_holding_registers, unsigned int nb_holding_registers,
        unsigned int start_input_registers, unsigned int nb_input_registers);

    void mapping_free();
};
}

#endif

void modbusTcpServerRoutineThreadFunc(void* arg);
