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
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "parameter_manager/parameter_manager_param_group.h"
#include "common_log.h"
#include "thread_help.h"

#include "modbus/modbus-private.h"
#include "modbus/modbus-tcp.h"
#include "modbus_server_param.h"
#include "local_ip.h"

using namespace std;
using namespace fst_hal;

namespace fst_hal
{
struct ModbusRegAddrInfo
{
    int addr;
    int max_nb;
};

struct ServerInfo
{
    string comm_type;
    string ip;
    int port;
    ModbusRegAddrInfo coil;
    ModbusRegAddrInfo discrepte_input;
    ModbusRegAddrInfo holding_reg;
    ModbusRegAddrInfo input_reg;
};

class ModbusTCPServer
{
public:
    ModbusTCPServer(string file_path);
    ~ModbusTCPServer();

    ServerInfo getInfo();
    ModbusRegAddrInfo getCoilInfo();
    ModbusRegAddrInfo getDiscrepteInputInfo();
    ModbusRegAddrInfo getHoldingRegInfo();
    ModbusRegAddrInfo getInputRegInfo();
    string getIp();
    int getPort();

    ErrorCode init();
    ErrorCode open();
    void closeServer();

    bool isRunning();
    void modbusTcpServerThreadFunc();

    bool initParam();

private:
    modbus_t* ctx_;
    modbus_mapping_t* mb_mapping_;

    int port_;
    int server_socket_;
    int connection_nb_;
    bool is_debug_;
    string comm_type_;
    ServerInfo server_info_;
    fst_ip::LocalIP local_ip_;

    uint8_t query_[MODBUS_TCP_MAX_ADU_LENGTH];

    ModbusServerParam* param_ptr_;
    fst_log::Logger* log_ptr_;

    int cycle_time_;
    bool is_running_;
    fst_base::ThreadHelp thread_ptr_;

    int fdmax_;
    fd_set refset_;

};
}

#endif

void modbusTcpServerRoutineThreadFunc(void* arg);
