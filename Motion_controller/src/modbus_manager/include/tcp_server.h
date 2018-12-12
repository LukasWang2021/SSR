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
#include "net_connection.h"

#include "modbus/modbus-private.h"
#include "modbus/modbus-tcp.h"
#include "modbus_server_param.h"
#include "local_ip.h"

using namespace std;
using namespace fst_hal;

namespace fst_hal
{

class ModbusTCPServer
{
public:
    ModbusTCPServer(string file_path);
    ~ModbusTCPServer();

    ErrorCode initParam();
    ErrorCode setConnectStatus(bool status);
    bool getConnectStatus();
    ErrorCode setConfig(ModbusServerConfig config);
    ModbusServerConfig getConfig();
    ModbusServerStartInfo getStartInfo();

    ModbusRegAddrInfo getCoilInfo();
    ModbusRegAddrInfo getDiscrepteInputInfo();
    ModbusRegAddrInfo getHoldingRegInfo();
    ModbusRegAddrInfo getInputRegInfo();

    ErrorCode openServer();
    void closeServer();

    bool isRunning();
    void modbusTcpServerThreadFunc();

    ErrorCode writeCoils(int addr, int nb, uint8_t *dest);
    ErrorCode readCoils(int addr, int nb, uint8_t *dest);
    ErrorCode readDiscreteInputs(int addr, int nb, uint8_t *dest);
    ErrorCode writeHoldingRegs(int addr, int nb, uint16_t *dest);
    ErrorCode readHoldingRegs(int addr, int nb, uint16_t *dest);
    ErrorCode readInputRegs(int addr, int nb, uint16_t *dest);

private:
    modbus_t* ctx_;
    modbus_mapping_t* mb_mapping_;

    int server_socket_;
    int fdmax_;
    fd_set refset_;

    int port_;
    int connection_nb_;
    bool is_debug_;
    bool is_enable_;
    string comm_type_;
    ModbusServerConfig config_;
    ModbusServerRegInfo reg_info_;
    fst_ip::LocalIP local_ip_;

    uint8_t query_[MODBUS_TCP_MAX_ADU_LENGTH];

    int cycle_time_;
    bool is_running_;
    fst_base::ThreadHelp thread_ptr_;

    ModbusServerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    NetConnection net_connect_;

    ErrorCode init();
    bool checkConnect();
    bool loadParam();
};
}

#endif

void modbusTcpServerRoutineThreadFunc(void* arg);
