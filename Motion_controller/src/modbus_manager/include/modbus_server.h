#ifndef _MODBUS_SERVER_HPP
#define _MODBUS_SERVER_HPP

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
#include <mutex>

#include "parameter_manager/parameter_manager_param_group.h"
#include "common_log.h"
#include "thread_help.h"
#include "net_connection.h"

#include "modbus/modbus-private.h"
#include "modbus/modbus-tcp.h"
#include "modbus_server_param.h"
#include "modbus_server_config_param.h"
#include "local_ip.h"

using namespace std;
using namespace fst_hal;

namespace fst_hal
{
struct ModbusServerConfigParams
{
    ModbusServerStartInfo start_info;
    ModbusServerRegInfo reg_info;
    bool is_enable;
};

enum {MODBUS_SERVER_COIL = 1, MODBUS_SERVER_DISCREPTE_INPUT = 2,
    MODBUS_SERVER_HOLDING_REG = 4, MODBUS_SERVER_INPUT_REG = 3, MODBUS_SERVER_REG_TYPE_INVALID = 0};

class ModbusServer
{
public:
    ModbusServer(string param_file_path, string config_param_file_path);
    ~ModbusServer();

    ErrorCode initParam();

    ErrorCode setStartInfo(ModbusServerStartInfo start_info);
    ModbusServerStartInfo getStartInfo();

    ErrorCode setRegInfo(ModbusServerRegInfo config_reg_info);
    ModbusServerRegInfo getRegInfo();

    ErrorCode setEnableStatus(bool status);
    bool getEnableStatus();

    ModbusServerConfigParams getConfigParams();

    ModbusRegAddrInfo getConfigCoilInfo();
    ModbusRegAddrInfo getConfigDiscrepteInputInfo();
    ModbusRegAddrInfo getConfigHoldingRegInfo();
    ModbusRegAddrInfo getConfigInputRegInfo();

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
    ErrorCode writeInputRegs(int addr, int nb, uint16_t *dest);
private:
    modbus_t* ctx_;
    modbus_mapping_t* mb_mapping_;
    uint8_t query_[MODBUS_TCP_MAX_ADU_LENGTH];

    int server_socket_;
    int fdmax_;
    fd_set refset_;

    int port_;
    int connection_nb_;
    bool is_debug_;
    string comm_type_;
    ModbusServerRegInfo reg_info_;

    bool is_enable_;
    ModbusServerStartInfo config_start_info_;
    ModbusServerRegInfo config_reg_info_;

    int cycle_time_;
    bool is_running_;
    int thread_priority_;
    fst_base::ThreadHelp thread_ptr_;

    ModbusServerParam* param_ptr_;
    ModbusServerConfigParam* config_param_ptr_;
    fst_log::Logger* log_ptr_;
    NetConnection net_connect_;
    fst_ip::LocalIP local_ip_;

    std::mutex tab_bit_mutex_;
    std::mutex tab_input_bit_mutex_;
    std::mutex tab_reg_mutex_;
    std::mutex tab_input_reg_mutex_;

    ErrorCode init();
    ModbusServer();
};
}

#endif

void* modbusServerRoutineThreadFunc(void* arg);