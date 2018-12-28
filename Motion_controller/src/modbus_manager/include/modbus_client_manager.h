#ifndef _MODBUS_CLIENT_MANAGER_HPP
#define _MODBUS_CLIENT_MANAGER_HPP

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>

#include "parameter_manager/parameter_manager_param_group.h"
#include "common_log.h"

#include "modbus/modbus-private.h"
#include "modbus/modbus-tcp.h"

#include "modbus_client.h"
#include "modbus_client_param.h"
#include "modbus_client_config_param.h"

using namespace std;
using namespace fst_hal;

namespace fst_hal
{

class ModbusClientManager
{
public:
    ModbusClientManager(string param_file_path, string config_file_path);
     ~ModbusClientManager();

    ErrorCode initParam();

    ErrorCode addClient(ModbusClientStartInfo start_info);
    ErrorCode deleteClient(int client_id);

    ErrorCode setEnableStatus(int client_id, bool status);
    ErrorCode setRegInfo(int client_id, ModbusClientRegInfo reg_info);
    ErrorCode updateStartInfo(int client_id, ModbusClientStartInfo start_info);

    ErrorCode getEnableStatus(int client_id, bool status);
    ErrorCode getStartInfo(int client_id, ModbusClientStartInfo start_info);
    ErrorCode getRegInfo(int client_id, ModbusClientRegInfo reg_info);
    vector<int> getClientIdList();

    ErrorCode openClient(int client_id);
    ErrorCode closeClient(int client_id);
    ErrorCode isRunning(int client_id, bool is_running);
    vector<int> getRunningClientIdList();//CONNECTED and OPERATIONAL

    vector<int> getScanRateList();
    vector<ModbusClientConfigParams> getConfigParamsList();
    bool isAnyClientClosed();

    int getScanRate(int client_id);
    int getCtrlState(int client_id);

    ErrorCode writeCoils(int client_id, int addr, int nb, uint8_t *dest);
    ErrorCode readCoils(int client_id, int addr, int nb, uint8_t *dest);
    ErrorCode readDiscreteInputs(int client_id, int addr, int nb, uint8_t *dest);
    ErrorCode writeHoldingRegs(int client_id, int addr, int nb, uint16_t *dest);
    ErrorCode readHoldingRegs(int client_id, int addr, int nb, uint16_t *dest);
    ErrorCode writeAndReadHoldingRegs(int client_id, int write_addr, int write_nb, const uint16_t *write_dest,
        int read_addr, int read_nb, uint16_t *read_dest);
    ErrorCode readInputRegs(int client_id, int addr, int nb, uint16_t *dest);

private:
    ModbusClientParam* param_ptr_;
    ModbusClientConfigParam* config_param_ptr_;
    fst_log::Logger* log_ptr_;

    string config_param_file_path_;

    string comm_type_;
    bool is_debug_;
    int client_number_;
    int scan_rate_min_;
    int scan_rate_max_;
    int port_min_;
    int port_max_;
    int response_timeout_min_;
    int response_timeout_max_;
    int coil_addr_min_;
    int coil_addr_max_;
    int discrepte_input_min_;
    int discrepte_input_max_;
    int holding_reg_min_;
    int holding_reg_max_;
    int input_reg_min_;
    int input_reg_max_;

    std::vector<ModbusClient> client_list_;
    //std::vector<ModbusClientConfigParams> client_config_param_list_;

    bool initClientListByConfigParamList();
    ModbusClientManager();
};
}

#endif
