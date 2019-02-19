#ifndef _MODBUS_CLIENT_CONFIG_PARAM_HPP
#define _MODBUS_CLIENT_CONFIG_PARAM_HPP

#include <string>
#include <mutex>
#include <time.h>
#include <sstream>

#include "parameter_manager/parameter_manager_param_group.h"
#include "common_file_path.h"
#include "modbus_server_config_param.h"
using namespace std;

namespace fst_hal
{
struct ModbusClientStartInfo
{
    int id;
    string name;
    string ip;
    int port;
    int scan_rate;
    int response_timeout;
};

struct ModbusClientRegInfo
{
    ModbusRegAddrInfo coil;
    ModbusRegAddrInfo discrepte_input;
    ModbusRegAddrInfo holding_reg;
    ModbusRegAddrInfo input_reg;
};

struct ModbusClientConfigParams
{
    bool is_added;
    bool is_enable;
    ModbusClientRegInfo reg_info;
    ModbusClientStartInfo start_info;
};

class ModbusClientConfigParam
{
public:
    ModbusClientConfigParam(string &file_path, int client_nb);
    ~ModbusClientConfigParam(){}

    bool loadParam();
    bool saveParam();

    bool saveStartInfo(ModbusClientStartInfo &start_info); // for client element
    bool saveRegInfo(int client_id, ModbusClientRegInfo &reg_info); // for client element
    bool saveEnableStatus(int client_id, bool &status); // for client element
    bool saveIsAdded(int client_id, bool &is_added);

    bool getStartInfo(ModbusClientStartInfo &start_info); // for client element
    bool getRegInfo(int client_id, ModbusClientRegInfo &reg_info); // for client element
    bool getEnableStatus(int client_id, bool &status); // for client element
    bool getIsAdded(int client_id, bool &is_added);

    std::mutex client_config_list_mutex_;
    std::vector<ModbusClientConfigParams> client_config_list_;

private:
    fst_parameter::ParamGroup yaml_help_;
    string file_path_;
    int client_nb_;

    ModbusClientConfigParam();
};
}

#endif


