#ifndef _MODBUS_SERVER_CONFIG_PARAM_HPP
#define _MODBUS_SERVER_CONFIG_PARAM_HPP

#include <string>
#include <mutex>
#include <time.h>

#include "parameter_manager/parameter_manager_param_group.h"
#include "common_file_path.h"
using namespace std;

namespace fst_hal
{
struct ModbusServerStartInfo
{
    string name;
    string ip;
    int response_delay;
};

struct ModbusRegAddrInfo
{
    int addr;
    int max_nb;
};

struct ModbusServerRegInfo
{
    ModbusRegAddrInfo coil;
    ModbusRegAddrInfo discrepte_input;
    ModbusRegAddrInfo holding_reg;
    ModbusRegAddrInfo input_reg;
};

class ModbusServerConfigParam
{
public:
    ModbusServerConfigParam(string file_path);
    ~ModbusServerConfigParam(){}

    bool loadParam();
    bool saveParam();

    bool saveStartInfo();
    bool saveRegInfo();
    bool saveEnableStatus();

    bool is_enable_;
    ModbusServerStartInfo start_info_;
    ModbusServerRegInfo reg_info_;

private:
    fst_parameter::ParamGroup yaml_help_;
    string file_path_;
};
}

#endif

