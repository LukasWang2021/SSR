#ifndef MODBUS_SERVER_PARAM
#define MODBUS_SERVER_PARAM

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
    string ip;
    int port;
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

struct ModbusServerConfig
{
    int response_delay;
    ModbusServerRegInfo reg_info;
};

class ModbusServerParam
{
public:
    ModbusServerParam(string file_path);
    ~ModbusServerParam(){}

    bool loadParam();

    bool loadConfig();
    bool saveConfig();

    bool saveConnectStatus();

    int log_level_;
    int port_;
    int cycle_time_;
    int connection_nb_;
    bool is_debug_;
    bool is_enable_;
    string comm_type_;
    ModbusServerRegInfo reg_info_;
    ModbusServerConfig config_;

private:
    fst_parameter::ParamGroup yaml_help_;
    string file_path_;
};
}

#endif

