#ifndef MODBUS_CLIENT_PARAM
#define MODBUS_CLIENT_PARAM

#include <string>
#include <mutex>
#include <time.h>

#include "parameter_manager/parameter_manager_param_group.h"
#include "common_file_path.h"
#include "modbus_server_param.h"
using namespace std;

namespace fst_hal
{
struct ModbusClientConfig
{
    string name;
    int port;
    string ip;
    int response_timeout_sec;
    int response_timeout_usec;
    int bytes_timeout_sec;
    int bytes_timeout_usec;
    ModbusServerRegInfo reg_info;
};

class ModbusClientParam
{
public:
    ModbusClientParam(string file_path);
    ~ModbusClientParam(){}

    bool loadParam();
    bool saveConfig();
    bool saveScanRate();
    bool saveConnectStatus();
    bool saveId();

    int scan_rate_;
    bool is_enable_;
    int id_;
    ModbusClientConfig config_;

    int log_level_;
    string comm_type_;
    bool is_debug_;

private:
    fst_parameter::ParamGroup yaml_help_;
    string file_path_;
};
}

#endif


