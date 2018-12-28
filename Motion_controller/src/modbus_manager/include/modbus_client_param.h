#ifndef _MODBUS_CLIENT_PARAM_HPP
#define _MODBUS_CLIENT_PARAM_HPP

#include <string>
#include <mutex>
#include <time.h>
#include <sstream>

#include "parameter_manager/parameter_manager_param_group.h"
#include "common_file_path.h"
#include "modbus_server_param.h"
#include "modbus_server_config_param.h"
using namespace std;

namespace fst_hal
{

class ModbusClientParam
{
public:
    ModbusClientParam(string file_path);
    ~ModbusClientParam(){}

    bool loadParam();
    bool saveParam();

    int log_level_;
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

private:
    fst_parameter::ParamGroup yaml_help_;
    string file_path_;
};
}

#endif


