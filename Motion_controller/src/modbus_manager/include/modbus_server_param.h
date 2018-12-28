#ifndef _MODBUS_SERVER_PARAM_HPP
#define _MODBUS_SERVER_PARAM_HPP

#include <string>
#include <mutex>
#include <time.h>

#include "parameter_manager/parameter_manager_param_group.h"
#include "common_file_path.h"

#include "modbus_server_config_param.h"
using namespace std;

namespace fst_hal
{

class ModbusServerParam
{
public:
    ModbusServerParam(string file_path);
    ~ModbusServerParam(){}

    bool loadParam();
    bool saveParam();

    int log_level_;
    int cycle_time_;
    int thread_priority_;
    string comm_type_; 
    int port_;

    int connection_nb_;
    bool is_debug_;

    ModbusServerRegInfo reg_info_;

private:
    fst_parameter::ParamGroup yaml_help_;
    string file_path_;
};
}

#endif

