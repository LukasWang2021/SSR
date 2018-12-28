#ifndef _MODBUS_MANAGER_PARAM_HPP
#define _MODBUS_MANAGER_PARAM_HPP

#include <string>
#include <mutex>
#include <time.h>

#include "parameter_manager/parameter_manager_param_group.h"
#include "common_file_path.h"
using namespace std;

namespace fst_hal
{
class ModbusManagerParam
{
public:
    ModbusManagerParam();
    ~ModbusManagerParam(){}

    bool loadParam();
    bool saveParam();
    bool saveStartMode();

    int log_level_;
    int start_mode_;
    string server_file_name_;
    string server_file_path_;
    string server_config_file_name_;
    string server_config_file_path_;

    string client_file_name_;
    string client_config_file_name_;
    string client_file_path_;
    string client_config_file_path_;

private:
    fst_parameter::ParamGroup yaml_help_;
    string file_path_;

};
}

#endif


