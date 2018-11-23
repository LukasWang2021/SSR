#ifndef MODBUS_MANAGER_PARAM
#define MODBUS_MANAGER_PARAM

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
    
    int log_level_;
    string tcp_server_file_name_;
    string tcp_client_file_name_;

    string client_file_path_;
    string server_file_path_;

private:
    fst_parameter::ParamGroup yaml_help_;
    string file_path_;

};
}

#endif


