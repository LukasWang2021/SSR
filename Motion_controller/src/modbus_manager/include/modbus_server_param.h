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

class ModbusServerParam
{
public:
    ModbusServerParam(string file_path);
    ~ModbusServerParam(){}

    bool loadParam();
    bool saveParam();

    int log_level_;
    int port_;
    int cycle_time_;
    int connection_nb_;
    bool is_debug_;
    string comm_type_;
    int coil_addr_;
    int coil_max_nb_;
    int discrepte_input_addr_;
    int discrepte_input_max_nb_;
    int input_register_addr_;
    int input_register_max_nb_;
    int holding_register_addr_;
    int holding_register_max_nb_;

private:
    fst_parameter::ParamGroup yaml_help_;
    string file_path_;
};
}

#endif
