#ifndef MODBUS_CLIENT_PARAM
#define MODBUS_CLIENT_PARAM

#include <string>
#include <mutex>
#include <time.h>

#include "parameter_manager/parameter_manager_param_group.h"
#include "common_file_path.h"
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

    bool saveIp();
    bool savePort();
    bool saveResponseTimeoutParam();
    bool saveBytesTimeoutParam();

    int log_level_;
    string ip_;
    int port_;
    string comm_type_;
    bool is_debug_;
    timeval response_timeout_;
    timeval bytes_timeout_;

private:
    fst_parameter::ParamGroup yaml_help_;
    string file_path_;
};
}

#endif



