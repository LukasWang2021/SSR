#ifndef MODBUS_MANAGER_PARAM
#define MODBUS_MANAGER_PARAM

#include <string>
#include <mutex>
#include <time.h>

#include "parameter_manager/parameter_manager_param_group.h"

using namespace std;

namespace fst_modbus
{
struct RegisterInfo //from io_mapping
{
    int coil_addr;
    int coil_nb; //
    int discrepte_input_addr;
    int discrepte_input_nb;
    int holding_register_addr;
    int holding_register_nb;
    int input_register_addr;
    int input_register_nb;
};

struct RegisterInfoOfDevice //from io_mapping
{
    int device_type;
    RegisterInfo reg_info;
};

class ModbusManagerParam
{
public:
    ModbusManagerParam();
    ~ModbusManagerParam(){}

    bool loadParam();
    bool saveParam();

    bool loadDeviceParam();
    bool saveDeviceParam();

    int log_level_;
    string server_ip_; // load yaml
    int server_port_; // load yaml
    int connection_number_;
    int device_number_;
    int cycle_time_;
    bool is_debug_;

    std::mutex device_reg_info_list_mutex_;
    std::vector<RegisterInfoOfDevice> device_reg_info_list_;

    RegisterInfo modbus_register_info_;

    timeval response_timeout_;
    timeval bytes_timeout_;
private:
    fst_parameter::ParamGroup yaml_help_;
    string device_manager_file_path_;
    string file_path_;

    enum DeviceType{ IO = 1,};
};
}

#endif


