#include "modbus_manager_param.h"
#include "common_file_path.h"
#include "common_log.h"
#include <string>
#include <iostream>
using namespace fst_hal;
using namespace std;


ModbusManagerParam::ModbusManagerParam():
    client_file_path_(MODBUS_DIR),
    server_file_path_(MODBUS_DIR),
    file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(fst_log::MSG_LEVEL_ERROR)
{
    file_path_ += "modbus_manager.yaml";
    tcp_client_file_name_ = "";
    tcp_server_file_name_ = "";
}

bool ModbusManagerParam::loadParam()
{
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("tcp_client_file_name", tcp_client_file_name_)
        || !yaml_help_.getParam("tcp_server_file_name", tcp_server_file_name_))
    {
        cout << " Failed load modbus_manager.yaml " << endl;
        return false;
    }

    client_file_path_ += tcp_client_file_name_;
    server_file_path_ += tcp_server_file_name_;

    return true;
}

bool ModbusManagerParam::saveParam()
{
    if (!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("tcp_client_file_name", tcp_client_file_name_)
        || !yaml_help_.setParam("tcp_server_file_name", tcp_server_file_name_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus_manager.yaml " << endl;
        return false;
    }

    return true;
}
