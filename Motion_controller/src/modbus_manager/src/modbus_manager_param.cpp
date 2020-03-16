#include "modbus_manager_param.h"
#include "common_file_path.h"
#include "common_log.h"
#include <string>
#include <iostream>
using namespace fst_hal;
using namespace std;


ModbusManagerParam::ModbusManagerParam():
    log_level_(fst_log::MSG_LEVEL_ERROR),
    server_file_path_(MODBUS_DIR),
    server_config_file_path_(MODBUS_DIR),
    client_file_path_(MODBUS_DIR),
    client_config_file_path_(MODBUS_DIR),
    file_path_(COMPONENT_PARAM_FILE_DIR)
{
    file_path_ += "modbus_manager.yaml";
    start_mode_ = 0;

    client_file_name_ = "";
    client_config_file_name_ = "";
    server_file_name_ = "";
    server_config_file_name_ = "";
}

bool ModbusManagerParam::loadParam()
{
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("start_mode", start_mode_)
        || !yaml_help_.getParam("client_file_name", client_file_name_)
        || !yaml_help_.getParam("client_config_file_name", client_config_file_name_)
        || !yaml_help_.getParam("server_file_name", server_file_name_)
        || !yaml_help_.getParam("server_config_file_name", server_config_file_name_))
    {
        cout << " Failed load modbus_manager.yaml " << endl;
        return false;
    }

    client_file_path_ += client_file_name_;
    client_config_file_path_ += client_config_file_name_;
    server_file_path_ += server_file_name_;
    server_config_file_path_ += server_config_file_name_;

    return true;
}

bool ModbusManagerParam::saveParam()
{
    if (!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("start_mode", start_mode_)
        || !yaml_help_.setParam("client_file_name", client_file_name_)
        || !yaml_help_.setParam("client_config_file_name", client_config_file_name_)
        || !yaml_help_.setParam("server_file_name", server_file_name_)
        || !yaml_help_.setParam("server_config_file_name", server_config_file_name_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus_manager.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusManagerParam::saveStartMode()
{
    if (!yaml_help_.setParam("start_mode", start_mode_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus_manager.yaml " << endl;
        return false;
    }

    return true;
}
