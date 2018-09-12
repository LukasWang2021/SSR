#include "modbus_manager_param.h"
#include "common_file_path.h"
#include "common_log.h"
#include <string>
#include <iostream>
using namespace fst_modbus;
using namespace std;


ModbusManagerParam::ModbusManagerParam():
    file_path_(COMPONENT_PARAM_FILE_DIR),
    device_manager_file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(fst_log::MSG_LEVEL_ERROR)
{
    file_path_ += "modbus_manager.yaml";
    device_manager_file_path_ += "modbus_device_manager.yaml";

    log_level_ = 1;
    server_ip_ = "";
    server_port_ = -1;
    connection_number_ = -1;
    device_number_ = -1;
    cycle_time_ = -1;
    is_debug_ = false;
}

bool ModbusManagerParam::loadParam()
{
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("cycle_time", cycle_time_)
        || !yaml_help_.getParam("modbus_info/server_ip", server_ip_)
        || !yaml_help_.getParam("modbus_info/server_port", server_port_)
        || !yaml_help_.getParam("modbus_info/connection_number", connection_number_)
        || !yaml_help_.getParam("modbus_info/is_debug", is_debug_)
        || !yaml_help_.getParam("modbus_info/device_number", device_number_))
    {
        cout << " Failed load modbus_manager.yaml " << endl;
        return false;
    }

    if (!yaml_help_.getParam("modbus_info/reg_info/coil/addr", modbus_register_info_.coil_addr)
        || !yaml_help_.getParam("modbus_info/reg_info/coil/max_nb", modbus_register_info_.coil_nb)
        || !yaml_help_.getParam("modbus_info/reg_info/discrepte_input/addr", modbus_register_info_.discrepte_input_addr)
        || !yaml_help_.getParam("modbus_info/reg_info/discrepte_input/max_nb", modbus_register_info_.discrepte_input_nb)
        || !yaml_help_.getParam("modbus_info/reg_info/input_register/addr", modbus_register_info_.holding_register_addr)
        || !yaml_help_.getParam("modbus_info/reg_info/input_register/max_nb", modbus_register_info_.holding_register_nb)
        || !yaml_help_.getParam("modbus_info/reg_info/holding_register/addr", modbus_register_info_.input_register_addr)
        || !yaml_help_.getParam("modbus_info/reg_info/holding_register/max_nb", modbus_register_info_.input_register_nb))
    {
        cout << " Failed load modbus_manager.yaml " << endl;
        return false;
    }

    int response_sec = 0;
    int response_usec = 0;
    int bytes_sec = 0;
    int bytes_usec = 0;

    if (!yaml_help_.getParam("modbus_info/response_timeout/tv_sec", response_sec)
        || !yaml_help_.getParam("modbus_info/response_timeout/tv_usec", response_usec)
        || !yaml_help_.getParam("modbus_info/bytes_timeout/tv_sec", bytes_sec)
        || !yaml_help_.getParam("modbus_info/bytes_timeout/tv_usec", bytes_usec))
    {
        cout << " Failed load modbus_manager.yaml " << endl;
        return false;
    }

    response_timeout_.tv_sec = response_sec;
    response_timeout_.tv_usec = response_usec;
    bytes_timeout_.tv_sec = bytes_sec;
    bytes_timeout_.tv_usec = bytes_usec;

    return true;
}

bool ModbusManagerParam::saveParam()
{
    if (!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("cycle_time", cycle_time_)
        || !yaml_help_.setParam("modbus_info/server_ip", server_ip_)
        || !yaml_help_.setParam("modbus_info/server_port", server_port_)
        || !yaml_help_.setParam("modbus_info/connection_number", connection_number_)
        || !yaml_help_.setParam("modbus_info/device_number", device_number_))
    {
        cout << " Failed save modbus_manager.yaml " << endl;
        return false;
    }
    if (!yaml_help_.setParam("modbus_info/reg_info/coil/addr", modbus_register_info_.coil_addr)
        || !yaml_help_.setParam("modbus_info/reg_info/coil/max_nb", modbus_register_info_.coil_nb)
        || !yaml_help_.setParam("modbus_info/reg_info/discrepte_input/addr", modbus_register_info_.discrepte_input_addr)
        || !yaml_help_.setParam("modbus_info/reg_info/discrepte_input/max_nb", modbus_register_info_.discrepte_input_nb)
        || !yaml_help_.setParam("modbus_info/reg_info/input_register/addr", modbus_register_info_.holding_register_addr)
        || !yaml_help_.setParam("modbus_info/reg_info/input_register/max_nb", modbus_register_info_.holding_register_nb)
        || !yaml_help_.setParam("modbus_info/reg_info/holding_register/addr", modbus_register_info_.input_register_addr)
        || !yaml_help_.setParam("modbus_info/reg_info/holding_register/max_nb", modbus_register_info_.input_register_nb))
    {
        cout << " Failed save modbus_manager.yaml " << endl;
        return false;
    }

    int response_sec = response_timeout_.tv_sec;
    int response_usec = response_timeout_.tv_usec;
    int bytes_sec = bytes_timeout_.tv_sec;
    int bytes_usec = bytes_timeout_.tv_usec;

    if (!yaml_help_.setParam("modbus_info/response_timeout/tv_sec", response_sec)
        || !yaml_help_.setParam("modbus_info/response_timeout/tv_usec", response_usec)
        || !yaml_help_.setParam("modbus_info/bytes_timeout/tv_sec", bytes_sec)
        || !yaml_help_.setParam("modbus_info/bytes_timeout/tv_usec", bytes_usec))
    {
        cout << " Failed save modbus_manager.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusManagerParam::loadDeviceParam()
{
    RegisterInfoOfDevice device_reg_info;
    device_reg_info.device_type = IO;

    if (!yaml_help_.loadParamFile(device_manager_file_path_.c_str()))
    {
        printf("load device yaml failed\n");
        return false;
    }

    if (!yaml_help_.getParam("io_info.coil.addr", device_reg_info.reg_info.coil_addr)
         || !yaml_help_.getParam("io_info/coil/nb", device_reg_info.reg_info.coil_nb)
         || !yaml_help_.getParam("io_info/discrepte_input/addr", device_reg_info.reg_info.discrepte_input_addr)
         || !yaml_help_.getParam("io_info/discrepte_input/nb", device_reg_info.reg_info.discrepte_input_nb)
         || !yaml_help_.getParam("io_info/holding_register/addr", device_reg_info.reg_info.holding_register_addr)
         || !yaml_help_.getParam("io_info/holding_register/nb", device_reg_info.reg_info.holding_register_nb)
         || !yaml_help_.getParam("io_info/input_register/addr", device_reg_info.reg_info.input_register_addr)
         || !yaml_help_.getParam("io_info/input_register/nb", device_reg_info.reg_info.input_register_nb))
    {
        printf("get device param failed\n");
        return false;
    }

    device_reg_info_list_mutex_.lock();
    device_reg_info_list_.push_back(device_reg_info);
    device_reg_info_list_mutex_.unlock();

    cout << "device_reg_info_list_.size = " << device_reg_info_list_.size() << endl;
    return true;
}

bool ModbusManagerParam::saveDeviceParam()
{
    RegisterInfoOfDevice device_reg_info;
    device_reg_info.device_type = IO;
    if (!yaml_help_.setParam("io_reg_info/coil_addr", device_reg_info.reg_info.coil_addr)
        || !yaml_help_.setParam("io_reg_info/coil_nb", device_reg_info.reg_info.coil_nb)
        || !yaml_help_.setParam("io_reg_info/discrepte_input_addr", device_reg_info.reg_info.discrepte_input_addr)
        || !yaml_help_.setParam("io_reg_info/discrepte_input_nb", device_reg_info.reg_info.discrepte_input_nb)
        || !yaml_help_.setParam("io_reg_info/holding_register_addr", device_reg_info.reg_info.holding_register_addr)
        || !yaml_help_.setParam("io_reg_info/holding_register_nb", device_reg_info.reg_info.holding_register_nb)
        || !yaml_help_.setParam("io_reg_info/input_register_addr", device_reg_info.reg_info.input_register_addr)
        || !yaml_help_.setParam("io_reg_info/input_register_nb", device_reg_info.reg_info.input_register_nb))
    {
        return false;
    }

    return true;
}
