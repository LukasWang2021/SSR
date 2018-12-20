#include "modbus_client_param.h"
#include "common_file_path.h"
#include "common_log.h"
#include <string>
#include <iostream>
using namespace fst_hal;
using namespace std;

ModbusClientParam::ModbusClientParam(string file_path):
    file_path_(file_path),
    log_level_(fst_log::MSG_LEVEL_ERROR)
{
    scan_rate_ = 0;
    is_enable_ = false;
    id_ = 0;
    config_.name = "Modbus";
    config_.ip = "";
    config_.port = -1;
    config_.response_timeout_sec = 0;
    config_.response_timeout_usec = 0;
    config_.bytes_timeout_sec = 0;
    config_.bytes_timeout_usec = 0;
    config_.reg_info.coil.addr = 0;
    config_.reg_info.coil.max_nb = 0;
    config_.reg_info.discrepte_input.addr = 0;
    config_.reg_info.discrepte_input.max_nb = 0;
    config_.reg_info.holding_reg.addr = 0;
    config_.reg_info.holding_reg.max_nb = 0;
    config_.reg_info.input_reg.addr = 0;
    config_.reg_info.input_reg.max_nb = 0;

    comm_type_ = "";
    is_debug_ = false;
}

bool ModbusClientParam::loadParam()
{
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("comm_type", comm_type_)
        || !yaml_help_.getParam("is_debug", is_debug_)
        || !yaml_help_.getParam("scan_rate", scan_rate_)
        || !yaml_help_.getParam("is_enable", is_enable_)
        || !yaml_help_.getParam("id", id_)
        || !yaml_help_.getParam("name", config_.name)
        || !yaml_help_.getParam("ip", config_.ip)
        || !yaml_help_.getParam("port", config_.port)
        || !yaml_help_.getParam("bytes_timeout/sec", config_.bytes_timeout_sec)
        || !yaml_help_.getParam("bytes_timeout/usec", config_.bytes_timeout_usec)
        || !yaml_help_.getParam("response_timeout/sec", config_.response_timeout_sec)
        || !yaml_help_.getParam("response_timeout/usec", config_.response_timeout_usec)
        || !yaml_help_.getParam("reg_info/coil/addr", config_.reg_info.coil.addr)
        || !yaml_help_.getParam("reg_info/coil/max_nb", config_.reg_info.coil.max_nb)
        || !yaml_help_.getParam("reg_info/discrepte_input/addr", config_.reg_info.discrepte_input.addr)
        || !yaml_help_.getParam("reg_info/discrepte_input/max_nb", config_.reg_info.discrepte_input.max_nb)
        || !yaml_help_.getParam("reg_info/input_register/addr", config_.reg_info.input_reg.addr)
        || !yaml_help_.getParam("reg_info/input_register/max_nb", config_.reg_info.input_reg.max_nb)
        || !yaml_help_.getParam("reg_info/holding_register/addr", config_.reg_info.holding_reg.addr)
        || !yaml_help_.getParam("reg_info/holding_register/max_nb", config_.reg_info.holding_reg.max_nb))
    {
        cout << " Failed load modbus tcp_client.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusClientParam::saveConfig()
{
    if (!yaml_help_.setParam("ip", config_.ip)
        || !yaml_help_.setParam("id", id_)
        || !yaml_help_.setParam("port", config_.port)
        || !yaml_help_.setParam("name", config_.name)
        || !yaml_help_.setParam("bytes_timeout/sec", config_.bytes_timeout_sec)
        || !yaml_help_.setParam("bytes_timeout/usec", config_.bytes_timeout_usec)
        || !yaml_help_.setParam("response_timeout/sec", config_.response_timeout_sec)
        || !yaml_help_.setParam("response_timeout/usec", config_.response_timeout_usec)
        || !yaml_help_.setParam("reg_info/coil/addr", config_.reg_info.coil.addr)
        || !yaml_help_.setParam("reg_info/coil/max_nb", config_.reg_info.coil.max_nb)
        || !yaml_help_.setParam("reg_info/discrepte_input/addr", config_.reg_info.discrepte_input.addr)
        || !yaml_help_.setParam("reg_info/discrepte_input/max_nb", config_.reg_info.discrepte_input.max_nb)
        || !yaml_help_.setParam("reg_info/input_register/addr", config_.reg_info.input_reg.addr)
        || !yaml_help_.setParam("reg_info/input_register/max_nb", config_.reg_info.input_reg.max_nb)
        || !yaml_help_.setParam("reg_info/holding_register/addr", config_.reg_info.holding_reg.addr)
        || !yaml_help_.setParam("reg_info/holding_register/max_nb", config_.reg_info.holding_reg.max_nb)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus tcp_client.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusClientParam::saveScanRate()
{
    if (!yaml_help_.setParam("scan_rate", scan_rate_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus tcp_client.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusClientParam::saveConnectStatus()
{
    if (!yaml_help_.setParam("is_enable", is_enable_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus tcp_client.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusClientParam::saveId()
{
    if (!yaml_help_.setParam("id", id_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus tcp_client.yaml " << endl;
        return false;
    }

    return true;
}
