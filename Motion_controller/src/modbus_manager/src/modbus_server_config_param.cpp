#include "modbus_server_config_param.h"
#include "common_file_path.h"
#include "common_log.h"
#include <string>
#include <iostream>
using namespace fst_hal;
using namespace std;

ModbusServerConfigParam::ModbusServerConfigParam(string file_path):
    file_path_(file_path)
{
    is_enable_ = false;

    start_info_.ip = "";
    start_info_.name = "";
    start_info_.response_delay = 0;

    reg_info_.coil.addr = 0;
    reg_info_.coil.max_nb = 0;
    reg_info_.discrepte_input.addr = 0;
    reg_info_.discrepte_input.max_nb = 0;
    reg_info_.holding_reg.addr = 0;
    reg_info_.holding_reg.max_nb = 0;
    reg_info_.input_reg.addr = 0;
    reg_info_.input_reg.max_nb = 0;
}

bool ModbusServerConfigParam::loadParam()
{
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("is_enable", is_enable_)
        || !yaml_help_.getParam("ip", start_info_.ip)
        || !yaml_help_.getParam("name", start_info_.name)
        || !yaml_help_.getParam("response_delay", start_info_.response_delay)
        || !yaml_help_.getParam("reg_info/coil/addr", reg_info_.coil.addr)
        || !yaml_help_.getParam("reg_info/coil/max_nb", reg_info_.coil.max_nb)
        || !yaml_help_.getParam("reg_info/discrepte_input/addr", reg_info_.discrepte_input.addr)
        || !yaml_help_.getParam("reg_info/discrepte_input/max_nb", reg_info_.discrepte_input.max_nb)
        || !yaml_help_.getParam("reg_info/input_register/addr", reg_info_.holding_reg.addr)
        || !yaml_help_.getParam("reg_info/input_register/max_nb", reg_info_.holding_reg.max_nb)
        || !yaml_help_.getParam("reg_info/holding_register/addr", reg_info_.input_reg.addr)
        || !yaml_help_.getParam("reg_info/holding_register/max_nb", reg_info_.input_reg.max_nb))
    {
        cout << " Failed load modbus server_config.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusServerConfigParam::saveParam()
{
    if (!yaml_help_.setParam("is_enable", is_enable_)
        || !yaml_help_.setParam("ip", start_info_.ip)
        || !yaml_help_.setParam("name", start_info_.name)
        || !yaml_help_.setParam("response_delay", start_info_.response_delay)
        || !yaml_help_.setParam("reg_info/coil/addr", reg_info_.coil.addr)
        || !yaml_help_.setParam("reg_info/coil/max_nb", reg_info_.coil.max_nb)
        || !yaml_help_.setParam("reg_info/discrepte_input/addr", reg_info_.discrepte_input.addr)
        || !yaml_help_.setParam("reg_info/discrepte_input/max_nb", reg_info_.discrepte_input.max_nb)
        || !yaml_help_.setParam("reg_info/input_register/addr", reg_info_.holding_reg.addr)
        || !yaml_help_.setParam("reg_info/input_register/max_nb", reg_info_.holding_reg.max_nb)
        || !yaml_help_.setParam("reg_info/holding_register/addr", reg_info_.input_reg.addr)
        || !yaml_help_.setParam("reg_info/holding_register/max_nb", reg_info_.input_reg.max_nb)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus server_config.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusServerConfigParam::saveStartInfo()
{
    if (!yaml_help_.setParam("name", start_info_.name)
        || !yaml_help_.setParam("response_delay", start_info_.response_delay)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus server_config.yaml " << endl;
        return false;
    }
    return true;
}

bool ModbusServerConfigParam::saveEnableStatus()
{
    if (!yaml_help_.setParam("is_enable", is_enable_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus server_config.yaml " << endl;
        return false;
    }

    return true;
}


bool ModbusServerConfigParam::saveRegInfo()
{
    if (!yaml_help_.setParam("reg_info/coil/addr", reg_info_.coil.addr)
        || !yaml_help_.setParam("reg_info/coil/max_nb", reg_info_.coil.max_nb)
        || !yaml_help_.setParam("reg_info/discrepte_input/addr", reg_info_.discrepte_input.addr)
        || !yaml_help_.setParam("reg_info/discrepte_input/max_nb", reg_info_.discrepte_input.max_nb)
        || !yaml_help_.setParam("reg_info/input_register/addr", reg_info_.holding_reg.addr)
        || !yaml_help_.setParam("reg_info/input_register/max_nb", reg_info_.holding_reg.max_nb)
        || !yaml_help_.setParam("reg_info/holding_register/addr", reg_info_.input_reg.addr)
        || !yaml_help_.setParam("reg_info/holding_register/max_nb", reg_info_.input_reg.max_nb)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus server_config.yaml " << endl;
        return false;
    }

    return true;
}
