#include "modbus_server_param.h"
#include "common_file_path.h"
#include "common_log.h"
#include <string>
#include <iostream>
using namespace fst_hal;
using namespace std;

ModbusServerParam::ModbusServerParam(string file_path):
    file_path_(file_path),
    log_level_(fst_log::MSG_LEVEL_ERROR)
{
    ModbusServerConfig config_;  

    is_debug_ = true;
    comm_type_ = "tcp";
    thread_priority_ = -1;

    port_ = 502;
    cycle_time_ = 1000;
    connection_nb_ = 5;

    reg_info_.coil.addr = 0;
    reg_info_.coil.max_nb = 0;
    reg_info_.discrepte_input.addr = 0;
    reg_info_.discrepte_input.max_nb = 0;
    reg_info_.input_reg.addr = 0;
    reg_info_.input_reg.max_nb = 0;
    reg_info_.holding_reg.addr = 0;
    reg_info_.holding_reg.max_nb = 0;

    config_.response_delay = 0;
    config_.reg_info.coil.addr = 0;
    config_.reg_info.coil.max_nb = 0;
    config_.reg_info.discrepte_input.addr = 0;
    config_.reg_info.discrepte_input.max_nb = 0;
    config_.reg_info.input_reg.addr = 0;
    config_.reg_info.input_reg.max_nb = 0;
    config_.reg_info.holding_reg.addr = 0;
    config_.reg_info.holding_reg.max_nb = 0;

    is_enable_ =  false;
}

bool ModbusServerParam::loadParam()
{
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("port", port_)
        || !yaml_help_.getParam("is_debug", is_debug_)
        || !yaml_help_.getParam("comm_type", comm_type_)
        || !yaml_help_.getParam("thread_priority", thread_priority_)
        || !yaml_help_.getParam("cycle_time", cycle_time_)
        || !yaml_help_.getParam("connection_nb", connection_nb_)
        || !yaml_help_.getParam("is_enable", is_enable_)
        || !yaml_help_.getParam("reg_info/coil/addr", reg_info_.coil.addr)
        || !yaml_help_.getParam("reg_info/coil/max_nb", reg_info_.coil.max_nb)
        || !yaml_help_.getParam("reg_info/discrepte_input/addr", reg_info_.discrepte_input.addr)
        || !yaml_help_.getParam("reg_info/discrepte_input/max_nb", reg_info_.discrepte_input.max_nb)
        || !yaml_help_.getParam("reg_info/input_register/addr", reg_info_.input_reg.addr)
        || !yaml_help_.getParam("reg_info/input_register/max_nb", reg_info_.input_reg.max_nb)
        || !yaml_help_.getParam("reg_info/holding_register/addr", reg_info_.holding_reg.addr)
        || !yaml_help_.getParam("reg_info/holding_register/max_nb", reg_info_.holding_reg.max_nb)
        || !yaml_help_.getParam("config/response_delay", config_.response_delay)
        || !yaml_help_.getParam("config/reg_info/coil/addr", config_.reg_info.coil.addr)
        || !yaml_help_.getParam("config/reg_info/coil/max_nb", config_.reg_info.coil.max_nb)
        || !yaml_help_.getParam("config/reg_info/coil/is_valid", config_.reg_info.coil.is_valid)
        || !yaml_help_.getParam("config/reg_info/discrepte_input/addr", config_.reg_info.discrepte_input.addr)
        || !yaml_help_.getParam("config/reg_info/discrepte_input/max_nb", config_.reg_info.discrepte_input.max_nb)
        || !yaml_help_.getParam("config/reg_info/discrepte_input/is_valid", config_.reg_info.discrepte_input.is_valid)
        || !yaml_help_.getParam("config/reg_info/input_register/addr", config_.reg_info.input_reg.addr)
        || !yaml_help_.getParam("config/reg_info/input_register/max_nb", config_.reg_info.input_reg.max_nb)
        || !yaml_help_.getParam("config/reg_info/input_register/is_valid", config_.reg_info.input_reg.is_valid)
        || !yaml_help_.getParam("config/reg_info/holding_register/addr", config_.reg_info.holding_reg.addr)
        || !yaml_help_.getParam("config/reg_info/holding_register/max_nb", config_.reg_info.holding_reg.max_nb)
        || !yaml_help_.getParam("config/reg_info/holding_register/is_valid", config_.reg_info.holding_reg.is_valid))
    {
        cout << " Failed load modbus tcp_server.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusServerParam::saveConfig()
{
    if (!yaml_help_.setParam("is_enable", is_enable_)
        || !yaml_help_.setParam("config/response_delay", config_.response_delay)
        || !yaml_help_.setParam("config/reg_info/coil/addr", config_.reg_info.coil.addr)
        || !yaml_help_.setParam("config/reg_info/coil/max_nb", config_.reg_info.coil.max_nb)
        || !yaml_help_.setParam("config/reg_info/coil/is_valid", config_.reg_info.coil.is_valid)
        || !yaml_help_.setParam("config/reg_info/discrepte_input/addr", config_.reg_info.discrepte_input.addr)
        || !yaml_help_.setParam("config/reg_info/discrepte_input/max_nb", config_.reg_info.discrepte_input.max_nb)
        || !yaml_help_.setParam("config/reg_info/discrepte_input/is_valid", config_.reg_info.discrepte_input.is_valid)
        || !yaml_help_.setParam("config/reg_info/input_register/addr", config_.reg_info.input_reg.addr)
        || !yaml_help_.setParam("config/reg_info/input_register/max_nb", config_.reg_info.input_reg.max_nb)
        || !yaml_help_.setParam("config/reg_info/input_register/is_valid", config_.reg_info.input_reg.is_valid)
        || !yaml_help_.setParam("config/reg_info/holding_register/addr", config_.reg_info.holding_reg.addr)
        || !yaml_help_.setParam("config/reg_info/holding_register/max_nb", config_.reg_info.holding_reg.max_nb)
        || !yaml_help_.setParam("config/reg_info/holding_register/is_valid", config_.reg_info.holding_reg.is_valid)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus tcp_server.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusServerParam::loadConfig()
{
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("config/response_delay", config_.response_delay)
        || !yaml_help_.getParam("config/reg_info/coil/addr", config_.reg_info.coil.addr)
        || !yaml_help_.getParam("config/reg_info/coil/max_nb", config_.reg_info.coil.max_nb)
        || !yaml_help_.getParam("config/reg_info/coil/is_valid", config_.reg_info.coil.is_valid)
        || !yaml_help_.getParam("config/reg_info/discrepte_input/addr", config_.reg_info.discrepte_input.addr)
        || !yaml_help_.getParam("config/reg_info/discrepte_input/max_nb", config_.reg_info.discrepte_input.max_nb)
        || !yaml_help_.getParam("config/reg_info/discrepte_input/is_valid", config_.reg_info.discrepte_input.is_valid)
        || !yaml_help_.getParam("config/reg_info/input_register/addr", config_.reg_info.input_reg.addr)
        || !yaml_help_.getParam("config/reg_info/input_register/max_nb", config_.reg_info.input_reg.max_nb)
        || !yaml_help_.getParam("config/reg_info/input_register/is_valid", config_.reg_info.input_reg.is_valid)
        || !yaml_help_.getParam("config/reg_info/holding_register/addr", config_.reg_info.holding_reg.addr)
        || !yaml_help_.getParam("config/reg_info/holding_register/max_nb", config_.reg_info.holding_reg.max_nb)
        || !yaml_help_.getParam("config/reg_info/holding_register/is_valid", config_.reg_info.holding_reg.is_valid))
    {
        cout << " Failed load modbus tcp_server.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusServerParam::saveConnectStatus()
{
    if (!yaml_help_.setParam("is_enable", is_enable_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus tcp_server.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusServerParam::saveCoilInfo()
{
    if (!yaml_help_.setParam("config/reg_info/coil/addr", config_.reg_info.coil.addr)
        || !yaml_help_.setParam("config/reg_info/coil/max_nb", config_.reg_info.coil.max_nb)
        || !yaml_help_.setParam("config/reg_info/coil/is_valid", config_.reg_info.coil.is_valid)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus tcp_server.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusServerParam::saveDiscrepteInputInfo()
{
    if (!yaml_help_.getParam("config/reg_info/discrepte_input/addr", config_.reg_info.discrepte_input.addr)
        || !yaml_help_.getParam("config/reg_info/discrepte_input/max_nb", config_.reg_info.discrepte_input.max_nb)
        || !yaml_help_.getParam("config/reg_info/discrepte_input/is_valid", config_.reg_info.discrepte_input.is_valid)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus tcp_server.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusServerParam::saveHoldingRegInfo()
{
    if (!yaml_help_.getParam("config/reg_info/input_register/addr", config_.reg_info.input_reg.addr)
        || !yaml_help_.getParam("config/reg_info/input_register/max_nb", config_.reg_info.input_reg.max_nb)
        || !yaml_help_.getParam("config/reg_info/input_register/is_valid", config_.reg_info.input_reg.is_valid)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus tcp_server.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusServerParam::saveInputRegInfo()
{
    if (!yaml_help_.setParam("config/reg_info/holding_register/addr", config_.reg_info.holding_reg.addr)
        || !yaml_help_.setParam("config/reg_info/holding_register/max_nb", config_.reg_info.holding_reg.max_nb)
        || !yaml_help_.setParam("config/reg_info/holding_register/is_valid", config_.reg_info.holding_reg.is_valid)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus tcp_server.yaml " << endl;
        return false;
    }

    return true;
}
