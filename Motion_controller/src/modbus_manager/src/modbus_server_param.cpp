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
    cycle_time_ = 0;
    thread_priority_ = 0;
    comm_type_ = ""; 
    port_ = -1;

    connection_nb_ = 0;
    is_debug_ = false;

    reg_info_.coil.addr = 0;
    reg_info_.coil.addr = 0;
    reg_info_.discrepte_input.addr = 0;
    reg_info_.discrepte_input.addr = 0;
    reg_info_.holding_reg.addr = 0;
    reg_info_.holding_reg.addr = 0;
    reg_info_.input_reg.addr = 0;
    reg_info_.input_reg.addr = 0;
}

bool ModbusServerParam::loadParam()
{
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("cycle_time", cycle_time_)
        || !yaml_help_.getParam("thread_priority", thread_priority_)
        || !yaml_help_.getParam("comm_type", comm_type_)
        || !yaml_help_.getParam("port", port_)
        || !yaml_help_.getParam("connection_nb", connection_nb_)
        || !yaml_help_.getParam("is_debug", is_debug_)
        || !yaml_help_.getParam("reg_info/coil/addr", reg_info_.coil.addr)
        || !yaml_help_.getParam("reg_info/coil/max_nb", reg_info_.coil.max_nb)
        || !yaml_help_.getParam("reg_info/discrepte_input/addr", reg_info_.discrepte_input.addr)
        || !yaml_help_.getParam("reg_info/discrepte_input/max_nb", reg_info_.discrepte_input.max_nb)
        || !yaml_help_.getParam("reg_info/input_register/addr", reg_info_.holding_reg.addr)
        || !yaml_help_.getParam("reg_info/input_register/max_nb", reg_info_.holding_reg.max_nb)
        || !yaml_help_.getParam("reg_info/holding_register/addr", reg_info_.input_reg.addr)
        || !yaml_help_.getParam("reg_info/holding_register/max_nb", reg_info_.input_reg.max_nb))
    {
        cout << " Failed load modbus server.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusServerParam::saveParam()
{

    if (!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("cycle_time", cycle_time_)
        || !yaml_help_.setParam("thread_priority", thread_priority_)
        || !yaml_help_.setParam("comm_type", comm_type_)
        || !yaml_help_.setParam("port", port_)
        || !yaml_help_.setParam("connection_nb", connection_nb_)
        || !yaml_help_.setParam("is_debug", is_debug_)
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
        cout << " Failed save modbus server.yaml " << endl;
        return false;
    }

    return true;
}
