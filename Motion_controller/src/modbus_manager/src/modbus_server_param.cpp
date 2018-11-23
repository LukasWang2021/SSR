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
    is_debug_ = true;
    comm_type_ = "tcp";

    port_ = 502;
    cycle_time_ = 1000;
    connection_nb_ = 5;

    coil_addr_ = 0;
    coil_max_nb_ = 1024;

    input_register_addr_ = 0;
    input_register_max_nb_ = 1024;

    discrepte_input_addr_ = 0;
    discrepte_input_max_nb_ = 1024;

    holding_register_addr_ = 0;
    holding_register_max_nb_ = 1024;
}

bool ModbusServerParam::loadParam()
{
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("port", port_)
        || !yaml_help_.getParam("is_debug", is_debug_)
        || !yaml_help_.getParam("comm_type", comm_type_)
        || !yaml_help_.getParam("cycle_time", cycle_time_)
        || !yaml_help_.getParam("connection_nb", connection_nb_)
        || !yaml_help_.getParam("reg_info/coil/addr", coil_addr_)
        || !yaml_help_.getParam("reg_info/coil/max_nb", coil_max_nb_)
        || !yaml_help_.getParam("reg_info/discrepte_input/addr", discrepte_input_addr_)
        || !yaml_help_.getParam("reg_info/discrepte_input/max_nb", discrepte_input_max_nb_)
        || !yaml_help_.getParam("reg_info/input_register/addr", input_register_addr_)
        || !yaml_help_.getParam("reg_info/input_register/max_nb", input_register_max_nb_)
        || !yaml_help_.getParam("reg_info/holding_register/addr", holding_register_addr_)
        || !yaml_help_.getParam("reg_info/holding_register/max_nb", holding_register_max_nb_))
    {
        cout << " Failed load modbus tcp_server.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusServerParam::saveParam()
{
    if (!yaml_help_.setParam("port", port_)
        || !yaml_help_.setParam("is_debug", is_debug_)
        || !yaml_help_.setParam("comm_type", comm_type_)
        || !yaml_help_.setParam("cycle_time", cycle_time_)
        || !yaml_help_.setParam("connection_nb", connection_nb_)
        || !yaml_help_.setParam("reg_info/coil/addr", coil_addr_)
        || !yaml_help_.setParam("reg_info/coil/max_nb", coil_max_nb_)
        || !yaml_help_.setParam("reg_info/discrepte_input/addr", discrepte_input_addr_)
        || !yaml_help_.setParam("reg_info/discrepte_input/max_nb", discrepte_input_max_nb_)
        || !yaml_help_.setParam("reg_info/input_register/addr", input_register_addr_)
        || !yaml_help_.setParam("reg_info/input_register/max_nb", input_register_max_nb_)
        || !yaml_help_.setParam("reg_info/holding_register/addr", holding_register_addr_)
        || !yaml_help_.setParam("reg_info/holding_register/max_nb", holding_register_max_nb_))
    {
        cout << " Failed save modbus tcp_server.yaml " << endl;
        return false;
    }

    return true;
}
