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
    comm_type_ = "";
    is_debug_ = false;
    client_number_ = 0;
}

bool ModbusClientParam::loadParam()
{
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("comm_type", comm_type_)
        || !yaml_help_.getParam("is_debug", is_debug_)
        || !yaml_help_.getParam("scan_rate_min", scan_rate_min_)
        || !yaml_help_.getParam("scan_rate_max", scan_rate_max_)
        || !yaml_help_.getParam("port_min", port_min_)
        || !yaml_help_.getParam("port_max", port_max_)
        || !yaml_help_.getParam("response_timeout_min", response_timeout_min_)
        || !yaml_help_.getParam("response_timeout_max", response_timeout_max_)
        || !yaml_help_.getParam("coil_addr_min", coil_addr_min_)
        || !yaml_help_.getParam("coil_addr_max", coil_addr_max_)
        || !yaml_help_.getParam("discrepte_input_min", discrepte_input_min_)
        || !yaml_help_.getParam("discrepte_input_max", discrepte_input_max_)
        || !yaml_help_.getParam("holding_reg_min", holding_reg_min_)
        || !yaml_help_.getParam("holding_reg_max", holding_reg_max_)
        || !yaml_help_.getParam("input_reg_min", input_reg_min_)
        || !yaml_help_.getParam("input_reg_max", input_reg_max_)
        || !yaml_help_.getParam("client_number", client_number_))
    {
        cout << " Failed load modbus client.yaml " << endl;
        return false;
    }

    return true;
}

bool ModbusClientParam::saveParam()
{
    if (!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("comm_type", comm_type_)
        || !yaml_help_.setParam("is_debug", is_debug_)
        || !yaml_help_.setParam("client_number", client_number_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        cout << " Failed save modbus client.yaml " << endl;
        return false;
    }

    return true;
}
