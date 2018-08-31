#include "modbus_manager_param.h"
//#include "common_file_path.h"
#include "common_log.h"
#include <string>

using namespace fst_modbus;

ModbusManagerParam::ModbusManagerParam():
    //file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(fst_log::MSG_LEVEL_ERROR)
{
    //file_path_ += "tp_comm.yaml";
}


bool ModbusManagerParam::loadParam()
{
    return true;
}

bool ModbusManagerParam::saveParam()
{
    return true;
}
