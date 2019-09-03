#include "anybus_manager_param.h"
#include "common_file_path.h"
#include <string>

using namespace fst_anybus;

AnybusManagerParam::AnybusManagerParam():
    file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(1)
{
    file_path_ += "anybus_manager.yaml";
    abcc_page_addr_ = 0xc0400000;
    appl_page_addr_ = 0xc00b000c;
}

AnybusManagerParam::~AnybusManagerParam()
{
}

bool AnybusManagerParam::loadParam()
{    
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("operation_mode", operation_mode_)
        || !yaml_help_.getParam("cycle_time", cycle_time_)
        || !yaml_help_.getParam("abcc_page_size", abcc_page_size_)
        || !yaml_help_.getParam("appl_page_size", appl_page_size_)
        || !yaml_help_.getParam("safety_enable", safety_enable_)
        || !yaml_help_.getParam("ethercat_config_file_name", ethercat_config_file_name_))
    {
        return false;
    }

    return true;
}

bool AnybusManagerParam::saveParam()
{
    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("operation_mode", operation_mode_)
        || !yaml_help_.setParam("cycle_time", cycle_time_)
        || !yaml_help_.setParam("abcc_page_size", abcc_page_size_)
        || !yaml_help_.setParam("appl_page_size", appl_page_size_)
        || !yaml_help_.setParam("safety_enable_", safety_enable_)
        || !yaml_help_.setParam("ethercat_config_file_name", ethercat_config_file_name_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }

    return true;
}

