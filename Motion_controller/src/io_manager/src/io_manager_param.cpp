#include "io_manager_param.h"
#include "common_file_path.h"
#include <string>

using namespace fst_hal;

IoManagerParam::IoManagerParam():
    file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(3),  // default is Error Level
    cycle_time_(10000)
{
    file_path_ += "io_manager.yaml";
}

IoManagerParam::~IoManagerParam()
{

}

bool IoManagerParam::loadParam()
{
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("cycle_time", cycle_time_))
    {
        return false;
    }
    else    
    {
        return true;
    } 
}

bool IoManagerParam::saveParam()
{
    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("cycle_time", cycle_time_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    } 
}

