#include "fst_safety_device_param.h"
#include "common_file_path.h"
#include <string>

using namespace fst_hal;

FstSafetyDeviceParam::FstSafetyDeviceParam():
    log_level_(3),  // default is Error Level
    cycle_time_(1000),
    is_virtual_(false),
    file_path_(COMPONENT_PARAM_FILE_DIR)
{
    file_path_ += "fst_safety_device.yaml";
}

FstSafetyDeviceParam::~FstSafetyDeviceParam()
{

}

bool FstSafetyDeviceParam::loadParam()
{ 
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("cycle_time", cycle_time_)
        || !yaml_help_.getParam("is_virtual", is_virtual_)
        || !yaml_help_.getParam("auto_mode_DO", auto_mode_DO_)
        || !yaml_help_.getParam("limited_manual_mode_DO", limited_manual_mode_DO_)
        || !yaml_help_.getParam("manual_mode_DO", manual_mode_DO_))
    {
        return false;
    }
    else    
    {
        return true;
    }
}

bool FstSafetyDeviceParam::saveParam()
{
    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("cycle_time", cycle_time_)
        || !yaml_help_.setParam("is_virtual", is_virtual_)
        || !yaml_help_.setParam("auto_mode_DO", auto_mode_DO_)
        || !yaml_help_.setParam("limited_manual_mode_DO", limited_manual_mode_DO_)
        || !yaml_help_.setParam("manual_mode_DO", manual_mode_DO_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    } 
}

