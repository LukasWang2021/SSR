#include "virtual_io_device_param.h"
#include "common_file_path.h"
#include <string>

using namespace fst_hal;

VirtualIoDeviceParam::VirtualIoDeviceParam():
    log_level_(3),
    max_DI_number_(0),
    max_DO_number_(0),
    file_path_(COMPONENT_PARAM_FILE_DIR)
{
    file_path_ += "virtual_io_device.yaml";
}

VirtualIoDeviceParam::~VirtualIoDeviceParam()
{

}

bool VirtualIoDeviceParam::loadParam()
{   
    int max_di, max_do;

    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("device_type", device_type_)
        || !yaml_help_.getParam("comm_type", comm_type_)
        || !yaml_help_.getParam("max_DI_number", max_di)
        || !yaml_help_.getParam("max_DO_number", max_do))
    {
        return false;
    }
    else    
    {
        if (max_di < 0 || max_do < 0) return false;
        max_DI_number_ = static_cast<uint32_t>(max_di);
        max_DO_number_ = static_cast<uint32_t>(max_do);
        return true;
    }
}

bool VirtualIoDeviceParam::saveParam()
{
    int max_di = static_cast<int>(max_DI_number_);
    int max_do = static_cast<int>(max_DO_number_);
    
    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("device_type", device_type_)
        || !yaml_help_.setParam("comm_type", comm_type_)
        || !yaml_help_.setParam("max_DI_number", max_di)
        || !yaml_help_.setParam("max_DO_number", max_do)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    }
}

