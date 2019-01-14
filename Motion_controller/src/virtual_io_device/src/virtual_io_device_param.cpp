#include "virtual_io_device_param.h"
#include "common_file_path.h"
#include <string>

using namespace fst_hal;

VirtualIoDeviceParam::VirtualIoDeviceParam():
    file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(3),
    max_DI_number_(0),
    max_DO_number_(0)
{
    file_path_ += "virtual_io_device.yaml";
}

VirtualIoDeviceParam::~VirtualIoDeviceParam()
{

}

bool VirtualIoDeviceParam::loadParam()
{    
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("device_type", device_type_)
        || !yaml_help_.getParam("comm_type", comm_type_)
        || !yaml_help_.getParam("max_DI_number", max_DI_number_)
        || !yaml_help_.getParam("max_DO_number", max_DO_number_))
    {
        return false;
    }
    else    
    {
        return true;
    }
}

bool VirtualIoDeviceParam::saveParam()
{
    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("device_type", device_type_)
        || !yaml_help_.setParam("comm_type", comm_type_)
        || !yaml_help_.setParam("max_DI_number", max_DI_number_)
        || !yaml_help_.setParam("max_DO_number", max_DO_number_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    }
}

