#include "fst_io_device_param.h"
#include "common_file_path.h"
#include <string>


using namespace fst_hal;

FstIoDeviceParam::FstIoDeviceParam():
    log_level_(3),  // default is Error Level
    cycle_time_(10000),
    max_DI_number_(0),
    max_DO_number_(0),
    max_RI_number_(0),
    max_RO_number_(0),
    is_virtual_(true),
    comm_tolerance_(10),
    file_path_(COMPONENT_PARAM_FILE_DIR)
{
    file_path_ += "fst_io_device.yaml";
}

FstIoDeviceParam::~FstIoDeviceParam()
{

}

bool FstIoDeviceParam::loadParam()
{
    int max_di, max_do, max_ri, max_ro;

    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("cycle_time", cycle_time_)
        || !yaml_help_.getParam("device_type", device_type_)
        || !yaml_help_.getParam("comm_type", comm_type_)
        || !yaml_help_.getParam("max_DI_number", max_di)
        || !yaml_help_.getParam("max_DO_number", max_do)
        || !yaml_help_.getParam("max_RI_number", max_ri)
        || !yaml_help_.getParam("max_RO_number", max_ro)
        || !yaml_help_.getParam("is_virtual", is_virtual_)
        || !yaml_help_.getParam("comm_tolerance", comm_tolerance_))
    {
        return false;
    }
    else    
    {
        if (max_di < 0 || max_do < 0 || max_ri < 0 || max_ro < 0) return false;
        max_DI_number_ = static_cast<uint32_t>(max_di);
        max_DO_number_ = static_cast<uint32_t>(max_do);
        max_RI_number_ = static_cast<uint32_t>(max_ri);
        max_RO_number_ = static_cast<uint32_t>(max_ro);
        return true;
    } 
}

bool FstIoDeviceParam::saveParam()
{
    int max_di = static_cast<int>(max_DI_number_);
    int max_do = static_cast<int>(max_DO_number_);
    int max_ri = static_cast<int>(max_RI_number_);
    int max_ro = static_cast<int>(max_RO_number_);

    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("cycle_time", cycle_time_)
        || !yaml_help_.setParam("device_type", device_type_)
        || !yaml_help_.setParam("comm_type", comm_type_)
        || !yaml_help_.setParam("max_DI_number", max_di)
        || !yaml_help_.setParam("max_DO_number", max_do)
        || !yaml_help_.setParam("max_RI_number", max_ri)
        || !yaml_help_.setParam("max_RO_number", max_ro)
        || !yaml_help_.setParam("is_virtual", is_virtual_)
        || !yaml_help_.setParam("comm_tolerance", comm_tolerance_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    } 
}

