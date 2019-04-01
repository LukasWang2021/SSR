#include "io_mapping_param.h"
#include "common_file_path.h"
#include <string>

using namespace fst_ctrl;

IoMappingParam::IoMappingParam():
    file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(3)
{
    file_path_ += "io_mapping.yaml";
    //file_path_ = "/home/fst/gitlab_iomapping_1015/Application/Motion_controller/install/share/runtime/component_param/io_mapping.yaml";//test only
}

IoMappingParam::~IoMappingParam()
{

}

bool IoMappingParam::loadParam()
{    
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("max_mapping_number", max_mapping_number_)
        || !yaml_help_.getParam("enable_set_io_in_auto", enable_set_io_in_auto_))
    {
        return false;
    }
    else    
    {
        return true;
    }
}

bool IoMappingParam::saveParam()
{
    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("max_mapping_number", max_mapping_number_)
        || !yaml_help_.setParam("enable_set_io_in_auto", enable_set_io_in_auto_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    }
}

