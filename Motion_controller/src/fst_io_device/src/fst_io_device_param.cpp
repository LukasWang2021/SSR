#include "fst_io_device_param.h"
#include "common_file_path.h"
#include <string>

using namespace fst_hal;

FstIoDeviceParam::FstIoDeviceParam():
    file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(3),  // default is Error Level
    cycle_time_(10000),
    max_dev_number_(0),
    max_DI_number_(0),
    max_DO_number_(0),
    max_RI_number_(0),
    max_RO_number_(0),
    is_virtual_(true),
    virtual_board_address_(16),
    virtual_DI_number_(2000),
    virtual_DO_number_(2000)
{
    file_path_ += "fst_io_device.yaml";
    //file_path_ = "/home/fst/gitlab_iomap_1015/Application/Motion_controller/install/share/runtime/component_param/fst_io_device.yaml";//test only
}

FstIoDeviceParam::~FstIoDeviceParam()
{

}

bool FstIoDeviceParam::loadParam()
{
/*    if (!yaml_help_.loadParamFile(file_path_.c_str()))
    {
        printf("load file faled\n");
        return false;
    }
    if (!yaml_help_.getParam("log_level", log_level_))
    {
        printf("load log_level faled\n");
        return false;
    }
    if (!yaml_help_.getParam("cycle_time", cycle_time_))
    {
        printf("load cycletime faled\n");
        return false;
    }
    if (!yaml_help_.getParam("max_dev_number", max_dev_number_))
    {
        printf("load max dev number faled\n");
        return false;
    }
    if (!yaml_help_.getParam("max_DI_number", max_DI_number_))
    {
        printf("load max di number faled\n");
        return false;
    }
    if (!yaml_help_.getParam("max_DO_number", max_DO_number_))
    {
        printf("load max do number faled\n");
        return false;
    }
    if (!yaml_help_.getParam("max_RI_number", max_RI_number_))
    {
        printf("load max ri number faled\n");
        return false;
    }
    if (!yaml_help_.getParam("max_RO_number", max_RO_number_))
    {
        printf("load max ro number faled\n");
        return false;
    }
    if (!yaml_help_.getParam("is_virtual", is_virtual_))
    {
        printf("load is virtual faled\n");
        return false;
    }
*/
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("cycle_time", cycle_time_)
        || !yaml_help_.getParam("max_dev_number", max_dev_number_)
        || !yaml_help_.getParam("max_DI_number", max_DI_number_)
        || !yaml_help_.getParam("max_DO_number", max_DO_number_)
        || !yaml_help_.getParam("max_RI_number", max_RI_number_)
        || !yaml_help_.getParam("max_RO_number", max_RO_number_)
        || !yaml_help_.getParam("is_virtual", is_virtual_)
        || !yaml_help_.getParam("virtual_board_address", virtual_board_address_)
        || !yaml_help_.getParam("virtual_DI_number", virtual_DI_number_)
        || !yaml_help_.getParam("virtual_DO_number", virtual_DO_number_))
    {
        return false;
    }
    else    
    {
        return true;
    } 
}

bool FstIoDeviceParam::saveParam()
{
    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("cycle_time", cycle_time_)
        || !yaml_help_.setParam("max_dev_number", max_dev_number_)
        || !yaml_help_.setParam("max_DI_number", max_DI_number_)
        || !yaml_help_.setParam("max_DO_number", max_DO_number_)
        || !yaml_help_.setParam("max_RI_number", max_RI_number_)
        || !yaml_help_.setParam("max_RO_number", max_RO_number_)
        || !yaml_help_.setParam("is_virtual", is_virtual_)
        || !yaml_help_.setParam("virtual_board_address", virtual_board_address_)
        || !yaml_help_.setParam("virtual_DI_number", virtual_DI_number_)
        || !yaml_help_.setParam("virtual_DO_number", virtual_DO_number_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    } 
}

