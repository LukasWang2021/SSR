#include "coordinate_manager_param.h"
#include "common_file_path.h"
#include "common_log.h"
#include <string>

using namespace fst_ctrl;

CoordinateManagerParam::CoordinateManagerParam():
    log_level_(fst_log::MSG_LEVEL_ERROR),
    max_name_length_(32),
    max_comment_length_(32),
    max_number_of_coords_(10),
    file_path_(COMPONENT_PARAM_FILE_DIR)
{
    file_path_ += "coordinate_manager.yaml";
}

CoordinateManagerParam::~CoordinateManagerParam()
{

}

bool CoordinateManagerParam::loadParam()
{    
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("max_name_length", max_name_length_)
        || !yaml_help_.getParam("max_comment_length", max_comment_length_)
        || !yaml_help_.getParam("max_number_of_coords", max_number_of_coords_)
        || !yaml_help_.getParam("coord_info_file_name", coord_info_file_name_))
    {
        return false;
    }
    else    
    {
        return true;
    }
}

bool CoordinateManagerParam::saveParam()
{
    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("max_name_length", max_name_length_)
        || !yaml_help_.setParam("max_comment_length", max_comment_length_)
        || !yaml_help_.setParam("max_number_of_coords", max_number_of_coords_)
        || !yaml_help_.setParam("coord_info_file_name", coord_info_file_name_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    }
}

