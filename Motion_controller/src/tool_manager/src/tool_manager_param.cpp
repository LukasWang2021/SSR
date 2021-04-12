#include "tool_manager_param.h"
#include "common_file_path.h"
#include <string>

using namespace fst_ctrl;

ToolManagerParam::ToolManagerParam():
    log_level_(0),
    max_name_length_(32),
    max_comment_length_(32),
    max_number_of_tools_(10),
    file_path_(COMPONENT_PARAM_FILE_DIR)
{
    file_path_ += "tool_manager.yaml";
}

ToolManagerParam::~ToolManagerParam()
{

}

bool ToolManagerParam::loadParam()
{    
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("max_name_length", max_name_length_)
        || !yaml_help_.getParam("max_comment_length", max_comment_length_)
        || !yaml_help_.getParam("max_number_of_tools", max_number_of_tools_)
        || !yaml_help_.getParam("tool_info_file_name", tool_info_file_name_))
    {
        return false;
    }
    else    
    {
        return true;
    }
}

bool ToolManagerParam::saveParam()
{
    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("max_name_length", max_name_length_)
        || !yaml_help_.setParam("max_comment_length", max_comment_length_)
        || !yaml_help_.setParam("max_number_of_tools", max_number_of_tools_)
        || !yaml_help_.setParam("tool_info_file_name", tool_info_file_name_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    }
}

