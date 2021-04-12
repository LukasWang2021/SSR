#include "log_manager_config.h"
#include "common_file_path.h"
#include <string>

using namespace log_space;

LogManagerConfig::LogManagerConfig():
	display_enable_(false),
	log_enable_(true),
	cycle_time_(10000),
	max_file_log_item_(0),
	max_log_size_(0),
	percent_log_retain_(0),
    file_path_(COMPONENT_PARAM_FILE_DIR)
{
    file_path_ += "log_manager.yaml";
}


bool LogManagerConfig::loadParam()
{

    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("display_enable", display_enable_)
        || !yaml_help_.getParam("log_enable", log_enable_)
        || !yaml_help_.getParam("cycle_time", cycle_time_)
        || !yaml_help_.getParam("max_file_log_item", max_file_log_item_)
        || !yaml_help_.getParam("max_log_size", max_log_size_)
        || !yaml_help_.getParam("percent_log_retain", percent_log_retain_)
        || !yaml_help_.getParam("log_path", log_path_))
    {
        return false;
    }
    else    
    {
        return true;
    }
}

bool LogManagerConfig::saveParam()
{
    if(!yaml_help_.setParam("display_enable", display_enable_)
        || !yaml_help_.setParam("log_enable", log_enable_)
        || !yaml_help_.setParam("cycle_time", cycle_time_)
        || !yaml_help_.setParam("max_file_log_item", max_file_log_item_)
        || !yaml_help_.setParam("max_log_size", max_log_size_)
        || !yaml_help_.setParam("percent_log_retain", percent_log_retain_)
        || !yaml_help_.setParam("log_path", log_path_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    }
}
