#include "program_launching_param.h"
#include "common_file_path.h"
#include <string>

using namespace fst_ctrl;

ProgramLaunchingParam::ProgramLaunchingParam():
    file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(3),  // default is Error Level
    cycle_count_(1)
{
    file_path_ += "program_launching.yaml";
}

ProgramLaunchingParam::~ProgramLaunchingParam()
{

}

bool ProgramLaunchingParam::loadParam()
{
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("cycle_count", cycle_count_))
    {
        return false;
    }
    else    
    {
        return true;
    } 
}

bool ProgramLaunchingParam::saveParam()
{
    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("cycle_time", cycle_count_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    } 
}

