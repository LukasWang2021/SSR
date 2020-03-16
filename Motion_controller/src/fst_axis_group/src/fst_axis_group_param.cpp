#include "fst_axis_group_param.h"
#include "common_file_path.h"
#include <string>

using namespace fst_mc;

FstAxisGroupParam::FstAxisGroupParam():
    log_level_(3),  // default is Error Level
    file_path_(COMPONENT_PARAM_FILE_DIR)
{
    file_path_ += "fst_axis_group.yaml";
}

FstAxisGroupParam::~FstAxisGroupParam()
{

}

bool FstAxisGroupParam::loadParam()
{    
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_))
    {
        return false;
    }
    else    
    {
        return true;
    }
}

bool FstAxisGroupParam::saveParam()
{
    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    }
}

