#include "process_comm_param.h"
#include "common_file_path.h"
#include <string>

using namespace fst_base;

ProcessCommParam::ProcessCommParam():
    file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(3)
{
    file_path_ += "process_comm.yaml";
}

ProcessCommParam::~ProcessCommParam()
{

}

bool ProcessCommParam::loadParam()
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

bool ProcessCommParam::saveParam()
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

