#include "interpreter_config.h"
#include "common_file_path.h"


InterpConfig::InterpConfig(/* args */):
    file_path_(COMPONENT_PARAM_FILE_DIR)
{
    file_path_ += "embed_python.yaml";
}

InterpConfig::~InterpConfig()
{
}

bool InterpConfig::loadConfig(void)
{
    if(yaml_help_.loadParamFile(file_path_.c_str()) && 
       yaml_help_.getParam("program_path", prog_path_) &&
       yaml_help_.getParam("module_path", module_path_) &&
       yaml_help_.getParam("interp_thread_priority", interp_thread_priority_))
       return true;
    
    return false;
}

std::string InterpConfig::getProgPath(void)
{
    return prog_path_;
}

std::string InterpConfig::getModulePath(void)
{
    return module_path_;
}

int InterpConfig::getThreadPriority(void)
{
    return interp_thread_priority_;
}



