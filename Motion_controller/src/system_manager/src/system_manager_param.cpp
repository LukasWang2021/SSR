#include "system_manager_param.h"
#include "common_file_path.h"
#include <string>

using namespace fst_ctrl;

SystemManagerParam::SystemManagerParam():
    file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(3)
{
    file_path_ += "system_manager.yaml";
}

SystemManagerParam::~SystemManagerParam()
{

}

bool SystemManagerParam::loadParam()
{
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("install_path", install_path_)
        || !yaml_help_.getParam("config_path", config_path_)
        || !yaml_help_.getParam("runtime_path", runtime_path_)
        || !yaml_help_.getParam("backup_install_path", backup_install_path_)
        || !yaml_help_.getParam("backup_config_path", backup_config_path_)
        || !yaml_help_.getParam("backup_runtime_path", backup_runtime_path_)
        || !yaml_help_.getParam("restore_install_path", restore_install_path_)
        || !yaml_help_.getParam("restore_config_path", restore_config_path_)
        || !yaml_help_.getParam("restore_runtime_path", restore_runtime_path_)
        || !yaml_help_.getParam("password", password_))
    {
        return false;
    }
    else    
    {
        return true;
    } 
}

bool SystemManagerParam::saveParam()
{
    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("install_path", install_path_)
        || !yaml_help_.setParam("config_path", config_path_)
        || !yaml_help_.setParam("runtime_path", runtime_path_)
        || !yaml_help_.setParam("backup_install_path", backup_install_path_)
        || !yaml_help_.setParam("backup_config_path", backup_config_path_)
        || !yaml_help_.setParam("backup_runtime_path", backup_runtime_path_)
        || !yaml_help_.setParam("restore_install_path", restore_install_path_)
        || !yaml_help_.setParam("restore_config_path", restore_config_path_)
        || !yaml_help_.setParam("restore_runtime_path", restore_runtime_path_)
        || !yaml_help_.setParam("password", password_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    } 
}

