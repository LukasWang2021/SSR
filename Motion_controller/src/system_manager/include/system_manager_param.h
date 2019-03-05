#ifndef SYSTEM_MANAGER_PARAM_H
#define SYSTEM_MANAGER_PARAM_H


#include "parameter_manager/parameter_manager_param_group.h"

namespace fst_ctrl
{
class SystemManagerParam
{
public:
    SystemManagerParam();
    ~SystemManagerParam();

    bool loadParam();
    bool saveParam();

    // param to load & save
    int log_level_;
    std::string install_path_;
    std::string config_path_;
    std::string runtime_path_;

    std::string backup_install_path_;
    std::string backup_config_path_;
    std::string backup_runtime_path_;

    std::string restore_install_path_;
    std::string restore_config_path_;
    std::string restore_runtime_path_;
    
    std::string password_;


private:
    fst_parameter::ParamGroup yaml_help_;
    std::string file_path_;
 
};

}


#endif

