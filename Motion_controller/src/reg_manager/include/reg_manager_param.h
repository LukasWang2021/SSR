#ifndef REG_MANAGER_PARAM_H
#define REG_MANAGER_PARAM_H


#include "parameter_manager/parameter_manager_param_group.h"
#include <string>

namespace fst_ctrl
{
class RegManagerParam
{
public:
    RegManagerParam();
    ~RegManagerParam();

    bool loadParam();
    bool saveParam();

    // param to load & save
    int log_level_;
    std::string reg_info_dir_;
    std::string pr_reg_file_name_;
    std::string hr_reg_file_name_;
    std::string mr_reg_file_name_;
    std::string sr_reg_file_name_;
    std::string r_reg_file_name_;
    int pr_reg_number_;
    int hr_reg_number_;
    int mr_reg_number_;
    int sr_reg_number_;
    int r_reg_number_;
    double pr_value_limit_;
    double hr_value_limit_;
    int mr_value_limit_;
    int sr_value_limit_;    // size in byte
    double r_value_limit_;
    
private:
    fst_parameter::ParamGroup yaml_help_;
    std::string file_path_;
};

}


#endif

