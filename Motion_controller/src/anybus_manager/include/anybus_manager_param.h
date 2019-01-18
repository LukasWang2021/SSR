#ifndef ANYBUS_MANAGER_PARAM_H
#define ANYBUS_MANAGER_PARAM_H


#include "parameter_manager/parameter_manager_param_group.h"

namespace fst_anybus
{
class AnybusManagerParam
{
public:
    AnybusManagerParam();
    ~AnybusManagerParam();

    bool loadParam();
    bool saveParam();

    // param to load & save
    int log_level_;
    int operation_mode_;
    int cycle_time_;
    int abcc_page_size_;
    unsigned int abcc_page_addr_;
    int appl_page_size_;
    unsigned int appl_page_addr_;
    bool safety_enable_;
    std::string ethercat_config_file_name_;

private:
    fst_parameter::ParamGroup yaml_help_;
    std::string file_path_;
};

}


#endif

