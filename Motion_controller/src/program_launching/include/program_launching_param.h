#ifndef PROGRAM_LAUNCHING_PARAM_H
#define PROGRAM_LAUNCHING_PARAM_H

#include "parameter_manager/parameter_manager_param_group.h"

namespace fst_ctrl
{
class ProgramLaunchingParam
{
public:
    ProgramLaunchingParam();
    ~ProgramLaunchingParam();

    bool loadParam();
    bool saveParam();

    // param to load & save
    int log_level_;
    int cycle_count_;

private:
    fst_parameter::ParamGroup yaml_help_;
    std::string file_path_;
};

}


#endif

