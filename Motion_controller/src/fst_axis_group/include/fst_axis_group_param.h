#ifndef FST_AXIS_GROUP_PARAM_H
#define FST_AXIS_GROUP_PARAM_H


#include "parameter_manager/parameter_manager_param_group.h"

namespace fst_mc
{
class FstAxisGroupParam
{
public:
    FstAxisGroupParam();
    ~FstAxisGroupParam();

    bool loadParam();
    bool saveParam();

    // param to load & save
    int log_level_;

private:
    fst_parameter::ParamGroup yaml_help_;
    std::string file_path_;
};

}


#endif

