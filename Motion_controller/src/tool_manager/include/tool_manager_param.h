#ifndef TOOL_MANAGER_PARAM_H
#define TOOL_MANAGER_PARAM_H


#include "parameter_manager/parameter_manager_param_group.h"
#include <string>

namespace fst_ctrl
{
class ToolManagerParam
{
public:
    ToolManagerParam();
    ~ToolManagerParam();

    bool loadParam();
    bool saveParam();

    // param to load & save
    int log_level_;
    int max_name_length_;       // in char
    int max_comment_length_;    // in char
    int max_number_of_tools_;   // without the condition of no tool
    std::string tool_info_file_name_;   // file name under TOOL_DIR

private:
    fst_parameter::ParamGroup yaml_help_;
    std::string file_path_;
};

}


#endif

