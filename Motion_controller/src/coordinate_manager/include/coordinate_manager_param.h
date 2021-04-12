#ifndef COORDINATE_MANAGER_PARAM_H
#define COORDINATE_MANAGER_PARAM_H

#include "yaml_help.h"
#include <string>


namespace fst_ctrl
{
class CoordinateManagerParam
{
public:
    CoordinateManagerParam();
    ~CoordinateManagerParam();

    bool loadParam();
    bool saveParam();

    // param to load & save
    int log_level_;
    int max_name_length_;       // in char
    int max_comment_length_;    // in char
    int max_number_of_coords_;   // without the condition of no tool
    std::string coord_info_file_name_;   // file name under COORD_DIR

private:
    base_space::YamlHelp yaml_help_;
    std::string file_path_;
};

}


#endif

