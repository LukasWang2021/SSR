#include "force_param.h"

using namespace system_model_space;
using namespace base_space;


ForceParam::ForceParam(std::string style_str, std::string file_path):
    ServoBase(style_str, file_path, Force_param_number,
                0, 0,
                0, 0,
                0, 0)
{

}

ForceParam::~ForceParam()
{

}


