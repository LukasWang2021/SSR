#include "axis_application_db.h"
#include "axis_application_param_1000.h"
#include "axis_application_param_1001.h"


using namespace system_model_space;

AxisApplicationDb::AxisApplicationDb()
{

}

AxisApplicationDb::~AxisApplicationDb()
{

}

ModelBase* AxisApplicationDb::newModel(std::string model_name, std::string file_path)
{
    ModelBase* model_ptr = NULL;
    size_t dot_pos = model_name.find_first_of('.');
    if(dot_pos != std::string::npos)
    {
        std::string name_str = model_name.substr(0, dot_pos);
        std::string style_str = model_name.substr(dot_pos + 1);
        if (name_str.compare("axis_application_1001") == 0)
        {
            model_ptr = new AxisApplicationParam1001(style_str, file_path);
        }
    }
    return model_ptr;     
}

