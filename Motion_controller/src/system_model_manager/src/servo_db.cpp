#include "servo_db.h"
#include "servo_param_1001.h"
#include "force_param.h"


using namespace system_model_space;

ServoDb::ServoDb()
{

}

ServoDb::~ServoDb()
{

}

ServoBase* ServoDb::newModel(std::string model_name, std::string file_path)
{
    ServoBase* model_ptr = NULL;
    size_t dot_pos = model_name.find_first_of('.');
    if(dot_pos != std::string::npos)
    {
        std::string name_str = model_name.substr(0, dot_pos);
        std::string style_str = model_name.substr(dot_pos + 1);
        if (name_str.compare("servo_1001") == 0)
        {
            model_ptr = new ServoParam1001(style_str, file_path);
        }
        else if(name_str.compare("force") == 0)
        {
            model_ptr = new ForceParam(style_str, file_path);
        } 
    }
    return model_ptr;        
}

