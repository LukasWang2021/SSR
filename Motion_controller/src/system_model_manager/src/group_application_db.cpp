#include "group_application_db.h"
#include "group_application_param_1000.h"


using namespace system_model_space;

GroupApplicationDb::GroupApplicationDb()
{

}

GroupApplicationDb::~GroupApplicationDb()
{

}

ModelBase* GroupApplicationDb::newModel(std::string model_name, std::string file_path)
{
    ModelBase* model_ptr = NULL;
    size_t dot_pos = model_name.find_first_of('.');
    if(dot_pos != std::string::npos)
    {
        std::string name_str = model_name.substr(0, dot_pos);
        std::string style_str = model_name.substr(dot_pos + 1);
        if(name_str.compare("group_application_1000") == 0)
        {
            model_ptr = new GroupApplicationParam1000(style_str, file_path);
        }
    }
    return model_ptr;     
}

