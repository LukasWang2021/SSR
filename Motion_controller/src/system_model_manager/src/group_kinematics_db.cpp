#include "group_kinematics_db.h"
#include "group_kinematics_param_1000.h"


using namespace system_model_space;

GroupKinematicsDb::GroupKinematicsDb()
{

}

GroupKinematicsDb::~GroupKinematicsDb()
{

}

ModelBase* GroupKinematicsDb::newModel(std::string model_name, std::string file_path)
{
    ModelBase* model_ptr = NULL;
    size_t dot_pos = model_name.find_first_of('.');
    if(dot_pos != std::string::npos)
    {
        std::string name_str = model_name.substr(0, dot_pos);
        std::string style_str = model_name.substr(dot_pos + 1);
        if(name_str.compare("group_kinematics_1000") == 0)
        {
            model_ptr = new GroupKinematicsParam1000(style_str, file_path);
        }
    }
    return model_ptr;    
}

