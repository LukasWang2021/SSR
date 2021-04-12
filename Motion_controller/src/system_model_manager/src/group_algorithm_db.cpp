#include "group_algorithm_db.h"
#include "group_algorithm_param_1000.h"


using namespace system_model_space;

GroupAlgorithmDb::GroupAlgorithmDb()
{

}

GroupAlgorithmDb::~GroupAlgorithmDb()
{

}

ModelBase* GroupAlgorithmDb::newModel(std::string model_name, std::string file_path)
{
    ModelBase* model_ptr = NULL;
    size_t dot_pos = model_name.find_first_of('.');
    if(dot_pos != std::string::npos)
    {
        std::string name_str = model_name.substr(0, dot_pos);
        std::string style_str = model_name.substr(dot_pos + 1);
        if(name_str.compare("group_algorithm_1000") == 0)
        {
            model_ptr = new GroupAlgorithmParam1000(style_str, file_path);
        }
    }
    return model_ptr;    
}

