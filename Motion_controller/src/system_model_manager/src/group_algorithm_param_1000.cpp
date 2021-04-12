#include "group_algorithm_param_1000.h"
#include "yaml_help.h"
#include <cstring>

using namespace system_model_space;
using namespace base_space;

GroupAlgorithmParam1000::GroupAlgorithmParam1000(std::string style_str, std::string file_path):
    ModelBase(MODEL_TYPE_GROUP_ALGORITHM, style_str, file_path, &param_[0], GroupAlgorithm1000__number)
{
    memset(&param_, 0, sizeof(ParamDetail_t) * GroupAlgorithm1000__number);
    addParam("model_id",            GroupAlgorithm1000__model_id);
    addParam("sampling_interval",   GroupAlgorithm1000__sampling_interval);
    addParam("dof",                 GroupAlgorithm1000__dof);
}

GroupAlgorithmParam1000::~GroupAlgorithmParam1000()
{

}

