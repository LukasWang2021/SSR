#include "group_application_param_1000.h"
#include "yaml_help.h"
#include <cstring>

using namespace system_model_space;
using namespace base_space;


GroupApplicationParam1000::GroupApplicationParam1000(std::string style_str, std::string file_path):
    ModelBase(MODEL_TYPE_GROUP_APPLICATION, style_str, file_path, &param_[0], GroupApplication1000__number)
{
    memset(&param_, 0, sizeof(ParamDetail_t) * GroupApplication1000__number);
    addParam("model_id",                    GroupApplication1000__model_id);
    addParam("target_reached_count_max",    GroupApplication1000__target_reached_count_max);
    addParam("coupling_numerator_5to6",     GroupApplication1000__coupling_numerator_5to6);
    addParam("coupling_denominator_5to6",   GroupApplication1000__coupling_denominator_5to6);
    addParam("coupling_direction_5to6",     GroupApplication1000__coupling_direction_5to6); 
}

GroupApplicationParam1000::~GroupApplicationParam1000()
{

}

