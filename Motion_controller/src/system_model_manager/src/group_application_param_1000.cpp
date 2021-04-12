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
    addParam("pdo_timeout_count_max",       GroupApplication1000__pdo_timeout_count_max);
    addParam("fbq_size",                    GroupApplication1000__fbq_size);
    addParam("tbq_size",                    GroupApplication1000__tbq_size);
    addParam("coupling_numerator_4to3",     GroupApplication1000__coupling_numerator_4to3);
    addParam("coupling_denominator_4to3",   GroupApplication1000__coupling_denominator_4to3);
    addParam("coupling_direction_4to3",     GroupApplication1000__coupling_direction_4to3);
    addParam("max_cartesian_acc_time",      GroupApplication1000__max_cartesian_acc_time);
    addParam("max_cartesian_vel",           GroupApplication1000__max_cartesian_vel);
    addParam("max_angular_acc_time",        GroupApplication1000__max_angular_acc_time);
    addParam("max_angular_vel",             GroupApplication1000__max_angular_vel);    
}

GroupApplicationParam1000::~GroupApplicationParam1000()
{

}

