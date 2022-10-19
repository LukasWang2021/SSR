#include "axis_application_param_1001.h"
#include "yaml_help.h"
#include <cstring>

using namespace system_model_space;
using namespace base_space;


AxisApplicationParam1001::AxisApplicationParam1001(std::string style_str, std::string file_path):
    ModelBase(MODEL_TYPE_AXIS_APPLICATION, style_str, file_path, &param_[0], AxisApplication1001__number)
{
    memset(&param_, 0, sizeof(ParamDetail_t) * AxisApplication1001__number);
    addParam("param_model_id",                     AxisApplication1001__param_model_id);
    addParam("model_class",                        AxisApplication1001__model_class);
    addParam("model_gear_ratio_numerator",         AxisApplication1001__model_gear_ratio_numerator);
    addParam("model_gear_ratio_denominator",       AxisApplication1001__model_gear_ratio_denominator);
    addParam("model_pitch",                        AxisApplication1001__model_pitch);
    addParam("model_linear_range",                 AxisApplication1001__model_linear_range);
    addParam("model_direction",                    AxisApplication1001__model_direction);
    addParam("model_hw_limit_enable",              AxisApplication1001__model_hw_limit_enable);
    addParam("model_hw_positive_limit",            AxisApplication1001__model_hw_positive_limit);
    addParam("model_hw_negative_limit",            AxisApplication1001__model_hw_negative_limit);
    addParam("encoder_pulse_per_revolution",       AxisApplication1001__encoder_pulse_per_revolution);
    addParam("motor_rated_torque",                 AxisApplication1001__motor_rated_torque);
    addParam("app_max_vel",                        AxisApplication1001__app_max_vel);
    addParam("app_acc_time",                       AxisApplication1001__app_acc_time);
    addParam("app_dec_time",                       AxisApplication1001__app_dec_time);
    addParam("app_sf_limit_enable",                AxisApplication1001__app_sf_limit_enable);
    addParam("app_sf_positive_limit",              AxisApplication1001__app_sf_positive_limit);
    addParam("app_sf_negative_limit",              AxisApplication1001__app_sf_negative_limit);
}

AxisApplicationParam1001::~AxisApplicationParam1001()
{

}

