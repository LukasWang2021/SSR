#ifndef AXIS_APPLICATION_PARAM_1001_H
#define AXIS_APPLICATION_PARAM_1001_H

/**
 * @file axis_application_param_1001.h
 * @brief The file defines the class AxisApplicationParam1001.
 * @author feng.wu
 */

#include "model_base.h"
/**
 * @brief system_model_space includes all system model related implementation.
 */
namespace system_model_space
{
/**
 * @brief Defines parameters for axis_application_1001 model.
 */
typedef enum
{
    AxisApplication1001__param_model_id = 0,     
    AxisApplication1001__model_class = 1,
    AxisApplication1001__model_gear_ratio_numerator = 2,
    AxisApplication1001__model_gear_ratio_denominator = 3,
    AxisApplication1001__model_pitch = 4,
    AxisApplication1001__model_linear_range = 5,
    AxisApplication1001__model_direction = 6,
    AxisApplication1001__model_hw_limit_enable = 7,
    AxisApplication1001__model_hw_positive_limit = 8,
    AxisApplication1001__model_hw_negative_limit = 9, 
    AxisApplication1001__encoder_pulse_per_revolution = 10, 
    AxisApplication1001__motor_rated_torque = 11, 
    AxisApplication1001__app_max_vel = 12,
    AxisApplication1001__app_acc_time = 13,
    AxisApplication1001__app_dec_time = 14,
    AxisApplication1001__app_sf_limit_enable = 15,
    AxisApplication1001__app_sf_positive_limit = 16,
    AxisApplication1001__app_sf_negative_limit = 17,
    AxisApplication1001__number = 18,
}AxisApplicationParam1001_e;
/**
 * @brief Defines the axis_application_1001 model.
 */
class AxisApplicationParam1001 : public ModelBase
{
public:
    /**
     * @brief Constructor of the class.
     * @param [in] style_str Model style.
     * @param [in] file_path The absolute file path of the yaml file.
     */    
    AxisApplicationParam1001(std::string style_str, std::string file_path);
    /**
     * @brief Destructor of the class.
     */    
    ~AxisApplicationParam1001();
    
private:
    ParamDetail_t param_[AxisApplication1001__number];
};

}

#endif

