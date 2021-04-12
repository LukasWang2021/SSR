#ifndef AXIS_APPLICATION_PARAM_1000_H
#define AXIS_APPLICATION_PARAM_1000_H

/**
 * @file axis_application_param_1000.h
 * @brief The file defines the class AxisApplicationParam1000.
 * @author zhengyu.shen
 */

#include "model_base.h"
/**
 * @brief system_model_space includes all system model related implementation.
 */
namespace system_model_space
{
/**
 * @brief Defines parameters for axis_application_1000 model.
 */
typedef enum
{
    AxisApplication1000__model_id = 0,       
    AxisApplication1000__sf_limit_enable = 1,
    AxisApplication1000__sf_positive_limit = 2,
    AxisApplication1000__sf_negative_limit = 3,
    AxisApplication1000__zero_offset_lsb = 4,
    AxisApplication1000__zero_offset_msb = 5,
    AxisApplication1000__max_jerk_time = 6,
    AxisApplication1000__max_acc_time = 7,
    AxisApplication1000__max_vel = 8,
    AxisApplication1000__target_reached_count_max = 9,
    AxisApplication1000__pdo_timeout_count_max = 10,
    AxisApplication1000__fbq_size = 11,
    AxisApplication1000__tbq_size = 12,
    AxisApplication1000__number = 13,
}AxisApplicationParam1000_e;
/**
 * @brief Defines the axis_application_1000 model.
 */
class AxisApplicationParam1000 : public ModelBase
{
public:
    /**
     * @brief Constructor of the class.
     * @param [in] style_str Model style.
     * @param [in] file_path The absolute file path of the yaml file.
     */    
    AxisApplicationParam1000(std::string style_str, std::string file_path);
    /**
     * @brief Destructor of the class.
     */    
    ~AxisApplicationParam1000();
    
private:
    ParamDetail_t param_[AxisApplication1000__number];
};

}

#endif

