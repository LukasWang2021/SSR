#ifndef GROUP_APPLICATION_PARAM_1000_H
#define GROUP_APPLICATION_PARAM_1000_H

/**
 * @file group_application_param_1000.h
 * @brief The file defines the class GroupApplicationParam1000.
 * @author zhengyu.shen
 */

#include "model_base.h"
/**
 * @brief system_model_space includes all system model related implementation.
 */
namespace system_model_space
{
/**
 * @brief Defines parameters for group_application_1000 model.
 */
typedef enum
{
    GroupApplication1000__model_id = 0,
    GroupApplication1000__target_reached_count_max = 1,
    GroupApplication1000__pdo_timeout_count_max = 2,
    GroupApplication1000__fbq_size = 3,
    GroupApplication1000__tbq_size = 4,
    GroupApplication1000__coupling_numerator_4to3 = 5,
    GroupApplication1000__coupling_denominator_4to3 = 6,
    GroupApplication1000__coupling_direction_4to3 = 7,
    GroupApplication1000__max_cartesian_acc_time = 8,
    GroupApplication1000__max_cartesian_vel = 9,
    GroupApplication1000__max_angular_acc_time = 10,
    GroupApplication1000__max_angular_vel = 11,    
    GroupApplication1000__number = 12,
}GroupApplicationParam1000_e;
/**
 * @brief Defines the group_application_1000 model.
 */
class GroupApplicationParam1000 : public ModelBase
{
public:
    /**
     * @brief Constructor of the class.
     * @param [in] style_str Model style.
     * @param [in] file_path The absolute file path of the yaml file.
     */    
    GroupApplicationParam1000(std::string style_str, std::string file_path);
    /**
     * @brief Destructor of the class.
     */    
    ~GroupApplicationParam1000();
    
private:
    ParamDetail_t param_[GroupApplication1000__number];
};

}

#endif

