#ifndef GROUP_KINEMATICS_PARAM_1000_H
#define GROUP_KINEMATICS_PARAM_1000_H

/**
 * @file group_kinematics_param_1000.h
 * @brief The file defines the class GroupKinematicsParam1000.
 * @author zhengyu.shen
 */

#include "model_base.h"
/**
 * @brief system_model_space includes all system model related implementation.
 */
namespace system_model_space
{
/**
 * @brief Defines parameters for group_kinematics_1000 model.
 */
typedef enum
{
    GroupKinematics1000__model_id = 0,
    GroupKinematics1000__BaseD = 1,
    GroupKinematics1000__BaseA = 2,
    GroupKinematics1000__BaseALPHA = 3,
    GroupKinematics1000__BaseTHETA = 4,
    GroupKinematics1000__BaseD_offset = 5,
    GroupKinematics1000__BaseA_offset = 6,
    GroupKinematics1000__BaseALPHA_offset = 7,
    GroupKinematics1000__BaseTHETA_offset = 8,    
    GroupKinematics1000__D0 = 9,
    GroupKinematics1000__D1 = 10,
    GroupKinematics1000__D2 = 11,
    GroupKinematics1000__D3 = 12,
    GroupKinematics1000__A0 = 13,
    GroupKinematics1000__A1 = 14,
    GroupKinematics1000__A2 = 15,
    GroupKinematics1000__A3 = 16,
    GroupKinematics1000__ALPHA0 = 17,
    GroupKinematics1000__ALPHA1 = 18,
    GroupKinematics1000__ALPHA2 = 19,
    GroupKinematics1000__ALPHA3 = 20,
    GroupKinematics1000__THETA0 = 21,
    GroupKinematics1000__THETA1 = 22,
    GroupKinematics1000__THETA2 = 23,
    GroupKinematics1000__THETA3 = 24,
    GroupKinematics1000__D0_offset = 25,
    GroupKinematics1000__D1_offset = 26,
    GroupKinematics1000__D2_offset = 27,
    GroupKinematics1000__D3_offset = 28,
    GroupKinematics1000__A0_offset = 29,
    GroupKinematics1000__A1_offset = 30,
    GroupKinematics1000__A2_offset = 31,
    GroupKinematics1000__A3_offset = 32,
    GroupKinematics1000__ALPHA0_offset = 33,
    GroupKinematics1000__ALPHA1_offset = 34,
    GroupKinematics1000__ALPHA2_offset = 35,
    GroupKinematics1000__ALPHA3_offset = 36,
    GroupKinematics1000__THETA0_offset = 37,
    GroupKinematics1000__THETA1_offset = 38,
    GroupKinematics1000__THETA2_offset = 39,
    GroupKinematics1000__THETA3_offset = 40,
    GroupKinematics1000__number = 41,
}GroupKinematicsParam1000_e;
/**
 * @brief Defines the group_kinematics_1000 model.
 */
class GroupKinematicsParam1000 : public ModelBase
{
public:
    /**
     * @brief Constructor of the class.
     * @param [in] style_str Model style.
     * @param [in] file_path The absolute file path of the yaml file.
     */    
    GroupKinematicsParam1000(std::string style_str, std::string file_path);
    /**
     * @brief Destructor of the class.
     */    
    ~GroupKinematicsParam1000();
    
private:
    ParamDetail_t param_[GroupKinematics1000__number];
};

}


#endif

