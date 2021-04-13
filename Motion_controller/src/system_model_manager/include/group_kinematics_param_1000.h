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
    GroupKinematics1000__D4 = 13,
    GroupKinematics1000__D5 = 14,
    GroupKinematics1000__A0 = 15,
    GroupKinematics1000__A1 = 16,
    GroupKinematics1000__A2 = 17,
    GroupKinematics1000__A3 = 18,
    GroupKinematics1000__A4 = 19,
    GroupKinematics1000__A5 = 20,
    GroupKinematics1000__ALPHA0 = 21,
    GroupKinematics1000__ALPHA1 = 22,
    GroupKinematics1000__ALPHA2 = 23,
    GroupKinematics1000__ALPHA3 = 24,
    GroupKinematics1000__ALPHA4 = 25,
    GroupKinematics1000__ALPHA5 = 26,
    GroupKinematics1000__THETA0 = 27,
    GroupKinematics1000__THETA1 = 28,
    GroupKinematics1000__THETA2 = 29,
    GroupKinematics1000__THETA3 = 30,
    GroupKinematics1000__THETA4 = 31,
    GroupKinematics1000__THETA5 = 32,
    GroupKinematics1000__D0_offset = 33,
    GroupKinematics1000__D1_offset = 34,
    GroupKinematics1000__D2_offset = 35,
    GroupKinematics1000__D3_offset = 36,
    GroupKinematics1000__D4_offset = 37,
    GroupKinematics1000__D5_offset = 38,
    GroupKinematics1000__A0_offset = 39,
    GroupKinematics1000__A1_offset = 40,
    GroupKinematics1000__A2_offset = 41,
    GroupKinematics1000__A3_offset = 42,
    GroupKinematics1000__A4_offset = 43,
    GroupKinematics1000__A5_offset = 44,
    GroupKinematics1000__ALPHA0_offset = 45,
    GroupKinematics1000__ALPHA1_offset = 46,
    GroupKinematics1000__ALPHA2_offset = 47,
    GroupKinematics1000__ALPHA3_offset = 48,
    GroupKinematics1000__ALPHA4_offset = 49,
    GroupKinematics1000__ALPHA5_offset = 50,
    GroupKinematics1000__THETA0_offset = 51,
    GroupKinematics1000__THETA1_offset = 52,
    GroupKinematics1000__THETA2_offset = 53,
    GroupKinematics1000__THETA3_offset = 54,
    GroupKinematics1000__THETA4_offset = 55,
    GroupKinematics1000__THETA5_offset = 56,
    GroupKinematics1000__number = 57,
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

