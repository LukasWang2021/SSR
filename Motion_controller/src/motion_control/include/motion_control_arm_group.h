/*************************************************************************
	> File Name: motion_control_arm_group.h
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 14时17分10秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_ARM_GROUP_H
#define _MOTION_CONTROL_ARM_GROUP_H

#include <motion_control_base_group.h>

#define JOINT_OF_ARM    6

namespace group_space
{

class ArmGroup : public BaseGroup
{
  public:
    ArmGroup() : BaseGroup() {};
    virtual ~ArmGroup() {};

    virtual ErrorCode initGroup(fst_ctrl::CoordinateManager *coordinate_manager_ptr, fst_ctrl::ToolManager *tool_manager_ptr, 
        std::map<int32_t, axis_space::Axis*>* axis_group_ptr, GroupSm* sm_ptr, servo_comm_space::ServoCpuCommBase* cpu_comm_ptr,
        system_model_space::GroupModel_t* db_ptr);

    virtual char* getModelName(char *buffer, size_t length);
    virtual size_t getNumberOfJoint(void);
    size_t getFIFOLength(void);
    bool isPostureMatch(const basic_alg::Posture &posture_1, const basic_alg::Posture &posture_2);

    virtual inline char* printDBLine(const int *data, char *buffer, size_t length);
    virtual inline char* printDBLine(const double *data, char *buffer, size_t length);

  private:
    ErrorCode LoadConstraintParameters(std::string dir_path);
    ErrorCode LoadBaseGroupParameters(std::string dir_path);
    ErrorCode LoadArmGroupParameters(std::string dir_path);
};

}

#endif
