/*************************************************************************
	> File Name: motion_control_arm_group.h
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 14时17分10秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_ARM_GROUP_H
#define _MOTION_CONTROL_ARM_GROUP_H

#include "common_log.h"
#include <motion_control_base_group.h>


#define JOINT_OF_ARM    6

namespace fst_mc
{

class ArmGroup : public BaseGroup
{
  public:
    ArmGroup(fst_log::Logger* plog) : BaseGroup(plog) {};
    ~ArmGroup() {};

    ErrorCode initGroup(fst_base::ErrorMonitor *error_monitor_ptr);

    ErrorCode autoMove(void);

    size_t getNumberOfJoint(void);
    size_t getFIFOLength(void);

    Calibrator* getCalibratorPtr(void);
  
  private:
    bool isJointInConstraint(Joint joint, JointConstraint constraint);
    
    ErrorCode pickFromManual(TrajectoryPoint *point, size_t &length);
    ErrorCode pickFromManualJoint(TrajectoryPoint *point, size_t &length);
    ErrorCode pickFromManualCartesian(TrajectoryPoint *point, size_t &length);

    inline char* printDBLine(const int *data, char *buffer, size_t length);
    inline char* printDBLine(const double *data, char *buffer, size_t length);
};









}

#endif
