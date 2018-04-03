/*************************************************************************
	> File Name: motion_plan_manual_teach.h
	> Author: 
	> Mail: 
	> Created Time: 2018年01月11日 星期四 15时27分15秒
 ************************************************************************/

#ifndef _MOTION_PLAN_MANUAL_TEACH_H
#define _MOTION_PLAN_MANUAL_TEACH_H

#include <fst_datatype.h>
#include <motion_plan_error_code.h>

namespace fst_controller
{


class ManualTeach
{
public:
    ManualTeach();
    ~ManualTeach();

    void setMode(const ManualMode mode);
    void setFrame(const ManualFrame frame);
    void setManualDirection(const ManualDirection *directions);
    void setManualTarget(const Joint &target);
    void setManualTarget(const PoseEuler &target);

    ErrorCode stepTeach(void);
    ErrorCode stepTeach(const ManualDirection *directions);
    ErrorCode stepTeach(const Joint &target);
    ErrorCode stepTeach(const PoseEuler &target);

    

private:
    ErrorCode   manualJoint(void);
    ErrorCode   manualCartesian(void);

    ErrorCode   manualJointStep(void);
    ErrorCode   manualJointContinuous(void);
    ErrorCode   manualJointAPoint(void);
    ErrorCode   manualCartesianStep(void);
    ErrorCode   manualCartesianContinuous(void);
    ErrorCode   manualCartesianAPoint(void);

    Joint       manual_target_joint_;
    PoseEuler   manual_target_pose_;

    ManualDirection manual_direction_[6];
    ManualMode      manual_mode_;
    ManualFrame     manual_frame_;
};





}




#endif
