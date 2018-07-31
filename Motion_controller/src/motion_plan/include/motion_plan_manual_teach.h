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

struct ManualCoef
{
    MotionTime start_time;
    MotionTime stable_time;
    MotionTime brake_time;
    MotionTime stop_time;

    double  start_alpha;
    double  brake_alpha;
};

struct ManualTrajectory
{
    ManualMode      mode;
    ManualFrame     frame;
    ManualDirection direction[6];

    Joint       joint_start;
    Joint       joint_ending;

    PoseEuler   cart_start;
    PoseEuler   cart_ending;

    MotionTime      duration;
    ManualCoef      coeff[6];
};

class ManualTeach
{
public:
    ManualTeach();
    ~ManualTeach();

    ErrorCode stepTeach(const ManualDirection *directions, MotionTime time, ManualTrajectory &traj);
    ErrorCode stepTeach(const Joint &target, MotionTime time, ManualTrajectory &traj);

    ErrorCode stopTeach(MotionTime time, ManualTrajectory &traj);
    ErrorCode repairCartesianContinuous(MotionTime time, MotionTime front, ManualTrajectory &traj);

private:
    ErrorCode   manualJoint(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj);
    ErrorCode   manualCartesian(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj);

    ErrorCode   manualJointStep(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj);
    ErrorCode   manualJointContinuous(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj);
    ErrorCode   manualJointAPoint(const Joint &target, MotionTime time, ManualTrajectory &traj);
    ErrorCode   manualCartesianStep(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj);
    ErrorCode   manualCartesianContinuous(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj);

    //ErrorCode   stopJointContinuous(MotionTime time);
    //ErrorCode   stopCartesianContinuous(MotionTime time);

    Joint       manual_target_joint_;
};





}




#endif
