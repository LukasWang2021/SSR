/*************************************************************************
	> File Name: motion_control_manual_teach.h
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 15时49分10秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_MANUAL_TEACH_H
#define _MOTION_CONTROL_MANUAL_TEACH_H

#include <error_code.h>
#include <motion_control_datatype.h>
#include <log_manager/log_manager_logger.h>

namespace fst_mc
{



class ManualTeach
{
public:
    ManualTeach(size_t joint_num, JointConstraint* pcons, fst_log::Logger* plog);
    ~ManualTeach(void);
    ErrorCode manualStepByDirect(const ManualDirection *directions, MotionTime time, ManualTrajectory &traj);
    ErrorCode manualContinuousByDirect(const ManualDirection *directions, MotionTime time, ManualTrajectory &traj);
    ErrorCode manualByTarget(const Joint &target, MotionTime time, ManualTrajectory &traj);
    ErrorCode manualStop(MotionTime time, ManualTrajectory &traj);

private:
    ErrorCode   manualJointStep(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj);
    ErrorCode   manualJointContinuous(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj);
    ErrorCode   manualJointAPoint(const Joint &target, MotionTime time, ManualTrajectory &traj);

    inline char* printDBLine(const int *data, size_t size, char *buffer, size_t length);
    inline char* printDBLine(const double *data, size_t size, char *buffer, size_t length);

    size_t joint_num_;
    fst_log::Logger* log_ptr_;
    double step_joint_;
    double step_position_;
    double step_orientation_;
    double vel_ratio_;
    double acc_ratio_;
    JointConstraint* joint_constraint_ptr_;
};


}

#endif
