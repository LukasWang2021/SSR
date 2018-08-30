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
#include <motion_control_constraint.h>
#include <log_manager/log_manager_logger.h>
#include <parameter_manager/parameter_manager_param_group.h>
#include <base_kinematics.h>

namespace fst_mc
{



class ManualTeach
{
public:
    ManualTeach(void);
    ~ManualTeach(void);

    ErrorCode init(BaseKinematics *pkinematics, Constraint *pcons, fst_log::Logger *plog, fst_parameter::ParamGroup &config);
    ErrorCode manualStepByDirect(const ManualDirection *directions, MotionTime time, ManualTrajectory &traj);
    ErrorCode manualContinuousByDirect(const ManualDirection *directions, MotionTime time, ManualTrajectory &traj);
    ErrorCode manualByTarget(const Joint &target, MotionTime time, ManualTrajectory &traj);
    ErrorCode manualStop(MotionTime time, ManualTrajectory &traj);

private:
    ErrorCode   manualJointStep(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj);
    ErrorCode   manualJointContinuous(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj);
    ErrorCode   manualJointAPoint(const Joint &target, MotionTime time, ManualTrajectory &traj);

    ErrorCode   manualCartesianStep(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj);
    ErrorCode   manualCartesianContinuous(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj);

    inline char* printDBLine(const int *data, char *buffer, size_t length);
    inline char* printDBLine(const double *data, char *buffer, size_t length);

    size_t joint_num_;
    double step_joint_;
    double step_position_;
    double step_orientation_;
    double vel_ratio_;
    double acc_ratio_;

    double axis_vel_[NUM_OF_JOINT];
    double axis_acc_[NUM_OF_JOINT];
    double position_vel_reference_;
    double position_acc_reference_;
    double orientation_omega_reference_;
    double orientation_alpha_reference_;

    Constraint *joint_constraint_ptr_;
    BaseKinematics *kinematics_ptr_;
    fst_log::Logger *log_ptr_;
};


}

#endif
