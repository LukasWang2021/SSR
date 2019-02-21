/*************************************************************************
	> File Name: motion_control_manual_teach.h
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 15时49分10秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_MANUAL_TEACH_H
#define _MOTION_CONTROL_MANUAL_TEACH_H

#include <string>
#include <error_code.h>
#include <motion_control_datatype.h>
#include <motion_control_constraint.h>
#include <log_manager/log_manager_logger.h>
#include <base_kinematics.h>

namespace fst_mc
{



class ManualTeach
{
public:
    ManualTeach(void);
    ~ManualTeach(void);

    ErrorCode init(BaseKinematics *pkinematics, Constraint *pcons, fst_log::Logger *plog, const std::string &config_file);

    double getGlobalVelRatio(void);
    double getGlobalAccRatio(void);
    double getManualStepAxis(void);
    double getManualStepPosition(void);
    double getManualStepOrientation(void);
    ErrorCode setGlobalVelRatio(double ratio);
    ErrorCode setGlobalAccRatio(double ratio);
    ErrorCode setManualStepAxis(double step);
    ErrorCode setManualStepPosition(double step);
    ErrorCode setManualStepOrientation(double step);

    ErrorCode manualStepByDirect(const ManualDirection *directions, MotionTime time, ManualTrajectory &traj);
    ErrorCode manualContinuousByDirect(const ManualDirection *directions, MotionTime time, ManualTrajectory &traj);
    ErrorCode manualByTarget(const basic_alg::Joint &target, MotionTime time, ManualTrajectory &traj);
    ErrorCode manualStop(MotionTime time, ManualTrajectory &traj);

private:
    ErrorCode   manualJointStep(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj);
    ErrorCode   manualJointContinuous(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj);
    ErrorCode   manualJointAPoint(const basic_alg::Joint &target, MotionTime time, ManualTrajectory &traj);

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
    std::string manual_config_file_;
};


}

#endif
