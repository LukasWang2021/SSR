/*************************************************************************
	> File Name: motion_control_manual_teach.h
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 15时49分10秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_MANUAL_TEACH_H
#define _MOTION_CONTROL_MANUAL_TEACH_H

#include <string>
#include <common_error_code.h>
#include <motion_control_datatype.h>
#include <joint_constraint.h>
#include <kinematics.h>
#include <dynamic_alg.h>
#include <ds_planner/ds_planner_single_jerk.h>
#include <trans_matrix.h>
#include "log_manager_producer.h"

namespace fst_mc
{



class ManualTeach
{
public:
    ManualTeach(void);
    ~ManualTeach(void);

    ErrorCode init(basic_alg::Kinematics *pkinematics, basic_alg::DynamicAlg* pdynamics, Constraint *pcons, const std::string &config_file);
    void getManualStepAxis(double *steps);
    double getManualStepPosition(void);
    double getManualStepOrientation(void);
    ErrorCode setManualStepAxis(const double *steps);
    ErrorCode setManualStepPosition(double step);
    ErrorCode setManualStepOrientation(double step);
    void setGlobalVelRatio(double ratio);
    void setGlobalAccRatio(double ratio);
    void setToolFrame(const basic_alg::PoseEuler &tf);
    void setUserFrame(const basic_alg::PoseEuler &uf);
    void setWorldFrame(const basic_alg::PoseEuler &wf);
    void setManualFrame(ManualFrame frame);
    ManualMode getManualMode(void);
    ManualFrame getManualFrame(void);

    ErrorCode manualStep(const ManualDirection *directions, const basic_alg::Joint &start);
    ErrorCode manualContinuous(const ManualDirection *directions, const basic_alg::Joint &start);
    ErrorCode manualContinuous(const ManualDirection *directions, double time);
    ErrorCode manualToJoint(const basic_alg::Joint &start, const basic_alg::Joint &end);
    ErrorCode manualStop(double stop_time);

    double getDuration(void);
    void getDirection(ManualDirection *directions);
    ErrorCode sampleTrajectory(double sample_time, const basic_alg::Joint &reference, JointState &state);

private:
    ErrorCode manualStepInJoint(const ManualDirection *dir, const basic_alg::Joint &start);
    ErrorCode manualStepInBase(const ManualDirection *dir, const basic_alg::Joint &start);
    ErrorCode manualStepInUser(const ManualDirection *dir, const basic_alg::Joint &start);
    ErrorCode manualStepInWorld(const ManualDirection *dir, const basic_alg::Joint &start);
    ErrorCode manualStepInTool(const ManualDirection *dir, const basic_alg::Joint &start);
    ErrorCode manualContinuousInJoint(const ManualDirection *dir, const basic_alg::Joint &start);
    ErrorCode manualContinuousInBase(const ManualDirection *dir, const basic_alg::Joint &start);
    ErrorCode manualContinuousInUser(const ManualDirection *dir, const basic_alg::Joint &start);
    ErrorCode manualContinuousInTool(const ManualDirection *dir, const basic_alg::Joint &start);
    ErrorCode manualContinuousInWorld(const ManualDirection *dir, const basic_alg::Joint &start);
    ErrorCode manualContinuousInJoint(const ManualDirection *dir, double time);
    ErrorCode manualContinuousInCartesian(const ManualDirection *dir, double time);
    basic_alg::PoseEuler interpolatPoseEuler(double p, const basic_alg::PoseEuler &start, const basic_alg::PoseEuler &end);

    inline char* printDBLine(const int *data, char *buffer, uint32_t length);
    inline char* printDBLine(const double *data, char *buffer, uint32_t length);

    size_t joint_num_;
    double step_axis_[NUM_OF_JOINT];
    double step_position_;
    double step_orientation_;
    double vel_ratio_;
    double acc_ratio_;

    double move_to_point_vel_[NUM_OF_JOINT];
    double axis_vel_[NUM_OF_JOINT];
    double axis_acc_[NUM_OF_JOINT];
    double axis_jerk_[NUM_OF_JOINT];
    double position_vel_reference_;
    double position_acc_reference_;
    double position_jerk_reference_;
    double orientation_omega_reference_;
    double orientation_alpha_reference_;
    double orientation_beta_reference_;

    Constraint *joint_constraint_ptr_;
    basic_alg::Kinematics *kinematics_ptr_;
    basic_alg::DynamicAlg *dynamics_ptr_;
    std::string manual_config_file_;

    SingleJerkDSCurvePlanner ds_curve_;
    SingleJerkDSCurvePlanner axis_ds_curve_[NUM_OF_JOINT];
    basic_alg::Joint joint_start_;          // 示教运动的开始关节位置
    basic_alg::Joint joint_end_;            // 示教运动的结束关节位置
    basic_alg::PoseEuler cart_start_;       // 示教运动开始的位姿
    basic_alg::PoseEuler cart_end_;         // 示教运动结束的位姿
    basic_alg::PoseEuler auxiliary_coord_;  // 辅助坐标系
    basic_alg::TransMatrix wf_matrix_;
    basic_alg::TransMatrix wf_matrix_inverse_;
    basic_alg::TransMatrix uf_matrix_;
    basic_alg::TransMatrix uf_matrix_inverse_;
    basic_alg::TransMatrix tf_matrix_;
    basic_alg::TransMatrix tf_matrix_inverse_;
    ManualDirection motion_direction_[NUM_OF_JOINT];
    bool axis_ds_curve_valid_[NUM_OF_JOINT];
    double axis_ds_curve_start_time_[NUM_OF_JOINT];
    double total_duration_;     // 示教运动的总耗时
    ManualMode motion_type_;     // 示教运动的运动模式，步进、连续、到给定目标点          
    ManualFrame frame_type_;    // 示教运动所在的坐标系，关节空间、基座坐标系、用户坐标系、世界坐标系、工具坐标系
    bool stop_teach_;

};


}

#endif
