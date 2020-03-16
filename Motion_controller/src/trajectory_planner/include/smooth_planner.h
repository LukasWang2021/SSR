/*************************************************************************
	> File Name: smooth_planner.h
	> Author: 
	> Mail: 
	> Created Time: 2019年11月26日 星期二 09时37分08秒
 ************************************************************************/

#ifndef _SMOOTH_PLANNER_H
#define _SMOOTH_PLANNER_H

#include <joint.h>
#include <motion_control_datatype.h>
#include <error_code.h>
#include <line_smooth_planner.h>
#include <joint_smooth_planner.h>
#include "line_planner.h"
#include "bezier_planner/orientation_planner.h"
#include "bezier_planner/postion_planner.h"
#include "spline_planner/cubic_spline_planner.h"
#include "spline_planner/spline_orientation_planner.h"
#include "traj_planner.h"


class SmoothPlanner
{
public:
  	SmoothPlanner(void);
	~SmoothPlanner(void);
	bool initPlanner(uint32_t joint_num, double cycle_time, basic_alg::Kinematics* pkinematics, fst_mc::Constraint* pconstraint, fst_log::Logger* log_ptr);
	void setLimit(const basic_alg::Joint &vel_limit_joint, const basic_alg::Joint &acc_limit_joint, const basic_alg::Joint &jerk_limit_joint, 
				  double vel_limit_position, double acc_limit_position, double jerk_limit_position, 
				  double vel_limit_orientation, double acc_limit_orientation, double jerk_limit_orientation);
	bool setUserTrans(const basic_alg::TransMatrix &uf);
	bool setToolTrans(const basic_alg::TransMatrix &tf);
	bool setUserFrame(const basic_alg::PoseEuler &uf);
	bool setToolFrame(const basic_alg::PoseEuler &tf);

	void setTrajectoryPlanner(fst_mc::TrajectoryPlanner& traj_planner_pre, fst_mc::TrajectoryPlanner& traj_planner_this);
	void updateTrajectoryInfo(double point_out_time, double point_in_time, double cycle_time,
		double &orientation_out_time, double &orientation_in_time);

	ErrorCode planTrajectory(const fst_mc::JointState &smooth_out_state, const basic_alg::Joint &fly_by_joint, const fst_mc::JointState &smooth_in_state);
	ErrorCode planCircleTrajectory(const basic_alg::PoseQuaternion pose_out, const basic_alg::PoseQuaternion pose_via, const basic_alg::PoseQuaternion pose_in,
		double position_out_vel, double position_in_vel,
		const fst_mc::JointState &smooth_out_state, const fst_mc::JointState &smooth_in_state);
	ErrorCode sampleTrajectory(double start_time, basic_alg::Joint reference, uint32_t &point_num, fst_mc::JointState *points);
	double getDuration(void);

private:
	ErrorCode planTrajectoryJ2J(const fst_mc::JointState &smooth_out_state, const fst_mc::JointState &smooth_in_state);
	ErrorCode planTrajectoryL2L(const fst_mc::JointState &smooth_out_state, const basic_alg::Joint &fly_by_joint, const fst_mc::JointState &smooth_in_state);

	void unifyPointUint(const basic_alg::Point &point, basic_alg::Point &unify_point);
	void returnPointUint(basic_alg::Point &point);
	ErrorCode sampleTrajectoryL2L(double sample_time, basic_alg::Joint reference, uint32_t &point_num, fst_mc::JointState *points);

	void doFK(const basic_alg::Joint &joint, basic_alg::PoseQuaternion &target);
	bool doIK(const basic_alg::PoseQuaternion &pose, const basic_alg::Joint reference, basic_alg::Joint &target_joint);

	ErrorCode checkL2LTrajectory(double time_start, double time_interval);

	fst_mc::MotionType motion_type_pre_;
	fst_mc::MotionType motion_type_this_;

	double uout_;
	double uin_;
	double vout_u_;
	double vin_u_;

	double point_smooth_total_time_;
	double t_total_;
	double t_former_;
	double t_last_;

	double orientation_out_time_;
	double orientation_in_time_;
	double point_out_time_;
	double point_in_time_;

	basic_alg::Joint joint_out_angle_;
	basic_alg::Quaternion quatern_out_;
	basic_alg::Quaternion quatern_in_;
	basic_alg::Quaternion quatern_mid_;
	basic_alg::Point point_out_;
	basic_alg::Point point_in_;
	basic_alg::PoseQuaternion via_pose_;
	double unit_unified_param_;
	double acc_limit_position_;

	fst_mc::TrajectoryPlanner traj_planner_pre_;
	fst_mc::TrajectoryPlanner traj_planner_this_;

	BezierPositionPlanner position_planner_;
	BezierOrientationPlanner orientation_planner_;
	SplineOrientationPlanner spline_orientation_planner_;

	uint32_t joint_num_;
	double cycle_time_;
	double delta_time_;
	basic_alg::TransMatrix uf_matrix_;
	basic_alg::TransMatrix tf_matrix_;
	basic_alg::TransMatrix uf_matrix_inverse_;
	basic_alg::TransMatrix tf_matrix_inverse_;
	basic_alg::Kinematics*	kinematics_ptr_;

	basic_alg::Joint omega_max_, alpha_max_, beta_max_;

	JointSmoothPlanner joint_smooth_planner_;

	fst_mc::Constraint* constraint_ptr_;
	fst_log::Logger* log_ptr_;
};


#endif
