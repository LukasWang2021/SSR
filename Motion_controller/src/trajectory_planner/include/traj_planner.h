/*************************************************************************
	> File Name: traj_planner.h
	> Author: 
	> Mail: 
	> Created Time: 2019年11月26日 星期二 16时45分01秒
 ************************************************************************/

#ifndef _TRAJ_PLANNER_H
#define _TRAJ_PLANNER_H

#include <trans_matrix.h>
#include <kinematics.h>
#include <dynamic_alg.h>
#include <joint_constraint.h>
#include <common_error_code.h>
#include <motion_control_datatype.h>
#include <line_planner.h>
#include <joint_planner.h>
#include <circle_planner.h>
#include <traj_params.h>


namespace fst_mc
{
class TrajectoryPlanner
{
public:
	TrajectoryPlanner(void);
	~TrajectoryPlanner(void);
	bool initPlanner(uint32_t joint_num, double cycle_time, basic_alg::Kinematics* pkinematics, basic_alg::DynamicAlg* pdynamics, fst_mc::Constraint* pconstraint);
	//void setLimit(const basic_alg::Joint &vel_limit_joint, const basic_alg::Joint &acc_limit_joint, const basic_alg::Joint &jerk_limit_joint, 
	//			  double vel_limit_position, double acc_limit_position, double jerk_limit_position,
	//			  double vel_limit_orientation, double acc_limit_orientation, double jerk_limit_orientation);
	bool setUserTrans(const basic_alg::TransMatrix &uf);
	bool setToolTrans(const basic_alg::TransMatrix &tf);
	bool setUserFrame(const basic_alg::PoseEuler &uf);
	bool setToolFrame(const basic_alg::PoseEuler &tf);
	ErrorCode planTrajectory(const basic_alg::Joint &start, const fst_mc::MotionInfo &target, double vel_ratio, double acc_ratio);
	ErrorCode sampleTrajectory(double sample_time, const basic_alg::Joint &reference, fst_mc::JointState &point);
	ErrorCode sampleTrajectory(double start_time, basic_alg::Joint reference, uint32_t &point_num, fst_mc::JointState *points);
	ErrorCode sampleCartesianTrajectory(double start_time, basic_alg::Joint reference, uint32_t &point_num, 
		fst_mc::JointState &point, basic_alg::PoseQuaternion &pose, double &postion_vel);
	void sampleLineNormalTrajectory(double sample_time, double &sample_u, double &sample_v, double &sample_a);
	double getDuration(void);
	double getSmoothInTime(double smooth_distance);
	double getSmoothOutTime(double smooth_distance);
	const fst_mc::MotionInfo& getMotionInfo(void) const;

	ErrorCode sampleCircleCartesianTrajectory(double sample_time, basic_alg::PoseQuaternion &pose, double &postion_vel);
	ErrorCode sampleLineCartesianTrajectory(double sample_time, basic_alg::PoseQuaternion &pose);
	ErrorCode sampleTrajectoryJoint(double start_time, basic_alg::Joint reference, basic_alg::Joint &point);

	void getTrajectoryLimit(basic_alg::Joint &omega_limit, basic_alg::Joint &alpha_limit, basic_alg::Joint *beta_limit, uint32_t &beta_limit_siz);
	void useSwiftParam(void);
	void useStableParam(void);
private:
	bool isEqual(const basic_alg::Joint &joint_a, const basic_alg::Joint &joint_b, double threshold = 0.001);
	// 仅在内部校核时使用,内部不计算逆动力学
	ErrorCode sampleTrajectory(double sample_time, fst_mc::JointState &point);
	ErrorCode planCircleTrajectory(double vel_ratio, double acc_ratio);
	ErrorCode planJointTrajectory(double vel_ratio, double acc_ratio);
	ErrorCode planLineTrajectory(double vel_ratio, double acc_ratio);
	ErrorCode checkTrajectory(void);
	double getSegmentTimeForOutPoint();
	double getSegmentTimeForInPoint();

	enum {CHECK_POINT_NUM_MIN = 5, CHECK_POINT_NUM = 25, CHECK_POINT_NUM_MAX = 50};

	uint32_t joint_num_;
	TrajParams traj_params_;
	double cycle_time_;
	double delta_time_;
	basic_alg::Joint start_joint_;
	fst_mc::MotionInfo target_info_;
	basic_alg::TransMatrix uf_matrix_;
	basic_alg::TransMatrix tf_matrix_;
	basic_alg::TransMatrix uf_matrix_inverse_;
	basic_alg::TransMatrix tf_matrix_inverse_;
	basic_alg::Kinematics* kinematics_ptr_;
	basic_alg::DynamicAlg* dynamics_ptr_;
	fst_mc::Constraint* constraint_ptr_;
	LinePlanner line_planner_;
	JointPlanner joint_planner_;
	CirclePlanner circle_planner_;

};

}

#endif