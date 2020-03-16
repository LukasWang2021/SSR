#ifndef _LINE_SMOOTH_PLANNER_H
#define _LINE_SMOOTH_PLANNER_H

#include "kinematics.h"
#include "pose_quaternion.h"
#include "motion_control_datatype.h"
#include "joint.h"
#include "error_code.h"
#include "bezier_planner.h"
class LineSmoothPlanner
{
public:
	LineSmoothPlanner(void);
	~LineSmoothPlanner(void);
	bool initPlanner(uint32_t joint_num, double cycle_time, basic_alg::Kinematics* pkinematics);
	void setLimit(double position_acc_limit);
	bool setUserTrans(const basic_alg::TransMatrix &uf);
	bool setToolTrans(const basic_alg::TransMatrix &tf);
	ErrorCode planTrajectory(const basic_alg::Joint& start, const basic_alg::Joint& via, const basic_alg::Joint& end, 
		double vout, double vin,
		double uout, double uin, double vout_u, double vin_u);
	ErrorCode sampleTrajectory(double start_time, basic_alg::Joint reference, uint32_t &point_num, fst_mc::JointState *points);
	double getDuration(void);

private:

	basic_alg::PoseQuaternion start_;
	basic_alg::PoseQuaternion via_;
	basic_alg::PoseQuaternion end_;
	BezierPlanner *bezier_curve_;

	double t_total_;
	basic_alg::TransMatrix uf_matrix_;
	basic_alg::TransMatrix tf_matrix_;
	basic_alg::Kinematics*	kinematics_ptr_;

	basic_alg::TransMatrix uf_matrix_inverse_;
	basic_alg::TransMatrix tf_matrix_inverse_;

	double unit_unified_param_;

	double acc_limit_; // mm/s^2
	uint32_t joint_num_;
	double cycle_time_;
	double delta_time_;

	void doFK(const basic_alg::Joint &joint, basic_alg::PoseQuaternion &pose);
	bool doIK(const basic_alg::PoseQuaternion &pose, const basic_alg::Joint reference, basic_alg::Joint &target_joint);

};

#endif