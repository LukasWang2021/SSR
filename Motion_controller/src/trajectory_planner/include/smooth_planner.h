/*************************************************************************
	> File Name: smooth_planner.h
	> Author: 
	> Mail: 
	> Created Time: 2019年11月26日 星期二 09时37分08秒
 ************************************************************************/

#ifndef _SMOOTH_PLANNER_H
#define _SMOOTH_PLANNER_H

#include <joint.h>
#include <trajectory_datatype.h>
#include <common_error_code.h>
#include "traj_planner.h"

#define CHECK_POINT_NUM (20)

namespace group_space
{


class SmoothPlanner
{
public:
  	SmoothPlanner(void);
	~SmoothPlanner(void);
	bool initPlanner(uint32_t joint_num, double cycle_time, basic_alg::Kinematics* pkinematics, basic_alg::DynamicAlg* pdynamics, Constraint* pconstraint);
	void setLimit(double vel_limit_position, double acc_limit_position, double jerk_limit_position, 
				  double vel_limit_orientation, double acc_limit_orientation, double jerk_limit_orientation);
	
	bool setCoord(const basic_alg::TransMatrix &coord);
	bool setCoord(const basic_alg::PoseEuler &coord);
	bool setTool(const basic_alg::TransMatrix &tool);
	bool setTool(const basic_alg::PoseEuler &tool);

	ErrorCode planTrajectory(TrajectoryPlanner& prev_planner, TrajectoryPlanner& next_planner, double smooth_distance);
	ErrorCode sampleTrajectory(double start_time, const basic_alg::Joint &reference, JointState &point);
	double getDuration(void);

private:
    void overlapTrajectory(const JointState &prev, const basic_alg::Joint &via, const JointState &next, JointState &result);
	bool checkOmega(const basic_alg::Joint &omega, const basic_alg::Joint &limit_omega);
	bool checkTorque(const basic_alg::Joint &torque);

	double smooth_duration_;
	double smooth_out_time_;
	double smooth_in_time_;

	basic_alg::Joint via_joint_;

	TrajectoryPlanner *prev_planner_ptr_;
	TrajectoryPlanner *next_planner_ptr_;

	std::vector<JointState> prev_traj_;
	std::vector<JointState> next_traj_;

	uint32_t joint_num_;
	double cycle_time_;
	double delta_time_;
	const static uint32_t segment_num_ = 20;

	basic_alg::TransMatrix coord_matrix_;
	basic_alg::TransMatrix coord_matrix_inverse_;
	basic_alg::TransMatrix tool_matrix_;
	basic_alg::TransMatrix tool_matrix_inverse_;

	basic_alg::Joint omega_max_;
	basic_alg::Joint alpha_max_;
	basic_alg::Joint torque_max_;

	basic_alg::Kinematics*	kinematics_ptr_;
	basic_alg::DynamicAlg* dynamics_ptr_;
	Constraint* constraint_ptr_;
};

}

#endif