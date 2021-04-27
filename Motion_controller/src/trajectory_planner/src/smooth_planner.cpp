
/*************************************************************************
	> File Name: smooth_planner.cpp
	> Author: 
	> Mail: 
	> Created Time: 2019年11月26日 星期二 09时38分28秒
 ************************************************************************/
#include <string.h>
#include <iostream>
#include <vector>
#include <common_file_path.h>
#include <basic_alg.h>
#include <trans_matrix.h>
#include <smooth_planner.h>
#include "yaml_help.h"
#include "log_manager_producer.h"

using namespace basic_alg;
using namespace group_space;
using namespace std;
using namespace log_space;

SmoothPlanner::SmoothPlanner(void)
{
	joint_num_ = 0;
	cycle_time_ = 0;
	delta_time_ = 0;
	smooth_duration_ = 0;
	smooth_out_time_ = 0;
	smooth_in_time_ = 0;

	memset(&coord_matrix_, 0, sizeof(coord_matrix_));
	memset(&tool_matrix_, 0, sizeof(tool_matrix_));
	memset(&coord_matrix_inverse_, 0, sizeof(coord_matrix_inverse_));
	memset(&tool_matrix_inverse_, 0, sizeof(tool_matrix_inverse_));
	coord_matrix_.rotation_matrix_.matrix_[0][0] = 1;
	coord_matrix_.rotation_matrix_.matrix_[1][1] = 1;
	coord_matrix_.rotation_matrix_.matrix_[2][2] = 1;
	tool_matrix_.rotation_matrix_.matrix_[0][0] = 1;
	tool_matrix_.rotation_matrix_.matrix_[1][1] = 1;
	tool_matrix_.rotation_matrix_.matrix_[2][2] = 1;
	coord_matrix_inverse_.rotation_matrix_.matrix_[0][0] = 1;
	coord_matrix_inverse_.rotation_matrix_.matrix_[1][1] = 1;
	coord_matrix_inverse_.rotation_matrix_.matrix_[2][2] = 1;
	tool_matrix_inverse_.rotation_matrix_.matrix_[0][0] = 1;
	tool_matrix_inverse_.rotation_matrix_.matrix_[1][1] = 1;
	tool_matrix_inverse_.rotation_matrix_.matrix_[2][2] = 1;

	dynamics_ptr_ = NULL;
	kinematics_ptr_ = NULL;
	constraint_ptr_ = NULL;
	prev_planner_ptr_ = NULL;
	next_planner_ptr_ = NULL;
	
	memset(&omega_max_, 0, sizeof(omega_max_));
	memset(&alpha_max_, 0, sizeof(alpha_max_));
}

SmoothPlanner::~SmoothPlanner(void)
{
}

bool SmoothPlanner::initPlanner(uint32_t joint_num, double cycle_time, Kinematics* pkinematics, DynamicAlg* pdynamics, Constraint* pconstraint)
{
	joint_num_ = joint_num;
	cycle_time_ = cycle_time;
	delta_time_ = cycle_time / 100;
	prev_traj_.resize(segment_num_ + 1);
	next_traj_.resize(segment_num_ + 1);

	kinematics_ptr_ = pkinematics;
	constraint_ptr_ = pconstraint;
	dynamics_ptr_ = pdynamics;

	vector<double> torque_max;
	vector<double> omega_max; 
    vector<double> alpha_max;
	base_space::YamlHelp yaml_help;
	string constraint_file_path = ALGORITHM_DIR;
	constraint_file_path += "constraint.yaml";

	if (!yaml_help.loadParamFile(constraint_file_path.c_str())
		|| !yaml_help.getParam("joint/torque_max", torque_max)
        || !yaml_help.getParam("joint/omega_max", omega_max)
        || !yaml_help.getParam("joint/alpha_max", alpha_max))
    {
        std::cout << " Failed load constraint.yaml " << std::endl;
        return false;
    }

	for (uint32_t j = 0; j < joint_num_; ++j)
    {
		torque_max_[j] = torque_max[j];
        omega_max_[j] = omega_max[j];
        alpha_max_[j] = alpha_max[j];
    }

	return true;
}


void SmoothPlanner::setLimit(double vel_limit_position, double acc_limit_position, double jerk_limit_position, 
				  double vel_limit_orientation, double acc_limit_orientation, double jerk_limit_orientation)
{
}

bool SmoothPlanner::setCoord(const TransMatrix &coord)
{
	TransMatrix inverse_coord = coord;

	if (!inverse_coord.inverse())
	{
		return false;
	}

	coord_matrix_ = coord;
	coord_matrix_inverse_ = inverse_coord;
	return true;
}

bool SmoothPlanner::setTool(const TransMatrix &tool)
{
	TransMatrix inverse_tool = tool;

	if (!inverse_tool.inverse())
	{
		return false;
	}

	tool_matrix_ = tool;
	tool_matrix_inverse_ = inverse_tool;
	return true;
}

bool SmoothPlanner::setCoord(const PoseEuler &coord)
{
	TransMatrix trans_coord, inverse_coord;
	coord.convertToTransMatrix(trans_coord);
	inverse_coord = trans_coord;

	if (!inverse_coord.inverse())
	{
		return false;
	}

	coord_matrix_ = trans_coord;
	coord_matrix_inverse_ = inverse_coord;
	return true;
}

bool SmoothPlanner::setTool(const PoseEuler &tool)
{
	TransMatrix trans_tool, inverse_tool;
	tool.convertToTransMatrix(trans_tool);
	inverse_tool = trans_tool;

	if (!inverse_tool.inverse())
	{
		return false;
	}

	tool_matrix_ = trans_tool;
	tool_matrix_inverse_ = inverse_tool;
	return true;
}


ErrorCode SmoothPlanner::planTrajectory(TrajectoryPlanner& prev_planner, TrajectoryPlanner& next_planner, double smooth_distance)
{
	prev_planner_ptr_ = &prev_planner;
	next_planner_ptr_ = &next_planner;
	//找到圆滑持续时间的初始值
	double smooth_in_time = next_planner_ptr_->getSmoothInTime(smooth_distance);
	double smooth_out_time = prev_planner_ptr_->getSmoothOutTime(smooth_distance);
	double smooth_in_duration = smooth_in_time;
	double smooth_out_duration = prev_planner_ptr_->getDuration() - smooth_out_time;
	double smooth_duration = smooth_in_duration < smooth_out_duration ? smooth_in_duration : smooth_out_duration;
	double sample_step_time = smooth_duration / segment_num_;
	smooth_in_time = smooth_duration;
	smooth_out_time = prev_planner_ptr_->getDuration() - smooth_duration;
	via_joint_ = prev_planner_ptr_->getMotionInfo().target.joint;
	Joint reference = via_joint_;
	Joint limit_omega = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	//找到圆滑运动中各轴的最大转速
	for (uint32_t i = 0; i <= segment_num_; i++)
	{
		double sample_time = smooth_out_time + sample_step_time * i;
		prev_planner_ptr_->sampleTrajectory(sample_time, reference, prev_traj_[i]);
		reference = prev_traj_[i].angle;
		
		for (uint32_t j = 0; j < joint_num_; j++)
		{
			if (fabs(prev_traj_[i].omega[j]) > limit_omega[j])
			{
				limit_omega[j] = fabs(prev_traj_[i].omega[j]);
			}
		}
	}

	reference = via_joint_;

	for (uint32_t i = 0; i <= segment_num_; i++)
	{
		double sample_time = sample_step_time * i;
		next_planner_ptr_->sampleTrajectory(sample_time, reference, next_traj_[i]);
		reference = next_traj_[i].angle;
		
		for (uint32_t j = 0; j < joint_num_; j++)
		{
			if (fabs(next_traj_[i].omega[j]) > limit_omega[j])
			{
				limit_omega[j] = fabs(next_traj_[i].omega[j]);
			}
		}
	}

	// 限制转速最大的轴在圆滑段的转速不超过原轨迹最高转速的120%
	uint32_t index = 0;
	double ratio = limit_omega[0] /  omega_max_[0];

	for (uint32_t j = 1; j < joint_num_; j++)
	{
		if (limit_omega[j] /  omega_max_[j] > ratio)
		{
			index = j;
			ratio = limit_omega[j] /  omega_max_[j];
		}
	}

	for (uint32_t j = 0; j < joint_num_; j++)
	{
		if (j != index)
		{
			limit_omega[j] = omega_max_[j];
		}
		else
		{
			double omg = limit_omega[j] * 1.2;
			limit_omega[j] = omg < omega_max_[j] ? omg : omega_max_[j];
		}
	}
	
	JointState check_state;
	JointVelocity omega;
	JointAcceleration alpha;
	JointTorque torque;
	uint32_t point_num = segment_num_ + 1;
	
	reference = via_joint_;
    //根据限制，计算圆滑实际的持续时间
	while (smooth_duration + MINIMUM_E6 > sample_step_time)
	{
		//FST_INFO("smooth_duration:%.6f, point_num:%u", smooth_duration, point_num);
		bool check_pass = true;

		for (uint32_t i = 0; i < point_num; i++)
		{
			overlapTrajectory(prev_traj_[segment_num_ + 1 - point_num], reference, next_traj_[i], check_state);
			memcpy(static_cast<void*>(&omega), static_cast<void*>(&check_state.omega), sizeof(omega));
			memcpy(static_cast<void*>(&alpha), static_cast<void*>(&check_state.alpha), sizeof(alpha));
			dynamics_ptr_->getTorqueInverseDynamics(check_state.angle, omega, alpha, torque);

			if (!constraint_ptr_->isJointInConstraint(check_state.angle))
			{
				check_pass = false;
				break;
			}

			if (!checkOmega(check_state.omega, limit_omega))
			{
				check_pass = false;
				break;
			}

			if (!checkTorque(*(Joint*)(&torque)))
			{
				check_pass = false;
				break;
			}
		}

		if (check_pass)
		{
			break;
		}

		smooth_duration -= sample_step_time;
		point_num--;
	}

	smooth_duration_ = smooth_duration;
	smooth_in_time_ = smooth_duration;
	smooth_out_time_ = prev_planner_ptr_->getDuration() - smooth_duration;
	
	return SUCCESS;
}


void SmoothPlanner::overlapTrajectory(const JointState &prev, const Joint &via, const JointState &next, JointState &result)
{
	for (uint32_t i = 0; i < joint_num_; i++)
	{
		result.angle[i] = prev.angle[i] + next.angle[i] - via[i];
		result.omega[i] = prev.omega[i] + next.omega[i];
		result.alpha[i] = prev.alpha[i] + next.alpha[i];
		result.jerk[i] = prev.jerk[i] + next.jerk[i];
	}
}


ErrorCode SmoothPlanner::sampleTrajectory(double sample_time, const Joint &reference, JointState &point)
{
	JointState prev_state, next_state;
	prev_planner_ptr_->sampleTrajectory(smooth_out_time_ + sample_time, reference, prev_state);
	next_planner_ptr_->sampleTrajectory(sample_time, reference, next_state);
	overlapTrajectory(prev_state, via_joint_, next_state, point);

	JointVelocity omega;
	JointAcceleration alpha;
	JointTorque torque;
	memcpy(static_cast<void*>(&omega), static_cast<void*>(&point.omega), sizeof(omega));
	memcpy(static_cast<void*>(&alpha), static_cast<void*>(&point.alpha), sizeof(alpha));
	dynamics_ptr_->getTorqueInverseDynamics(point.angle, omega, alpha, torque);
	memcpy(static_cast<void*>(&point.torque), static_cast<void*>(&torque), sizeof(point.torque));

	return SUCCESS;
}


double SmoothPlanner::getDuration(void)
{
	return smooth_duration_;
}


bool SmoothPlanner::checkOmega(const Joint &omega, const Joint &limit_omega)
{
	for (uint32_t j = 0; j < joint_num_; ++j)
	{
		if (fabs(omega[j]) > limit_omega[j])
		{
			LogProducer::warn("traj_planner","Joint omega out of constraint: omega[%d] > limit[%d] : %.6f, %.6f", j, j, omega[j], limit_omega[j]);
			return false;
		}
	}

	return true;
}

bool SmoothPlanner::checkTorque(const Joint &torque)
{
	for (uint32_t j = 0; j < joint_num_; ++j)
	{
		if (fabs(torque[j]) > torque_max_[j])
		{
			LogProducer::warn("traj_planner","Joint torque out of constraint: torque[%d] > limit[%d] : %.6f, %.6f", j, j, torque[j], torque_max_[j]);
			return false;
		}
	}

	return true;
}


