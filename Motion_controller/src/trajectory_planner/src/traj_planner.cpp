/*************************************************************************
	> File Name: traj_planner.cpp
	> Author: 
	> Mail: 
	> Created Time: 2019年11月27日 星期三 11时06分43秒
 ************************************************************************/

#include <string.h>
#include <math.h>
#include <iostream>
#include <basic_alg.h>
#include <traj_planner.h>

using namespace fst_mc;
using namespace basic_alg;

namespace fst_mc
{

TrajectoryPlanner::TrajectoryPlanner(void)
{
	joint_num_ = 0;
	cycle_time_ = 0;
	delta_time_ = 0;
	target_info_.type = MOTION_NONE;
	memset(&uf_matrix_, 0, sizeof(uf_matrix_));
	uf_matrix_.rotation_matrix_.matrix_[0][0] = 1;
	uf_matrix_.rotation_matrix_.matrix_[1][1] = 1;
	uf_matrix_.rotation_matrix_.matrix_[2][2] = 1;
	memset(&tf_matrix_, 0, sizeof(tf_matrix_));
	tf_matrix_.rotation_matrix_.matrix_[0][0] = 1;
	tf_matrix_.rotation_matrix_.matrix_[1][1] = 1;
	tf_matrix_.rotation_matrix_.matrix_[2][2] = 1;
	kinematics_ptr_ = NULL;
}

TrajectoryPlanner::~TrajectoryPlanner(void)
{}

bool TrajectoryPlanner::initPlanner(uint32_t joint_num, double cycle_time, Kinematics* pkinematics, DynamicAlg* pdynamics, Constraint* pconstraint, fst_log::Logger* log_ptr)
{
	joint_num_ = joint_num;
	cycle_time_ = cycle_time;
	delta_time_ = cycle_time / 100;
	log_ptr_ = log_ptr;
	dynamics_ptr_ = pdynamics;
	kinematics_ptr_ = pkinematics;
	constraint_ptr_ = pconstraint;
	target_info_.type = MOTION_NONE;
	memset(&uf_matrix_, 0, sizeof(uf_matrix_));
	uf_matrix_.rotation_matrix_.matrix_[0][0] = 1;
	uf_matrix_.rotation_matrix_.matrix_[1][1] = 1;
	uf_matrix_.rotation_matrix_.matrix_[2][2] = 1;
	memset(&tf_matrix_, 0, sizeof(tf_matrix_));
	tf_matrix_.rotation_matrix_.matrix_[0][0] = 1;
	tf_matrix_.rotation_matrix_.matrix_[1][1] = 1;
	tf_matrix_.rotation_matrix_.matrix_[2][2] = 1;
	traj_params_.setNumOfJoint(joint_num);
	
	if (!traj_params_.loadConstraint() || !traj_params_.loadConfig())
	{
		return false;
	}
    
	if (!line_planner_.initPlanner(joint_num, traj_params_.max_jerk_num_) || 
		!joint_planner_.initPlanner(joint_num, traj_params_.max_jerk_num_) ||
		!circle_planner_.initPlanner(joint_num, traj_params_.max_jerk_num_))
	{
		return false;
	}

	line_planner_.setLimit(traj_params_.position_vel_max_, traj_params_.position_acc_max_, traj_params_.position_jerk_max_, 
							traj_params_.quaternion_vel_max_, traj_params_.quaternion_acc_max_, traj_params_.quaternion_jerk_max_);
	circle_planner_.setLimit(traj_params_.position_vel_max_, traj_params_.position_acc_max_, traj_params_.position_jerk_max_, 
							traj_params_.quaternion_vel_max_, traj_params_.quaternion_acc_max_, traj_params_.quaternion_jerk_max_);
	joint_planner_.setLimit(traj_params_.omega_max_, traj_params_.alpha_max_, traj_params_.beta_max_);
	return true;
}

/*
void TrajectoryPlanner::setLimit(const Joint &vel_limit_joint, const Joint &acc_limit_joint, const Joint &jerk_limit_joint, 
								double vel_limit_position, double acc_limit_position, double jerk_limit_position, 
								double vel_limit_orientation, double acc_limit_orientation, double jerk_limit_orientation)
{
	line_planner_.setLimit(vel_limit_position, acc_limit_position, jerk_limit_position, vel_limit_orientation, acc_limit_orientation, jerk_limit_orientation);
	joint_planner_.setLimit(vel_limit_joint, acc_limit_joint, jerk_limit_joint);
}
*/

bool TrajectoryPlanner::setUserTrans(const TransMatrix &uf)
{
	TransMatrix inverse_uf = uf;

	if (!inverse_uf.inverse())
	{
		return false;
	}

	uf_matrix_ = uf;
	uf_matrix_inverse_ = inverse_uf;
	return true;
}

bool TrajectoryPlanner::setToolTrans(const TransMatrix &tf)
{
	TransMatrix inverse_tf = tf;

	if (!inverse_tf.inverse())
	{
		return false;
	}

	tf_matrix_ = tf;
	tf_matrix_inverse_ = inverse_tf;
	return true;
}

bool TrajectoryPlanner::setUserFrame(const PoseEuler &uf)
{
	TransMatrix trans_uf, inverse_uf;
	uf.convertToTransMatrix(trans_uf);
	inverse_uf = trans_uf;

	if (!inverse_uf.inverse())
	{
		return false;
	}

	uf_matrix_ = trans_uf;
	uf_matrix_inverse_ = inverse_uf;
	return true;
}

bool TrajectoryPlanner::setToolFrame(const PoseEuler &tf)
{
	TransMatrix trans_tf, inverse_tf;
	tf.convertToTransMatrix(trans_tf);
	inverse_tf = trans_tf;

	if (!inverse_tf.inverse())
	{
		return false;
	}

	tf_matrix_ = trans_tf;
	tf_matrix_inverse_ = inverse_tf;
	return true;
}

ErrorCode TrajectoryPlanner::planTrajectory(const Joint &start, const MotionInfo &target, double vel_ratio, double acc_ratio)
{
	start_joint_ = start;
	target_info_ = target;

	if (target.type == MOTION_JOINT)
	{
		return planJointTrajectory(vel_ratio, acc_ratio);
	}
	else if (target.type == MOTION_LINE)
	{
		return planLineTrajectory(vel_ratio, acc_ratio);
	}
	else if (target.type == MOTION_CIRCLE)
	{
		return planCircleTrajectory(vel_ratio, acc_ratio);;
	}
	else
	{
		return INVALID_PARAMETER;
	}

	return SUCCESS;
}

ErrorCode TrajectoryPlanner::planJointTrajectory(double vel_ratio, double acc_ratio)
{
	double vel = target_info_.vel * vel_ratio;
	double acc = (traj_params_.adjust_acc_by_vel_ && (acc_ratio > vel)) ?  vel : acc_ratio;
	double jerk = acc;
	
	TransMatrix matrix;
	target_info_.target.user_frame.convertToTransMatrix(matrix);
	setUserTrans(matrix);
	target_info_.target.tool_frame.convertToTransMatrix(matrix);
	setToolTrans(matrix);
	joint_planner_.planTrajectory(start_joint_, target_info_.target.joint, vel, acc, jerk);
	
	if (!traj_params_.dynamics_check_)
	{
		return SUCCESS;
	}

	double check_time_acc = joint_planner_.getSegmentEndingTime(CONSTANT_ACCELERATION);
	double check_time_dec = joint_planner_.getSegmentEndingTime(INCREASE_DECELERATION);
	JointState check_state_acc, check_state_dec;
	bool torque_over_limit = false;
	int32_t max_loop_num = 10;

	while (max_loop_num > 0)
	{
		// 采样匀加速段末尾/匀减速段起始两个点的力矩,用电机额定力矩进行校核
		torque_over_limit = false;
		sampleTrajectory(check_time_acc, check_state_acc);
		sampleTrajectory(check_time_dec, check_state_dec);
		JointVelocity omega;
		JointAcceleration alpha;
		JointTorque torque_acc, torque_dec;
		memcpy(static_cast<void*>(&omega), static_cast<void*>(&check_state_acc.omega), sizeof(omega));
		memcpy(static_cast<void*>(&alpha), static_cast<void*>(&check_state_acc.alpha), sizeof(alpha));
		dynamics_ptr_->getTorqueInverseDynamics(check_state_acc.angle, omega, alpha, torque_acc);
		memcpy(static_cast<void*>(&omega), static_cast<void*>(&check_state_dec.omega), sizeof(omega));
		memcpy(static_cast<void*>(&alpha), static_cast<void*>(&check_state_dec.alpha), sizeof(alpha));
		dynamics_ptr_->getTorqueInverseDynamics(check_state_dec.angle, omega, alpha, torque_dec);
		
		for (uint32_t i = 0; i < joint_num_; i++)
		{
			if (fabs(torque_acc[i]) > traj_params_.torque_max_[i])
			{
				torque_over_limit = true;
				break;
			}

			if (fabs(torque_dec[i]) > traj_params_.torque_max_[i])
			{
				torque_over_limit = true;
				break;
			}
		}

		if (torque_over_limit)
		{
			// 电机力矩校核失败,降低轨迹加速度和加加速度后再次迭代
			acc = acc * 0.75;
			jerk = jerk * 0.75;
			joint_planner_.planTrajectory(start_joint_, target_info_.target.joint, vel, acc, jerk);
			check_time_acc = joint_planner_.getSegmentEndingTime(CONSTANT_ACCELERATION);
			check_time_dec = joint_planner_.getSegmentEndingTime(INCREASE_DECELERATION);
		}
		else
		{
			// 电机力矩校核通过
			break;
		}

		max_loop_num--;
	}

	if (max_loop_num <= 0)
	{
		// 电机力矩不满足轨迹需求,规划失败
		return MC_TRAJECTORY_PLANNING_FAIL;
	}
	
	return SUCCESS;
}

ErrorCode TrajectoryPlanner::checkLineTrajectory(void)
{
	double line_duration = line_planner_.getDuration();
	double duration_step = line_duration / 100;
	ErrorCode err = SUCCESS;
	Joint reference = start_joint_;
	Joint check_joint;

	for (double t = duration_step; t < line_duration + MINIMUM_E6; t += duration_step)
	{
		PoseQuaternion pose;
		TransMatrix trans_pose;
		line_planner_.sampleTrajectory(t, pose);
		pose.convertToTransMatrix(trans_pose);
		trans_pose.leftMultiply(uf_matrix_).rightMultiply(tf_matrix_inverse_);

		if (!kinematics_ptr_->doIK(trans_pose, reference, check_joint))
		{
			PoseEuler pe;
			trans_pose.convertToPoseQuaternion(pose);
			pose.convertToPoseEuler(pe);
			FST_ERROR("Fail to compute IK on line trajectory, t = %.6f.", t);
			FST_ERROR("Pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.quaternion_.x_, pose.quaternion_.y_, pose.quaternion_.z_, pose.quaternion_.w_);
			FST_ERROR("Reference: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", reference.j1_, reference.j2_, reference.j3_, reference.j4_, reference.j5_, reference.j6_);
			err = MC_COMPUTE_IK_FAIL;
			break;
		}

		if (!constraint_ptr_->isJointInConstraint(check_joint))
		{
			PoseEuler pe;
			trans_pose.convertToPoseQuaternion(pose);
			pose.convertToPoseEuler(pe);
			FST_ERROR("Pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.quaternion_.x_, pose.quaternion_.y_, pose.quaternion_.z_, pose.quaternion_.w_);
			FST_ERROR("Reference: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", reference.j1_, reference.j2_, reference.j3_, reference.j4_, reference.j5_, reference.j6_);
			FST_ERROR("Result: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", check_joint.j1_, check_joint.j2_, check_joint.j3_, check_joint.j4_, check_joint.j5_, check_joint.j6_);
			FST_ERROR("Trajectory out of constraint, t = %.6f.", t);
			err = JOINT_OUT_OF_CONSTRAINT;
			break;
		}

		reference = check_joint;
	}

	if (err != SUCCESS)
	{
		FST_ERROR("Trajectory check failed.");
		return err;
	}

	if (!isEqual(check_joint, target_info_.target.joint))
	{
		FST_ERROR("Last joint of the trajectory not equal with target joint");
		FST_ERROR("Last point:   %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", check_joint[0], check_joint[1], check_joint[2], check_joint[3], check_joint[4], check_joint[5]);
		FST_ERROR("Target point: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", target_info_.target.joint[0], target_info_.target.joint[1], target_info_.target.joint[2], target_info_.target.joint[3], target_info_.target.joint[4], target_info_.target.joint[5]);
		return MC_TRAJECTORY_OUT_OF_TARGET;
	}

	return SUCCESS;
}

ErrorCode TrajectoryPlanner::planLineTrajectory(double vel_ratio, double acc_ratio)
{
	TransMatrix matrix;
	PoseQuaternion start_pose, target_pose;
	target_info_.target.user_frame.convertToTransMatrix(matrix);
	setUserTrans(matrix);
	target_info_.target.tool_frame.convertToTransMatrix(matrix);
	setToolTrans(matrix);
	kinematics_ptr_->doFK(start_joint_, start_pose);
	start_pose.convertToTransMatrix(matrix);
	matrix.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_).convertToPoseQuaternion(start_pose);
	target_info_.target.pose.pose.convertToPoseQuaternion(target_pose);
	double vel = target_info_.vel * vel_ratio / traj_params_.position_vel_max_;
	double acc = (traj_params_.adjust_acc_by_vel_ && (acc_ratio > vel)) ?  vel : acc_ratio;
	double jerk = acc;
	bool alternative_trajectory = false;
	line_planner_.planTrajectory(start_pose, target_pose, vel, acc, jerk);
	ErrorCode err = checkLineTrajectory();

	if (err != SUCCESS)
	{
		FST_INFO("Plan alternative trajectory");
		alternative_trajectory = true;
		line_planner_.planAlternativeTrajectory(start_pose, target_pose, vel, acc, jerk);
		err = checkLineTrajectory();

		if (err != SUCCESS)
		{
			FST_ERROR("Trajectory plan failed.");
			return err;
		}
	}

	double check_time_acc = line_planner_.getSegmentEndingTime(CONSTANT_ACCELERATION);
	double check_time_dec = line_planner_.getSegmentEndingTime(INCREASE_DECELERATION);
	JointState check_state_acc, check_state_dec, check_state_vel;
	bool torque_over_limit = false;
	bool speed_over_limit = false;
	int32_t max_loop_num = 10;

	while (max_loop_num > 0)
	{
		// 均匀采样整个轨迹段,用电机额定转速进行校核
		FST_INFO("Motor speed check, vel = %.6f, acc = %.6f, jerk = %.6f", vel, acc, jerk);
		speed_over_limit = false;

		double check_duration = line_planner_.getDuration();
		uint32_t  check_num = check_duration < 0.1 ? 5 : (check_duration < 0.5 ? 25 : 50);
		double check_step = check_duration / check_num;

		for (double t = check_step; t < check_duration + MINIMUM_E6; t += check_step)
		{
			sampleTrajectory(t, check_state_vel);

			for (uint32_t i = 0; i < joint_num_; i++)
			{
				if (fabs(check_state_vel.omega[i]) > traj_params_.omega_max_of_cart_motion_[i])
				{
					speed_over_limit = true;
					break;
				}
			}
		}

		if (speed_over_limit)
		{
			// 电机额定转速校核失败,降低轨迹速度后再次迭代
			vel = vel * 0.75;
			if (!alternative_trajectory)
				line_planner_.planTrajectory(start_pose, target_pose, vel, acc, jerk);
			else
				line_planner_.planAlternativeTrajectory(start_pose, target_pose, vel, acc, jerk);
		}
		else
		{
			// 电机额定转速校核通过
			break;
		}

		max_loop_num--;
	}

	if (max_loop_num <= 0)
	{
		// 电机转速不满足轨迹需求,规划失败
		FST_ERROR("vel = %.6f, acc = %.6f, jerk = %.6f", vel, acc, jerk);
		FST_ERROR("Motor speed check failed, trajectory plan abort.");
		return MC_TRAJECTORY_PLANNING_FAIL;
	}

	if (!traj_params_.dynamics_check_)
	{
		return SUCCESS;
	}

	max_loop_num = 10;

	while (max_loop_num > 0)
	{
		// 采样匀加速段末尾/匀减速段起始两个点的力矩,用电机额定力矩进行校核
		FST_INFO("Inverse dynamics check, vel = %.6f, acc = %.6f, jerk = %.6f", vel, acc, jerk);
		torque_over_limit = false;
		sampleTrajectory(check_time_acc, check_state_acc);
		sampleTrajectory(check_time_dec, check_state_dec);
		JointVelocity omega;
		JointAcceleration alpha;
		JointTorque torque_acc, torque_dec;
		memcpy(static_cast<void*>(&omega), static_cast<void*>(&check_state_acc.omega), sizeof(omega));
		memcpy(static_cast<void*>(&alpha), static_cast<void*>(&check_state_acc.alpha), sizeof(alpha));
		dynamics_ptr_->getTorqueInverseDynamics(check_state_acc.angle, omega, alpha, torque_acc);
		memcpy(static_cast<void*>(&omega), static_cast<void*>(&check_state_dec.omega), sizeof(omega));
		memcpy(static_cast<void*>(&alpha), static_cast<void*>(&check_state_dec.alpha), sizeof(alpha));
		dynamics_ptr_->getTorqueInverseDynamics(check_state_dec.angle, omega, alpha, torque_dec);

		for (uint32_t i = 0; i < joint_num_; i++)
		{
			if (fabs(torque_acc[i]) > traj_params_.torque_max_[i])
			{
				torque_over_limit = true;
				break;
			}

			if (fabs(torque_dec[i]) > traj_params_.torque_max_[i])
			{
				torque_over_limit = true;
				break;
			}
		}

		if (torque_over_limit)
		{
			// 电机力矩校核失败,降低轨迹加速度和加加速度后再次迭代
			acc = acc * 0.75;
			jerk = jerk * 0.75;
			
			if (!alternative_trajectory)
				line_planner_.planTrajectory(start_pose, target_pose, vel, acc, jerk);
			else
				line_planner_.planAlternativeTrajectory(start_pose, target_pose, vel, acc, jerk);

			check_time_acc = line_planner_.getSegmentEndingTime(CONSTANT_ACCELERATION);
			check_time_dec = line_planner_.getSegmentEndingTime(INCREASE_DECELERATION);
		}
		else
		{
			// 电机力矩校核通过
			break;
		}

		max_loop_num--;
	}

	if (max_loop_num <= 0)
	{
		// 电机力矩不满足轨迹需求,规划失败
		FST_ERROR("vel = %.6f, acc = %.6f, jerk = %.6f", vel, acc, jerk);
		FST_ERROR("Inverse dynamics check failed, trajectory plan abort.");
		return MC_TRAJECTORY_PLANNING_FAIL;
	}

	return SUCCESS;
}

ErrorCode TrajectoryPlanner::planCircleTrajectory(double vel_ratio, double acc_ratio)
{
	TransMatrix matrix;
	PoseQuaternion start_pose, via_pose, target_pose;
	target_info_.target.user_frame.convertToTransMatrix(matrix);
	setUserTrans(matrix);
	target_info_.target.tool_frame.convertToTransMatrix(matrix);
	setToolTrans(matrix);
	kinematics_ptr_->doFK(start_joint_, start_pose);
	start_pose.convertToTransMatrix(matrix);
	matrix.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_).convertToPoseQuaternion(start_pose);
	target_info_.via.pose.pose.convertToPoseQuaternion(via_pose);
	target_info_.target.pose.pose.convertToPoseQuaternion(target_pose);
	double vel = target_info_.vel * vel_ratio / traj_params_.position_vel_max_;
	double acc = (traj_params_.adjust_acc_by_vel_ && (acc_ratio > vel)) ?  vel : acc_ratio;
	double jerk = acc;
	circle_planner_.planTrajectory(start_pose, via_pose, target_pose, vel, acc, jerk);
	double circle_duration = circle_planner_.getDuration();
	double duration_step = circle_duration / 100;
	ErrorCode err = SUCCESS;
	Joint reference = start_joint_;
	Joint check_joint;

	for (double t = duration_step; t < circle_duration + MINIMUM_E6; t += duration_step)
	{
		PoseQuaternion pose;
		TransMatrix trans_pose;
		circle_planner_.sampleTrajectory(t, pose);
		pose.convertToTransMatrix(trans_pose);
		trans_pose.leftMultiply(uf_matrix_).rightMultiply(tf_matrix_inverse_);

		if (!kinematics_ptr_->doIK(trans_pose, reference, check_joint))
		{
			FST_ERROR("Fail to compute IK on circle trajectory.");
			err = MC_COMPUTE_IK_FAIL;
			break;
		}

		if (!constraint_ptr_->isJointInConstraint(check_joint))
		{
			FST_ERROR("Trajectory out of constraint.");
			err = JOINT_OUT_OF_CONSTRAINT;
			break;
		}

		reference = check_joint;
	}

	if (err != SUCCESS)
	{
		FST_ERROR("Trajectory check failed.");
		return err;
	}

	if (!isEqual(check_joint, target_info_.target.joint))
	{
		FST_ERROR("Last point of the trajectory not equal with target joint");
		FST_ERROR("Last point:   %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", check_joint[0], check_joint[1], check_joint[2], check_joint[3], check_joint[4], check_joint[5]);
		FST_ERROR("Target point: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", target_info_.target.joint[0], target_info_.target.joint[1], target_info_.target.joint[2], target_info_.target.joint[3], target_info_.target.joint[4], target_info_.target.joint[5]);
		return MC_TRAJECTORY_OUT_OF_TARGET;
	}

	double check_time_acc = circle_planner_.getSegmentEndingTime(CONSTANT_ACCELERATION);
	double check_time_dec = circle_planner_.getSegmentEndingTime(INCREASE_DECELERATION);
	JointState check_state_acc, check_state_dec, check_state_vel;
	bool torque_over_limit = false;
	bool speed_over_limit = false;
	int32_t max_loop_num = 10;

	while (max_loop_num > 0)
	{
		// 采样匀速段起始/中间/末尾三个点的关节转速,用电机额定转速进行校核
		FST_INFO("Motor speed check, vel = %.6f, acc = %.6f, jerk = %.6f", vel, acc, jerk);
		speed_over_limit = false;
		double check_duration = circle_planner_.getDuration();
		uint32_t  check_num = check_duration < 0.1 ? 5 : (check_duration < 0.5 ? 25 : 50);
		double check_step = check_duration / check_num;

		for (double t = check_step; t < check_duration + MINIMUM_E6; t += check_step)
		{
			sampleTrajectory(t, check_state_vel);

			for (uint32_t i = 0; i < joint_num_; i++)
			{
				if (fabs(check_state_vel.omega[i]) > traj_params_.omega_max_of_cart_motion_[i])
				{
					speed_over_limit = true;
					break;
				}
			}
		}

		if (speed_over_limit)
		{
			// 电机额定转速校核失败,降低轨迹速度后再次迭代
			vel = vel * 0.75;
			circle_planner_.planTrajectory(start_pose, via_pose, target_pose, vel, acc, jerk);
		}
		else
		{
			// 电机额定转速校核通过
			break;
		}

		max_loop_num--;
	}

	if (max_loop_num <= 0)
	{
		// 电机转速不满足轨迹需求,规划失败
		FST_ERROR("vel = %.6f, acc = %.6f, jerk = %.6f", vel, acc, jerk);
		FST_ERROR("Motor speed check failed, trajectory plan abort.");
		return MC_TRAJECTORY_PLANNING_FAIL;
	}

	if (!traj_params_.dynamics_check_)
	{
		return SUCCESS;
	}

	max_loop_num = 10;

	while (max_loop_num > 0)
	{
		// 采样匀加速段末尾/匀减速段起始两个点的力矩,用电机额定力矩进行校核
		FST_INFO("Inverse dynamics check, vel = %.6f, acc = %.6f, jerk = %.6f", vel, acc, jerk);
		torque_over_limit = false;
		sampleTrajectory(check_time_acc, check_state_acc);
		sampleTrajectory(check_time_dec, check_state_dec);
		JointVelocity omega;
		JointAcceleration alpha;
		JointTorque torque_acc, torque_dec;
		memcpy(static_cast<void*>(&omega), static_cast<void*>(&check_state_acc.omega), sizeof(omega));
		memcpy(static_cast<void*>(&alpha), static_cast<void*>(&check_state_acc.alpha), sizeof(alpha));
		dynamics_ptr_->getTorqueInverseDynamics(check_state_acc.angle, omega, alpha, torque_acc);
		memcpy(static_cast<void*>(&omega), static_cast<void*>(&check_state_dec.omega), sizeof(omega));
		memcpy(static_cast<void*>(&alpha), static_cast<void*>(&check_state_dec.alpha), sizeof(alpha));
		dynamics_ptr_->getTorqueInverseDynamics(check_state_dec.angle, omega, alpha, torque_dec);

		for (uint32_t i = 0; i < joint_num_; i++)
		{
			if (fabs(torque_acc[i]) > traj_params_.torque_max_[i])
			{
				torque_over_limit = true;
				break;
			}

			if (fabs(torque_dec[i]) > traj_params_.torque_max_[i])
			{
				torque_over_limit = true;
				break;
			}
		}

		if (torque_over_limit)
		{
			// 电机力矩校核失败,降低轨迹加速度和加加速度后再次迭代
			acc = acc * 0.75;
			jerk = jerk * 0.75;
			circle_planner_.planTrajectory(start_pose, via_pose, target_pose, vel, acc, jerk);
			check_time_acc = circle_planner_.getSegmentEndingTime(CONSTANT_ACCELERATION);
			check_time_dec = circle_planner_.getSegmentEndingTime(INCREASE_DECELERATION);
		}
		else
		{
			// 电机力矩校核通过
			break;
		}

		max_loop_num--;
	}

	if (max_loop_num <= 0)
	{
		// 电机力矩不满足轨迹需求,规划失败
		FST_ERROR("vel = %.6f, acc = %.6f, jerk = %.6f", vel, acc, jerk);
		FST_ERROR("Inverse dynamics check failed, trajectory plan abort.");
		return MC_TRAJECTORY_PLANNING_FAIL;
	}

	return SUCCESS;
}

// 仅在内部速度校核时使用,内部不计算逆动力学,不保证回转圈数正确
ErrorCode TrajectoryPlanner::sampleTrajectory(double sample_time, JointState &point)
{
	if (target_info_.type == MOTION_JOINT)
	{
		joint_planner_.sampleTrajectory(sample_time, point);
	}
	else if (target_info_.type == MOTION_LINE || target_info_.type == MOTION_CIRCLE)
	{
		Posture posture = kinematics_ptr_->getPostureByJoint(start_joint_);
		Joint joint_delta1, joint_delta2;
		PoseQuaternion pose, pose_delta1, pose_delta2;
		TransMatrix trans_pose, trans_delta1, trans_delta2;
		
		if (target_info_.type == MOTION_LINE)
		{
			line_planner_.sampleTrajectory(sample_time, pose);
			line_planner_.sampleTrajectory(sample_time + delta_time_, pose_delta1);
			line_planner_.sampleTrajectory(sample_time + delta_time_ * 2, pose_delta2);
		}
		else
		{
			circle_planner_.sampleTrajectory(sample_time, pose);
			circle_planner_.sampleTrajectory(sample_time + delta_time_, pose_delta1);
			circle_planner_.sampleTrajectory(sample_time + delta_time_ * 2, pose_delta2);
		}
		
		//FST_INFO("sample_time_delta0=%.6f: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", sample_time, pose.point_.x_,  pose.point_.y_,  pose.point_.z_, pose.quaternion_.w_, pose.quaternion_.x_, pose.quaternion_.y_, pose.quaternion_.z_);
		//FST_INFO("sample_time_delta1=%.6f: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", sample_time + delta_time_, pose_delta1.point_.x_,  pose_delta1.point_.y_,  pose_delta1.point_.z_, pose_delta1.quaternion_.w_, pose_delta1.quaternion_.x_, pose_delta1.quaternion_.y_, pose_delta1.quaternion_.z_);
		//FST_INFO("sample_time_delta2=%.6f: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", sample_time + delta_time_ * 2, pose_delta2.point_.x_,  pose_delta2.point_.y_,  pose_delta2.point_.z_, pose_delta2.quaternion_.w_, pose_delta2.quaternion_.x_, pose_delta2.quaternion_.y_, pose_delta2.quaternion_.z_);
		pose.convertToTransMatrix(trans_pose);
		pose_delta1.convertToTransMatrix(trans_delta1);
		pose_delta2.convertToTransMatrix(trans_delta2);
		trans_pose.leftMultiply(uf_matrix_).rightMultiply(tf_matrix_inverse_);
		trans_delta1.leftMultiply(uf_matrix_).rightMultiply(tf_matrix_inverse_);
		trans_delta2.leftMultiply(uf_matrix_).rightMultiply(tf_matrix_inverse_);

		if (!kinematics_ptr_->doIK(trans_pose, posture, point.angle) ||
			!kinematics_ptr_->doIK(trans_delta1, posture, joint_delta1) ||
			!kinematics_ptr_->doIK(trans_delta2, posture, joint_delta2))
		{
			return MC_COMPUTE_IK_FAIL;
		}

		for (uint32_t j = 0; j < joint_num_; j++)
		{
			point.omega[j] = (joint_delta1[j] - point.angle[j]) / delta_time_;
			point.alpha[j] = ((joint_delta2[j] - joint_delta1[j]) / delta_time_ - point.omega[j]) / delta_time_;
		}

		//FST_INFO("angle: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", point.angle[0], point.angle[1], point.angle[2], point.angle[3], point.angle[4], point.angle[5]);
		//FST_INFO("omega: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", point.omega[0], point.omega[1], point.omega[2], point.omega[3], point.omega[4], point.omega[5]);
		//FST_INFO("alpha: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", point.alpha[0], point.alpha[1], point.alpha[2], point.alpha[3], point.alpha[4], point.alpha[5]);
	}
	else
	{
		return INVALID_PARAMETER;
	}

	return SUCCESS;
}

ErrorCode TrajectoryPlanner::sampleTrajectory(double start_time, Joint reference, uint32_t &point_num, JointState *points)
{
	double total_time;
	double sample_time = start_time;
	uint32_t picked_num = 0;
	ErrorCode err = SUCCESS;

	if (target_info_.type == MOTION_JOINT)
	{
		total_time = joint_planner_.getDuration();
		
		for (uint32_t i = 0; i < point_num; i++)
		{
			joint_planner_.sampleTrajectory(sample_time, points[i]);
			JointVelocity omega;
			JointAcceleration alpha;
			JointTorque torque;
			memcpy(static_cast<void*>(&omega), static_cast<void*>(&points[i].omega), sizeof(omega));
			memcpy(static_cast<void*>(&alpha), static_cast<void*>(&points[i].alpha), sizeof(alpha));
			dynamics_ptr_->getTorqueInverseDynamics(points[i].angle, omega, alpha, torque);
			memcpy(static_cast<void*>(&points[i].torque), static_cast<void*>(&torque), sizeof(points[i].torque));
			picked_num ++;

			if (sample_time > total_time)
			{
				break;
			}

			sample_time += cycle_time_;
		}

		point_num = picked_num;
		return err;
	}
	else if (target_info_.type == MOTION_LINE || target_info_.type == MOTION_CIRCLE)
	{
		Joint joint_delta1, joint_delta2;
		PoseQuaternion pose, pose_delta1, pose_delta2;
		TransMatrix trans_pose, trans_delta1, trans_delta2;
		total_time = line_planner_.getDuration();

		for (uint32_t i = 0; i < point_num; i++)
		{
			if (target_info_.type == MOTION_LINE)
			{
				line_planner_.sampleTrajectory(sample_time, pose);
				line_planner_.sampleTrajectory(sample_time + delta_time_, pose_delta1);
				line_planner_.sampleTrajectory(sample_time + delta_time_ * 2, pose_delta2);
			}
			else
			{
				circle_planner_.sampleTrajectory(sample_time, pose);
				circle_planner_.sampleTrajectory(sample_time + delta_time_, pose_delta1);
				circle_planner_.sampleTrajectory(sample_time + delta_time_ * 2, pose_delta2);
			}
			
			pose.convertToTransMatrix(trans_pose);
			pose_delta1.convertToTransMatrix(trans_delta1);
			pose_delta2.convertToTransMatrix(trans_delta2);
			trans_pose.leftMultiply(uf_matrix_).rightMultiply(tf_matrix_inverse_);
			trans_delta1.leftMultiply(uf_matrix_).rightMultiply(tf_matrix_inverse_);
			trans_delta2.leftMultiply(uf_matrix_).rightMultiply(tf_matrix_inverse_);

			if (!kinematics_ptr_->doIK(trans_pose, reference, points[i].angle) ||
				!kinematics_ptr_->doIK(trans_delta1, reference, joint_delta1) ||
				!kinematics_ptr_->doIK(trans_delta2, reference, joint_delta2))
			{
				err = MC_COMPUTE_IK_FAIL;
				break;
			}

			for (uint32_t j = 0; j < joint_num_; j++)
			{
				points[i].omega[j] = (joint_delta1[j] - points[i].angle[j]) / delta_time_;
				points[i].alpha[j] = ((joint_delta2[j] - joint_delta1[j]) / delta_time_ - points[i].omega[j]) / delta_time_;
			}

			JointVelocity omega;
			JointAcceleration alpha;
			JointTorque torque;
			memcpy(static_cast<void*>(&omega), static_cast<void*>(&points[i].omega), sizeof(omega));
			memcpy(static_cast<void*>(&alpha), static_cast<void*>(&points[i].alpha), sizeof(alpha));
			dynamics_ptr_->getTorqueInverseDynamics(points[i].angle, omega, alpha, torque);
			memcpy(static_cast<void*>(&points[i].torque), static_cast<void*>(&torque), sizeof(points[i].torque));
			picked_num ++;

			if (sample_time > total_time)
			{
				break;
			}

			sample_time += cycle_time_;
			reference = points[i].angle;
		}

		point_num = picked_num;
		return err;
	}
	else
	{
		point_num = picked_num;
		return err;
	}
}

ErrorCode TrajectoryPlanner::sampleCircleCartesianTrajectory(double sample_time, basic_alg::PoseQuaternion &pose, double &postion_vel)
{
	double total_time = circle_planner_.getDuration();
	if (total_time < sample_time) sample_time = total_time;
	circle_planner_.sampleTrajectory(sample_time, postion_vel, pose);

	// TransMatrix trans_pose;
	// pose.convertToTransMatrix(trans_pose);
	// trans_pose.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_).convertToPoseQuaternion(pose);

	return SUCCESS;
}

ErrorCode TrajectoryPlanner::sampleLineCartesianTrajectory(double sample_time, basic_alg::PoseQuaternion &pose)
{
	double total_time = line_planner_.getDuration();
	if (total_time < sample_time) sample_time = total_time;
	line_planner_.sampleTrajectory(sample_time, pose);

	// TransMatrix trans_pose;
	// pose.convertToTransMatrix(trans_pose);
	// trans_pose.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_).convertToPoseQuaternion(pose);
	return SUCCESS;
}



ErrorCode TrajectoryPlanner::sampleCartesianTrajectory(double start_time, basic_alg::Joint reference, uint32_t &point_num, 
		fst_mc::JointState &point, basic_alg::PoseQuaternion &pose, double &postion_vel)
{
	double total_time;
	double sample_time = start_time;
	uint32_t picked_num = 0;
	ErrorCode err = SUCCESS;

	Joint joint_delta1, joint_delta2;
	PoseQuaternion pose_delta1, pose_delta2;
	TransMatrix trans_pose, trans_delta1, trans_delta2;

	if (target_info_.type == MOTION_CIRCLE)
	{
		total_time = circle_planner_.getDuration();
	}
	else if (target_info_.type == MOTION_LINE)
	{
		total_time = line_planner_.getDuration();
	}
	else 
	{
		point_num = picked_num;
		return err;
	}

	for (uint32_t i = 0; i < point_num; i++)
	{
		if (target_info_.type == MOTION_CIRCLE)
		{
			circle_planner_.sampleTrajectory(sample_time, postion_vel, pose);
			circle_planner_.sampleTrajectory(sample_time + delta_time_, pose_delta1);
			circle_planner_.sampleTrajectory(sample_time + delta_time_ * 2, pose_delta2);
		}
		else if (target_info_.type == MOTION_LINE)
		{
			line_planner_.sampleTrajectory(sample_time, pose);
			line_planner_.sampleTrajectory(sample_time + delta_time_, pose_delta1);
			line_planner_.sampleTrajectory(sample_time + delta_time_ * 2, pose_delta2);
		}
		else 
		{
			point_num = picked_num;
			return err;
		}
		
		pose.convertToTransMatrix(trans_pose);
		pose_delta1.convertToTransMatrix(trans_delta1);
		pose_delta2.convertToTransMatrix(trans_delta2);
		trans_pose.leftMultiply(uf_matrix_).rightMultiply(tf_matrix_inverse_);
		trans_delta1.leftMultiply(uf_matrix_).rightMultiply(tf_matrix_inverse_);
		trans_delta2.leftMultiply(uf_matrix_).rightMultiply(tf_matrix_inverse_);
		if (!kinematics_ptr_->doIK(trans_pose, reference, point.angle) ||
			!kinematics_ptr_->doIK(trans_delta1, reference, joint_delta1) ||
			!kinematics_ptr_->doIK(trans_delta2, reference, joint_delta2))
		{
			err = MC_COMPUTE_IK_FAIL;
			break;
		}
		for (uint32_t j = 0; j < joint_num_; j++)
		{
			point.omega[j] = (joint_delta1[j] - point.angle[j]) / delta_time_;
			point.alpha[j] = ((joint_delta2[j] - joint_delta1[j]) / delta_time_ - point.omega[j]) / delta_time_;
		}

		JointVelocity omega;
		JointAcceleration alpha;
		JointTorque torque;
		memcpy(static_cast<void*>(&omega), static_cast<void*>(&point.omega), sizeof(omega));
		memcpy(static_cast<void*>(&alpha), static_cast<void*>(&point.alpha), sizeof(alpha));
		dynamics_ptr_->getTorqueInverseDynamics(point.angle, omega, alpha, torque);
		memcpy(static_cast<void*>(&point.torque), static_cast<void*>(&torque), sizeof(point.torque));
		//dynamics_ptr_->getTorqueInverseDynamics(point.angle, (*(JointVelocity*)(&point.omega)), (*(JointAcceleration*)(&point.alpha)), (*(JointTorque*)(&point.torque)));
		picked_num ++;
		if (sample_time > total_time)
		{
			break;
		}
		sample_time += cycle_time_;
		reference = point.angle;
	}
	point_num = picked_num;

	return err;
}

void TrajectoryPlanner::sampleLineNormalTrajectory(double sample_time, double &sample_u, double &sample_v, double &sample_a)
{
	line_planner_.sampleNormalTrajectory(sample_time, sample_u, sample_v, sample_a);
}

double TrajectoryPlanner::getSmoothInTime(double smooth_distance)
{
	double trip = 0;
	double half_time = getDuration() / 2;
	double sample_step = ceil(half_time / cycle_time_ / 100) * cycle_time_;
	double sample_time = sample_step;

	JointState sample_state;
	PoseEuler last_pose;
	TransMatrix matrix;
	kinematics_ptr_->doFK(start_joint_, last_pose);
	last_pose.convertToTransMatrix(matrix);
	matrix.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_).convertToPoseEuler(last_pose);
	double total_distance = target_info_.type == MOTION_CIRCLE ? 
								getDistance(target_info_.target.pose.pose.point_, target_info_.via.pose.pose.point_) + getDistance(target_info_.via.pose.pose.point_, last_pose.point_) :
							  	getDistance(target_info_.target.pose.pose.point_, last_pose.point_);

	FST_INFO("getSmoothInTime: duration = %.6f, sample-step = %.6f, smooth-diatance = %.6f", getDuration(), sample_step, smooth_distance);

	if (total_distance < 1)
	{
		return 0;
	}

	if (target_info_.type == MOTION_JOINT || target_info_.type == MOTION_CIRCLE)
	{
		while (sample_time < half_time)
		{
			sampleTrajectory(sample_time, sample_state);
			kinematics_ptr_->doFK(sample_state.angle, matrix);
			matrix.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_);
			trip += getDistance(matrix.trans_vector_, last_pose.point_);
			if (trip > smooth_distance) break;
			sample_time += sample_step;
			last_pose.point_ = matrix.trans_vector_;
		}

		return sample_time - sample_step;
	}
	else if (target_info_.type == MOTION_LINE)
	{
		while (sample_time < half_time)
		{
			sampleTrajectory(sample_time, sample_state);
			kinematics_ptr_->doFK(sample_state.angle, matrix);
			matrix.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_);
			trip = getDistance(matrix.trans_vector_, last_pose.point_);
			//FST_INFO("sample_time = %.6f, trip-diatance = %.6f", sample_time, trip);
			if (trip > smooth_distance) break;
			sample_time += sample_step;
		}

		return sample_time - sample_step;
	}
	else
	{
		return 0;
	}
}

void TrajectoryPlanner::getSmoothOutTimeAndDistance(double given_smooth_distance, double time_offset, double &smooth_time, double &smooth_distance)
{
	double trip = 0;
	double half_time = getDuration() / 2;
	double sample_step = ceil(half_time / cycle_time_ / 100) * cycle_time_;
	double sample_time = floor(getDuration() / sample_step) * sample_step + time_offset;
	sample_time = sample_time > getDuration() ? sample_time - sample_step : sample_time;
	TransMatrix matrix;
	JointState sample_state;
	PoseEuler last_pose = target_info_.target.pose.pose;
	PoseEuler start_pose;
	kinematics_ptr_->doFK(start_joint_, start_pose);
	start_pose.convertToTransMatrix(matrix);
	matrix.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_).convertToPoseEuler(start_pose);
	double total_distance = target_info_.type == MOTION_CIRCLE ? 
								getDistance(last_pose.point_, target_info_.via.pose.pose.point_) + getDistance(target_info_.via.pose.pose.point_, start_pose.point_) :
								getDistance(last_pose.point_, start_pose.point_);
	FST_INFO("getSmoothOutTimeAndDistance: duration = %.6f, total-distance = %.6f, sample-step = %.6f, sample-time = %.6f", getDuration(), total_distance, sample_step, sample_time);

	if (total_distance < 1)
	{
		smooth_time = getDuration();
		smooth_distance = 0;
		return;
	}

	if (target_info_.type == MOTION_JOINT || target_info_.type == MOTION_CIRCLE)
	{
		while (sample_time > half_time)
		{
			sampleTrajectory(sample_time, sample_state);
			kinematics_ptr_->doFK(sample_state.angle, matrix);
			matrix.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_);
			trip += getDistance(matrix.trans_vector_, last_pose.point_);
			
			if (trip > given_smooth_distance)
			{
				trip -= getDistance(matrix.trans_vector_, last_pose.point_);
				break;
			}

			sample_time -= sample_step;
			last_pose.point_ = matrix.trans_vector_;
		}

		sample_time += sample_step;

		if (sample_time > getDuration())
		{
			smooth_time = getDuration();
			smooth_distance = 0;
		}

		smooth_time = sample_time;
		smooth_distance = trip;
	}
	else if (target_info_.type == MOTION_LINE)
	{
		while (sample_time > half_time)
		{
			sampleTrajectory(sample_time, sample_state);
			kinematics_ptr_->doFK(sample_state.angle, matrix);
			matrix.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_);
			trip = getDistance(matrix.trans_vector_, last_pose.point_);

			if (trip > given_smooth_distance)
			{
				break;
			}

			sample_time -= sample_step;
		}

		sample_time += sample_step;

		if (sample_time > getDuration())
		{
			smooth_time = getDuration();
			smooth_distance = 0;
		}

		smooth_time = sample_time;
		sampleTrajectory(sample_time, sample_state);
		kinematics_ptr_->doFK(sample_state.angle, matrix);
		matrix.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_);
		smooth_distance = getDistance(matrix.trans_vector_, last_pose.point_);
	}
	else
	{
		smooth_time = getDuration();
		smooth_distance = 0;
	}
}

double TrajectoryPlanner::getDuration(void)
{
	double duration = 0;

	if (target_info_.type == MOTION_JOINT)
	{
		duration = joint_planner_.getDuration();
	}
	else if (target_info_.type == MOTION_LINE)
	{
		duration = line_planner_.getDuration();
	}
	else if (target_info_.type == MOTION_CIRCLE)
	{
		duration = circle_planner_.getDuration();
	}

	return duration;
}

const fst_mc::MotionInfo& TrajectoryPlanner::getMotionInfo(void) const
{
	return target_info_;
}

bool TrajectoryPlanner::isEqual(const Joint &joint_a, const Joint &joint_b, double threshold)
{
	for (uint32_t j = 0; j < joint_num_; j++)
	{
		if (fabs(joint_a[j] - joint_b[j]) > threshold)
		{
			return false;
		}
	}

	return true;
}

}
