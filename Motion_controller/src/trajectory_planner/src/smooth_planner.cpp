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
#include <error_code.h>
#include <basic_alg.h>
#include <trans_matrix.h>
#include <smooth_planner.h>

using namespace basic_alg;
using namespace fst_mc;
using namespace std;

SmoothPlanner::SmoothPlanner(void)
{
	motion_type_pre_ = MOTION_NONE;
	motion_type_this_ = MOTION_NONE;
	joint_num_ = 0;
	cycle_time_ = 0;
	delta_time_ = 0;
	memset(&uf_matrix_, 0, sizeof(uf_matrix_));
	memset(&tf_matrix_, 0, sizeof(tf_matrix_));
	memset(&uf_matrix_inverse_, 0, sizeof(uf_matrix_inverse_));
	memset(&tf_matrix_inverse_, 0, sizeof(tf_matrix_inverse_));
	uf_matrix_.rotation_matrix_.matrix_[0][0] = 1;
	uf_matrix_.rotation_matrix_.matrix_[1][1] = 1;
	uf_matrix_.rotation_matrix_.matrix_[2][2] = 1;
	tf_matrix_.rotation_matrix_.matrix_[0][0] = 1;
	tf_matrix_.rotation_matrix_.matrix_[1][1] = 1;
	tf_matrix_.rotation_matrix_.matrix_[2][2] = 1;
	uf_matrix_inverse_.rotation_matrix_.matrix_[0][0] = 1;
	uf_matrix_inverse_.rotation_matrix_.matrix_[1][1] = 1;
	uf_matrix_inverse_.rotation_matrix_.matrix_[2][2] = 1;
	tf_matrix_inverse_.rotation_matrix_.matrix_[0][0] = 1;
	tf_matrix_inverse_.rotation_matrix_.matrix_[1][1] = 1;
	tf_matrix_inverse_.rotation_matrix_.matrix_[2][2] = 1;
	kinematics_ptr_ = NULL;

	uout_ = 0.0;
	uin_ = 0.0;
	vout_u_ = 0.0;
	vin_u_ = 0.0;
	point_smooth_total_time_ = 0.0;
	unit_unified_param_ = 1000; // m->mm / mm -> m
	acc_limit_position_ = 0.0;

	motion_type_pre_ = MOTION_NONE;
	motion_type_this_ = MOTION_NONE;

	t_total_ = 0.0;
	t_former_ = 0.0;
	t_last_ = 0.0;

	orientation_out_time_ = 0.0;
	orientation_in_time_ = 0.0;
	point_out_time_ = 0.0;
	point_in_time_ = 0.0;
}

SmoothPlanner::~SmoothPlanner(void)
{}

bool SmoothPlanner::initPlanner(uint32_t joint_num, double cycle_time, Kinematics* pkinematics, fst_mc::Constraint* pconstraint, fst_log::Logger* log_ptr)
{
	motion_type_pre_ = MOTION_NONE;
	motion_type_this_ = MOTION_NONE;
	joint_num_ = joint_num;
	cycle_time_ = cycle_time;
	delta_time_ = cycle_time / 100;
	kinematics_ptr_ = pkinematics;
	memset(&uf_matrix_, 0, sizeof(uf_matrix_));
	memset(&tf_matrix_, 0, sizeof(tf_matrix_));
	memset(&uf_matrix_inverse_, 0, sizeof(uf_matrix_inverse_));
	memset(&tf_matrix_inverse_, 0, sizeof(tf_matrix_inverse_));
	uf_matrix_.rotation_matrix_.matrix_[0][0] = 1;
	uf_matrix_.rotation_matrix_.matrix_[1][1] = 1;
	uf_matrix_.rotation_matrix_.matrix_[2][2] = 1;
	tf_matrix_.rotation_matrix_.matrix_[0][0] = 1;
	tf_matrix_.rotation_matrix_.matrix_[1][1] = 1;
	tf_matrix_.rotation_matrix_.matrix_[2][2] = 1;
	uf_matrix_inverse_.rotation_matrix_.matrix_[0][0] = 1;
	uf_matrix_inverse_.rotation_matrix_.matrix_[1][1] = 1;
	uf_matrix_inverse_.rotation_matrix_.matrix_[2][2] = 1;
	tf_matrix_inverse_.rotation_matrix_.matrix_[0][0] = 1;
	tf_matrix_inverse_.rotation_matrix_.matrix_[1][1] = 1;
	tf_matrix_inverse_.rotation_matrix_.matrix_[2][2] = 1;

	// line_smooth_planner_.initPlanner(joint_num, cycle_time, pkinematics);
	joint_smooth_planner_.initPlanner(joint_num, cycle_time);

	constraint_ptr_ = pconstraint;
	log_ptr_ = log_ptr;

	vector<double> omega_max; 
    vector<double> alpha_max;
    vector<double> beta_max;
	fst_parameter::ParamGroup yaml_help;
	string constraint_file_path = ALGORITHM_DIR;
	constraint_file_path += "constraint.yaml";

	if (!yaml_help.loadParamFile(constraint_file_path.c_str())
        || !yaml_help.getParam("joint/omega_max", omega_max)
        || !yaml_help.getParam("joint/alpha_max", alpha_max)
        || !yaml_help.getParam("joint/beta_max_acc", beta_max))
    {
        std::cout << " Failed load constraint.yaml " << std::endl;
        return false;
    }

	for (uint32_t j = 0; j < joint_num_; ++j)
    {
        omega_max_[j] = omega_max[j];
        alpha_max_[j] = alpha_max[j];
        beta_max_[j] =  beta_max[j];
    }

	joint_smooth_planner_.setLimit(omega_max_, alpha_max_, beta_max_);
	return true;
}


void SmoothPlanner::setLimit(const Joint &vel_limit_joint, const Joint &acc_limit_joint, const Joint &jerk_limit_joint, 
				  double vel_limit_position, double acc_limit_position, double jerk_limit_position, 
				  double vel_limit_orientation, double acc_limit_orientation, double jerk_limit_orientation)
{
	// line_smooth_planner_.setLimit(acc_limit_position);
	acc_limit_position_ = acc_limit_position;
	//joint_smooth_planner_.setLimit(vel_limit_joint, acc_limit_joint, jerk_limit_joint);
}

bool SmoothPlanner::setUserTrans(const TransMatrix &uf)
{
	TransMatrix inverse_uf = uf;

	if (!inverse_uf.inverse())
	{
		return false;
	}

	uf_matrix_ = uf;
	uf_matrix_inverse_ = inverse_uf;
	// line_smooth_planner_.setUserTrans(uf_matrix_);
	return true;
}


bool SmoothPlanner::setToolTrans(const TransMatrix &tf)
{
	TransMatrix inverse_tf = tf;

	if (!inverse_tf.inverse())
	{
		return false;
	}

	tf_matrix_ = tf;
	tf_matrix_inverse_ = inverse_tf;
	// line_smooth_planner_.setToolTrans(tf_matrix_);
	return true;
}

bool SmoothPlanner::setUserFrame(const PoseEuler &uf)
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
	// line_smooth_planner_.setUserTrans(uf_matrix_);
	return true;
}

bool SmoothPlanner::setToolFrame(const PoseEuler &tf)
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
	// line_smooth_planner_.setToolTrans(tf_matrix_);
	return true;
}


ErrorCode SmoothPlanner::planTrajectory(const JointState &smooth_out_state, const Joint &fly_by_joint, const JointState &smooth_in_state)
{
	if ((motion_type_pre_ != MOTION_LINE && motion_type_pre_ != MOTION_JOINT && motion_type_pre_ != MOTION_CIRCLE) ||
		(motion_type_this_ != MOTION_LINE && motion_type_this_ != MOTION_JOINT && motion_type_this_ != MOTION_CIRCLE))
	{
		return INVALID_PARAMETER;
	}

	if (motion_type_pre_ == MOTION_LINE && motion_type_this_ == MOTION_LINE)
	{
		return planTrajectoryL2L(smooth_out_state, fly_by_joint, smooth_in_state);
	}

	return planTrajectoryJ2J(smooth_out_state, smooth_in_state);
}

ErrorCode SmoothPlanner::planCircleTrajectory(const basic_alg::PoseQuaternion pose_out, const basic_alg::PoseQuaternion pose_via, const basic_alg::PoseQuaternion pose_in,
		double position_out_vel, double position_in_vel,
		const fst_mc::JointState &smooth_out_state, const fst_mc::JointState &smooth_in_state)
{
	return planTrajectoryJ2J(smooth_out_state, smooth_in_state);
}

ErrorCode SmoothPlanner::planTrajectoryL2L(const JointState &smooth_out_state, const Joint &fly_by_joint, const JointState &smooth_in_state)
{
	joint_out_angle_ = smooth_out_state.angle;

	/* get via pose */
	TransMatrix trans_via;
	kinematics_ptr_->doFK(fly_by_joint, via_pose_);
	via_pose_.convertToTransMatrix(trans_via);
	trans_via.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_).convertToPoseQuaternion(via_pose_);

	/* compute vout and vin */
	TransMatrix trans_in, trans_out, trans_delta;
	kinematics_ptr_->doFK(smooth_in_state.angle, trans_in);
	kinematics_ptr_->doFK(smooth_out_state.angle, trans_out);
	trans_in.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_);
	trans_out.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_);
	Joint delta_in, delta_out;

	for (uint32_t i = 0; i < joint_num_; i++)
	{
		delta_in[i] = smooth_in_state.angle[i] + smooth_in_state.omega[i] * delta_time_;
		delta_out[i] = smooth_out_state.angle[i] + smooth_out_state.omega[i] * delta_time_;
	}

	kinematics_ptr_->doFK(delta_in, trans_delta);
	trans_delta.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_);
	double vin = fabs(getDistance(trans_delta.trans_vector_, trans_in.trans_vector_)) / delta_time_;
	kinematics_ptr_->doFK(delta_out, trans_delta);
	trans_delta.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_);
	double vout = fabs(getDistance(trans_delta.trans_vector_, trans_out.trans_vector_)) / delta_time_;

	/* unify point uint */
	Point unified_point_out, unified_point_via, unified_point_in;
	unifyPointUint(point_out_, unified_point_out);
	unifyPointUint(via_pose_.point_, unified_point_via);
	unifyPointUint(point_in_, unified_point_in);

	/* plan point trajectory by bezier */
	position_planner_.planCurve(unified_point_out, unified_point_via, unified_point_in, 
		vout / unit_unified_param_, vin/unit_unified_param_, acc_limit_position_ / unit_unified_param_);
	point_smooth_total_time_ = position_planner_.getDuration();
	t_total_ = fabs(orientation_out_time_ - point_out_time_) + fabs(orientation_in_time_ - point_in_time_) + point_smooth_total_time_;
	double t_min_curva = position_planner_.getMinCurvaTime();

	/* plan quaternion trajctory by cubic spline */
	t_former_ = t_min_curva +  fabs(orientation_out_time_ - point_out_time_);
	t_last_ = t_total_ - t_former_;
	spline_orientation_planner_.planCurve(uout_, uin_, vout_u_, vin_u_, t_former_, t_last_);

	Quaternion quatern_via;
	quatern_via.x_ = via_pose_.quaternion_.x_;
	quatern_via.y_ = via_pose_.quaternion_.y_;
	quatern_via.z_ = via_pose_.quaternion_.z_;
	quatern_via.w_ = via_pose_.quaternion_.w_;

	spline_orientation_planner_.setQuaternions(quatern_out_, quatern_via, quatern_in_);

	ErrorCode err = SUCCESS;

	if (checkL2LTrajectory(0, t_former_) != SUCCESS)
	{
		quatern_via.x_ = -via_pose_.quaternion_.x_;
		quatern_via.y_ = -via_pose_.quaternion_.y_;
		quatern_via.z_ = -via_pose_.quaternion_.z_;
		quatern_via.w_ = -via_pose_.quaternion_.w_;

		FST_WARN("reverse via quaternion");
		spline_orientation_planner_.setQuaternions(quatern_out_, quatern_via, quatern_in_);

		err = checkL2LTrajectory(0, t_former_);
		if (err != SUCCESS)
		{
			FST_ERROR("Trajectory check failed.");
			return err;
		} 
	}

	if (checkL2LTrajectory(t_former_, t_last_) != SUCCESS)
	{
		quatern_in_.x_ = -quatern_in_.x_;
		quatern_in_.y_ = -quatern_in_.y_;
		quatern_in_.z_ = -quatern_in_.z_;
		quatern_in_.w_ = -quatern_in_.w_;

		FST_WARN("reverse in quaternion");
		spline_orientation_planner_.setQuaternions(quatern_out_, quatern_via, quatern_in_);

		err = checkL2LTrajectory(t_former_, t_last_);
		if (err != SUCCESS)
		{
			FST_ERROR("Trajectory check failed.");
		} 
	}

	return err;
}

ErrorCode SmoothPlanner::planTrajectoryJ2J(const JointState &smooth_out_state, const JointState &smooth_in_state)
{
	return joint_smooth_planner_.planTrajectory(smooth_out_state, smooth_in_state) ? SUCCESS : MC_TRAJECTORY_SMOOTH_FAIL;
}

ErrorCode SmoothPlanner::sampleTrajectoryL2L(double sample_time, basic_alg::Joint reference, uint32_t &point_num, fst_mc::JointState *points)
{
	uint32_t picked_num = 0;
	ErrorCode err = SUCCESS;

	Joint joint_delta1, joint_delta2;
	PoseQuaternion pose, pose_delta1, pose_delta2;
	double sample_time_temp = 0.0;
	for (uint32_t i = 0; i < point_num; i++)
	{
		if (sample_time < t_total_ - fabs(orientation_in_time_ - point_in_time_) - point_smooth_total_time_)
		{
			sample_time_temp = sample_time + orientation_out_time_;
			traj_planner_pre_.sampleLineCartesianTrajectory(sample_time_temp, pose);
			traj_planner_pre_.sampleLineCartesianTrajectory(sample_time_temp + delta_time_, pose_delta1);
			traj_planner_pre_.sampleLineCartesianTrajectory(sample_time_temp + delta_time_ * 2, pose_delta2);
		}
		else if (sample_time < t_total_ - fabs(orientation_in_time_ - point_in_time_))
		{
			sample_time_temp = sample_time - fabs(point_out_time_ - orientation_out_time_);
			position_planner_.sampleCurve(sample_time_temp, pose.point_);
			position_planner_.sampleCurve(sample_time_temp + delta_time_, pose_delta1.point_);
			position_planner_.sampleCurve(sample_time_temp + delta_time_ * 2, pose_delta2.point_);
			returnPointUint(pose.point_);
			returnPointUint(pose_delta1.point_);
			returnPointUint(pose_delta2.point_);
		}
		else
		{
			sample_time_temp = sample_time - fabs(point_out_time_ - orientation_out_time_) - point_smooth_total_time_ + point_in_time_;
			traj_planner_this_.sampleLineCartesianTrajectory(sample_time_temp, pose);
			traj_planner_this_.sampleLineCartesianTrajectory(sample_time_temp + delta_time_, pose_delta1);
			traj_planner_this_.sampleLineCartesianTrajectory(sample_time_temp + delta_time_ * 2, pose_delta2);
		}

		spline_orientation_planner_.sampleQuaternion(sample_time, pose.quaternion_);
		spline_orientation_planner_.sampleQuaternion(sample_time + delta_time_, pose_delta1.quaternion_);
		spline_orientation_planner_.sampleQuaternion(sample_time + delta_time_ * 2, pose_delta2.quaternion_);

		if (!doIK(pose, reference, points[i].angle) ||
			!doIK(pose_delta1, reference, joint_delta1) ||
			!doIK(pose_delta2, reference, joint_delta2))
		{
			err = MC_COMPUTE_IK_FAIL;
			break;
		}

		for (uint32_t j = 0; j < joint_num_; j++)
		{
			points[i].omega[j] = (joint_delta1[j] - points[i].angle[j]) / delta_time_;
			points[i].alpha[j] = ((joint_delta2[j] - joint_delta1[j]) / delta_time_ - points[i].omega[j]) / delta_time_;
		}

		picked_num++;

		if (sample_time > t_total_)
		{
			break;
		}

		sample_time += cycle_time_;
		reference = points[i].angle;
	}

	point_num = picked_num;
	return err;
}

ErrorCode SmoothPlanner::sampleTrajectory(double start_time, Joint reference, uint32_t &point_num, JointState *points)
{
	if ((motion_type_pre_ != MOTION_LINE && motion_type_pre_ != MOTION_JOINT && motion_type_pre_ != MOTION_CIRCLE) ||
		(motion_type_this_ != MOTION_LINE && motion_type_this_ != MOTION_JOINT && motion_type_this_ != MOTION_CIRCLE))
	{
		return INVALID_PARAMETER;
	}

	if (motion_type_pre_ == MOTION_LINE && motion_type_this_ == MOTION_LINE)
	{
		// return line_smooth_planner_.sampleTrajectory(start_time, reference, point_num, points);
		return sampleTrajectoryL2L(start_time, reference, point_num, points);
	}
	else
	{
		joint_smooth_planner_.sampleTrajectory(start_time, point_num, points);
		return SUCCESS;
	}
}

double SmoothPlanner::getDuration(void)
{
	if ((motion_type_pre_ != MOTION_LINE && motion_type_pre_ != MOTION_JOINT && motion_type_pre_ != MOTION_CIRCLE) ||
		(motion_type_this_ != MOTION_LINE && motion_type_this_ != MOTION_JOINT && motion_type_this_ != MOTION_CIRCLE))
	{
		return 0;
	}

	if (motion_type_pre_ == MOTION_LINE && motion_type_this_ == MOTION_LINE)
	{
		// return line_smooth_planner_.getDuration();
		return t_total_;
	}
	else
	{
		return joint_smooth_planner_.getDuration();
	}
}


void SmoothPlanner::setTrajectoryPlanner(fst_mc::TrajectoryPlanner& traj_planner_pre, fst_mc::TrajectoryPlanner& traj_planner_this)
{
	traj_planner_pre_ = traj_planner_pre;
	traj_planner_this_ = traj_planner_this;
}


void SmoothPlanner::updateTrajectoryInfo(double point_out_time, double point_in_time, double cycle_time,
	double &orientation_out_time, double &orientation_in_time)
{
	motion_type_pre_ = traj_planner_pre_.getMotionInfo().type;
	motion_type_this_ = traj_planner_this_.getMotionInfo().type;

	if (motion_type_pre_ == MOTION_LINE && motion_type_this_ == MOTION_LINE)
	{
		orientation_out_time_ = ceil(traj_planner_pre_.getDuration() / cycle_time / 2) * cycle_time;
		orientation_out_time_ = orientation_out_time_ > point_out_time ? point_out_time : orientation_out_time_;
		orientation_in_time_ = floor(traj_planner_this_.getDuration() / cycle_time / 2) * cycle_time;
		orientation_in_time_ = orientation_in_time_ < point_in_time ? point_in_time : orientation_in_time_;

		orientation_in_time = orientation_in_time_;
		orientation_out_time = orientation_out_time_;

		point_out_time_ = point_out_time;
		point_in_time_ = point_in_time;

		double a_out;
		traj_planner_pre_.sampleLineNormalTrajectory(orientation_out_time_, uout_, vout_u_, a_out);
		traj_planner_this_.sampleLineNormalTrajectory(orientation_in_time_, uin_, vin_u_, a_out);

		PoseQuaternion pose_out, pose_in;
		traj_planner_pre_.sampleLineCartesianTrajectory(orientation_out_time_, pose_out);
		traj_planner_this_.sampleLineCartesianTrajectory(orientation_in_time_, pose_in);
		quatern_out_.x_ = pose_out.quaternion_.x_; quatern_out_.y_ = pose_out.quaternion_.y_;
		quatern_out_.z_ = pose_out.quaternion_.z_; quatern_out_.w_ = pose_out.quaternion_.w_;
		quatern_in_.x_ = pose_in.quaternion_.x_; quatern_in_.y_ = pose_in.quaternion_.y_;
		quatern_in_.z_ = pose_in.quaternion_.z_; quatern_in_.w_ = pose_in.quaternion_.w_;

		traj_planner_pre_.sampleLineCartesianTrajectory(point_out_time_, pose_out);
		traj_planner_this_.sampleLineCartesianTrajectory(point_in_time_, pose_in);
		point_out_ = pose_out.point_;
		point_in_ = pose_in.point_;
	}
	else 
	{
		orientation_out_time_ = point_out_time;
		orientation_in_time_ = point_in_time;
		point_out_time_ = point_out_time;
		point_in_time_ = point_in_time;
		orientation_out_time = point_out_time;
		orientation_in_time = point_in_time;
	}
}

void SmoothPlanner::unifyPointUint(const basic_alg::Point &point, basic_alg::Point &unify_point)
{
	unify_point.x_ = point.x_ / unit_unified_param_;
	unify_point.y_ = point.y_ / unit_unified_param_;
	unify_point.z_ = point.z_ / unit_unified_param_;
}

void SmoothPlanner::returnPointUint(basic_alg::Point &point)
{
	point.x_ = point.x_ * unit_unified_param_;
	point.y_ = point.y_ * unit_unified_param_;
	point.z_ = point.z_ * unit_unified_param_;
}

void SmoothPlanner::doFK(const basic_alg::Joint &joint, basic_alg::PoseQuaternion &target)
{
	TransMatrix matrix;
	kinematics_ptr_->doFK(joint, target);
	target.convertToTransMatrix(matrix);
	matrix.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_).convertToPoseQuaternion(target);
}

bool SmoothPlanner::doIK(const basic_alg::PoseQuaternion &pose, const basic_alg::Joint reference, basic_alg::Joint &target_joint)
{
	TransMatrix trans_pose;
	pose.convertToTransMatrix(trans_pose);

	trans_pose.leftMultiply(uf_matrix_).rightMultiply(tf_matrix_inverse_);
	return kinematics_ptr_->doIK(trans_pose, reference, target_joint);
}

ErrorCode SmoothPlanner::checkL2LTrajectory(double time_start, double time_interval)
{
	double l2l_duration = time_interval;
	double duration_step = l2l_duration / 100;
	ErrorCode err = SUCCESS;
	Joint reference = joint_out_angle_;
	Joint check_joint;
	uint32_t point_num = 1;
	JointState point;

	TransMatrix matrix;
	PoseQuaternion pose;
	PoseEuler pe;
	for (double t = time_start + duration_step; t < time_start + l2l_duration + MINIMUM_E6; t += duration_step)
	{
		sampleTrajectoryL2L(t, reference, point_num, &point);
		check_joint = point.angle;

		if (!constraint_ptr_->isJointInConstraint(check_joint))
		{
			doFK(check_joint, pose);
			pose.convertToPoseEuler(pe);
			FST_WARN("PoseQuaternion: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.quaternion_.x_, pose.quaternion_.y_, pose.quaternion_.z_, pose.quaternion_.w_);
			FST_WARN("PoseEuler: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe.point_.x_, pe.point_.y_, pe.point_.z_, pe.euler_.a_, pe.euler_.b_, pe.euler_.c_);
			FST_WARN("Reference: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", reference.j1_, reference.j2_, reference.j3_, reference.j4_, reference.j5_, reference.j6_);
			FST_WARN("Result: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", check_joint.j1_, check_joint.j2_, check_joint.j3_, check_joint.j4_, check_joint.j5_, check_joint.j6_);
			FST_WARN("Trajectory out of constraint, total_time = %lf, t = %.6f.", l2l_duration, t);

			err = JOINT_OUT_OF_CONSTRAINT;
			break;
		}

		reference = check_joint;
	}

	return err;
}
