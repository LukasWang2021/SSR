#include "pose_quaternion.h"
#include "basic_alg.h"
#include "error_code.h"
#include "line_smooth_planner.h"

using namespace std;
using namespace basic_alg;

LineSmoothPlanner::LineSmoothPlanner(void):
	t_total_(0.0),
	unit_unified_param_(1000),
	acc_limit_(0.0),
	joint_num_(0),
	cycle_time_(0.0),
	delta_time_(0.0)
{
	kinematics_ptr_ = NULL;
	bezier_curve_ = new BezierPlanner();
	start_.point_.x_ = 0.0;
	start_.point_.y_ = 0.0;
	start_.point_.z_ = 0.0;
	start_.quaternion_.x_ = 0.0;
	start_.quaternion_.y_ = 0.0;
	start_.quaternion_.z_ = 0.0;
	start_.quaternion_.w_ = 0.0;
	via_.point_.x_ = 0.0;
	via_.point_.y_ = 0.0;
	via_.point_.z_ = 0.0;
	via_.quaternion_.x_ = 0.0;
	via_.quaternion_.y_ = 0.0;
	via_.quaternion_.z_ = 0.0;
	via_.quaternion_.w_ = 0.0;
	end_.point_.x_ = 0.0;
	end_.point_.y_ = 0.0;
	end_.point_.z_ = 0.0;
	end_.quaternion_.x_ = 0.0;
	end_.quaternion_.y_ = 0.0;
	end_.quaternion_.z_ = 0.0;
	end_.quaternion_.w_ = 0.0;
}

LineSmoothPlanner::~LineSmoothPlanner(void)
{
	if (bezier_curve_ != NULL)
	{
		delete bezier_curve_;
		bezier_curve_ = NULL;
	}
}

bool LineSmoothPlanner::initPlanner(uint32_t joint_num, double cycle_time, basic_alg::Kinematics* pkinematics)
{
	joint_num_ = joint_num;
	cycle_time_ = cycle_time;
	delta_time_ = cycle_time_ / 100;
	kinematics_ptr_ = pkinematics;

	if (kinematics_ptr_ == NULL) return false;

	return true;
}

void LineSmoothPlanner::setLimit(double position_acc_limit)
{
	acc_limit_ = position_acc_limit;
}

bool LineSmoothPlanner::setUserTrans(const basic_alg::TransMatrix &uf)
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

bool LineSmoothPlanner::setToolTrans(const basic_alg::TransMatrix &tf)
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

ErrorCode LineSmoothPlanner::planTrajectory(const basic_alg::Joint& start, const basic_alg::Joint& via, const basic_alg::Joint& end, 
		double vout, double vin,
		double uout, double uin, double vout_u, double vin_u)
{
	doFK(start, start_);
	doFK(via, via_);
	doFK(end, end_);

	start_.point_.x_ /= unit_unified_param_; start_.point_.y_ /= unit_unified_param_; start_.point_.z_ /= unit_unified_param_;
	via_.point_.x_ /= unit_unified_param_; 	via_.point_.y_ /= unit_unified_param_; via_.point_.z_ /= unit_unified_param_;
	end_.point_.x_ /= unit_unified_param_; 	end_.point_.y_ /= unit_unified_param_; end_.point_.z_ /= unit_unified_param_;

	bezier_curve_->planCurve(start_, via_, end_, vout/unit_unified_param_, 
		vin/unit_unified_param_, acc_limit_/unit_unified_param_,
		uout, uin, vout_u, vin_u);

	t_total_ = bezier_curve_->getDuration();

	return SUCCESS;
}

ErrorCode LineSmoothPlanner::sampleTrajectory(double start_time, basic_alg::Joint reference, uint32_t &point_num, fst_mc::JointState *points)
{
	double sample_time = start_time;
	uint32_t picked_num = 0;
	ErrorCode err = SUCCESS;

	Joint joint_delta1, joint_delta2;
	PoseQuaternion pose, pose_delta1, pose_delta2;

	for (uint32_t i = 0; i < point_num; i++)
	{
		bezier_curve_->sampleCurve(sample_time, pose);
		bezier_curve_->sampleCurve(sample_time + delta_time_, pose_delta1);
		bezier_curve_->sampleCurve(sample_time + delta_time_ * 2, pose_delta2);

		pose.point_.x_ *= unit_unified_param_; pose.point_.y_ *= unit_unified_param_; pose.point_.z_ *= unit_unified_param_;
		pose_delta1.point_.x_ *= unit_unified_param_; pose_delta1.point_.y_ *= unit_unified_param_; pose_delta1.point_.z_ *= unit_unified_param_;
		pose_delta2.point_.x_ *= unit_unified_param_; pose_delta2.point_.y_ *= unit_unified_param_; pose_delta2.point_.z_ *= unit_unified_param_;

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

double LineSmoothPlanner::getDuration(void)
{
	return t_total_;
}

void LineSmoothPlanner::doFK(const basic_alg::Joint &joint, basic_alg::PoseQuaternion &target)
{
	TransMatrix matrix;
	kinematics_ptr_->doFK(joint, target);
	target.convertToTransMatrix(matrix);
	matrix.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_).convertToPoseQuaternion(target);
}


bool LineSmoothPlanner::doIK(const basic_alg::PoseQuaternion &pose,  const basic_alg::Joint reference, basic_alg::Joint &target_joint)
{
	TransMatrix trans_pose;
	pose.convertToTransMatrix(trans_pose);

	trans_pose.leftMultiply(uf_matrix_).rightMultiply(tf_matrix_inverse_);
	return kinematics_ptr_->doIK(trans_pose, reference, target_joint);
}


