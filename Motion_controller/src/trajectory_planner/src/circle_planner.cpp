/*************************************************************************
	> File Name: circle_planner.cpp
	> Author: 
	> Mail: 
	> Created Time: 2019年12月13日 星期五 09时47分52秒
 ************************************************************************/
#include <string.h>
#include <math.h>
#include <assert.h>
#include <iostream>
#include "ds_planner/ds_planner_single_jerk.h"
#include "ds_planner/ds_planner_three_jerk.h"
#include "ds_planner/ds_planner_two_jerk.h"
#include <circle_planner.h>
#include <basic_alg.h>

using namespace std;
using namespace basic_alg;

CirclePlanner::CirclePlanner(void)
{
	ds_curve_ = NULL;
	jerk_num_ = 1;
	radius_ = 0;
	position_angle_ = 0;
	orientation_angle_ = 0;
	limit_vel_position_ = 0;
	limit_acc_position_ = 0;
	memset(limit_jerk_position_, 0, sizeof(limit_jerk_position_));;
	limit_vel_orientation_ = 0;
	limit_acc_orientation_ = 0;
	memset(limit_jerk_orientation_, 0, sizeof(limit_jerk_orientation_));;
}

CirclePlanner::~CirclePlanner(void)
{
	if (ds_curve_ != NULL)
	{
		delete ds_curve_;
		ds_curve_ = NULL;
	}
}

bool CirclePlanner::initPlanner(uint32_t joint_num, uint32_t jerk_num)
{
	jerk_num_ = jerk_num;

	if (ds_curve_ != NULL)
	{
		delete ds_curve_;
		ds_curve_ = NULL;
	}

    switch (jerk_num_)
    {
        case 1:
            ds_curve_ = new SingleJerkDSCurvePlanner();
        break;
        case 2: 
            ds_curve_ = new TwoJerkDSCurvePlanner();
        break; 
        case 3: 
            ds_curve_ = new ThreeJerkDSCurvePlanner();
        break; 
        default:
            return false;
    }

	return ds_curve_ != NULL;
}

void CirclePlanner::setLimit(double vel_limit_position, double acc_limit_position, double *jerk_limit_position, 
				  			double vel_limit_orientation, double acc_limit_orientation, double *jerk_limit_orientation)
{
	limit_vel_position_ = vel_limit_position;
	limit_acc_position_ = acc_limit_position;
	memcpy(limit_jerk_position_, jerk_limit_position, sizeof(limit_jerk_position_));
	limit_vel_orientation_ = vel_limit_orientation;
	limit_acc_orientation_ = acc_limit_orientation;
	memcpy(limit_jerk_orientation_, jerk_limit_orientation, sizeof(limit_jerk_orientation_));
}

void CirclePlanner::planTrajectory(const PoseQuaternion &start, const PoseQuaternion &via, const PoseQuaternion &end, 
	//double vel, double acc, double jerk)
	double vel, double vel_ratio, double acc_ratio, double jerk_ratio)
{
	Point ab = via.point_ - start.point_;
	Point ac = end.point_ - start.point_;
	double labab = ab.x_ * ab.x_ + ab.y_ * ab.y_ + ab.z_ * ab.z_;
	double lacac = ac.x_ * ac.x_ + ac.y_ * ac.y_ + ac.z_ * ac.z_;
	double labac = ab.x_ * ac.x_ + ab.y_ * ac.y_ + ab.z_ * ac.z_;
	double u = (labac * lacac - lacac * labab) / (labab * lacac - labac * labac) / 2;
	double v = (labac * labab - labab * lacac) / (labab * lacac - labac * labac) / 2;
	Point oa = ab * u + ac * v;
	Point center = start.point_ - oa;
	Point ob = via.point_ - center;
	Point oc = end.point_ - center;
	trans_matrix_.trans_vector_ = center;
	double vector_oa[3] = {oa.x_, oa.y_, oa.z_};
	double vector_ob[3] = {ob.x_, ob.y_, ob.z_};
	//double vector_oc[3] = {oc.x_, oc.y_, oc.z_};
	double unit_vector_z[3];
	crossProduct(vector_oa, vector_ob, unit_vector_z);
	mulScalar2Vector(1 / norm(unit_vector_z), unit_vector_z);
	double norm_oa = norm(vector_oa);
	//double norm_ob = norm(vector_ob);
	//double norm_oc = norm(vector_oc);
	double unit_vector_x[3] = {vector_oa[0] / norm_oa, vector_oa[1] / norm_oa, vector_oa[2] / norm_oa};
	double unit_vector_y[3];
	crossProduct(unit_vector_z, unit_vector_x, unit_vector_y);
	trans_matrix_.rotation_matrix_.matrix_[0][0] = unit_vector_x[0];
	trans_matrix_.rotation_matrix_.matrix_[0][1] = unit_vector_y[0];
	trans_matrix_.rotation_matrix_.matrix_[0][2] = unit_vector_z[0];
	trans_matrix_.rotation_matrix_.matrix_[1][0] = unit_vector_x[1];
	trans_matrix_.rotation_matrix_.matrix_[1][1] = unit_vector_y[1];
	trans_matrix_.rotation_matrix_.matrix_[1][2] = unit_vector_z[1];
	trans_matrix_.rotation_matrix_.matrix_[2][0] = unit_vector_x[2];
	trans_matrix_.rotation_matrix_.matrix_[2][1] = unit_vector_y[2];
	trans_matrix_.rotation_matrix_.matrix_[2][2] = unit_vector_z[2];
	trans_matrix_inverse_= trans_matrix_;
	trans_matrix_inverse_.inverse();
	TransMatrix trans_matrix;
	start.convertToTransMatrix(trans_matrix);
	trans_matrix.leftMultiply(trans_matrix_inverse_);
	Point qa = trans_matrix.trans_vector_;
	via.convertToTransMatrix(trans_matrix);
	trans_matrix.leftMultiply(trans_matrix_inverse_);
	Point qb = trans_matrix.trans_vector_;
	end.convertToTransMatrix(trans_matrix);
	trans_matrix.leftMultiply(trans_matrix_inverse_);
	Point qc = trans_matrix.trans_vector_;
	theta1_ = atan2(qb.y_, qb.x_);
	theta2_ = atan2(qc.y_, qc.x_);
	assert(theta1_ > 0);
	theta2_ = theta2_ < 0 ? theta2_ + M_PI * 2 : theta2_;
	rotate_direction_ = theta1_ < theta2_ ? 1 : -1;
	position_angle_ = rotate_direction_ > 0 ? theta2_ : M_PI * 2 - theta2_;
	radius_ = norm_oa;

	double distance = position_angle_ * radius_;
	double lim_cart_vel = limit_vel_position_ * vel / distance;
	// double lim_cart_vel = limit_vel_position_ / distance;
	double lim_cart_acc = limit_acc_position_ * acc_ratio / distance;

	start_ = start;
	via_ = via;
	end_ = end;
	orientation_angle_ = end_.quaternion_.getIncludedAngle(start_.quaternion_);

#if 0
	if (orientation_angle_ > M_PI / 2)
	{
		end_.quaternion_.w_ = -end_.quaternion_.w_;
		end_.quaternion_.x_ = -end_.quaternion_.x_;
		end_.quaternion_.y_ = -end_.quaternion_.y_;
		end_.quaternion_.z_ = -end_.quaternion_.z_;
		orientation_angle_ = end_.quaternion_.getIncludedAngle(start_.quaternion_);
	}
#endif
	double lim_orientation_vel = limit_vel_orientation_ * vel / orientation_angle_;
	double lim_orientation_acc = limit_acc_orientation_ * acc_ratio / orientation_angle_;
	double max_jerk[MAX_JERK_NUM];
	double max_pos_jerk;
	double max_ori_jerk;

    for (uint32_t i = 0; i < jerk_num_; ++i)
    {
        max_pos_jerk = limit_jerk_position_[i] * jerk_ratio / distance;
		max_ori_jerk = limit_jerk_orientation_[i] * jerk_ratio / orientation_angle_;
		max_jerk[i] = max_pos_jerk < max_ori_jerk ? max_pos_jerk : max_ori_jerk;
    }

	double max_vel = lim_cart_vel < lim_orientation_vel ? lim_cart_vel : lim_orientation_vel;
	double max_acc = lim_cart_acc < lim_orientation_acc ? lim_cart_acc : lim_orientation_acc;
	ds_curve_->planDSCurve(0, 1, max_vel, max_acc, max_jerk, vel_ratio);
}

void CirclePlanner::planAlternativeTrajectory(const basic_alg::PoseQuaternion &start, const basic_alg::PoseQuaternion &via, const basic_alg::PoseQuaternion &end, 
	double vel, double vel_ratio, double acc_ratio, double jerk_ratio)
{
	Point ab = via.point_ - start.point_;
	Point ac = end.point_ - start.point_;
	double labab = ab.x_ * ab.x_ + ab.y_ * ab.y_ + ab.z_ * ab.z_;
	double lacac = ac.x_ * ac.x_ + ac.y_ * ac.y_ + ac.z_ * ac.z_;
	double labac = ab.x_ * ac.x_ + ab.y_ * ac.y_ + ab.z_ * ac.z_;
	double u = (labac * lacac - lacac * labab) / (labab * lacac - labac * labac) / 2;
	double v = (labac * labab - labab * lacac) / (labab * lacac - labac * labac) / 2;
	Point oa = ab * u + ac * v;
	Point center = start.point_ - oa;
	Point ob = via.point_ - center;
	Point oc = end.point_ - center;
	trans_matrix_.trans_vector_ = center;
	double vector_oa[3] = {oa.x_, oa.y_, oa.z_};
	double vector_ob[3] = {ob.x_, ob.y_, ob.z_};
	//double vector_oc[3] = {oc.x_, oc.y_, oc.z_};
	double unit_vector_z[3];
	crossProduct(vector_oa, vector_ob, unit_vector_z);
	mulScalar2Vector(1 / norm(unit_vector_z), unit_vector_z);
	double norm_oa = norm(vector_oa);
	//double norm_ob = norm(vector_ob);
	//double norm_oc = norm(vector_oc);
	double unit_vector_x[3] = {vector_oa[0] / norm_oa, vector_oa[1] / norm_oa, vector_oa[2] / norm_oa};
	double unit_vector_y[3];
	crossProduct(unit_vector_z, unit_vector_x, unit_vector_y);
	trans_matrix_.rotation_matrix_.matrix_[0][0] = unit_vector_x[0];
	trans_matrix_.rotation_matrix_.matrix_[0][1] = unit_vector_y[0];
	trans_matrix_.rotation_matrix_.matrix_[0][2] = unit_vector_z[0];
	trans_matrix_.rotation_matrix_.matrix_[1][0] = unit_vector_x[1];
	trans_matrix_.rotation_matrix_.matrix_[1][1] = unit_vector_y[1];
	trans_matrix_.rotation_matrix_.matrix_[1][2] = unit_vector_z[1];
	trans_matrix_.rotation_matrix_.matrix_[2][0] = unit_vector_x[2];
	trans_matrix_.rotation_matrix_.matrix_[2][1] = unit_vector_y[2];
	trans_matrix_.rotation_matrix_.matrix_[2][2] = unit_vector_z[2];
	trans_matrix_inverse_= trans_matrix_;
	trans_matrix_inverse_.inverse();
	TransMatrix trans_matrix;
	start.convertToTransMatrix(trans_matrix);
	trans_matrix.leftMultiply(trans_matrix_inverse_);
	Point qa = trans_matrix.trans_vector_;
	via.convertToTransMatrix(trans_matrix);
	trans_matrix.leftMultiply(trans_matrix_inverse_);
	Point qb = trans_matrix.trans_vector_;
	end.convertToTransMatrix(trans_matrix);
	trans_matrix.leftMultiply(trans_matrix_inverse_);
	Point qc = trans_matrix.trans_vector_;
	theta1_ = atan2(qb.y_, qb.x_);
	theta2_ = atan2(qc.y_, qc.x_);
	assert(theta1_ > 0);
	theta2_ = theta2_ < 0 ? theta2_ + M_PI * 2 : theta2_;
	rotate_direction_ = theta1_ < theta2_ ? 1 : -1;
	position_angle_ = rotate_direction_ > 0 ? theta2_ : M_PI * 2 - theta2_;
	radius_ = norm_oa;

	double distance = position_angle_ * radius_;
	double lim_cart_vel = limit_vel_position_ * vel / distance;
	double lim_cart_acc = limit_acc_position_ * acc_ratio / distance;

	start_ = start;
	via_ = via;
	end_ = end;
	end_.quaternion_.x_ = -end.quaternion_.x_;
	end_.quaternion_.y_ = -end.quaternion_.y_;
	end_.quaternion_.z_ = -end.quaternion_.z_;
	end_.quaternion_.w_ = -end.quaternion_.w_;

	orientation_angle_ = end_.quaternion_.getIncludedAngle(start_.quaternion_);

	double lim_orientation_vel = limit_vel_orientation_ * vel / orientation_angle_;
	double lim_orientation_acc = limit_acc_orientation_ * acc_ratio / orientation_angle_;
	double max_jerk[MAX_JERK_NUM];
	double max_pos_jerk;
	double max_ori_jerk;

    for (uint32_t i = 0; i < jerk_num_; ++i)
    {
        max_pos_jerk = limit_jerk_position_[i] * jerk_ratio / distance;
		max_ori_jerk = limit_jerk_orientation_[i] * jerk_ratio / orientation_angle_;
		max_jerk[i] = max_pos_jerk < max_ori_jerk ? max_pos_jerk : max_ori_jerk;
    }

	double max_vel = lim_cart_vel < lim_orientation_vel ? lim_cart_vel : lim_orientation_vel;
	double max_acc = lim_cart_acc < lim_orientation_acc ? lim_cart_acc : lim_orientation_acc;
	ds_curve_->planDSCurve(0, 1, max_vel, max_acc, max_jerk, vel_ratio);
}

void CirclePlanner::sampleTrajectory(double t, basic_alg::PoseQuaternion &sample)
{
	double p, v, a;
	ds_curve_->sampleDSCurve(t, p, v, a);

	// 圆心角差值,位置差值
	TransMatrix trans;
	double theta = p * position_angle_ * rotate_direction_;
	trans.trans_vector_.x_ = radius_ * cos(theta);
	trans.trans_vector_.y_ = radius_ * sin(theta);
	trans.trans_vector_.z_ = 0;
	trans.leftMultiply(trans_matrix_);
	sample.point_ = trans.trans_vector_;

	if (orientation_angle_ < 0.1)
	{
		// 姿态夹角小于0.1rad,姿态线性差值
		sample.quaternion_.w_ = (1 - p) * start_.quaternion_.w_ + p * end_.quaternion_.w_;
		sample.quaternion_.x_ = (1 - p) * start_.quaternion_.x_ + p * end_.quaternion_.x_;
		sample.quaternion_.y_ = (1 - p) * start_.quaternion_.y_ + p * end_.quaternion_.y_;
		sample.quaternion_.z_ = (1 - p) * start_.quaternion_.z_ + p * end_.quaternion_.z_;
	}
	else
	{
		// 姿态夹角大于0.1rad,姿态球面差值
		sample.quaternion_.w_ = (sin((1 - p) * orientation_angle_) * start_.quaternion_.w_ + sin(p * orientation_angle_) * end_.quaternion_.w_) / sin(orientation_angle_);
		sample.quaternion_.x_ = (sin((1 - p) * orientation_angle_) * start_.quaternion_.x_ + sin(p * orientation_angle_) * end_.quaternion_.x_) / sin(orientation_angle_);
		sample.quaternion_.y_ = (sin((1 - p) * orientation_angle_) * start_.quaternion_.y_ + sin(p * orientation_angle_) * end_.quaternion_.y_) / sin(orientation_angle_);
		sample.quaternion_.z_ = (sin((1 - p) * orientation_angle_) * start_.quaternion_.z_ + sin(p * orientation_angle_) * end_.quaternion_.z_) / sin(orientation_angle_);
	}
}

void CirclePlanner::sampleTrajectory(double t, double &sample_position_vel, basic_alg::PoseQuaternion &sample_pose)
{
	double p, v, a;
	ds_curve_->sampleDSCurve(t, p, v, a);

	// 圆心角差值,位置差值
	TransMatrix trans;
	double theta = p * position_angle_ * rotate_direction_;
	trans.trans_vector_.x_ = radius_ * cos(theta);
	trans.trans_vector_.y_ = radius_ * sin(theta);
	trans.trans_vector_.z_ = 0;
	trans.leftMultiply(trans_matrix_);
	sample_pose.point_ = trans.trans_vector_;
	sample_position_vel = position_angle_ * v * radius_; // v = wr

	if (orientation_angle_ < 0.1)
	{
		// 姿态夹角小于0.1rad,姿态线性差值
		sample_pose.quaternion_.w_ = (1 - p) * start_.quaternion_.w_ + p * end_.quaternion_.w_;
		sample_pose.quaternion_.x_ = (1 - p) * start_.quaternion_.x_ + p * end_.quaternion_.x_;
		sample_pose.quaternion_.y_ = (1 - p) * start_.quaternion_.y_ + p * end_.quaternion_.y_;
		sample_pose.quaternion_.z_ = (1 - p) * start_.quaternion_.z_ + p * end_.quaternion_.z_;
	}
	else
	{
		// 姿态夹角大于0.1rad,姿态球面差值
		sample_pose.quaternion_.w_ = (sin((1 - p) * orientation_angle_) * start_.quaternion_.w_ + sin(p * orientation_angle_) * end_.quaternion_.w_) / sin(orientation_angle_);
		sample_pose.quaternion_.x_ = (sin((1 - p) * orientation_angle_) * start_.quaternion_.x_ + sin(p * orientation_angle_) * end_.quaternion_.x_) / sin(orientation_angle_);
		sample_pose.quaternion_.y_ = (sin((1 - p) * orientation_angle_) * start_.quaternion_.y_ + sin(p * orientation_angle_) * end_.quaternion_.y_) / sin(orientation_angle_);
		sample_pose.quaternion_.z_ = (sin((1 - p) * orientation_angle_) * start_.quaternion_.z_ + sin(p * orientation_angle_) * end_.quaternion_.z_) / sin(orientation_angle_);
	}
}

double CirclePlanner::getDuration(void)
{
	return ds_curve_->getDuration();
}

double CirclePlanner::getSegmentEndingTime(DSSetment segment)
{
	return ds_curve_->getSegmentEndingTime(segment);
}

void CirclePlanner::planStopTrajectory(double stop_time)
{
	ds_curve_->planStopDSCurve(stop_time);
}

