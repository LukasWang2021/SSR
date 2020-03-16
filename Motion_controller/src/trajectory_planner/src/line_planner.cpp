/*************************************************************************
	> File Name: line_planner.cpp
	> Author: 
	> Mail: 
	> Created Time: 2019年11月27日 星期三 10时01分55秒
 ************************************************************************/
#include <string.h>
#include <iostream>
#include "ds_planner/ds_planner_single_jerk.h"
#include "ds_planner/ds_planner_three_jerk.h"
#include "ds_planner/ds_planner_two_jerk.h"
#include <line_planner.h>
#include <basic_alg.h>

using namespace std;
using namespace basic_alg;

LinePlanner::LinePlanner(void)
{
	ds_curve_ = NULL;
	jerk_num_ = 1;
	orientation_angle_ = 0;
	limit_vel_position_ = 0;
	limit_acc_position_ = 0;
	memset(limit_jerk_position_, 0, sizeof(limit_jerk_position_));;
	limit_vel_orientation_ = 0;
	limit_acc_orientation_ = 0;
	memset(limit_jerk_orientation_, 0, sizeof(limit_jerk_orientation_));;
}

LinePlanner::~LinePlanner(void)
{
	if (ds_curve_ != NULL)
	{
		delete ds_curve_;
		ds_curve_ = NULL;
	}
}

bool LinePlanner::initPlanner(uint32_t joint_num, uint32_t jerk_num)
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

void LinePlanner::setLimit(double vel_limit_position, double acc_limit_position, double *jerk_limit_position, 
				  			double vel_limit_orientation, double acc_limit_orientation, double *jerk_limit_orientation)
{
	limit_vel_position_ = vel_limit_position;
	limit_acc_position_ = acc_limit_position;
	memcpy(limit_jerk_position_, jerk_limit_position, sizeof(limit_jerk_position_));
	limit_vel_orientation_ = vel_limit_orientation;
	limit_acc_orientation_ = acc_limit_orientation;
	memcpy(limit_jerk_orientation_, jerk_limit_orientation, sizeof(limit_jerk_orientation_));
}

void LinePlanner::planTrajectory(const PoseQuaternion &start, const PoseQuaternion &end, double vel, double acc, double jerk)
{
	double distance = getDistance(end.point_, start.point_);
	double lim_cart_vel = limit_vel_position_ * vel / distance;
	double lim_cart_acc = limit_acc_position_ * acc / distance;
	//double lim_cart_jerk = limit_jerk_position_ / distance;

	start_ = start;
	end_ = end;

	double angle = end_.quaternion_.getIncludedAngle(start_.quaternion_);
	orientation_angle_ = angle;
	double lim_orientation_vel = limit_vel_orientation_ * vel / angle;
	double lim_orientation_acc = limit_acc_orientation_ * acc / angle;
	//double lim_orientation_jerk = limit_jerk_orientation_ / angle;

	double max_jerk[MAX_JERK_NUM];
	double max_pos_jerk[MAX_JERK_NUM];
	double max_ori_jerk[MAX_JERK_NUM];

    for (uint32_t i = 0; i < jerk_num_; ++i)
    {
        max_pos_jerk[i] = limit_jerk_position_[i] * jerk / distance;
		max_ori_jerk[i] = limit_jerk_orientation_[i] * jerk / angle;
		max_jerk[i] = max_pos_jerk[i] < max_ori_jerk[i] ? max_pos_jerk[i] : max_ori_jerk[i];
    }

	double max_vel = lim_cart_vel < lim_orientation_vel ? lim_cart_vel : lim_orientation_vel;
	double max_acc = lim_cart_acc < lim_orientation_acc ? lim_cart_acc : lim_orientation_acc;
	//double max_jerk = lim_cart_jerk < lim_orientation_jerk ? lim_cart_jerk : lim_orientation_jerk;

	ds_curve_->planDSCurve(0, 1, max_vel, max_acc, max_jerk);
}


void LinePlanner::planAlternativeTrajectory(const PoseQuaternion &start, const PoseQuaternion &end, double vel, double acc, double jerk)
{
	double distance = getDistance(end.point_, start.point_);
	double lim_cart_vel = limit_vel_position_ * vel / distance;
	double lim_cart_acc = limit_acc_position_ * acc / distance;
	//double lim_cart_jerk = limit_jerk_position_ / distance;

	start_ = start;
	end_ = end;
	end_.quaternion_.w_ = -end_.quaternion_.w_;
	end_.quaternion_.x_ = -end_.quaternion_.x_;
	end_.quaternion_.y_ = -end_.quaternion_.y_;
	end_.quaternion_.z_ = -end_.quaternion_.z_;

	double angle = end_.quaternion_.getIncludedAngle(start_.quaternion_);
	orientation_angle_ = angle;
	double lim_orientation_vel = limit_vel_orientation_ * vel / angle;
	double lim_orientation_acc = limit_acc_orientation_ * acc / angle;
	//double lim_orientation_jerk = limit_jerk_orientation_ / angle;

	double max_jerk[MAX_JERK_NUM];
	double max_pos_jerk[MAX_JERK_NUM];
	double max_ori_jerk[MAX_JERK_NUM];

    for (uint32_t i = 0; i < jerk_num_; ++i)
    {
        max_pos_jerk[i] = limit_jerk_position_[i] * jerk / distance;
		max_ori_jerk[i] = limit_jerk_orientation_[i] * jerk / angle;
		max_jerk[i] = max_pos_jerk[i] < max_ori_jerk[i] ? max_pos_jerk[i] : max_ori_jerk[i];
    }

	double max_vel = lim_cart_vel < lim_orientation_vel ? lim_cart_vel : lim_orientation_vel;
	double max_acc = lim_cart_acc < lim_orientation_acc ? lim_cart_acc : lim_orientation_acc;
	//double max_jerk = lim_cart_jerk < lim_orientation_jerk ? lim_cart_jerk : lim_orientation_jerk;

	ds_curve_->planDSCurve(0, 1, max_vel, max_acc, max_jerk);
}

void LinePlanner::sampleNormalTrajectory(double t, double &u, double &v, double &a)
{
	ds_curve_->sampleDSCurve(t, u, v, a);
}

void LinePlanner::sampleTrajectory(double t, PoseQuaternion &sample)
{
	double p, v, a;
	ds_curve_->sampleDSCurve(t, p, v, a);

	// 位置线性差值
	sample.point_.x_ = (1 - p) * start_.point_.x_ + p * end_.point_.x_;
	sample.point_.y_ = (1 - p) * start_.point_.y_ + p * end_.point_.y_;
	sample.point_.z_ = (1 - p) * start_.point_.z_ + p * end_.point_.z_;

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

double LinePlanner::getDuration(void)
{
	return ds_curve_->getDuration();
}

double LinePlanner::getSegmentEndingTime(DSSetment segment)
{
	return ds_curve_->getSegmentEndingTime(segment);
}

void LinePlanner::planStopTrajectory(double stop_time)
{
	ds_curve_->planStopDSCurve(stop_time);
}
