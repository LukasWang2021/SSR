/*************************************************************************
    > File Name: joint_planner.cpp
    > Author: 
    > Mail: 
    > Created Time: 2019年11月27日 星期三 13时06分55秒
 ************************************************************************/
#include <string.h>
#include <iostream>
#include <basic_alg.h>
#include <joint_planner.h>

using namespace group_space;
using namespace basic_alg;

JointPlanner::JointPlanner(void)
{
    ds_curve_ = NULL;
    joint_num_ = 6;

    memset(&start_, 0, sizeof(start_));
    memset(&end_, 0, sizeof(end_));
    memset(&vel_limit_, 0, sizeof(vel_limit_));
    memset(&acc_limit_, 0, sizeof(acc_limit_));
    memset(&jerk_limit_, 0, sizeof(jerk_limit_));

    jerk_num_ = 0;
}
JointPlanner::~JointPlanner(void)
{
    if (ds_curve_ != NULL)
    {
        delete ds_curve_;
        ds_curve_ = NULL;
    }
}
bool JointPlanner::initPlanner(uint32_t joint_num, uint32_t jerk_num)
{
    jerk_num_ = jerk_num;
    joint_num_ = joint_num;

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

void JointPlanner::setLimit(const Joint &vel_limit, const Joint &acc_limit, const Joint *jerk_limit)
{
    vel_limit_ = vel_limit;
    acc_limit_ = acc_limit;

    for (uint32_t i = 0; i < jerk_num_; ++i)
    {
        jerk_limit_[i] = jerk_limit[i];
    }
}


void JointPlanner::planTrajectory(const Joint &start, const Joint &end, double vel, double vel_ratio, double acc_ratio, double jerk_ratio)
{
    start_ = start;
    end_ = end;

    double dif_pos[NUM_OF_JOINT], lim_vel[NUM_OF_JOINT], lim_acc[NUM_OF_JOINT], lim_jerk[3][NUM_OF_JOINT];
    //速度加速度加加速度归一化处理
    for (uint32_t j = 0; j < joint_num_; j++)
    {  
        dif_pos[j] = fabs(end_[j] - start_[j]);
        lim_vel[j] = vel_limit_[j] * vel / dif_pos[j];
        // lim_vel[j] = vel_limit_[j] / dif_pos[j];
        lim_acc[j] = acc_limit_[j] * acc_ratio / dif_pos[j];
    }
    
    for (uint32_t i = 0; i != jerk_num_; ++i)
    {
        for (uint32_t j = 0; j != joint_num_; ++j)
        {
            lim_jerk[i][j] = jerk_limit_[i][j] * jerk_ratio / dif_pos[j];
        }
    }
    
    double max_vel = minInArray(lim_vel, joint_num_);
    double max_acc = minInArray(lim_acc, joint_num_);
    double max_jerk[MAX_JERK_NUM];
    
    for (uint32_t i = 0; i != jerk_num_; ++i)
    {
        max_jerk[i] = minInArray(lim_jerk[i], joint_num_);
    }
    
    ds_curve_->planDSCurve(0, 1, max_vel, max_acc, max_jerk, vel_ratio);
}

void JointPlanner::sampleTrajectory(double t, JointState &sample)
{
    double dif_pos[NUM_OF_JOINT];

    for (uint32_t j = 0; j < joint_num_; j++)
    {
        dif_pos[j] =end_[j] - start_[j];
    }

    double ps, vs, as;
    ds_curve_->sampleDSCurve(t, ps, vs, as);

    for (uint32_t j = 0; j < joint_num_; j++)
    {
        sample.angle[j] = start_[j] + ps * dif_pos[j];
        sample.omega[j] = vs * dif_pos[j];
        sample.alpha[j] = as * dif_pos[j];
    }
}

double JointPlanner::getDuration(void)
{
    return ds_curve_->getDuration();
}

double JointPlanner::getSegmentEndingTime(DSSetment segment)
{
	return ds_curve_->getSegmentEndingTime(segment);
}

void JointPlanner::planStopTrajectory(double stop_time)
{
    ds_curve_->planStopDSCurve(stop_time);
}