/*************************************************************************
    > File Name: joint_planner.h
    > Author: 
    > Mail: 
    > Created Time: 2019年11月26日 星期二 17时00分18秒
 ************************************************************************/

#ifndef _JOINT_PLANNER_H
#define _JOINT_PLANNER_H

#include "joint.h"
#include "ds_planner/ds_planner.h"
#include "ds_planner/ds_planner_single_jerk.h"
#include "ds_planner/ds_planner_three_jerk.h"
#include "ds_planner/ds_planner_two_jerk.h"
#include "motion_control_datatype.h"
#include "basic_alg_datatype.h"

class JointPlanner
{
public:
	JointPlanner(void);
	~JointPlanner(void);
	bool initPlanner(uint32_t joint_num, uint32_t jerk_num);
	void setLimit(const basic_alg::Joint &vel_limit, const basic_alg::Joint &acc_limit, const basic_alg::Joint *jerk_limit);
	void planStopTrajectory(double stop_time);
	void planTrajectory(const basic_alg::Joint &start, const basic_alg::Joint &end, double vel, double vel_ratio, double acc_ratio, double jerk_ratio);
	void sampleTrajectory(double t, fst_mc::JointState &sample);
	double getDuration(void);
	double getSegmentEndingTime(DSSetment segment);

private:
	uint32_t joint_num_;
	uint32_t jerk_num_;
	basic_alg::Joint start_;
	basic_alg::Joint end_;
	basic_alg::Joint vel_limit_;
	basic_alg::Joint acc_limit_;
	basic_alg::Joint jerk_limit_[3];
	DSCurvePlanner *ds_curve_;
};



#endif
