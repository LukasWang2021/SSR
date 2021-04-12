/*************************************************************************
	> File Name: line_planner.h
	> Author: 
	> Mail: 
	> Created Time: 2019年11月26日 星期二 17时07分55秒
 ************************************************************************/

#ifndef _LINE_PLANNER_H
#define _LINE_PLANNER_H

#include <pose_quaternion.h>
#include <ds_planner/ds_planner.h>

class LinePlanner
{
public:
	LinePlanner(void);
	~LinePlanner(void);
	bool initPlanner(uint32_t joint_num, uint32_t jerk_num);
	void setLimit(double vel_limit_position, double acc_limit_position, double *jerk_limit_position, 
				  double vel_limit_orientation, double acc_limit_orientation, double *jerk_limit_orientation);
	void planTrajectory(const basic_alg::PoseQuaternion &start, const basic_alg::PoseQuaternion &end, double vel, double vel_ratio, double acc_ratio, double jerk_ratio);
	void planAlternativeTrajectory(const basic_alg::PoseQuaternion &start, const basic_alg::PoseQuaternion &end, double vel, double vel_ratio, double acc_ratio, double jerk_ratio);
	void planStopTrajectory(double stop_time);
	void sampleTrajectory(double t, basic_alg::PoseQuaternion &sample);
	void sampleNormalTrajectory(double t, double &u, double &v, double &a);
	double getDuration(void);
	double getSegmentEndingTime(DSSetment segment);

private:
	uint32_t jerk_num_;
	basic_alg::PoseQuaternion 	start_;
	basic_alg::PoseQuaternion 	end_;
	DSCurvePlanner *ds_curve_;
	double orientation_angle_;
	double limit_vel_position_;
	double limit_acc_position_;
	double limit_jerk_position_[3];
	double limit_vel_orientation_;
	double limit_acc_orientation_;
	double limit_jerk_orientation_[3];
};

#endif
