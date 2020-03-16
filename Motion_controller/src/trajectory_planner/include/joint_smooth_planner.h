#ifndef _JOINT_SMOOTH_PLANNER_H
#define _JOINT_SMOOTH_PLANNER_H

#include "joint.h"
#include "error_code.h"
#include "motion_control_datatype.h"

struct TrajectorySegment
{
	double duration;
	double data[7];
};

struct SmoothTrajectory
{
	uint32_t num_of_segment;
	TrajectorySegment segments[6];
};


class JointSmoothPlanner
{
public:
	JointSmoothPlanner(void);
	~JointSmoothPlanner(void);
	void initPlanner(uint32_t joint_num, double cycle_time);
	void setLimit(const basic_alg::Joint &omega_limit, const basic_alg::Joint &alpha_limit, const basic_alg::Joint &beta_limit);
	bool planTrajectory(const fst_mc::JointState& start, const fst_mc::JointState& end);
	void sampleTrajectory(double start_time, uint32_t &point_num, fst_mc::JointState *points);
	double getDuration(void);

private:
	fst_mc::JointState start_;
	fst_mc::JointState end_;

	uint32_t joint_num_;
	double t_total_;
	double cycle_time_;

	double a0_[NUM_OF_JOINT];
	double a1_[NUM_OF_JOINT];
	double a2_[NUM_OF_JOINT];
	double a3_[NUM_OF_JOINT];
	double a4_[NUM_OF_JOINT];
	double a5_[NUM_OF_JOINT];
	double omega_max_[NUM_OF_JOINT];
	double alpha_max_[NUM_OF_JOINT];
	double beta_max_[NUM_OF_JOINT];
	SmoothTrajectory trajectory_[NUM_OF_JOINT];
	void computeQuinticCoeff();
	void sampleTrajectory(double sample_time, fst_mc::JointState &point);


};

#endif