#include <math.h>
#include <string.h>
#include <float.h>
#include "joint_smooth_planner.h"

using namespace basic_alg;


JointSmoothPlanner::JointSmoothPlanner(void)
{
	for (int j = 0; j != NUM_OF_JOINT; ++j)
	{
		start_.angle[j] = 0.0;
		start_.omega[j] = 0.0;
		start_.alpha[j] = 0.0;

		end_.angle[j] = 0.0;
		end_.omega[j] = 0.0;
		end_.alpha[j] = 0.0;
	}

	t_total_ = 0.0;
	joint_num_ = 0;
	cycle_time_ = 0.0;
}

JointSmoothPlanner::~JointSmoothPlanner(void)
{

}
void JointSmoothPlanner::initPlanner(uint32_t joint_num, double cycle_time)
{
	joint_num_ = joint_num;
	cycle_time_ = cycle_time;
}
void JointSmoothPlanner::setLimit(const Joint &omega_limit, const Joint &alpha_limit, const Joint &beta_limit)
{
	memcpy(omega_max_, &omega_limit, joint_num_ * sizeof(double));
	memcpy(alpha_max_, &alpha_limit, joint_num_ * sizeof(double));
	memcpy(beta_max_, &beta_limit, joint_num_ * sizeof(double));
}
bool JointSmoothPlanner::planTrajectory(const fst_mc::JointState& start, const fst_mc::JointState& end)
{
	start_ = start;
	end_ = end;
	double smooth_duration_min = 0, smooth_duration_max = DBL_MAX;
	double duration_max[NUM_OF_JOINT], duration_min[NUM_OF_JOINT], s[NUM_OF_JOINT];
	bool error_assert = false;

	for (uint32_t j = 0; j < joint_num_; j++)
	{
		double tc = fabs((end.omega[j] - start.omega[j]) / alpha_max_[j]);
		s[j] = end.angle[j] - start.angle[j] - (end.omega[j] + start.omega[j]) / 2 * tc;

		if ((s[j] > MINIMUM_E9 && end.omega[j] < 0 && start.omega[j] < 0) || (s[j] < -MINIMUM_E9 && end.omega[j] > 0 && start.omega[j] > 0))
		{
			error_assert = true;
			break;
		}

		if (fabs(s[j]) < MINIMUM_E9)
		{
			duration_min[j] = tc;
			duration_max[j] = DBL_MAX;
		}
		else if (s[j] > 0)
		{
			if (start.omega[j] > MINIMUM_E6 && end.omega[j] > MINIMUM_E6)
			{
				double t1 = s[j] / start.omega[j] + tc;
				double t2 = s[j] / end.omega[j] + tc;
				duration_min[j] = t1 < t2 ? t1 : t2;
				duration_max[j] = t1 < t2 ? t2 : t1;
			}
			else if (start.omega[j] > MINIMUM_E6)
			{
				duration_min[j] = s[j] / start.omega[j] + tc;
				duration_max[j] = DBL_MAX;
			}
			else if (end.omega[j] > MINIMUM_E6)
			{
				duration_min[j] = s[j] / end.omega[j] + tc;
				duration_max[j] = DBL_MAX;
			}
			else
			{}
		}
		else
		{
			if (start.omega[j] < -MINIMUM_E6 && end.omega[j] < -MINIMUM_E6)
			{
				double t1 = s[j] / start.omega[j] + tc;
				double t2 = s[j] / end.omega[j] + tc;
				duration_min[j] = t1 < t2 ? t1 : t2;
				duration_max[j] = t1 < t2 ? t2 : t1;
			}
			else if (start.omega[j] < -MINIMUM_E6)
			{
				duration_min[j] = s[j] / start.omega[j] + tc;
				duration_max[j] = DBL_MAX;
			}
			else if (end.omega[j] < -MINIMUM_E6)
			{
				duration_min[j] = s[j] / end.omega[j] + tc;
				duration_max[j] = DBL_MAX;
			}
			else
			{}
		}

		if (duration_min[j] > smooth_duration_min)
		{
			smooth_duration_min = duration_min[j];
		}

		if (duration_max[j] < smooth_duration_max)
		{
			smooth_duration_max = duration_max[j];
		}

		printf("tc: %.6f, s: %.6f, duration_min: %.6f\n", tc, s[j], duration_min[j]);
	}

	t_total_ = smooth_duration_min;
	computeQuinticCoeff();
	return true;
	/*
	//printf("min: %.6f, max: %.6f\n", smooth_duration_min, smooth_duration_max);

	if (error_assert || smooth_duration_min > smooth_duration_max)
	{
		printf("Duration generate failed: min = %.6f, max = %.6f\n", smooth_duration_min, smooth_duration_max);
		printf("duration_min: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f\n", duration_min[0], duration_min[1], duration_min[2], duration_min[3], duration_min[4], duration_min[5]);
		printf("duration_max: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f\n", duration_max[0], duration_max[1], duration_max[2], duration_max[3], duration_max[4], duration_max[5]);
		return false;
	}

	double smooth_duration = smooth_duration_min;
	t_total_ = smooth_duration;
	printf("smooth duration: %.6f\n", smooth_duration);

	for (uint32_t j = 0; j < joint_num_; j++)
	{
		double t2 = fabs((end.omega[j] - start.omega[j]) / alpha_max_[j]);
		double t3 = fabs(end.omega[j] - start.omega[j]) < MINIMUM_E6 ? 0 : (s[j] - (smooth_duration - t2) * start.omega[j]) / (end.omega[j] - start.omega[j]);
		double t1 = smooth_duration - t2 - t3;	

		if (t1 < -MINIMUM_E6 || t3 < -MINIMUM_E6)
		{
			printf("j: %d, t1: %.6f, t2: %.6f, t3: %.6f\n", j, t1, t2, t3);
			printf("vs: %.6f, ve: %.6f, trip: %.6f, a: %.6f, s: %.6f\n", start.omega[j], end.omega[j], end.angle[j] - start.angle[j], alpha_max_[j], s[j]);
			return false;
		}

		SmoothTrajectory &trajectory = trajectory_[j];
		trajectory.num_of_segment = 3;
		trajectory.segments[0].duration = t1;
		trajectory.segments[1].duration = t2;
		trajectory.segments[2].duration = t3;
		memset(trajectory.segments[0].data, 0, sizeof(trajectory.segments[0].data));
		memset(trajectory.segments[1].data, 0, sizeof(trajectory.segments[1].data));
		memset(trajectory.segments[2].data, 0, sizeof(trajectory.segments[2].data));
		trajectory.segments[0].data[0] = start.angle[j];
		trajectory.segments[0].data[1] = start.omega[j];
		trajectory.segments[1].data[0] = start.angle[j] + start.omega[j] * t1;
		trajectory.segments[1].data[1] = start.omega[j];
		trajectory.segments[1].data[2] = (end.omega[j] - start.omega[j]) / t2 / 2;
		trajectory.segments[2].data[0] = end.angle[j] - end.omega[j] * t3;
		trajectory.segments[2].data[1] = end.omega[j];
	}

	return true;
	*/
}

void JointSmoothPlanner::sampleTrajectory(double start_time, uint32_t &point_num, fst_mc::JointState *points)
{
	uint32_t picked_num = 0;
	double sample_time = start_time;

	for (uint32_t i = 0; i < point_num; i++)
	{
		sampleTrajectory(sample_time, points[i]);
		picked_num++;
		
		if (sample_time > t_total_)
		{
			break;
		}

		sample_time += cycle_time_;
	}

	point_num = picked_num;
}

double JointSmoothPlanner::getDuration(void)
{
	return t_total_;
}

void JointSmoothPlanner::sampleTrajectory(double sample_time, fst_mc::JointState &point)
{
	sample_time = sample_time > 0 ? sample_time : 0;
	sample_time = sample_time > t_total_ ? t_total_ : sample_time;
	double time_array[6];
	time_array[1] = sample_time;
	time_array[2] = time_array[1] * sample_time;
	time_array[3] = time_array[2] * sample_time;
	time_array[4] = time_array[3] * sample_time;
	time_array[5] = time_array[4] * sample_time;

	for (int j = 0; j != joint_num_; ++j)
	{
		point.angle[j] =
			a0_[j]
			+ a1_[j] * time_array[1]
			+ a2_[j] * time_array[2]
			+ a3_[j] * time_array[3]
			+ a4_[j] * time_array[4]
			+ a5_[j] * time_array[5];

		point.omega[j] =
			a1_[j]
			+ a2_[j] * time_array[1] * 2
			+ a3_[j] * time_array[2] * 3
			+ a4_[j] * time_array[3] * 4
			+ a5_[j] * time_array[4] * 5;

		point.alpha[j] =
			+a2_[j] * 2
			+ a3_[j] * time_array[1] * 6
			+ a4_[j] * time_array[2] * 12
			+ a5_[j] * time_array[3] * 20;

		point.jerk[j] =
			+a3_[j] * 6
			+ a4_[j] * time_array[1] * 24
			+ a5_[j] * time_array[2] * 60;
	}
	/*
	sample_time = sample_time > 0 ? sample_time : 0;
	sample_time = sample_time < t_total_ ? sample_time : t_total_;

	for (uint32_t j = 0; j < joint_num_; j++)
	{
		uint32_t seg_num;
		double t = sample_time;
		const SmoothTrajectory &trajectory = trajectory_[j];

		for (seg_num = 0; seg_num < trajectory.num_of_segment; seg_num++)
		{
			if (t < trajectory.segments[seg_num].duration)
			{
				break;
			}

			t -= trajectory.segments[seg_num].duration;
		}

		if (seg_num == trajectory.num_of_segment)
		{
			seg_num = trajectory.num_of_segment - 1;
			t = trajectory.segments[seg_num].duration;
		}

		point.angle[j] = trajectory.segments[seg_num].data[0] + trajectory.segments[seg_num].data[1] * t + trajectory.segments[seg_num].data[2] * t * t + trajectory.segments[seg_num].data[3] * t * t * t;
		point.omega[j] = trajectory.segments[seg_num].data[1] + trajectory.segments[seg_num].data[2] * t * 2 + trajectory.segments[seg_num].data[3] * t * t * 3;
		point.alpha[j] = trajectory.segments[seg_num].data[2] * 2 + trajectory.segments[seg_num].data[3] * t * 6;
	}
	*/
}



void JointSmoothPlanner::computeQuinticCoeff()
{
	if (t_total_ < MINIMUM_E6)
	{
		for (int j = 0; j != joint_num_; ++j)
		{
			a0_[j] = start_.angle[j];
			a1_[j] = start_.omega[j];
			a2_[j] = start_.alpha[j] / 2;
			a3_[j] = 0;
			a4_[j] = 0;
			a5_[j] = 0;
		}
		
		return;
	}

	double t[6];
	t[1] = t_total_;
	t[2] = t[1] * t_total_;
	t[3] = t[2] * t_total_;
	t[4] = t[3] * t_total_;
	t[5] = t[4] * t_total_;

	for (int j = 0; j != joint_num_; ++j)
	{
		a0_[j] = start_.angle[j];
		a1_[j] = start_.omega[j];
		a2_[j] = start_.alpha[j] / 2;
		a3_[j] = (end_.angle[j] * 20 - start_.angle[j] * 20 + end_.alpha[j] * t[2]
			- start_.alpha[j] * t[2] * 3 - start_.omega[j] * t[1] * 12 - end_.omega[j] * t[1] * 8) / (t[3] * 2);

		a4_[j] = (start_.angle[j] * 30 - end_.angle[j] * 30 + start_.omega[j] * t[1] * 16 + end_.omega[j] * t[1] * 14
			+ start_.alpha[j] * t[2] * 3 - end_.alpha[j] * t[2] * 2) / (t[4] * 2);
		a5_[j] = (end_.angle[j] * 12 - start_.angle[j] * 12 - start_.omega[j] * t[1] * 6 - end_.omega[j] * t[1] * 6
			- start_.alpha[j] * t[2] + end_.alpha[j] * t[2]) / (t[5] * 2);
	}
}

