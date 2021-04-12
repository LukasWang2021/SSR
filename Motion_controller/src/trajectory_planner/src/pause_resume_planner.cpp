#include "pause_resume_planner.h"
#include "log_manager_producer.h"
#include <fstream>
#include <sstream>
#include <sys/file.h>

#define DOUBLE_MIN	1E-10
#define DOUBLE_MAX	1E10
#define DOUBLE_PAUSE_CONSTANT -0.003000
#define DOUBLE_RESUME_CONSTANT 0.003000
#define DOUBLE_CONSTANT_UP 0.001000
#define DOUBLE_CONSTANT_DOWN -0.001000
#define INPUT_POINTS 20

using namespace fst_mc;
using namespace log_space;

PauseResumePlanner::PauseResumePlanner(void)
{
	d_ratio_ = 0.000000;
	joint_num_ = 0;
	b_end_flag_ = false;
	memset(v_speed_change_flag_, false, sizeof(v_speed_change_flag_));
	g_cur_pos_in_old_time_ = 0;
	memset(g_cur_pause_speed_, 0, sizeof(g_cur_pause_speed_));
	memset(g_cur_pause_pos_, 0, sizeof(g_cur_pause_pos_));
	memset(acc_ratio_, 0, sizeof(acc_ratio_));
	n_input_point_counts_ = 0;
}

PauseResumePlanner::~PauseResumePlanner(void)
{
	v_traj_.clear();
}

bool PauseResumePlanner::initPausePlanner(uint32_t joint_num)
{
	joint_num_ = joint_num;
	LogProducer::info("traj_planner","joint num = %d", joint_num_);

	return true;
}

ErrorCode PauseResumePlanner::planPauseTrajectory(double d_ratio, std::vector<JointState> &trajectory, std::vector<JointState> &pause_trajectory, uint32_t &pause_at)
{
	LogProducer::info("traj_planner","Begin plan Pause Trajectory");

	//init global data
	memset(v_speed_change_flag_, false, sizeof(v_speed_change_flag_));
	n_input_point_counts_ = trajectory.size();
	LogProducer::info("traj_planner","Input trajectory point counts = %d", n_input_point_counts_);
	if(n_input_point_counts_ < 2)
	{
		LogProducer::error("traj_planner","No planned trajectory points!");
		return MC_PAUSE_FAILED;
	}

	v_traj_.clear();
	v_traj_.assign(trajectory.begin(), trajectory.end());

	// std::ofstream out_before_pause("/root/before_pause_compute.csv");
	// for (auto it : v_traj_) 
	// {
    //     out_before_pause << 
	// 	it.angle[0] << "," << it.angle[1] << "," << it.angle[2] << "," << it.angle[3] << "," << it.angle[4] << "," << it.angle[5] << "," << 
	// 	it.omega[0] << "," << it.omega[1] << "," << it.omega[2] << "," << it.omega[3] << "," << it.omega[4] << "," << it.omega[5] << "," << 
	// 	it.alpha[0] << "," << it.alpha[1] << "," << it.alpha[2] << "," << it.alpha[3] << "," << it.alpha[4] << "," << it.alpha[5] << "," << std::endl;
	// }
	// out_before_pause.close();

	pause_trajectory.clear();
	pause_trajectory.push_back(v_traj_[0]);

	d_ratio = d_ratio < 0.25 ? 0.25 : d_ratio;
	LogProducer::info("traj_planner","%f", d_ratio);
	d_ratio_ = d_ratio;

	g_cur_pos_in_old_time_ = 0;
    for(uint32_t i = 0; i < joint_num_; i++)
    {
		getSpeedOld(g_cur_pos_in_old_time_, i, &(g_cur_pause_speed_[i]));
		getPosOld(g_cur_pos_in_old_time_, i, &(g_cur_pause_pos_[i]));
	}

	double k = 1.000000;
	acc_ratio_[0] = k * 24.000000 * d_ratio;
	acc_ratio_[1] = k * 20.000000 * d_ratio;
	acc_ratio_[2] = k * 24.000000 * d_ratio;
	acc_ratio_[3] = k * 32.000000 * d_ratio;
	acc_ratio_[4] = k * 28.800000 * d_ratio;
	acc_ratio_[5] = k * 40.000000 * d_ratio;

	//pause task
	// std::vector<double> v_max_time;
	// v_max_time.clear();
	double tmp1_pre[NUM_OF_JOINT];
	memset(&tmp1_pre, 0, sizeof(tmp1_pre));
	double d_constant = DOUBLE_PAUSE_CONSTANT;
	double d_delta_time = 1.000000;
	do
	{
		//1.calc the current pos by max acc
		double acc_martch[NUM_OF_JOINT];
		double tmp1, tmp2;
		double speed_old;
		double pause_length_min[NUM_OF_JOINT];
		double nxt_pause_pos[NUM_OF_JOINT];
		for(uint32_t i = 0; i < joint_num_; i++)
		{
			if(g_cur_pause_speed_[i] > 0)
				acc_martch[i] = -acc_ratio_[i];
			else
				acc_martch[i] = acc_ratio_[i];

			//check if speed is very small and we should not use full acc
			if(fabs(g_cur_pause_speed_[i]) < fabs(acc_martch[i] * 0.001))
			{
				//too slow, get the new t
				//t = v / a
				tmp2 = fabs(g_cur_pause_speed_[i] / acc_martch[i]);
				tmp1 = g_cur_pause_speed_[i] * tmp2 + 0.5 * acc_martch[i] * tmp2 * tmp2;
				// LogProducer::info("traj_planner","new time: tmp2 = %.20f; tmp1 = %.20f", tmp2, tmp1);
			}
			else
			{
				//tmp1 = current_speed * 1ms + 0.5 * acc_martch[i] * 1ms * 1ms
				tmp1 = g_cur_pause_speed_[i] * 0.001 + 0.5 * acc_martch[i] * 0.001 * 0.001;
				// LogProducer::info("traj_planner","time: 0.001; tmp1 = %.20f", tmp1);
			}

			if(tmp1_pre[i] * tmp1 < 0)
			{
				// LogProducer::info("traj_planner","Acceleration causes the trajectory to back");
				pause_length_min[i] = 0.000000;	
			}
			else
			{
				getSpeedOld(g_cur_pos_in_old_time_, i, &speed_old);
				//limit the speed, not faster than the old one.
				if(fabs(tmp1) > fabs(speed_old * 0.001))
					pause_length_min[i] = speed_old * 0.001;
				else
					pause_length_min[i] = tmp1;
			}
			tmp1_pre[i] = tmp1;

			//add to next pos
			nxt_pause_pos[i] = g_cur_pause_pos_[i] + pause_length_min[i];
			// LogProducer::info("traj_planner","axis = %d: acc = %f, delta = %g, cur_pos = %g, nxt_pos = %g", i+1, acc_martch[i], pause_length_min[i], g_cur_pause_pos_[i], nxt_pause_pos[i]);
		}

		//2. find the time for next step
		double nxt_step_pos_time_old[NUM_OF_JOINT];
		for(uint32_t i = 0; i < joint_num_; i++)
		{
			getMartchTime(nxt_pause_pos[i], i, &(nxt_step_pos_time_old[i]));
			// LogProducer::info("traj_planner","axis: %d, next time is: %.20f", i + 1, nxt_step_pos_time_old[i]);	
		}

		//3. find the max time one
		tmp1 = 0.0;
		for(uint32_t i = 0; i < joint_num_; i++)
		{
			if(nxt_step_pos_time_old[i] > tmp1)
				tmp1 = nxt_step_pos_time_old[i];
		}

		// for(uint32_t i = 0; i < joint_num_; i++)
		// {
		// 	v_max_time.push_back(nxt_step_pos_time_old[i]);
		// }
		// v_max_time.push_back(tmp1);

		// LogProducer::info("traj_planner","final next time is: %.20f", tmp1);
		// LogProducer::info("traj_planner","g_cur_pos_in_old_time_ is: %.20f", g_cur_pos_in_old_time_);

		if(((tmp1 - g_cur_pos_in_old_time_) - d_delta_time - d_constant) > DOUBLE_CONSTANT_UP )
		{
			d_constant = d_constant + DOUBLE_CONSTANT_UP;
			tmp1 = g_cur_pos_in_old_time_ + d_delta_time + d_constant;
			d_delta_time = tmp1 - g_cur_pos_in_old_time_;	 
		}
		else if(((tmp1 - g_cur_pos_in_old_time_) - d_delta_time - d_constant) < DOUBLE_CONSTANT_DOWN)
		{
			d_constant = d_constant + DOUBLE_CONSTANT_DOWN;
			tmp1 = g_cur_pos_in_old_time_ + d_delta_time + d_constant;
			d_delta_time = tmp1 - g_cur_pos_in_old_time_;
		}
		else
		{
			d_delta_time = tmp1 - g_cur_pos_in_old_time_;
		}
		if(tmp1 < g_cur_pos_in_old_time_)
		{
			tmp1 = g_cur_pos_in_old_time_;
		}
		// LogProducer::info("traj_planner","d_delta_time: %.20f", d_delta_time);

		//4. if the max time is zero, we have done.
		if(fabs(tmp1 - g_cur_pos_in_old_time_) < DOUBLE_MIN)
		{
			LogProducer::info("traj_planner","pause Done!");
			b_end_flag_ = true;
		}

		//5.update the time and the position and speed and acc
		fst_mc::JointState temp_JointState;
		memset(&temp_JointState, 0, sizeof(temp_JointState));

		g_cur_pos_in_old_time_ = tmp1;
		// LogProducer::info("traj_planner","Update cur time: %.20f", g_cur_pos_in_old_time_);

		if(floor(g_cur_pos_in_old_time_) < ( n_input_point_counts_ - 1))
		{
			//get all pos
			for(uint32_t i = 0; i < joint_num_; i++)
			{
				double tmp_new_pos, tmp_delta_pos/*, tmp_delta_acc*/;
				//get the new pos 
				getPosOld(g_cur_pos_in_old_time_, i, &(tmp_new_pos));
				tmp_delta_pos = tmp_new_pos - g_cur_pause_pos_[i];
				g_cur_pause_pos_[i] = tmp_new_pos;
				temp_JointState.angle[i] = g_cur_pause_pos_[i];
				acc_martch[i] = (tmp_delta_pos * 1000 - g_cur_pause_speed_[i]) / 0.001;
				temp_JointState.alpha[i] = acc_martch[i];

				//use diff pos get the speed.
				g_cur_pause_speed_[i] = tmp_delta_pos * 1000;
				// LogProducer::info("traj_planner","axis = %d, new_pos = %f, new speed = %f", i + 1, g_cur_pause_pos_[i], g_cur_pause_speed_[i]);
				temp_JointState.omega[i] = g_cur_pause_speed_[i];
			}

			pause_trajectory.push_back(temp_JointState);
			pause_at = ceil(g_cur_pos_in_old_time_);
		}
		else
		{
			LogProducer::error("traj_planner","There are not enough input points for planning the trajectory");
			LogProducer::info("traj_planner","trajectory point total count: %d", pause_trajectory.size());
			b_end_flag_ = false;
			return MC_PAUSE_FAILED;
		}
	}
	while(!b_end_flag_);
	b_end_flag_ = false;

	LogProducer::info("traj_planner","trajectory point total count: %d", pause_trajectory.size());
	// for(uint32_t i = 0; i < pause_trajectory.size(); i++)
	// {
	// 	LogProducer::info("traj_planner","%f, %f, %f, %f, %f, %f", pause_trajectory[i].angle[0], pause_trajectory[i].angle[1], pause_trajectory[i].angle[2], 
	// 	pause_trajectory[i].angle[3], pause_trajectory[i].angle[4], pause_trajectory[i].angle[5]);
	// }

	// std::ofstream out_pause_time("/root/pause_max_time.csv");
	// for (auto it = v_max_time.begin(); it != v_max_time.end(); ++it)
	// {
	// 	out_pause_time << *it << "," << *(++it) << "," << *(++it) << "," << *(++it) << "," << *(++it) << "," << *(++it) << "," << *(++it) << "," << std::endl;
	// }
	// out_pause_time.close();

	// std::ofstream out_pause("/root/pause_compute.csv");
	// for (auto it : pause_trajectory) 
	// {
    //     out_pause << 
	// 	it.angle[0] << "," << it.angle[1] << "," << it.angle[2] << "," << it.angle[3] << "," << it.angle[4] << "," << it.angle[5] << "," << 
	// 	it.omega[0] << "," << it.omega[1] << "," << it.omega[2] << "," << it.omega[3] << "," << it.omega[4] << "," << it.omega[5] << "," << 
	// 	it.alpha[0] << "," << it.alpha[1] << "," << it.alpha[2] << "," << it.alpha[3] << "," << it.alpha[4] << "," << it.alpha[5] << "," << std::endl;
	// }
	// out_pause.close();

	LogProducer::info("traj_planner","Finish plan Pause Trajectory");

    return SUCCESS;
}

ErrorCode PauseResumePlanner::planResumeTrajectory(std::vector<JointState> &trajectory, std::vector<JointState> &resume_trajectory, uint32_t &resume_at, double d_ratio)
{
	LogProducer::info("traj_planner","Begin plan Resume Trajectory");
	n_input_point_counts_ = trajectory.size();
	LogProducer::info("traj_planner","Input trajectory point counts = %d", n_input_point_counts_);
	if(n_input_point_counts_ < 3)
	{
		LogProducer::info("traj_planner","input trajectory points < 3");

		resume_trajectory.clear();
		resume_trajectory.assign(trajectory.begin(), trajectory.end());
		resume_at = n_input_point_counts_ - 1;

		return SUCCESS;
	}

	v_traj_.clear();
	v_traj_.assign(trajectory.begin(), trajectory.end());
	g_cur_pos_in_old_time_ = 0;
	fst_mc::JointState temp_JointState;
	memset(&temp_JointState, 0, sizeof(temp_JointState));
	if(n_input_point_counts_ < INPUT_POINTS)
	{
		LogProducer::info("traj_planner","Input trajectory point counts < 20");
		double acc_martch[NUM_OF_JOINT];
		memset(&acc_martch, 0, sizeof(acc_martch));
		for(int j = 0; j < (2 * (n_input_point_counts_ -1)); j++)
		{
			g_cur_pos_in_old_time_ = j * j / (4 * ((double)n_input_point_counts_ - 1.0));
			if(g_cur_pos_in_old_time_ >= (double)n_input_point_counts_ -1.0)
			{
				g_cur_pos_in_old_time_ = n_input_point_counts_ - DOUBLE_MIN;
			}
			for(uint32_t i = 0; i < joint_num_; i++)
			{
				double tmp_new_pos, tmp_delta_pos;
				//get the new pos
				getPosOld(g_cur_pos_in_old_time_, i, &(tmp_new_pos));
				tmp_delta_pos = tmp_new_pos - g_cur_pause_pos_[i];
				g_cur_pause_pos_[i] = tmp_new_pos;
				temp_JointState.angle[i] = g_cur_pause_pos_[i];
				//get the acc.
				acc_martch[i] = (tmp_delta_pos * 1000 - g_cur_pause_speed_[i]) / 0.001;
				temp_JointState.alpha[i] = acc_martch[i];
				//get the speed.
				g_cur_pause_speed_[i] = tmp_delta_pos * 1000;
				temp_JointState.omega[i] = g_cur_pause_speed_[i];

				// LogProducer::info("traj_planner","forward compute, axis: %d, new_pos = %.20f, new speed = %.20f", i + 1, g_cur_pause_pos_[i], g_cur_pause_speed_[i]);
			}

			resume_trajectory.push_back(temp_JointState);
		}

		resume_at = n_input_point_counts_ - 1;

		return SUCCESS;
	}

	//init global data
	memset(v_speed_change_flag_, false, sizeof(v_speed_change_flag_));
	double d_resume_costant;
	double d_constant_up;
	double d_constant_down;
	d_resume_costant = 1.0 / (double)(n_input_point_counts_ - 1);
	d_constant_up = d_resume_costant;
	d_constant_down = -d_constant_up;
	LogProducer::info("traj_planner","d_resume_costant = %.20f, d_constant_up = %.20f, d_constant_down = %.20f", d_resume_costant, d_constant_up, d_constant_down);

	v_resume_traj_temp_.clear();
	resume_trajectory.clear();

	double k = 1.000000;
	acc_ratio_[0] = k * 24.000000 * d_ratio;
	acc_ratio_[1] = k * 20.000000 * d_ratio;
	acc_ratio_[2] = k * 24.000000 * d_ratio;
	acc_ratio_[3] = k * 32.000000 * d_ratio;
	acc_ratio_[4] = k * 28.800000 * d_ratio;
	acc_ratio_[5] = k * 40.000000 * d_ratio;
	
    for(uint32_t i = 0; i < joint_num_; i++)
    {
		g_cur_pause_speed_[i] = 0.000000;
		// LogProducer::info("traj_planner","new axis: %d, g_cur_pause_speed_ = %.20f", i + 1, g_cur_pause_speed_[i]);
		getPosOld(g_cur_pos_in_old_time_, i, &(g_cur_pause_pos_[i]));
		// LogProducer::info("traj_planner","axis: %d, g_cur_pause_pos_ = %.20f", i + 1, g_cur_pause_pos_[i]);
	}

	//resume task
	std::vector<double> v_min_time;
	v_min_time.clear();
	double d_speed[NUM_OF_JOINT];
	double d_pos[NUM_OF_JOINT];
	// fst_mc::JointState temp_JointState;
	// memset(&temp_JointState, 0, sizeof(temp_JointState));
	double tmp1_pre[NUM_OF_JOINT];
	memset(&tmp1_pre, 0, sizeof(tmp1_pre));
	double d_constant = d_resume_costant;
	double d_delta_time = 0.000000;
	do
	{
		//1. calc the next step length by max acc
		//2. find the time for next step
		double tmp1, tmp2;
		double acc_martch[NUM_OF_JOINT];
		double nxt_pause_pos[NUM_OF_JOINT];
		double nxt_step_pos_time_old[NUM_OF_JOINT];

		for(uint32_t i = 0; i < joint_num_; i++)
		{
			// LogProducer::info("traj_planner","axis: %d, time = %.20f;", i + 1, g_cur_pos_in_old_time_);
			getPosOld(g_cur_pos_in_old_time_ + 1.0, i, &tmp2);
			tmp1 = tmp2 - g_cur_pause_pos_[i];
			acc_martch[i] = 2 * (tmp1 - g_cur_pause_speed_[i] * 0.001) / (0.001 * 0.001);
			// LogProducer::info("traj_planner","axis: %d, acc_martch = %.20f; acc_ratio_ = %.20f; tmp2 = %.20f; tmp1 = %.20f", i + 1, acc_martch[i], acc_ratio_[i], tmp2, tmp1);
			if(fabs(acc_martch[i]) > acc_ratio_[i])
			{
				// LogProducer::info("traj_planner","fabs(acc_martch[i]) > acc_ratio_[i]");
				if(acc_martch[i] > 0)
					acc_martch[i] = acc_ratio_[i];
				else
					acc_martch[i] = -acc_ratio_[i];
				tmp1 = g_cur_pause_speed_[i] * 0.001 + 0.5 * acc_martch[i] * 0.001 * 0.001;

				if(tmp1_pre[i] * tmp1 < 0)
				{
					// LogProducer::info("traj_planner","Acceleration causes the trajectory to back");
					nxt_pause_pos[i] = g_cur_pause_pos_[i];	
				}
				else
				{
					//add to current step
					nxt_pause_pos[i] = g_cur_pause_pos_[i] + tmp1;
				}
				tmp1_pre[i] = tmp1;

				//find the time for this axis
				getMartchTimeResume(nxt_pause_pos[i], i, &(nxt_step_pos_time_old[i]));
			}
			else
			{
				// LogProducer::info("traj_planner","acc_martch <= acc_ratio_, axis: %d", i +1);
				nxt_step_pos_time_old[i] = g_cur_pos_in_old_time_ + 1.0;
			}
			// LogProducer::info("traj_planner","axis: %d, next time is: %.20f", i + 1, nxt_step_pos_time_old[i]);
		}

		//3. find the min time one
		tmp1 = DOUBLE_MAX;
		for(uint32_t i = 0; i < joint_num_; i++)
		{
			if(nxt_step_pos_time_old[i] < tmp1)
				tmp1 = nxt_step_pos_time_old[i];
		}

		for(uint32_t i = 0; i < joint_num_; i++)
		{
			v_min_time.push_back(nxt_step_pos_time_old[i]);
		}
		v_min_time.push_back(tmp1);

		if(d_constant < 0)
		{
			d_constant = fabs(d_constant_down);
		}
		if(((tmp1 - g_cur_pos_in_old_time_) - d_delta_time - d_constant) > d_constant_up )
		{
			d_constant = d_constant + d_constant_up;
			tmp1 = g_cur_pos_in_old_time_ + d_delta_time + d_constant;
			d_delta_time = tmp1 - g_cur_pos_in_old_time_;	 
		}
		else if(((tmp1 - g_cur_pos_in_old_time_) - d_delta_time - d_constant) < d_constant_down)
		{
			d_constant = d_constant + d_constant_down;
			tmp1 = g_cur_pos_in_old_time_ + d_delta_time + d_constant;
			d_delta_time = tmp1 - g_cur_pos_in_old_time_;
		}
		else
		{
			d_delta_time = tmp1 - g_cur_pos_in_old_time_;
		}
		// LogProducer::info("traj_planner","d_delta_time: %.20f", d_delta_time);
		v_min_time.push_back(tmp1);

		// LogProducer::info("traj_planner","find the min time one, final next time is: %.20f", tmp1);

		//4. if the max time is great than or equal to 1, we have done.
		if (tmp1 - g_cur_pos_in_old_time_ < 0)
		{
			LogProducer::info("traj_planner","Find neg time");
		}
		tmp2 = fabs(tmp1 - g_cur_pos_in_old_time_);
		// LogProducer::info("traj_planner","diff in time: %g", tmp2);
		if(tmp2 > (1.0 - DOUBLE_MIN))
		{
			LogProducer::info("traj_planner","forward planning Resume Done!");
			b_end_flag_ = true;
		}

		//5. update the time and the position and speed
		g_cur_pos_in_old_time_ = tmp1;
		// LogProducer::info("traj_planner","Update cur time: %.20f", g_cur_pos_in_old_time_);

		if(floor(g_cur_pos_in_old_time_) < ( n_input_point_counts_ - 2))
		{
			for(uint32_t i = 0; i < joint_num_; i++)
			{
				double tmp_new_pos, tmp_delta_pos/*, tmp_delta_acc*/;
				//get the new pos
				getPosOld(g_cur_pos_in_old_time_, i, &(tmp_new_pos));
				tmp_delta_pos = tmp_new_pos - g_cur_pause_pos_[i];
				g_cur_pause_pos_[i] = tmp_new_pos;
				temp_JointState.angle[i] = g_cur_pause_pos_[i];
				//get the acc.
				acc_martch[i] = (tmp_delta_pos * 1000 - g_cur_pause_speed_[i]) / 0.001;
				temp_JointState.alpha[i] = acc_martch[i];
				//get the speed.
				g_cur_pause_speed_[i] = tmp_delta_pos * 1000;
				temp_JointState.omega[i] = g_cur_pause_speed_[i];

				// LogProducer::info("traj_planner","forward compute, axis: %d, new_pos = %.20f, new speed = %.20f", i + 1, g_cur_pause_pos_[i], g_cur_pause_speed_[i]);
			}
			// LogProducer::info("traj_planner","forward compute, p = %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, v = %.9f, %.9f, %.9f, %.9f, %.9f, %.9f", 
			// g_cur_pause_pos_[0], g_cur_pause_pos_[1], g_cur_pause_pos_[2], g_cur_pause_pos_[3], g_cur_pause_pos_[4], g_cur_pause_pos_[5], 
			// g_cur_pause_speed_[0], g_cur_pause_speed_[1], g_cur_pause_speed_[2], g_cur_pause_speed_[3], g_cur_pause_speed_[4], g_cur_pause_speed_[5]);

			if(b_end_flag_ == true)
			{
				LogProducer::info("traj_planner","forward compute discard point, p = %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, v = %.9f, %.9f, %.9f, %.9f, %.9f, %.9f", 
				g_cur_pause_pos_[0], g_cur_pause_pos_[1], g_cur_pause_pos_[2], g_cur_pause_pos_[3], g_cur_pause_pos_[4], g_cur_pause_pos_[5], 
				g_cur_pause_speed_[0], g_cur_pause_speed_[1], g_cur_pause_speed_[2], g_cur_pause_speed_[3], g_cur_pause_speed_[4], g_cur_pause_speed_[5]);

				LogProducer::info("traj_planner","The all speed has been achieved");
				resume_at = ceil(g_cur_pos_in_old_time_);
				LogProducer::info("traj_planner","resume_at = %d", resume_at);
				for (uint32_t i = 0; i < joint_num_; i++)
				{
					getPosOld(ceil(g_cur_pos_in_old_time_), i, &(d_pos[i]));
					g_cur_pause_pos_[i] = d_pos[i];
					temp_JointState.angle[i] = g_cur_pause_pos_[i];
					getSpeedOld(ceil(g_cur_pos_in_old_time_), i, &(d_speed[i]));
					g_cur_pause_speed_[i] = d_speed[i];
					temp_JointState.omega[i] = g_cur_pause_speed_[i];
					temp_JointState.alpha[i] = v_traj_[resume_at].alpha[i];
					
				}
				// LogProducer::info("traj_planner","forward compute, the last point, p = %.9f, %.9f, %.9f, %.9f, %.9f, %.9f, v = %.9f, %.9f, %.9f, %.9f, %.9f, %.9f", 
				// 	g_cur_pause_pos_[0], g_cur_pause_pos_[1], g_cur_pause_pos_[2], g_cur_pause_pos_[3], g_cur_pause_pos_[4], g_cur_pause_pos_[5], 
				// 	g_cur_pause_speed_[0], g_cur_pause_speed_[1], g_cur_pause_speed_[2], g_cur_pause_speed_[3], g_cur_pause_speed_[4], g_cur_pause_speed_[5]);
				v_resume_traj_temp_.push_back(temp_JointState);
				LogProducer::info("traj_planner","Point by point forward planning Done!");
			}
			else
			{
				// LogProducer::info("traj_planner","push point!!!");
				v_resume_traj_temp_.push_back(temp_JointState);
			}				
		}
		else
		{
			LogProducer::error("traj_planner","There are not enough input points for planning the resume trajectory");
			LogProducer::info("traj_planner","trajectory point total count: %d", v_resume_traj_temp_.size());
			b_end_flag_ = false;
			return MC_RESUME_FAILED;
		}
	}
	while(!b_end_flag_);
	b_end_flag_ = false;

	std::ofstream out_time("/root/forward_min_time.csv");
	for (auto it = v_min_time.begin(); it != v_min_time.end(); ++it)
	{
		// out_time << *it << "," << *(++it) << "," << *(++it) << "," << *(++it) << "," << *(++it) << "," << *(++it) << "," << *(++it) << "," << std::endl;
		out_time << *it << std::endl;
	}
	out_time.close();

	std::ofstream out_forward("/root/forward_compute.csv");
	for (auto it : v_resume_traj_temp_) 
	{
        out_forward << 
		it.angle[0] << "," << it.angle[1] << "," << it.angle[2] << "," << it.angle[3] << "," << it.angle[4] << "," << it.angle[5] << "," << 
		it.omega[0] << "," << it.omega[1] << "," << it.omega[2] << "," << it.omega[3] << "," << it.omega[4] << "," << it.omega[5] << "," << 
		it.alpha[0] << "," << it.alpha[1] << "," << it.alpha[2] << "," << it.alpha[3] << "," << it.alpha[4] << "," << it.alpha[5] << "," << std::endl;
	}
	out_forward.close();

	LogProducer::info("traj_planner","forward trajectory point total count: %d", v_resume_traj_temp_.size());

	if(v_resume_traj_temp_.size() == 1)
	{
		resume_trajectory.push_back(temp_JointState);
		for(uint32_t i = 0; i < joint_num_; i++)
		{
			temp_JointState.alpha[i] = 0.000000;
			temp_JointState.omega[i] = 0.000000;
			temp_JointState.angle[i] = v_traj_.front().angle[i];
		}
		resume_trajectory.push_back(temp_JointState);

		std::ofstream out_backward("/root/backward_compute.csv");
		for (auto it : resume_trajectory) 
		{
			out_backward << 
			it.angle[0] << "," << it.angle[1] << "," << it.angle[2] << "," << it.angle[3] << "," << it.angle[4] << "," << it.angle[5] << "," << 
			it.omega[0] << "," << it.omega[1] << "," << it.omega[2] << "," << it.omega[3] << "," << it.omega[4] << "," << it.omega[5] << "," << 
			it.alpha[0] << "," << it.alpha[1] << "," << it.alpha[2] << "," << it.alpha[3] << "," << it.alpha[4] << "," << it.alpha[5] << "," << std::endl;
		}
		out_backward.close();

		reverse(resume_trajectory.begin(), resume_trajectory.end());

		LogProducer::info("traj_planner","backward trajectory point total count: %d", resume_trajectory.size());

		std::ofstream out_backward_reverse("/root/backward_reverse_compute.csv");
		for (auto it : resume_trajectory) 
		{
			out_backward_reverse << 
			it.angle[0] << "," << it.angle[1] << "," << it.angle[2] << "," << it.angle[3] << "," << it.angle[4] << "," << it.angle[5] << "," << 
			it.omega[0] << "," << it.omega[1] << "," << it.omega[2] << "," << it.omega[3] << "," << it.omega[4] << "," << it.omega[5] << "," << 
			it.alpha[0] << "," << it.alpha[1] << "," << it.alpha[2] << "," << it.alpha[3] << "," << it.alpha[4] << "," << it.alpha[5] << "," << std::endl;
		}
		out_backward_reverse.close();

		LogProducer::info("traj_planner","Finish plan resume Trajectory");

		return SUCCESS;
	}

	resume_trajectory.push_back(temp_JointState);
	
	double tmp;
	double d_temp_time = 0.000000;
	double d_time[NUM_OF_JOINT];
	double d_acc[NUM_OF_JOINT];
	std::vector<JointState>::reverse_iterator rit_cur = v_resume_traj_temp_.rbegin();
	std::vector<JointState>::reverse_iterator rit_next = ++(v_resume_traj_temp_.rbegin());
	
	tmp = 0.001000;
	for(uint32_t i = 0; i < joint_num_; i++)
	{
		if(fabs((*rit_cur).omega[i] + (*rit_next).omega[i]) < DOUBLE_MIN)
		{
			// LogProducer::info("traj_planner","Vcur + Vnext = 0");
			d_time[i] = 0.000000;
		}
		else
		{
			//P0  P1......Pn......Pnext  Pcur......
			//t = (Pcur - Pnext) / ((Vcur + Vnext) / 2)
			d_time[i] = ((*rit_cur).angle[i] - (*rit_next).angle[i]) / (((*rit_cur).omega[i] + (*rit_next).omega[i]) / 2);
			tmp = DOUBLE_MIN;
			if(d_time[i] > tmp)
				tmp = d_time[i];
		}
	}
	LogProducer::info("traj_planner","Final max tmp = %.20f s", tmp);

	LogProducer::info("traj_planner","Final d_temp_time = %.20f", d_temp_time);
	d_temp_time = ((tmp + d_temp_time) * 1000 -1) / 1000;
	LogProducer::info("traj_planner","Final d_temp_time = %.20f", d_temp_time);

	for(uint32_t i = 0; i < joint_num_; i++)
	{
		d_acc[i] = (*rit_cur).alpha[i];
		temp_JointState.alpha[i] = d_acc[i];

		d_speed[i] = (*rit_next).omega[i] + ((*rit_cur).omega[i] - (*rit_next).omega[i]) * d_temp_time / tmp;
		temp_JointState.omega[i] = d_speed[i];

		d_pos[i] = (*rit_next).angle[i] + ((*rit_cur).angle[i] - (*rit_next).angle[i]) * d_temp_time / tmp;
		temp_JointState.angle[i] = d_pos[i];

		// LogProducer::info("traj_planner","@Z@ backward compute, Final axis: %d, new_pos = %.20f, new speed = %.20f", i + 1, d_pos[i], d_speed[i]);
	}
	++rit_cur;
	++rit_next;
	resume_trajectory.push_back(temp_JointState);

	for (; rit_next != v_resume_traj_temp_.rend(); )
	{	
		for(uint32_t i = 0; i < joint_num_; i++)
		{
			d_acc[i] = (*rit_cur).alpha[i];
			temp_JointState.alpha[i] = d_acc[i];

			d_speed[i] = (*rit_next).omega[i] + ((*rit_cur).omega[i] - (*rit_next).omega[i]) * d_temp_time * 1000;
			temp_JointState.omega[i] = d_speed[i];

			d_pos[i] = (*rit_next).angle[i] + ((*rit_cur).angle[i] - (*rit_next).angle[i]) * d_temp_time * 1000;
			temp_JointState.angle[i] = d_pos[i];
			// LogProducer::info("traj_planner","@Z@ backward compute, Final axis: %d, new_pos = %.20f, new speed = %.20f", i + 1, d_pos[i], d_speed[i]);
		}

		++rit_cur;
		++rit_next;
		resume_trajectory.push_back(temp_JointState);
	}

	for(uint32_t i = 0; i < joint_num_; i++)
	{
		temp_JointState.alpha[i] = 0.000000;
		temp_JointState.omega[i] = 0.000000;
		temp_JointState.angle[i] = v_traj_.front().angle[i];
	}
	resume_trajectory.push_back(temp_JointState);

	std::ofstream out_backward("/root/backward_compute.csv");
	for (auto it : resume_trajectory) 
	{
        out_backward << 
		it.angle[0] << "," << it.angle[1] << "," << it.angle[2] << "," << it.angle[3] << "," << it.angle[4] << "," << it.angle[5] << "," << 
		it.omega[0] << "," << it.omega[1] << "," << it.omega[2] << "," << it.omega[3] << "," << it.omega[4] << "," << it.omega[5] << "," << 
		it.alpha[0] << "," << it.alpha[1] << "," << it.alpha[2] << "," << it.alpha[3] << "," << it.alpha[4] << "," << it.alpha[5] << "," << std::endl;
	}
	out_backward.close();

	reverse(resume_trajectory.begin(), resume_trajectory.end());
	
	LogProducer::info("traj_planner","backward trajectory point total count: %d", resume_trajectory.size());

	std::ofstream out_backward_reverse("/root/backward_reverse_compute.csv");
	for (auto it : resume_trajectory) 
	{
        out_backward_reverse << 
		it.angle[0] << "," << it.angle[1] << "," << it.angle[2] << "," << it.angle[3] << "," << it.angle[4] << "," << it.angle[5] << "," << 
		it.omega[0] << "," << it.omega[1] << "," << it.omega[2] << "," << it.omega[3] << "," << it.omega[4] << "," << it.omega[5] << "," << 
		it.alpha[0] << "," << it.alpha[1] << "," << it.alpha[2] << "," << it.alpha[3] << "," << it.alpha[4] << "," << it.alpha[5] << "," << std::endl;
	}
	out_backward_reverse.close();

	LogProducer::info("traj_planner","Finish plan resume Trajectory");

	return SUCCESS;
}

void PauseResumePlanner::getMartchTime(double pos, int axis, double *new_time)
{
	// LogProducer::info("traj_planner","get Martch Time");
	//find time
	double d_pre_time, d_next_time;
	double d_pre_time_speed, d_next_time_speed;
	d_pre_time = floor(g_cur_pos_in_old_time_);
	d_next_time = ceil(g_cur_pos_in_old_time_);
	getSpeedOld(d_pre_time, axis, &d_pre_time_speed);
	getSpeedOld(d_next_time, axis, &d_next_time_speed);
	if(d_pre_time_speed * d_next_time_speed < 0.0)
	{
		// LogProducer::info("traj_planner","change the direction, axis: %d", axis + 1);
		v_speed_change_flag_[axis] = true;
		*new_time = g_cur_pos_in_old_time_;
		return;
	}
	if(fabs(d_pre_time_speed) < MINIMUM_E6 || fabs(d_next_time_speed) < MINIMUM_E6)
	{
		// LogProducer::info("traj_planner","d_pre_time_speed < MINIMUM_E6 || d_next_time_speed < MINIMUM_E6, axis: %d", axis + 1);
		v_speed_change_flag_[axis] = true;
		*new_time = g_cur_pos_in_old_time_;
		return;
	}
	if(true == v_speed_change_flag_[axis])
	{
		// LogProducer::info("traj_planner","change the direction finish, axis: %d", axis + 1);
		*new_time = g_cur_pos_in_old_time_;
		return;
	}
	getTimeOldPause(pos, axis, new_time);
	if(*new_time < DOUBLE_MIN)
	{
		// LogProducer::info("traj_planner","*new_time <= 0");
		*new_time = g_cur_pos_in_old_time_;
		// LogProducer::info("traj_planner","*new_time = %f", *new_time);
		return;
	}
	else if(*new_time > 1.0)
	{
		// LogProducer::info("traj_planner","*new_time > 1.0");
		*new_time = g_cur_pos_in_old_time_ + 1.0;
		// LogProducer::info("traj_planner","*new_time = %f", *new_time);
		return;
	}
	else
	{
		*new_time = g_cur_pos_in_old_time_ + *new_time;
		// LogProducer::info("traj_planner","*new_time = %f", *new_time);
	}
}

void PauseResumePlanner::getMartchTimeResume(double pos, int axis, double *new_time)
{
	// LogProducer::info("traj_planner","get Martch Time Resume");
	//find time

	double d_pre_time, d_next_time;
	double d_pre_time_speed, d_next_time_speed;
	d_pre_time = floor(g_cur_pos_in_old_time_);
	d_next_time = ceil(g_cur_pos_in_old_time_);
	getSpeedOld(d_pre_time, axis, &d_pre_time_speed);
	getSpeedOld(d_next_time, axis, &d_next_time_speed);
	if(d_pre_time_speed * d_next_time_speed < 0.0)
	{
		// LogProducer::info("traj_planner","change the direction, axis: %d", axis + 1);
		v_speed_change_flag_[axis] = true;
		*new_time = g_cur_pos_in_old_time_ + 1.0;
		return;
	}
	if(fabs(d_pre_time_speed) < MINIMUM_E6 || fabs(d_next_time_speed) < MINIMUM_E6)
	{
		// LogProducer::info("traj_planner","d_pre_time_speed < MINIMUM_E6 || d_next_time_speed < MINIMUM_E6, axis: %d", axis + 1);
		v_speed_change_flag_[axis] = true;
		*new_time = g_cur_pos_in_old_time_ + 1.0;
		return;
	}
	if(true == v_speed_change_flag_[axis])
	{
		// LogProducer::info("traj_planner","change the direction finish, axis: %d", axis + 1);
		*new_time = g_cur_pos_in_old_time_ + 1.0;
		return;
	}

	getTimeOldResume(pos, axis, new_time);
	if(fabs(*new_time) < DOUBLE_MIN)
	{
		// LogProducer::info("traj_planner","*new_time <= 0, axis: %d", axis + 1);
		*new_time = g_cur_pos_in_old_time_ + 1.0;
	}
	else if(*new_time > 1.0)
	{
		// LogProducer::info("traj_planner","*new_time > 1.0, axis: %d", axis + 1);
		*new_time = g_cur_pos_in_old_time_ + 1.0;
	}
	else if(*new_time < 0.0)
	{
		// LogProducer::info("traj_planner","*new_time < 0.0, axis: %d", axis + 1);
		*new_time = g_cur_pos_in_old_time_ + 1.0;
		// LogProducer::info("traj_planner","*new_time = %f", *new_time);
	}
	else
	{
		*new_time = g_cur_pos_in_old_time_ + *new_time;
		// LogProducer::info("traj_planner","*new_time = %f", *new_time);
	}
}

//get speed at current time(planed before)
int PauseResumePlanner::getSpeedOld(double d_time, int axis, double *speed)
{
	if(speed == NULL)
        return -1;

	*speed = 0.0;

	int section_base = static_cast<int>(d_time);
	if(section_base < 0)
        return -1;

	double section_ratio = d_time - (double)section_base;
	// LogProducer::info("traj_planner","getSpeedOld: time = %.20f, section = %d, section_ratio = %.20f", d_time, section_base, section_ratio);
	*speed = ((v_traj_[section_base + 1].omega[axis] - v_traj_[section_base].omega[axis]) * section_ratio) 
		+ v_traj_[section_base].omega[axis];
	// LogProducer::info("traj_planner","getSpeedOld: *speed = %.20f", *speed);

	return 0;
}

//get pos at current time(planed before)
int PauseResumePlanner::getPosOld(double d_time, int axis, double *pos)
{
	if(pos == NULL)
        return -1;
	*pos = 0.0;

	int section_base = static_cast<int>(d_time);
	if(section_base < 0)
        return -1;

	double section_ratio = d_time - (double)section_base;
	// LogProducer::info("traj_planner","getPosOld: time = %.20f, section = %d, section_ratio = %.20f", d_time, section_base, section_ratio);
	*pos = ((v_traj_[section_base + 1].angle[axis] - v_traj_[section_base].angle[axis]) * section_ratio) 
		+ v_traj_[section_base].angle[axis];
	// LogProducer::info("traj_planner","getPosOld: *pos = %.20f", *pos);

	return 0;
}

void PauseResumePlanner::getTimeOldPause(double pos, int axis, double *p_time)
{
	// LogProducer::info("traj_planner","get Time Old Pause");
	double y1, y0;
	y0 = v_traj_[(int)(g_cur_pos_in_old_time_) + 0].angle[axis];
	y1 = v_traj_[(int)(g_cur_pos_in_old_time_) + 1].angle[axis];
	// LogProducer::info("traj_planner","y0: %.20f,  y1: %.20f", y0, y1);

	if(fabs(y1 - y0) < DOUBLE_MIN)
    {
		// LogProducer::info("traj_planner","y1 = y0");
		*p_time = 0;
		return;
	}

	if(y1 > y0)
	{
		if(y0 < pos)
		{
			// LogProducer::info("traj_planner","y1 > y0, y0 < aim pos");
			double d_cur_pos;
			getPosOld(g_cur_pos_in_old_time_, axis, &(d_cur_pos));
			*p_time = (pos - d_cur_pos) / (y1 - y0);
			// LogProducer::info("traj_planner","*p_time = %f, axis = %d", *p_time, axis + 1);
			return;
		}
		else
		{
			// LogProducer::info("traj_planner","y1 > y0, aim pos <= y0");
			*p_time = 0;
			return;
		}
	}

	if(y1 < y0)
	{
		if(pos < y0)
		{
			// LogProducer::info("traj_planner","y1 < y0, aim pos < y0");
			double d_cur_pos;
			getPosOld(g_cur_pos_in_old_time_, axis, &(d_cur_pos));
			*p_time = (d_cur_pos - pos) / (y0 - y1);
			// LogProducer::info("traj_planner","*p_time = %f, axis = %d", *p_time, axis + 1);
			return;
		}
		else
		{
			// LogProducer::info("traj_planner","y1 < y0, aim pos >= y0");
			*p_time = 0;
			return;
		}
	}
}

void PauseResumePlanner::getTimeOldResume(double pos, int axis, double *p_time)
{
	// LogProducer::info("traj_planner","get Time Old Resume");
	double y1, y0;
	y0 = v_traj_[(int)(g_cur_pos_in_old_time_) + 0].angle[axis];
	y1 = v_traj_[(int)(g_cur_pos_in_old_time_) + 1].angle[axis];
	// LogProducer::info("traj_planner","y0: %.20f,  y1: %.20f", y0, y1);

	if(fabs(y1 - y0) < DOUBLE_MIN)
	{
		// LogProducer::info("traj_planner","y1 = y0");
		*p_time = 1;
		return;
	}

	if(y1 > y0)
	{
		if(y0 < pos)
		{
			// LogProducer::info("traj_planner","y1 > y0, y0 < aim pos");
			double d_cur_pos;
			getPosOld(g_cur_pos_in_old_time_, axis, &(d_cur_pos));
			*p_time = (pos - d_cur_pos) / (y1 - y0);
			// LogProducer::info("traj_planner","*p_time = %f, axis = %d", *p_time, axis + 1);
			return;
		}
		else
		{
			// LogProducer::info("traj_planner","y1 > y0, aim pos <= y0");
			*p_time = 1;
			return;
		}
	}

	if(y1 < y0)
	{
		if(pos < y0)
		{
			// LogProducer::info("traj_planner","y1 < y0, aim pos < y0");
			double d_cur_pos;
			getPosOld(g_cur_pos_in_old_time_, axis, &(d_cur_pos));
			*p_time = (d_cur_pos - pos) / (y0 - y1);
			// LogProducer::info("traj_planner","*p_time = %f, axis = %d", *p_time, axis + 1);
			return;
		}
		else
		{
			// LogProducer::info("traj_planner","y1 < y0, aim pos >= y0");
			*p_time = 1;
			return;
		}
	}
}