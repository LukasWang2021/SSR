/*************************************************************************
	> File Name: pause_resume_planner.h
	> Author: 
	> Mail: 
	> Created Time: 2020年03月26日 星期一 10时26分01秒
 ************************************************************************/

#ifndef _PAUSE_RESUME_PLANNER_H
#define _PAUSE_RESUME_PLANNER_H

#include <string.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <common_error_code.h>
#include <motion_control_datatype.h>
#include <algorithm>


namespace fst_mc
{

class PauseResumePlanner
{
public:
    PauseResumePlanner(void);
    ~PauseResumePlanner(void);

    bool initPausePlanner(uint32_t joint_num);
    ErrorCode planPauseTrajectory(double d_ratio, std::vector<JointState> &trajectory, std::vector<JointState> &pause_trajectory, uint32_t &pause_at);
    ErrorCode planResumeTrajectory(std::vector<JointState> &trajectory, std::vector<JointState> &resume_trajectory, uint32_t &resume_at, double d_ratio);

private:
    void getMartchTime(double pos, int axis, double *new_time);
    void getMartchTimeResume(double pos, int axis, double *new_time);

    int getSpeedOld(double d_time, int axis, double *speed);
    int getPosOld(double d_time, int axis, double *pos);
    
    void getTimeOldPause(double pos, int axis, double *p_time);
    void getTimeOldResume(double pos, int axis, double *p_time);

    double d_ratio_;
    uint32_t joint_num_;
    bool b_end_flag_;
    bool v_speed_change_flag_[NUM_OF_JOINT];
    double g_cur_pos_in_old_time_;
    double g_cur_pause_speed_[NUM_OF_JOINT];
    double g_cur_pause_pos_[NUM_OF_JOINT];
    double acc_ratio_[NUM_OF_JOINT];
    int n_input_point_counts_;
    std::vector<JointState> v_traj_;
    std::vector<JointState> v_resume_traj_temp_;

};

}

#endif // _PAUSE_RESUME_PLANNER_H