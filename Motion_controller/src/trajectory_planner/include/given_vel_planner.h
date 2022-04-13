#ifndef _GIVEN_VEL_PLANNER_H_
#define _GIVEN_VEL_PLANNER_H_
#include <vector>
#include "basic_alg.h"
#include "quaternion.h"
#include "matrix33.h"
#include "vector3.h"
#include "algorithm"
#include "pose_euler.h"

class GivenVelocityPlanner
{
public:
    GivenVelocityPlanner();
    ~GivenVelocityPlanner();
    /*
    Generate trajectory through given via points. The trajectory velocity is constant.
    1. get the via points' quaternion
    2. if via points more than 10 then
    3. smooth the xyz with fit and abc with smooth quaternion
    4. calculate the path info: 
       distance between every via point, acceleration time and position 
       even speed time and position deceleration time and position
    5. use the path info and poly5 replan the via points' time
    6. xyz trajectory resampling with spline
    7. abc transfer to quaternion and resampling with spline
    */
    int32_t viaPoints2Traj(std::vector<basic_alg::PoseEuler> via_points);

    std::vector<basic_alg::PoseEuler> getResampledTraj(void) { return resampled_traj_; }
    void setSamplingFreq(double fs) { sampling_freq_ = fs; }
    void setTrajTime(double t) { traj_time_set_ = t; }
    void setAccTimeRatio(double rate) { acc_time_ratio_ = rate; }
    void setDecTimeRatio(double rate) { dec_time_ratio_ = rate; }
    void xyzSetFitRate(double rate) { xyz_fit_rate_ = rate; }
    void abcSetSmoothWindow(int window) { abc_smooth_window_ = window; }

    std::vector<basic_alg::Quaternion> testQuatSmooth(const std::vector<basic_alg::Quaternion>& quat_in);
    std::vector<basic_alg::PoseEuler> testFitSmooth(const std::vector<basic_alg::PoseEuler>& pos_in);

private:
    /*smooth abc with fit*/
    bool xyzFitSmooth(double fit_rate);// not use now
    /*smooth abc with quaternion average*/
    bool abcQuatSmooth(int32_t smooth_window);

    bool pathInfoCalc(void);
    /*calculate the velocity in acceleration even deceleration*/
    bool trajPlanWithPathInfo(void);

    /*initialize the resampling trajectory vector*/
    //bool trajTimeResampling(void);

    bool xyzTrajResampling(void);
    bool abcTrajResampling(void);

private:
    /*
      Interpolation between pos_start and pos_end with 5th order polynome and then 
      find the give via point's time in whole trajectory.
      fs: sampling frequency
      pos_give: via point's distance between first via point
    */
    bool poly5(
        double pos_start, double pos_end,
        double vel_start, double vel_end,
        double acc_start, double acc_end,
        double time_start, double time_end,
        double fs, double pos_give);
    /*spline function sepreated with xyz and abc for the efficiency of calculation */
    bool xyzSpline(void);
    bool abcSpline(void);

private:
    double sampling_freq_;
    double traj_time_set_;
    double acc_time_ratio_;
    double dec_time_ratio_;

    int32_t abc_smooth_window_;
    double xyz_fit_rate_;

private:
    int32_t via_points_cnt_;
    std::vector<basic_alg::PoseEuler> input_via_points_;
    std::vector<basic_alg::Quaternion> via_points_quat_;
    std::vector<double> via_points_dist_;

    double vel_even_;
    double acc_;
    double dec_;

    double acc_end_time_;
    double even_end_time_;
    double dec_end_time_;

    double acc_end_dist_;
    double even_end_dist_;
    double dec_end_dist_;

    std::vector<double> via_points_time_new_;
    std::vector<basic_alg::PoseEuler> resampled_traj_;
};

#endif
