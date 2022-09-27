#ifndef _VPATH_PLANNER_H_
#define _VPATH_PLANNER_H_
#include <vector>
#include "basic_alg.h"
#include "quaternion.h"
#include "matrix33.h"
#include "vector3.h"
#include "algorithm"
#include "pose_euler.h"
#include "traj_params.h"

class VpathPlanner
{
public:
    VpathPlanner();
    ~VpathPlanner();
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
    bool viaPoints2Traj(double traj_vel);
    bool viaPoints2Traj(const std::vector<basic_alg::PoseEuler> &via_points, double traj_vel);
    bool setViaPoints(const std::vector<basic_alg::PoseEuler> &via_points, bool is_new);
    void reset(void);
    
    bool trajPausePlan(uint32_t index, double vel, double vel_ratio, double acc_ratio, double jerk_ratio);
    bool trajResumePlan(double vel, double vel_ratio, double acc_ratio, double jerk_ratio);

    std::vector<basic_alg::PoseEuler> getResampledTraj(void) { return resampled_traj_; }
    std::vector<basic_alg::PoseEuler> getPauseTraj(void) { return pause_traj_; }

    void setSamplingFreq(double fs) { sampling_freq_ = fs; }
    // void setTrajTime(double t) { traj_time_set_ = t; }
    void setAccTimeRatio(double rate) { acc_time_ratio_ = rate; }
    void setDecTimeRatio(double rate) { dec_time_ratio_ = rate; }
    void xyzSetFitRate(double rate) { xyz_fit_rate_ = rate; }
    void abcSetSmoothWindow(int window) { abc_smooth_window_ = window; }

	void setLimit(TrajParams *param) { traj_param_ = param; }

    std::vector<basic_alg::Quaternion> testQuatSmooth(const std::vector<basic_alg::Quaternion>& quat_in);
    std::vector<basic_alg::Point> testPoseSmooth(const std::vector<basic_alg::Point>& pose_in);

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

    bool calcResampledPointDist(void);

private:
    /*
      Interpolation between pos_start and pos_end with 5th order polynome and then 
      find the give via points' time in the whole trajectory.
      fs: sampling frequency
      pos_give: via point's distance between first via point
    */
    bool poly5(
        double pos_start, double pos_end,
        double vel_start, double vel_end,
        double acc_start, double acc_end,
        double time_start, double time_end,
        double fs, double pos_give, 
        double results[4]);

    /*spline function sepreated with xyz and abc for the efficiency of calculation */
    bool spline(
        const std::vector<basic_alg::Point> &via_points_pose,
        const std::vector<basic_alg::Quaternion> &via_points_quat,
        const basic_alg::Point &init_pose_vel,
        const basic_alg::Point &final_pose_vel,
        const basic_alg::Quaternion &init_quat_vel,
        const basic_alg::Quaternion &final_quat_vel,
        const std::vector<double> &via_points_time,
        std::vector<basic_alg::PoseEuler>& out_traj);

    bool spline(
        const std::vector<double>& data, 
        const double& init, 
        const double& final, 
        const std::vector<double> times, 
        std::vector<double>& out);

    void spline_value(
        const std::vector<double> &via_points_time, 
        const std::vector<double> &resampled_time, 
        std::vector<basic_alg::PoseEuler>& out_traj);

private:
    double sampling_freq_;
    double acc_time_ratio_;
    double dec_time_ratio_;

    int32_t abc_smooth_window_;
    double xyz_fit_rate_;

private:
    int32_t via_points_cnt_;
    // std::vector<basic_alg::PoseEuler> input_via_points_;
    std::vector<basic_alg::Point> via_points_pose_; // mm
    std::vector<basic_alg::Quaternion> via_points_quat_;
    std::vector<double> via_points_dist_;
    std::vector<double> traj_points_dist_;

    double traj_vel_set_; // mm/s
    double traj_time_set_; // s
    size_t traj_size_;

    double acc_, vel_, dec_;

    double acc_end_time_, acc_end_dist_;   // acceleration time and distance
    double even_end_time_, even_end_dist_; // speed hold on time and distance
    double dec_end_time_, dec_end_dist_;   // decceleration time and distance

    int paused_index_;

    std::vector<double> fa0_px_, fa0_py_, fa0_pz_;
    std::vector<double> fa1_px_, fa1_py_, fa1_pz_;
    std::vector<double> fa2_px_, fa2_py_, fa2_pz_;
    std::vector<double> fa3_px_, fa3_py_, fa3_pz_;

    std::vector<double> fa0_qw_, fa0_qx_, fa0_qy_, fa0_qz_;
    std::vector<double> fa1_qw_, fa1_qx_, fa1_qy_, fa1_qz_;
    std::vector<double> fa2_qw_, fa2_qx_, fa2_qy_, fa2_qz_;
    std::vector<double> fa3_qw_, fa3_qx_, fa3_qy_, fa3_qz_;

    std::vector<double> via_points_time_new_;
    std::vector<basic_alg::PoseEuler> resampled_traj_;
    std::vector<basic_alg::PoseEuler> pause_traj_;
    std::vector<basic_alg::PoseEuler> resume_traj_;

private:
    TrajParams *traj_param_;
};

#endif
