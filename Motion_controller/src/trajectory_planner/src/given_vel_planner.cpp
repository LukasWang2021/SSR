#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>
#include <sstream>
#include "given_vel_planner.h"
#include "log_manager_producer.h"

using namespace std;
using namespace basic_alg;
using namespace log_space;

GivenVelocityPlanner::GivenVelocityPlanner()
{
    sampling_freq_ = 0.001;
    traj_time_set_ = 10;
    acc_time_ratio_ = 0.33333333;
    dec_time_ratio_ = 0.33333333;
    via_points_cnt_ = 0;
    xyz_fit_rate_ = 0.99999;
    abc_smooth_window_ = 7;
}

GivenVelocityPlanner::~GivenVelocityPlanner()
{

}

bool GivenVelocityPlanner::setViaPoints(const vector<PoseEuler> &via_points, bool is_new)
{
    if(is_new) // plan a new trajectory
    {
        via_points_pose_.clear();
        via_points_quat_.clear();
        via_points_dist_.clear();
        via_points_time_new_.clear();
        resampled_traj_.clear();
        via_points_cnt_ = 0;
    }

    Point tmp_pose;
    Quaternion tmp_quat;
    for (auto iter = via_points.begin(); iter != via_points.end(); ++iter)
    {
        tmp_pose = iter->point_;
        tmp_quat = Euler2Quaternion(iter->euler_);
        normalizeQuaternion(tmp_quat);
        via_points_quat_.push_back(tmp_quat);
        via_points_pose_.push_back(tmp_pose);
        ++via_points_cnt_;
    }

    return true;
}

bool GivenVelocityPlanner::viaPoints2Traj(double traj_vel)
{
    if(traj_vel < MINIMUM_E6)
    {
        LogProducer::error("offline_planner","trajectory speed %lf invalid", traj_vel);
        return false;
    }
    traj_vel_set_ = traj_vel;
    if (via_points_cnt_ >= 10 && (!xyzFitSmooth(xyz_fit_rate_) || !abcQuatSmooth(abc_smooth_window_)))
    {
        LogProducer::error("offline_planner","via points smooth failed");
        return false;
    }
    // 4. calculate the path info
    pathInfoCalc();
    trajPlanWithPathInfo();
    // trajTimeResampling();
    // 5. resampling xyz trajectory with 0.001s
    xyzTrajResampling();
    // 6. resampling abc trajectory with 0.001s
    abcTrajResampling();

     /* the trajectory speed on begin and end are 0 */
    Point init_pose_vel;
    init_pose_vel.zero();
    Point final_pose_vel;
    final_pose_vel.zero();
    Quaternion init_quat_vel;
    init_quat_vel.zero();
    Quaternion final_quat_vel;
    final_quat_vel.zero();

    if (spline(via_points_pose_, via_points_quat_,
        init_pose_vel, final_pose_vel,
        init_quat_vel, final_quat_vel,
        via_points_time_new_, resampled_traj_) == false)
    {
        return false;
    }

    calcResampledPointDist();

    return true;
}

bool GivenVelocityPlanner::viaPoints2Traj(vector<PoseEuler> via_points)
{
    via_points_pose_.clear();
    via_points_quat_.clear();
    via_points_dist_.clear();
    via_points_time_new_.clear();
    resampled_traj_.clear();
    // 1. transfer abc to quaternion and normalize it
    Point tmp_pose;
    Quaternion tmp_quat;
    for (auto iter = via_points.begin(); iter != via_points.end(); ++iter)
    {
        tmp_pose = iter->point_;
        tmp_quat = Euler2Quaternion(iter->euler_);
        normalizeQuaternion(tmp_quat);
        via_points_quat_.push_back(tmp_quat);
        via_points_pose_.push_back(tmp_pose);
        ++via_points_cnt_;
    }
    // if the number of via point more than 10 do 2 and 3 else not
    // 2. smooth xyz
    // 3. smooth abc
    if (via_points_cnt_ >= 10 && (!xyzFitSmooth(xyz_fit_rate_) || !abcQuatSmooth(abc_smooth_window_)))
    {
        return false;
    }
    // 4. calculate the path info
    pathInfoCalc();
    trajPlanWithPathInfo();
    // trajTimeResampling();
    // 5. resampling xyz trajectory with 0.001s
    xyzTrajResampling();
    // 6. resampling abc trajectory with 0.001s
    abcTrajResampling();
     /* the trajectory speed on begin and end are 0 */
    Point init_pose_vel;
    init_pose_vel.zero();
    Point final_pose_vel;
    final_pose_vel.zero();
    Quaternion init_quat_vel;
    init_quat_vel.zero();
    Quaternion final_quat_vel;
    final_quat_vel.zero();

    if (spline(via_points_pose_, via_points_quat_,
        init_pose_vel, final_pose_vel,
        init_quat_vel, final_quat_vel,
        via_points_time_new_, resampled_traj_) == false)
    {
        return -1;
    }

    calcResampledPointDist();

    return true;
}

bool GivenVelocityPlanner::xyzFitSmooth(double fit_rate)
{
    return true;
}

bool GivenVelocityPlanner::abcQuatSmooth(int32_t smooth_window)
{
    if (smooth_window % 2 == 0)
    {
        return false;
    }

    vector<Quaternion> smooth_quat;
    vector<Quaternion> mean_quat;

    for (size_t idx = 0; idx < via_points_quat_.size(); ++idx)
    {
        if (idx <= (size_t)((smooth_window - 1) / 2 - 1) || idx >= (via_points_quat_.size() - (smooth_window - 1) / 2))
        {
            smooth_quat.push_back(via_points_quat_[idx]);
            continue;
        }
        mean_quat.clear();
        for (int32_t i = 0; i < smooth_window; ++i)
        {
            mean_quat.push_back(via_points_quat_[idx + (i - (smooth_window - 1) / 2)]);
        }
        Matrix44 quat_matrix_sum;
        double eigvec[4];
        double eigval = 0;
        Quaternion quat;
        memset(quat_matrix_sum.matrix_, 0, sizeof(double)*16);
        for (size_t i = 0; i < mean_quat.size(); ++i)
        {
            quat_matrix_sum.matrix_[0][0] += mean_quat[i].x_ * mean_quat[i].x_;
            quat_matrix_sum.matrix_[0][1] += mean_quat[i].x_ * mean_quat[i].y_;
            quat_matrix_sum.matrix_[0][2] += mean_quat[i].x_ * mean_quat[i].z_;
            quat_matrix_sum.matrix_[0][3] += mean_quat[i].x_ * mean_quat[i].w_;

            quat_matrix_sum.matrix_[1][0] += mean_quat[i].y_ * mean_quat[i].x_;
            quat_matrix_sum.matrix_[1][1] += mean_quat[i].y_ * mean_quat[i].y_;
            quat_matrix_sum.matrix_[1][2] += mean_quat[i].y_ * mean_quat[i].z_;
            quat_matrix_sum.matrix_[1][3] += mean_quat[i].y_ * mean_quat[i].w_;

            quat_matrix_sum.matrix_[2][0] += mean_quat[i].z_ * mean_quat[i].x_;
            quat_matrix_sum.matrix_[2][1] += mean_quat[i].z_ * mean_quat[i].y_;
            quat_matrix_sum.matrix_[2][2] += mean_quat[i].z_ * mean_quat[i].z_;
            quat_matrix_sum.matrix_[2][3] += mean_quat[i].z_ * mean_quat[i].w_;

            quat_matrix_sum.matrix_[3][0] += mean_quat[i].w_ * mean_quat[i].x_;
            quat_matrix_sum.matrix_[3][1] += mean_quat[i].w_ * mean_quat[i].y_;
            quat_matrix_sum.matrix_[3][2] += mean_quat[i].w_ * mean_quat[i].z_;
            quat_matrix_sum.matrix_[3][3] += mean_quat[i].w_ * mean_quat[i].w_;
        }
        quat_matrix_sum.matrix_[0][0] /= mean_quat.size();
        quat_matrix_sum.matrix_[0][1] /= mean_quat.size();
        quat_matrix_sum.matrix_[0][2] /= mean_quat.size();
        quat_matrix_sum.matrix_[0][3] /= mean_quat.size();

        quat_matrix_sum.matrix_[1][0] /= mean_quat.size();
        quat_matrix_sum.matrix_[1][1] /= mean_quat.size();
        quat_matrix_sum.matrix_[1][2] /= mean_quat.size();
        quat_matrix_sum.matrix_[1][3] /= mean_quat.size();

        quat_matrix_sum.matrix_[2][0] /= mean_quat.size();
        quat_matrix_sum.matrix_[2][1] /= mean_quat.size();
        quat_matrix_sum.matrix_[2][2] /= mean_quat.size();
        quat_matrix_sum.matrix_[2][3] /= mean_quat.size();

        quat_matrix_sum.matrix_[3][0] /= mean_quat.size();
        quat_matrix_sum.matrix_[3][1] /= mean_quat.size();
        quat_matrix_sum.matrix_[3][2] /= mean_quat.size();
        quat_matrix_sum.matrix_[3][3] /= mean_quat.size();
        if (!eigens(&quat_matrix_sum.matrix_[0][0], 4, eigvec, &eigval))
        {
            return false;
        }
        quat.x_ = eigvec[0]; quat.y_ = eigvec[1];
        quat.z_ = eigvec[2]; quat.w_ = eigvec[3];
        smooth_quat.push_back(quat);
    }
    via_points_quat_ = smooth_quat;
    return true;
}

bool GivenVelocityPlanner::pathInfoCalc(void)
{
    /*calculate distance between via points*/
    double distance = 0;
    via_points_dist_.push_back(distance);
    for (int cnt = 0; cnt < via_points_cnt_ - 1; ++cnt)
    {
        distance += getDistance(via_points_pose_[cnt+1], via_points_pose_[cnt]);
        via_points_dist_.push_back(distance);
    }
    traj_time_set_ = distance / traj_vel_set_ * 1.5;
    acc_end_time_ = acc_time_ratio_ * traj_time_set_;
    even_end_time_ = acc_end_time_ + traj_time_set_ * (1 - (acc_time_ratio_ + dec_time_ratio_));
    dec_end_time_ = traj_time_set_;

    vel_even_ = via_points_dist_.back() / traj_time_set_ * (1 / (1 - (acc_time_ratio_ + dec_time_ratio_) / 2));
    acc_ = vel_even_ / acc_end_time_;

    acc_end_dist_ = vel_even_ * 0.5 * acc_end_time_;
    even_end_dist_ = acc_end_dist_ + vel_even_ * (even_end_time_ - acc_end_time_);
    dec_end_dist_ = via_points_dist_.back();

    return true;
}


bool GivenVelocityPlanner::trajPlanWithPathInfo(void)
{
    double pos_give = 0.0;
    for (int i = 0; i < via_points_cnt_; ++i)
    {
        pos_give = via_points_dist_[i];
        if (pos_give >= 0 && pos_give < acc_end_dist_)
        {
            poly5(0, acc_end_dist_, 0, vel_even_, 0, 0, 0, acc_end_time_, sampling_freq_, pos_give);
        }
        else if (pos_give >= acc_end_dist_ && pos_give < even_end_dist_)
        {
            poly5(acc_end_dist_, even_end_dist_, vel_even_, vel_even_, 0, 0, acc_end_time_, even_end_time_, sampling_freq_, pos_give);
        }
        else if (pos_give >= even_end_dist_ && pos_give <= dec_end_dist_)
        {
            poly5(even_end_dist_, dec_end_dist_, vel_even_, 0, 0, 0, even_end_time_, dec_end_time_, sampling_freq_, pos_give);
        }
    }
    via_points_time_new_.back() = traj_time_set_;
    // for(auto iter = via_points_time_new_.begin(); iter != via_points_time_new_.end(); ++iter)
    // {
    //     LogProducer::error("mc","####vp time new %lf", *iter);
    // }

    return true;
}

bool GivenVelocityPlanner::poly5(
    double pos_start, double pos_end, 
    double vel_start, double vel_end, 
    double acc_start, double acc_end, 
    double time_start, double time_end, 
    double fs, double pos_give
)
{
    double T = time_end - time_start;
    double a0 = pos_start;
    double a1 = vel_start;
    double a2 = acc_start / 2;
    double a3 = -(20 * pos_start - 20 * pos_end + 12 * T * vel_start + 8 * T * vel_end 
        + 3 * pow(T, 2) * acc_start - pow(T, 2) * acc_end) / (2 * pow(T, 3));

    double a4 = (30 * pos_start - 30 * pos_end + 16 * T * vel_start + 14 * T * vel_end 
        + 3 * pow(T, 2) * acc_start - 2 * pow(T, 2) * acc_end) / (2 * pow(T, 4));

    double a5 = -(12 * pos_start - 12 * pos_end + 6 * T * vel_start + 6 * T * vel_end 
        + pow(T, 2) * acc_start - pow(T, 2) * acc_end) / (2 * pow(T, 5));

    double tmp_pos = 0;
    double t_set = 0, pos_set = 0, vel_set = 0, acc_set = 0;
    for (double t = time_start; t <= time_end; t += fs)
    {
        tmp_pos = a0 + a1 * pow((t - time_start), 1) + a2 * pow((t - time_start), 2) +
            a3 * pow((t - time_start), 3) + a4 * pow(t - time_start, 4) + a5 * pow(t - time_start, 5);

        if (tmp_pos <= pos_give) t_set = t;
    }

    via_points_time_new_.push_back(t_set);

    pos_set = a0 + a1 * pow((t_set - time_start), 1) + a2 * pow((t_set - time_start), 2) 
        + a3 * pow((t_set - time_start), 3) + a4 * pow(t_set - time_start, 4) + a5 * pow(t_set - time_start, 5);

    vel_set = a1 + 2 * a2 * pow((t_set - time_start), 1) + 3 * a3 * pow((t_set - time_start), 2) 
        + 4 * a4 * pow(t_set - time_start, 3) + 5 * a5 * pow(t_set - time_start, 4);

    acc_set = 2 * a2 + 6 * a3 * pow((t_set - time_start), 1) + 12 * a4 * pow(t_set - time_start, 2) 
        + 20 * a5 * pow(t_set - time_start, 3);

    return true;
}

bool GivenVelocityPlanner::xyzTrajResampling(void)
{
    // if (!xyzSpline()) return false;

    return true;
}

bool GivenVelocityPlanner::abcTrajResampling(void)
{
    double dq = 0.0;
    vector<Quaternion> resampled_quat;
    vector<Quaternion> tmp_quat_vec = via_points_quat_;
    for (size_t i = 0; i < tmp_quat_vec.size() - 1; ++i)
    {
        dq = tmp_quat_vec[i].w_ * tmp_quat_vec[i + 1].w_
            + tmp_quat_vec[i].x_ * tmp_quat_vec[i + 1].x_
            + tmp_quat_vec[i].y_ * tmp_quat_vec[i + 1].y_
            + tmp_quat_vec[i].z_ * tmp_quat_vec[i + 1].z_;

        if (dq < 0)
        {
            tmp_quat_vec[i + 1].w_ = -tmp_quat_vec[i + 1].w_;
            tmp_quat_vec[i + 1].x_ = -tmp_quat_vec[i + 1].x_;
            tmp_quat_vec[i + 1].y_ = -tmp_quat_vec[i + 1].y_;
            tmp_quat_vec[i + 1].z_ = -tmp_quat_vec[i + 1].z_;
        }
    }
    via_points_quat_ = tmp_quat_vec;
    // if (!abcSpline()) return false;

    return true;
}

bool GivenVelocityPlanner::calcResampledPointDist(void)
{
    double distance = 0;
    traj_points_dist_.clear();
    traj_points_dist_.push_back(distance);
    for (size_t cnt = 0; cnt < resampled_traj_.size() - 1; ++cnt)
    {
        distance += getDistance(resampled_traj_[cnt+1].point_, resampled_traj_[cnt].point_);
        traj_points_dist_.push_back(distance);
    }
    return true;
}

bool GivenVelocityPlanner::trajPausePlan(int32_t index, double vel, double vel_ratio, double acc_ratio, double jerk_ratio)
{
    // calc the line speed of the origon traj
    vector<double> cart_vel_org;
    for (size_t i = 0; i < traj_points_dist_.size() - 1; ++i)
    {
        cart_vel_org.push_back((traj_points_dist_[i + 1] - traj_points_dist_[i]) / sampling_freq_);
    }
    // calc the stop time
    double dec_max = 5; // may calc with acc_ratio
    double line_stop_time = cart_vel_org[index] / dec_max;

    // calc the distance of pause traj use the specified acceleration
    double min_stop_dist = 0.5 * dec_max * pow(line_stop_time, 2);

    // find the nearlist point on the orginal traj
    double stop_pos = 0;
    //double stop_while = 0;
    int stop_index = 0;
    for (size_t i = index; i < traj_points_dist_.size() - 1; ++i)
    {
        if (traj_points_dist_[i] > min_stop_dist + traj_points_dist_[index])
        {
            stop_index = i;
            stop_pos = traj_points_dist_[stop_index];
            break;
            //stop_while = stop_index * sampling_freq_;
        }
    }
    // means can not stop in time just go on org traj
    if (stop_index == 0)
    {
        return true;
    }

    double pause_time = index * sampling_freq_;
    double stop_time = sqrt(2 * (stop_pos - traj_points_dist_[index])) + pause_time;
    stop_time = ceil(stop_time / sampling_freq_) * sampling_freq_;

    // fit the stop traj
    vector<Point> xyz_pause_vp;
    xyz_pause_vp.push_back(resampled_traj_[index].point_);
    xyz_pause_vp.push_back(resampled_traj_[stop_index].point_);

    vector<double> vp_time;
    vp_time.push_back(pause_time);
    vp_time.push_back(stop_time);

    // calc the xyz speed of the origon traj on pause and stop point
    Point pause_pose_vel = (resampled_traj_[index].point_ - resampled_traj_[index - 1].point_) / sampling_freq_;
    Point stop_pose_vel; stop_pose_vel.x_ = 0; stop_pose_vel.y_ = 0; stop_pose_vel.z_ = 0;

    Quaternion pause_quat = Euler2Quaternion(resampled_traj_[index].euler_);
    Quaternion stop_quat = Euler2Quaternion(resampled_traj_[stop_index].euler_);
    double dq = 0.0;
    dq = pause_quat.w_ * stop_quat.w_
        + pause_quat.x_ * stop_quat.x_
        + pause_quat.y_ * stop_quat.y_
        + pause_quat.z_ * stop_quat.z_;

    if (dq < 0)
    {
        stop_quat.w_ = -stop_quat.w_;
        stop_quat.x_ = -stop_quat.x_;
        stop_quat.y_ = -stop_quat.y_;
        stop_quat.z_ = -stop_quat.z_;
    }

    vector<Quaternion> abc_pause_vp;
    abc_pause_vp.push_back(pause_quat);
    abc_pause_vp.push_back(stop_quat);

    Quaternion pause_quat_vel = (Euler2Quaternion(resampled_traj_[index].euler_) - Euler2Quaternion(resampled_traj_[index - 1].euler_));
    pause_quat_vel = pause_quat_vel / sampling_freq_;
    Quaternion stop_quat_vel; stop_quat_vel.w_ = 0; stop_quat_vel.x_ = 0; stop_quat_vel.y_ = 0; stop_quat_vel.z_ = 0;

    //double abc_vs[4] = { pause_quat_vel.x_, pause_quat_vel.y_, pause_quat_vel.z_, pause_quat_vel.w_ };
    //double abc_ve[4] = { 0, 0, 0, 0 };

    if (spline(xyz_pause_vp, abc_pause_vp,
        pause_pose_vel, stop_pose_vel,
        pause_quat_vel, stop_quat_vel,
        vp_time, pause_traj_) == false)
    {
        return false;
    }

    return true;
}

bool GivenVelocityPlanner::trajResumePlan(void)
{
    return true;
}

vector<Quaternion> GivenVelocityPlanner::testQuatSmooth(const vector<Quaternion>& quat_in)
{
    via_points_quat_ = quat_in;
    abcQuatSmooth(abc_smooth_window_);
    return via_points_quat_;
}

vector<Point> GivenVelocityPlanner::testPoseSmooth(const vector<Point>& pos_in)
{
    return via_points_pose_;
}


bool GivenVelocityPlanner::spline
(
    const vector<Point> via_points_pose,
    const vector<Quaternion> via_points_quat,
    const Point init_pose_vel,
    const Point final_pose_vel,
    const Quaternion init_quat_vel,
    const Quaternion final_quat_vel,
    const vector<double> via_points_time,
    vector<PoseEuler>& out_traj
)
{
    if (via_points_pose.size() != via_points_quat.size()
        || via_points_time.size() != via_points_pose.size()
        || via_points_pose.size() <= 1)
    {
        LogProducer::error("mc","spline factor size invalid pose=%ld, quat=%ld time=%ld", via_points_pose.size(), via_points_quat.size(), via_points_time.size());
        return false;
    }

    size_t points_cnts = via_points_pose.size();

    double* px_Y = new double[points_cnts];
    double* py_Y = new double[points_cnts];
    double* pz_Y = new double[points_cnts];

    double* qw_Y = new double[points_cnts];
    double* qx_Y = new double[points_cnts];
    double* qy_Y = new double[points_cnts];
    double* qz_Y = new double[points_cnts];

    double* H = new double[points_cnts * points_cnts];
    double* inv_H = new double[points_cnts * points_cnts];

    double* inv_H_px_Y = new double[points_cnts];
    double* inv_H_py_Y = new double[points_cnts];
    double* inv_H_pz_Y = new double[points_cnts];

    double* inv_H_qw_Y = new double[points_cnts];
    double* inv_H_qx_Y = new double[points_cnts];
    double* inv_H_qy_Y = new double[points_cnts];
    double* inv_H_qz_Y = new double[points_cnts];

    memset(px_Y, 0, sizeof(double) * points_cnts);
    memset(py_Y, 0, sizeof(double) * points_cnts);
    memset(pz_Y, 0, sizeof(double) * points_cnts);

    memset(qw_Y, 0, sizeof(double) * points_cnts);
    memset(qx_Y, 0, sizeof(double) * points_cnts);
    memset(qy_Y, 0, sizeof(double) * points_cnts);
    memset(qz_Y, 0, sizeof(double) * points_cnts);
    memset(H, 0, sizeof(double) * points_cnts * points_cnts);
    memset(inv_H, 0, sizeof(double) * points_cnts * points_cnts);

    memset(inv_H_px_Y, 0, sizeof(double) * points_cnts);
    memset(inv_H_py_Y, 0, sizeof(double) * points_cnts);
    memset(inv_H_pz_Y, 0, sizeof(double) * points_cnts);

    memset(inv_H_qw_Y, 0, sizeof(double) * points_cnts);
    memset(inv_H_qx_Y, 0, sizeof(double) * points_cnts);
    memset(inv_H_qy_Y, 0, sizeof(double) * points_cnts);
    memset(inv_H_qz_Y, 0, sizeof(double) * points_cnts);

    for (size_t i = 0; i < points_cnts; ++i)
    {
        if (i == 0)
        {
            H[0] = 2 * (via_points_time[1] - via_points_time[0]);
            H[1] = via_points_time[1] - via_points_time[0];

            //point factor calc
            px_Y[0] = 6 * (
                (via_points_pose[1].x_ - via_points_pose[0].x_)
                / (via_points_time[1] - via_points_time[0]) - init_pose_vel.x_
                );

            py_Y[0] = 6 * (
                (via_points_pose[1].y_ - via_points_pose[0].y_)
                / (via_points_time[1] - via_points_time[0]) - init_pose_vel[1]
                );

            pz_Y[0] = 6 * (
                (via_points_pose[1].z_ - via_points_pose[0].z_)
                / (via_points_time[1] - via_points_time[0]) - init_pose_vel[2]
                );

            // quaternion factor calc
            qw_Y[0] = 6 * (
                (via_points_quat[1].w_ - via_points_quat[0].w_)
                / (via_points_time[1] - via_points_time[0]) - init_quat_vel.w_
                );

            qx_Y[0] = 6 * (
                (via_points_quat[1].x_ - via_points_quat[0].x_)
                / (via_points_time[1] - via_points_time[0]) - init_quat_vel.x_
                );

            qy_Y[0] = 6 * (
                (via_points_quat[1].y_ - via_points_quat[0].y_)
                / (via_points_time[1] - via_points_time[0]) - init_quat_vel.y_
                );

            qz_Y[0] = 6 * (
                (via_points_quat[1].z_ - via_points_quat[0].z_)
                / (via_points_time[1] - via_points_time[0]) - init_quat_vel.z_
                );
        }
        else if (i == points_cnts - 1)
        {
            H[i * points_cnts + i - 1] = via_points_time[i] - via_points_time[i - 1];
            H[i * points_cnts + i - 0] = 2 * (via_points_time[i] - via_points_time[i - 1]);

            //matlab:Y(i) = 6 * (diff(p(i:i + 1)) / diff(tnew(i:i + 1)) - v0);
            //point factor calc
            px_Y[i] = 6 * (final_pose_vel.x_
                - (via_points_pose[i].x_ - via_points_pose[i - 1].x_)
                / (via_points_time[i] - via_points_time[i - 1])
                );

            py_Y[i] = 6 * (final_pose_vel.y_
                - (via_points_pose[i].y_ - via_points_pose[i - 1].y_)
                / (via_points_time[i] - via_points_time[i - 1])
                );

            pz_Y[i] = 6 * (final_pose_vel.z_
                - (via_points_pose[i].z_ - via_points_pose[i - 1].z_)
                / (via_points_time[i] - via_points_time[i - 1])
                );
            /************************************************************/
            qw_Y[i] = 6 * (final_quat_vel.w_
                - (via_points_quat[i].w_ - via_points_quat[i - 1].w_)
                / (via_points_time[i] - via_points_time[i - 1])
                );

            qx_Y[i] = 6 * (final_quat_vel.x_
                - (via_points_quat[i].x_ - via_points_quat_[i - 1].x_)
                / (via_points_time[i] - via_points_time[i - 1])
                );

            qy_Y[i] = 6 * (final_quat_vel.y_
                - (via_points_quat[i].y_ - via_points_quat[i - 1].y_)
                / (via_points_time[i] - via_points_time[i - 1])
                );

            qz_Y[i] = 6 * (final_quat_vel.z_
                - (via_points_quat[i].z_ - via_points_quat[i - 1].z_)
                / (via_points_time[i] - via_points_time[i - 1])
                );
        }
        else
        {
            H[i * points_cnts + i - 1] = via_points_time[i] - via_points_time[i - 1];
            /*a[i] - a[i-1] + a[i+1] - a[i] --> a[i+1] - a[i-1]*/
            H[i * points_cnts + i - 0] = 2 * (via_points_time[i + 1] - via_points_time[i - 1]);
            H[i * points_cnts + i + 1] = via_points_time[i + 1] - via_points_time[i];
            //Y(i) = 6 * (diff(p(i:i + 1)) / diff(tnew(i:i + 1)) - diff(p(i - 1:i) / diff(tnew(i - 1:i))));
            px_Y[i] = 6 * (
                (via_points_pose[i + 1].x_ - via_points_pose[i].x_)
                / (via_points_time[i + 1] - via_points_time[i])
                - (via_points_pose[i].x_ - via_points_pose[i - 1].x_)
                / (via_points_time[i] - via_points_time[i - 1])
                );

            py_Y[i] = 6 * (
                (via_points_pose[i + 1].y_ - via_points_pose[i].y_)
                / (via_points_time[i + 1] - via_points_time[i])
                - (via_points_pose[i].y_ - via_points_pose[i - 1].y_)
                / (via_points_time[i] - via_points_time[i - 1])
                );

            pz_Y[i] = 6 * (
                (via_points_pose[i + 1].z_ - via_points_pose[i].z_)
                / (via_points_time[i + 1] - via_points_time[i])
                - (via_points_pose[i].z_ - via_points_pose[i - 1].z_)
                / (via_points_time[i] - via_points_time[i - 1])
                );
            /******************************************************************/
            qw_Y[i] = 6 * (
                (via_points_quat[i + 1].w_ - via_points_quat[i].w_)
                / (via_points_time[i + 1] - via_points_time[i])
                - (via_points_quat[i].w_ - via_points_quat[i - 1].w_)
                / (via_points_time[i] - via_points_time[i - 1])
                );
            qx_Y[i] = 6 * (
                (via_points_quat[i + 1].x_ - via_points_quat[i].x_)
                / (via_points_time[i + 1] - via_points_time[i])
                - (via_points_quat[i].x_ - via_points_quat[i - 1].x_)
                / (via_points_time[i] - via_points_time[i - 1])
                );

            qy_Y[i] = 6 * (
                (via_points_quat[i + 1].y_ - via_points_quat[i].y_)
                / (via_points_time[i + 1] - via_points_time[i])
                - (via_points_quat[i].y_ - via_points_quat[i - 1].y_)
                / (via_points_time[i] - via_points_time[i - 1])
                );

            qz_Y[i] = 6 * (
                (via_points_quat[i + 1].z_ - via_points_quat[i].z_)
                / (via_points_time[i + 1] - via_points_time[i])
                - (via_points_quat[i].z_ - via_points_quat[i - 1].z_)
                / (via_points_time[i] - via_points_time[i - 1])
                );
        }
    }
    if (!inverse(H, points_cnts, inv_H))
    {
        LogProducer::error("offline_planner", "inverse H failed");
        return false;
    }

    //m = inv(H) * Y;
    for (size_t i = 0; i < points_cnts; ++i)
    {
        for (size_t j = 0; j < points_cnts; ++j)
        {
            inv_H_px_Y[i] += inv_H[i * points_cnts + j] * px_Y[j];
            inv_H_py_Y[i] += inv_H[i * points_cnts + j] * py_Y[j];
            inv_H_pz_Y[i] += inv_H[i * points_cnts + j] * pz_Y[j];

            inv_H_qw_Y[i] += inv_H[i * points_cnts + j] * qw_Y[j];
            inv_H_qx_Y[i] += inv_H[i * points_cnts + j] * qx_Y[j];
            inv_H_qy_Y[i] += inv_H[i * points_cnts + j] * qy_Y[j];
            inv_H_qz_Y[i] += inv_H[i * points_cnts + j] * qz_Y[j];
        }
    }

    vector<double> fa0_px, fa0_py, fa0_pz;
    vector<double> fa1_px, fa1_py, fa1_pz;
    vector<double> fa2_px, fa2_py, fa2_pz;
    vector<double> fa3_px, fa3_py, fa3_pz;

    vector<double> fa0_qw, fa0_qx, fa0_qy, fa0_qz;
    vector<double> fa1_qw, fa1_qx, fa1_qy, fa1_qz;
    vector<double> fa2_qw, fa2_qx, fa2_qy, fa2_qz;
    vector<double> fa3_qw, fa3_qx, fa3_qy, fa3_qz;

    for (size_t i = 0; i < points_cnts - 1; ++i)
    {
        fa0_px.push_back(via_points_pose[i].x_);
        fa0_py.push_back(via_points_pose[i].y_);
        fa0_pz.push_back(via_points_pose[i].z_);

        fa0_qw.push_back(via_points_quat[i].w_);
        fa0_qx.push_back(via_points_quat[i].x_);
        fa0_qy.push_back(via_points_quat[i].y_);
        fa0_qz.push_back(via_points_quat[i].z_);

        fa1_px.push_back((via_points_pose[i + 1].x_ - via_points_pose[i].x_)
            / (via_points_time[i + 1] - via_points_time[i])
            - inv_H_px_Y[i] * (via_points_time[i + 1] - via_points_time[i]) / 2
            - (inv_H_px_Y[i + 1] - inv_H_px_Y[i]) * (via_points_time[i + 1] - via_points_time[i]) / 6);

        fa1_py.push_back((via_points_pose[i + 1].y_ - via_points_pose[i].y_)
            / (via_points_time[i + 1] - via_points_time[i])
            - inv_H_py_Y[i] * (via_points_time[i + 1] - via_points_time[i]) / 2
            - (inv_H_py_Y[i + 1] - inv_H_py_Y[i]) * (via_points_time[i + 1] - via_points_time[i]) / 6);

        fa1_pz.push_back((via_points_pose[i + 1].z_ - via_points_pose[i].z_)
            / (via_points_time[i + 1] - via_points_time[i])
            - inv_H_pz_Y[i] * (via_points_time[i + 1] - via_points_time[i]) / 2
            - (inv_H_pz_Y[i + 1] - inv_H_pz_Y[i]) * (via_points_time[i + 1] - via_points_time[i]) / 6);
        /************************/
        fa1_qw.push_back((via_points_quat[i + 1].w_ - via_points_quat[i].w_)
            / (via_points_time[i + 1] - via_points_time[i])
            - inv_H_qw_Y[i] * (via_points_time[i + 1] - via_points_time[i]) / 2
            - (inv_H_qw_Y[i + 1] - inv_H_qw_Y[i]) * (via_points_time[i + 1] - via_points_time[i]) / 6);

        fa1_qx.push_back((via_points_quat[i + 1].x_ - via_points_quat[i].x_)
            / (via_points_time[i + 1] - via_points_time[i])
            - inv_H_qx_Y[i] * (via_points_time[i + 1] - via_points_time[i]) / 2
            - (inv_H_qx_Y[i + 1] - inv_H_qx_Y[i]) * (via_points_time[i + 1] - via_points_time[i]) / 6);

        fa1_qy.push_back((via_points_quat[i + 1].y_ - via_points_quat[i].y_)
            / (via_points_time[i + 1] - via_points_time[i])
            - inv_H_qy_Y[i] * (via_points_time[i + 1] - via_points_time[i]) / 2
            - (inv_H_qy_Y[i + 1] - inv_H_qy_Y[i]) * (via_points_time[i + 1] - via_points_time[i]) / 6);

        fa1_qz.push_back((via_points_quat[i + 1].z_ - via_points_quat[i].z_)
            / (via_points_time[i + 1] - via_points_time[i])
            - inv_H_qz_Y[i] * (via_points_time[i + 1] - via_points_time[i]) / 2
            - (inv_H_qz_Y[i + 1] - inv_H_qz_Y[i]) * (via_points_time[i + 1] - via_points_time[i]) / 6);

        fa2_px.push_back(inv_H_px_Y[i] / 2);
        fa2_py.push_back(inv_H_py_Y[i] / 2);
        fa2_pz.push_back(inv_H_pz_Y[i] / 2);

        fa2_qw.push_back(inv_H_qw_Y[i] / 2);
        fa2_qx.push_back(inv_H_qx_Y[i] / 2);
        fa2_qy.push_back(inv_H_qy_Y[i] / 2);
        fa2_qz.push_back(inv_H_qz_Y[i] / 2);

        fa3_px.push_back((inv_H_px_Y[i + 1] - inv_H_px_Y[i])
            / (6 * (via_points_time[i + 1] - via_points_time[i])));

        fa3_py.push_back((inv_H_py_Y[i + 1] - inv_H_py_Y[i])
            / (6 * (via_points_time[i + 1] - via_points_time[i])));

        fa3_pz.push_back((inv_H_pz_Y[i + 1] - inv_H_pz_Y[i])
            / (6 * (via_points_time[i + 1] - via_points_time[i])));
        /**********************/
        fa3_qw.push_back((inv_H_qw_Y[i + 1] - inv_H_qw_Y[i])
            / (6 * (via_points_time[i + 1] - via_points_time[i])));

        fa3_qx.push_back((inv_H_qx_Y[i + 1] - inv_H_qx_Y[i])
            / (6 * (via_points_time[i + 1] - via_points_time[i])));

        fa3_qy.push_back((inv_H_qy_Y[i + 1] - inv_H_qy_Y[i])
            / (6 * (via_points_time[i + 1] - via_points_time[i])));

        fa3_qz.push_back((inv_H_qz_Y[i + 1] - inv_H_qz_Y[i])
            / (6 * (via_points_time[i + 1] - via_points_time[i])));
    }
    int index = 0;
    PoseEuler tmp_pos;
    Quaternion tmp_quat;
    for (double t = via_points_time.front(); t <= via_points_time.back(); t += sampling_freq_)
    {
        for (int i = 0; ; ++i)
        {
            if (via_points_time[i] > t)
            {
                index = i - 1;
                break;
            }
        }
        tmp_pos.point_.x_ = fa3_px[index] * pow(t - via_points_time[index], 3)
            + fa2_px[index] * pow(t - via_points_time[index], 2)
            + fa1_px[index] * (t - via_points_time[index])
            + fa0_px[index];

        tmp_pos.point_.y_ = fa3_py[index] * pow(t - via_points_time[index], 3)
            + fa2_py[index] * pow(t - via_points_time[index], 2)
            + fa1_py[index] * (t - via_points_time[index])
            + fa0_py[index];

        tmp_pos.point_.z_ = fa3_pz[index] * pow(t - via_points_time[index], 3)
            + fa2_pz[index] * pow(t - via_points_time[index], 2)
            + fa1_pz[index] * (t - via_points_time[index])
            + fa0_pz[index];

        tmp_quat.w_ = fa3_qw[index] * pow(t - via_points_time[index], 3)
            + fa2_qw[index] * pow(t - via_points_time[index], 2)
            + fa1_qw[index] * (t - via_points_time[index])
            + fa0_qw[index];

        tmp_quat.x_ = fa3_qx[index] * pow(t - via_points_time[index], 3)
            + fa2_qx[index] * pow(t - via_points_time[index], 2)
            + fa1_qx[index] * (t - via_points_time[index])
            + fa0_qx[index];

        tmp_quat.y_ = fa3_qy[index] * pow(t - via_points_time[index], 3)
            + fa2_qy[index] * pow(t - via_points_time[index], 2)
            + fa1_qy[index] * (t - via_points_time[index])
            + fa0_qy[index];

        tmp_quat.z_ = fa3_qz[index] * pow(t - via_points_time[index], 3)
            + fa2_qz[index] * pow(t - via_points_time[index], 2)
            + fa1_qz[index] * (t - via_points_time[index])
            + fa0_qz[index];

        tmp_pos.euler_ = Quaternion2Euler(tmp_quat);

        out_traj.push_back(tmp_pos);
    }

    delete[] px_Y;
    delete[] py_Y;
    delete[] pz_Y;

    delete[] qw_Y;
    delete[] qx_Y;
    delete[] qy_Y;
    delete[] qz_Y;

    delete[] H;
    delete[] inv_H;

    delete[] inv_H_px_Y;
    delete[] inv_H_py_Y;
    delete[] inv_H_pz_Y;

    delete[] inv_H_qw_Y;
    delete[] inv_H_qx_Y;
    delete[] inv_H_qy_Y;
    delete[] inv_H_qz_Y;

    return true;
}

void GivenVelocityPlanner::setLimit(const basic_alg::Joint &vel_limit, const basic_alg::Joint &acc_limit)
{
    vel_limit_ = vel_limit;
    acc_limit_ = acc_limit;
}
