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

void GivenVelocityPlanner::reset(void)
{
    via_points_pose_.clear();
    via_points_quat_.clear();
    via_points_dist_.clear();
    traj_points_dist_.clear();
    via_points_time_new_.clear();
    resampled_traj_.clear();
    pause_traj_.clear();
    resume_traj_.clear();
    via_points_cnt_ = 0;

    fa0_px_.clear(); fa0_py_.clear(); fa0_pz_.clear();
    fa1_px_.clear(); fa1_py_.clear(); fa1_pz_.clear();
    fa2_px_.clear(); fa2_py_.clear(); fa2_pz_.clear();
    fa3_px_.clear(); fa3_py_.clear(); fa3_pz_.clear();

    fa0_qw_.clear(); fa0_qx_.clear(); fa0_qy_.clear(); fa0_qz_.clear();
    fa1_qw_.clear(); fa1_qx_.clear(); fa1_qy_.clear(); fa1_qz_.clear();
    fa2_qw_.clear(); fa2_qx_.clear(); fa2_qy_.clear(); fa2_qz_.clear();
    fa3_qw_.clear(); fa3_qx_.clear(); fa3_qy_.clear(); fa3_qz_.clear();
}

bool GivenVelocityPlanner::setViaPoints(const vector<PoseEuler> &via_points, bool is_new)
{
     /* plan a new trajectory */
    if(is_new) { reset(); }

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
    if(traj_vel < MINIMUM_E3)
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

bool GivenVelocityPlanner::viaPoints2Traj(vector<PoseEuler> via_points, double traj_vel)
{
    setViaPoints(via_points, true);
    return viaPoints2Traj(traj_vel);
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

bool GivenVelocityPlanner::trajPausePlan(uint32_t index, double vel, double vel_ratio, double acc_ratio, double jerk_ratio)
{
    if(index > traj_points_dist_.size() - 1) return false;
    pause_traj_.clear();
    // calc the line speed of the origon traj
    vector<double> cart_vel_org;

    for (size_t i = 0; i < traj_points_dist_.size() - 1; ++i)
    {
        cart_vel_org.push_back((traj_points_dist_[i + 1] - traj_points_dist_[i]) / sampling_freq_);
    }

    // calc the stop time
    double dec_max = 500.0; //traj_param_->position_acc_max_; // may calc with acc_ratio
    double line_stop_time = cart_vel_org[index] / dec_max;

    // calc the distance of pause traj use the specified acceleration
    double min_stop_dist = 0.5 * dec_max * pow(line_stop_time, 2);

    // find the nearest point on the orginal traj
    double paused_pos = 0;
    for (size_t i = index; i < traj_points_dist_.size() - 1; ++i)
    {
        if (traj_points_dist_[i] > min_stop_dist + traj_points_dist_[index])
        {
            paused_index_ = i;
            LogProducer::info("GivenVelocityPlanner", "paused_index %d", paused_index_);
            paused_pos = traj_points_dist_[paused_index_];
            break;
        }
    }
    // means can not stop in time just go on org traj
    if (paused_index_ == 0)
    {
        for(size_t i = index; i < resampled_traj_.size(); ++i)
        {
            pause_traj_.push_back(resampled_traj_[i]);
        }
        return true;
    }

    double pause_time = index * sampling_freq_;
    double stop_time = sqrt(2 * (paused_pos - traj_points_dist_[index])) + pause_time;
    stop_time = ceil(stop_time / sampling_freq_) * sampling_freq_;

    vector<double> org_traj_time;
    org_traj_time.push_back(pause_time);
    org_traj_time.push_back(paused_index_ * sampling_freq_);

    vector<double> vp_time;
    vp_time.push_back(pause_time);
    vp_time.push_back(stop_time);

    vector<double> pause_traj_time;
    if(!spline(org_traj_time, 1.0, 0.0, vp_time, pause_traj_time))
    {
        return false;
    }
    spline_value(via_points_time_new_, pause_traj_time, pause_traj_);

#if 0
    // fit the stop traj
    vector<Point> xyz_pause_vp;
    xyz_pause_vp.push_back(resampled_traj_[index].point_);
    xyz_pause_vp.push_back(resampled_traj_[paused_index_].point_);

    // calc the xyz speed of the origon traj on pause and stop point
    Point pause_pose_vel = (resampled_traj_[index].point_ - resampled_traj_[index - 1].point_) / sampling_freq_;
    Point stop_pose_vel; stop_pose_vel.x_ = 0; stop_pose_vel.y_ = 0; stop_pose_vel.z_ = 0;

    Quaternion pause_quat = Euler2Quaternion(resampled_traj_[index].euler_);
    Quaternion stop_quat = Euler2Quaternion(resampled_traj_[paused_index_].euler_);
    double dq = pause_quat.w_ * stop_quat.w_ 
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

    if (spline(xyz_pause_vp, abc_pause_vp,
        pause_pose_vel, stop_pose_vel,
        pause_quat_vel, stop_quat_vel,
        vp_time, pause_traj_) == false)
    {
        return false;
    }
#endif
    return true;
}

bool GivenVelocityPlanner::trajResumePlan(double vel, double vel_ratio, double acc_ratio, double jerk_ratio)
{
    // calc the rest of traj distance 
    // double dist = traj_points_dist_.back() - traj_points_dist_[paused_index_];
    // // calc the time
    // double t = dist / traj_vel_set_;
    // double acc = 0, max_acc = 222;
    // do{
    //     acc = traj_vel_set_ / (0.333333 * t);
    //     if(acc > max_acc) t = t * 2;
    // }while(acc > max_acc);

    // get the new via points
    int resume_vp_interval = (int)round((resampled_traj_.size() - paused_index_) / 3);
    vector<PoseEuler> new_vp;
    for(size_t i = paused_index_; i < resampled_traj_.size(); i+=resume_vp_interval)
    {
        new_vp.push_back(resampled_traj_[i]);
    }
    new_vp.push_back(resampled_traj_.back());
    viaPoints2Traj(new_vp, traj_vel_set_);
    resume_traj_ = resampled_traj_;
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

    for (size_t i = 0; i < points_cnts - 1; ++i)
    {
        fa0_px_.push_back(via_points_pose[i].x_);
        fa0_py_.push_back(via_points_pose[i].y_);
        fa0_pz_.push_back(via_points_pose[i].z_);

        fa0_qw_.push_back(via_points_quat[i].w_);
        fa0_qx_.push_back(via_points_quat[i].x_);
        fa0_qy_.push_back(via_points_quat[i].y_);
        fa0_qz_.push_back(via_points_quat[i].z_);

        fa1_px_.push_back((via_points_pose[i + 1].x_ - via_points_pose[i].x_)
            / (via_points_time[i + 1] - via_points_time[i])
            - inv_H_px_Y[i] * (via_points_time[i + 1] - via_points_time[i]) / 2
            - (inv_H_px_Y[i + 1] - inv_H_px_Y[i]) * (via_points_time[i + 1] - via_points_time[i]) / 6);

        fa1_py_.push_back((via_points_pose[i + 1].y_ - via_points_pose[i].y_)
            / (via_points_time[i + 1] - via_points_time[i])
            - inv_H_py_Y[i] * (via_points_time[i + 1] - via_points_time[i]) / 2
            - (inv_H_py_Y[i + 1] - inv_H_py_Y[i]) * (via_points_time[i + 1] - via_points_time[i]) / 6);

        fa1_pz_.push_back((via_points_pose[i + 1].z_ - via_points_pose[i].z_)
            / (via_points_time[i + 1] - via_points_time[i])
            - inv_H_pz_Y[i] * (via_points_time[i + 1] - via_points_time[i]) / 2
            - (inv_H_pz_Y[i + 1] - inv_H_pz_Y[i]) * (via_points_time[i + 1] - via_points_time[i]) / 6);
        /******************************************************************************************************/
        fa1_qw_.push_back((via_points_quat[i + 1].w_ - via_points_quat[i].w_)
            / (via_points_time[i + 1] - via_points_time[i])
            - inv_H_qw_Y[i] * (via_points_time[i + 1] - via_points_time[i]) / 2
            - (inv_H_qw_Y[i + 1] - inv_H_qw_Y[i]) * (via_points_time[i + 1] - via_points_time[i]) / 6);

        fa1_qx_.push_back((via_points_quat[i + 1].x_ - via_points_quat[i].x_)
            / (via_points_time[i + 1] - via_points_time[i])
            - inv_H_qx_Y[i] * (via_points_time[i + 1] - via_points_time[i]) / 2
            - (inv_H_qx_Y[i + 1] - inv_H_qx_Y[i]) * (via_points_time[i + 1] - via_points_time[i]) / 6);

        fa1_qy_.push_back((via_points_quat[i + 1].y_ - via_points_quat[i].y_)
            / (via_points_time[i + 1] - via_points_time[i])
            - inv_H_qy_Y[i] * (via_points_time[i + 1] - via_points_time[i]) / 2
            - (inv_H_qy_Y[i + 1] - inv_H_qy_Y[i]) * (via_points_time[i + 1] - via_points_time[i]) / 6);

        fa1_qz_.push_back((via_points_quat[i + 1].z_ - via_points_quat[i].z_)
            / (via_points_time[i + 1] - via_points_time[i])
            - inv_H_qz_Y[i] * (via_points_time[i + 1] - via_points_time[i]) / 2
            - (inv_H_qz_Y[i + 1] - inv_H_qz_Y[i]) * (via_points_time[i + 1] - via_points_time[i]) / 6);

        fa2_px_.push_back(inv_H_px_Y[i] / 2);
        fa2_py_.push_back(inv_H_py_Y[i] / 2);
        fa2_pz_.push_back(inv_H_pz_Y[i] / 2);

        fa2_qw_.push_back(inv_H_qw_Y[i] / 2);
        fa2_qx_.push_back(inv_H_qx_Y[i] / 2);
        fa2_qy_.push_back(inv_H_qy_Y[i] / 2);
        fa2_qz_.push_back(inv_H_qz_Y[i] / 2);

        fa3_px_.push_back((inv_H_px_Y[i + 1] - inv_H_px_Y[i])
            / (6 * (via_points_time[i + 1] - via_points_time[i])));

        fa3_py_.push_back((inv_H_py_Y[i + 1] - inv_H_py_Y[i])
            / (6 * (via_points_time[i + 1] - via_points_time[i])));

        fa3_pz_.push_back((inv_H_pz_Y[i + 1] - inv_H_pz_Y[i])
            / (6 * (via_points_time[i + 1] - via_points_time[i])));
        /******************************************************************************************************/
        fa3_qw_.push_back((inv_H_qw_Y[i + 1] - inv_H_qw_Y[i])
            / (6 * (via_points_time[i + 1] - via_points_time[i])));

        fa3_qx_.push_back((inv_H_qx_Y[i + 1] - inv_H_qx_Y[i])
            / (6 * (via_points_time[i + 1] - via_points_time[i])));

        fa3_qy_.push_back((inv_H_qy_Y[i + 1] - inv_H_qy_Y[i])
            / (6 * (via_points_time[i + 1] - via_points_time[i])));

        fa3_qz_.push_back((inv_H_qz_Y[i + 1] - inv_H_qz_Y[i])
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
        tmp_pos.point_.x_ = fa3_px_[index] * pow(t - via_points_time[index], 3)
            + fa2_px_[index] * pow(t - via_points_time[index], 2)
            + fa1_px_[index] * (t - via_points_time[index])
            + fa0_px_[index];

        tmp_pos.point_.y_ = fa3_py_[index] * pow(t - via_points_time[index], 3)
            + fa2_py_[index] * pow(t - via_points_time[index], 2)
            + fa1_py_[index] * (t - via_points_time[index])
            + fa0_py_[index];

        tmp_pos.point_.z_ = fa3_pz_[index] * pow(t - via_points_time[index], 3)
            + fa2_pz_[index] * pow(t - via_points_time[index], 2)
            + fa1_pz_[index] * (t - via_points_time[index])
            + fa0_pz_[index];

        tmp_quat.w_ = fa3_qw_[index] * pow(t - via_points_time[index], 3)
            + fa2_qw_[index] * pow(t - via_points_time[index], 2)
            + fa1_qw_[index] * (t - via_points_time[index])
            + fa0_qw_[index];

        tmp_quat.x_ = fa3_qx_[index] * pow(t - via_points_time[index], 3)
            + fa2_qx_[index] * pow(t - via_points_time[index], 2)
            + fa1_qx_[index] * (t - via_points_time[index])
            + fa0_qx_[index];

        tmp_quat.y_ = fa3_qy_[index] * pow(t - via_points_time[index], 3)
            + fa2_qy_[index] * pow(t - via_points_time[index], 2)
            + fa1_qy_[index] * (t - via_points_time[index])
            + fa0_qy_[index];

        tmp_quat.z_ = fa3_qz_[index] * pow(t - via_points_time[index], 3)
            + fa2_qz_[index] * pow(t - via_points_time[index], 2)
            + fa1_qz_[index] * (t - via_points_time[index])
            + fa0_qz_[index];

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

void GivenVelocityPlanner::spline_value
(
    const std::vector<double> via_points_time, 
    const std::vector<double> resampled_time, 
    std::vector<basic_alg::PoseEuler>& out_traj
)
{
    int index = 0;
    PoseEuler tmp_pos;
    Quaternion tmp_quat;

    for (double t = resampled_time.front(); t <= resampled_time.back(); t += sampling_freq_)
    {
        for (int i = 0; ; ++i)
        {
            if (via_points_time[i] > t)
            {
                index = i - 1;
                break;
            }
        }
        tmp_pos.point_.x_ = fa3_px_[index] * pow(t - via_points_time[index], 3)
            + fa2_px_[index] * pow(t - via_points_time[index], 2)
            + fa1_px_[index] * (t - via_points_time[index])
            + fa0_px_[index];

        tmp_pos.point_.y_ = fa3_py_[index] * pow(t - via_points_time[index], 3)
            + fa2_py_[index] * pow(t - via_points_time[index], 2)
            + fa1_py_[index] * (t - via_points_time[index])
            + fa0_py_[index];

        tmp_pos.point_.z_ = fa3_pz_[index] * pow(t - via_points_time[index], 3)
            + fa2_pz_[index] * pow(t - via_points_time[index], 2)
            + fa1_pz_[index] * (t - via_points_time[index])
            + fa0_pz_[index];

        tmp_quat.w_ = fa3_qw_[index] * pow(t - via_points_time[index], 3)
            + fa2_qw_[index] * pow(t - via_points_time[index], 2)
            + fa1_qw_[index] * (t - via_points_time[index])
            + fa0_qw_[index];

        tmp_quat.x_ = fa3_qx_[index] * pow(t - via_points_time[index], 3)
            + fa2_qx_[index] * pow(t - via_points_time[index], 2)
            + fa1_qx_[index] * (t - via_points_time[index])
            + fa0_qx_[index];

        tmp_quat.y_ = fa3_qy_[index] * pow(t - via_points_time[index], 3)
            + fa2_qy_[index] * pow(t - via_points_time[index], 2)
            + fa1_qy_[index] * (t - via_points_time[index])
            + fa0_qy_[index];

        tmp_quat.z_ = fa3_qz_[index] * pow(t - via_points_time[index], 3)
            + fa2_qz_[index] * pow(t - via_points_time[index], 2)
            + fa1_qz_[index] * (t - via_points_time[index])
            + fa0_qz_[index];

        tmp_pos.euler_ = Quaternion2Euler(tmp_quat);

        out_traj.push_back(tmp_pos);
    }
}

bool GivenVelocityPlanner::spline
(
    const vector<double>& data, 
    const double& init, 
    const double& final, 
    const std::vector<double> times, 
    vector<double>& out
)
{
    size_t cnts = data.size();

    double* Y = new double[cnts];

    double* H = new double[cnts * cnts];
    double* inv_H = new double[cnts * cnts];

    double* inv_H_Y = new double[cnts];

    memset(Y, 0, sizeof(double) * cnts);
    memset(H, 0, sizeof(double) * cnts * cnts);
    memset(inv_H, 0, sizeof(double) * cnts * cnts);

    memset(inv_H_Y, 0, sizeof(double) * cnts);

    for (size_t i = 0; i < cnts; ++i)
    {
        if (i == 0)
        {
            H[0] = 2 * (times[1] - times[0]);
            H[1] = times[1] - times[0];

            //point factor calc
            Y[0] = 6 * ((data[1] - data[0]) / (times[1] - times[0]) - init);
        }
        else if (i == cnts - 1)
        {
            H[i * cnts + i - 1] = times[i] - times[i - 1];
            H[i * cnts + i - 0] = 2 * (times[i] - times[i - 1]);

            //matlab:Y(i) = 6 * (diff(p(i:i + 1)) / diff(tnew(i:i + 1)) - v0);
            //point factor calc
            Y[i] = 6 * (final - (data[i] - data[i - 1]) / (data[i] - data[i - 1]));
        }
        else
        {
            H[i * cnts + i - 1] = times[i] - times[i - 1];
            /*a[i] - a[i-1] + a[i+1] - a[i] --> a[i+1] - a[i-1]*/
            H[i * cnts + i - 0] = 2 * (times[i + 1] - times[i - 1]);
            H[i * cnts + i + 1] = times[i + 1] - times[i];
            //Y(i) = 6 * (diff(p(i:i + 1)) / diff(tnew(i:i + 1)) - diff(p(i - 1:i) / diff(tnew(i - 1:i))));
            Y[i] = 6 * ((data[i + 1] - data[i]) / (times[i + 1] - times[i]) - (data[i] - data[i - 1]) / (times[i] - times[i - 1]));
        }
    }
    if (!inverse(H, cnts, inv_H))
    {
        LogProducer::error("offline_planner", "inverse H failed");
        return false;
    }

    //m = inv(H) * Y;
    for (size_t i = 0; i < cnts; ++i)
    {
        for (size_t j = 0; j < cnts; ++j)
        {
            inv_H_Y[i] += inv_H[i * cnts + j] * Y[j];
        }
    }

    vector<double> fa0, fa1, fa2, fa3;

    for (size_t i = 0; i < cnts - 1; ++i)
    {
        fa0.push_back(data[i]);

        fa1.push_back(
            (data[i + 1] - data[i]) 
            / (times[i + 1] - times[i]) 
            - inv_H_Y[i] * (times[i + 1] - times[i]) / 2 
            - (inv_H_Y[i + 1] - inv_H_Y[i]) * (times[i + 1] - times[i]) / 6);

        fa2.push_back(inv_H_Y[i] / 2);

        fa3.push_back((inv_H_Y[i + 1] - inv_H_Y[i]) / (6 * (times[i + 1] - times[i])));
    }

    int index = 0;
    double tmp = 0;
    for (double t = times.front(); t <= times.back(); t += sampling_freq_)
    {
        for (int i = 0; ; ++i)
        {
            if (times[i] > t)
            {
                index = i - 1;
                break;
            }
        }
        tmp = fa3[index] * pow(t - times[index], 3)
            + fa2[index] * pow(t - times[index], 2)
            + fa1[index] * (t - times[index])
            + fa0[index];

        out.push_back(tmp);
    }

    delete[] Y;
    delete[] H;
    delete[] inv_H;
    delete[] inv_H_Y;

    return true;
}
