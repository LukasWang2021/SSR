#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>
#include <sstream>
#include "given_vel_planner.h"

using namespace std;
using namespace basic_alg;

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

int32_t GivenVelocityPlanner::viaPoints2Traj(vector<PoseEuler> via_points)
{
    input_via_points_.clear();
    via_points_quat_.clear();
    via_points_dist_.clear();
    via_points_time_new_.clear();
    resampled_traj_.clear();
    // 1. transfer abc to quaternion and normalize it
    input_via_points_ = via_points;
    for (auto iter = input_via_points_.begin(); iter != input_via_points_.end(); ++iter)
    {
        Quaternion tmp_quat;
        tmp_quat = Euler2Quaternion(iter->euler_);
        normalizeQuaternion(tmp_quat);
        via_points_quat_.push_back(tmp_quat);
        ++via_points_cnt_;
    }
    // if the number of via point more than 10 do 2 and 3 else not
    // 2. smooth xyz
    // 3. smooth abc
    if (via_points_cnt_ >= 10 && (!xyzFitSmooth(xyz_fit_rate_) || !abcQuatSmooth(abc_smooth_window_)))
    {
        return -1;
    }
    // 4. calculate the path info
    pathInfoCalc();
    trajPlanWithPathInfo();
    // trajTimeResampling();
    // 5. resampling xyz trajectory with 0.001s
    xyzTrajResampling();
    // 6. resampling abc trajectory with 0.001s
    abcTrajResampling();

    return 0;
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

    for (int32_t idx = 0; idx < via_points_quat_.size(); ++idx)
    {
        if (idx <= ((smooth_window - 1) / 2 - 1) || idx >= (via_points_quat_.size() - (smooth_window - 1) / 2))
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
        for (int32_t i = 0; i < mean_quat.size(); ++i)
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
    int32_t point_cnt = input_via_points_.size();
    double distance = 0;
    via_points_dist_.push_back(distance);
    for (int cnt = 0; cnt < point_cnt - 1; ++cnt)
    {
        distance += getDistance(input_via_points_[cnt+1].point_, input_via_points_[cnt].point_);
        via_points_dist_.push_back(distance);
    }
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


//bool  GivenVelocityPlanner::trajTimeResampling(void)
//{
//    PoseEuler pos;
//    for (double t = 0; t <= traj_time_set_; t += sampling_freq_)
//    {
//        resampled_traj_.push_back(pos);
//    }
//    return true;
//}

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
    if (!xyzSpline()) return false;

    return true;
}

bool GivenVelocityPlanner::xyzSpline(void)
{
    double v_start = 0.0, v_end = 0.0;
    double *px_Y = new double[via_points_cnt_];
    double *py_Y = new double[via_points_cnt_];
    double *pz_Y = new double[via_points_cnt_];
    double *H = new double[via_points_cnt_ * via_points_cnt_];
    double *inv_H = new double[via_points_cnt_ * via_points_cnt_];
    double *inv_H_px_Y = new double[via_points_cnt_];
    double* inv_H_py_Y = new double[via_points_cnt_];
    double* inv_H_pz_Y = new double[via_points_cnt_];

    memset(px_Y, 0, sizeof(double) * via_points_cnt_);
    memset(py_Y, 0, sizeof(double) * via_points_cnt_);
    memset(pz_Y, 0, sizeof(double) * via_points_cnt_);
    memset(H, 0, sizeof(double)* via_points_cnt_* via_points_cnt_);
    memset(inv_H, 0, sizeof(double) * via_points_cnt_ * via_points_cnt_);
    memset(inv_H_px_Y, 0, sizeof(double) * via_points_cnt_);
    memset(inv_H_py_Y, 0, sizeof(double) * via_points_cnt_);
    memset(inv_H_pz_Y, 0, sizeof(double) * via_points_cnt_);

    for (int i = 0; i < via_points_cnt_; ++i)
    {
        if (i == 0)
        {
            H[0] = 2 * (via_points_time_new_[1] - via_points_time_new_[0]);
            H[1] = via_points_time_new_[1] - via_points_time_new_[0];

            px_Y[0] = 6 * (
                (input_via_points_[1].point_.x_ - input_via_points_[0].point_.x_) 
                / (via_points_time_new_[1] - via_points_time_new_[0]) - v_start
                );
            
            py_Y[0] = 6 * (
                (input_via_points_[1].point_.y_ - input_via_points_[0].point_.y_) 
                / (via_points_time_new_[1] - via_points_time_new_[0]) - v_start
                );

            pz_Y[0] = 6 * (
                (input_via_points_[1].point_.z_ - input_via_points_[0].point_.z_) 
                / (via_points_time_new_[1] - via_points_time_new_[0]) - v_start
                );
        }
        else if (i == via_points_cnt_ - 1)
        {
            H[i * via_points_cnt_ + i - 1] = via_points_time_new_[i] - via_points_time_new_[i - 1];
            H[i * via_points_cnt_ + i - 0] = 2 * (via_points_time_new_[i] - via_points_time_new_[i - 1]);
            
            //Y(i) = 6 * (diff(p(i:i + 1)) / diff(tnew(i:i + 1)) - v0);
            px_Y[i] = 6 * (v_end 
                - (input_via_points_[i].point_.x_ - input_via_points_[i - 1].point_.x_) 
                / (via_points_time_new_[i] - via_points_time_new_[i - 1])
                );

            py_Y[i] = 6 * (v_end 
                - (input_via_points_[i].point_.y_ - input_via_points_[i - 1].point_.y_) 
                / (via_points_time_new_[i] - via_points_time_new_[i - 1])
                );

            pz_Y[i] = 6 * (v_end 
                - (input_via_points_[i].point_.z_ - input_via_points_[i - 1].point_.z_) 
                / (via_points_time_new_[i] - via_points_time_new_[i - 1])
                );
        }
        else
        {
            H[i * via_points_cnt_ + i - 1] = via_points_time_new_[i] - via_points_time_new_[i - 1];
            /*a[i] - a[i-1] + a[i+1] - a[i] --> a[i+1] - a[i-1]*/
            H[i * via_points_cnt_ + i - 0] = 2 * (via_points_time_new_[i + 1] - via_points_time_new_[i - 1]);
            H[i * via_points_cnt_ + i + 1] = via_points_time_new_[i + 1] - via_points_time_new_[i];
            //Y(i) = 6 * (diff(p(i:i + 1)) / diff(tnew(i:i + 1)) - diff(p(i - 1:i) / diff(tnew(i - 1:i))));
            px_Y[i] = 6 * (
                (input_via_points_[i + 1].point_.x_ - input_via_points_[i].point_.x_) 
                / (via_points_time_new_[i + 1] - via_points_time_new_[i]) 
                - (input_via_points_[i].point_.x_ - input_via_points_[i - 1].point_.x_) 
                / (via_points_time_new_[i] - via_points_time_new_[i - 1])
                );

            py_Y[i] = 6 * (
                (input_via_points_[i + 1].point_.y_ - input_via_points_[i].point_.y_) 
                / (via_points_time_new_[i + 1] - via_points_time_new_[i]) 
                - (input_via_points_[i].point_.y_ - input_via_points_[i - 1].point_.y_) 
                / (via_points_time_new_[i] - via_points_time_new_[i - 1])
                );

            pz_Y[i] = 6 * (
                (input_via_points_[i + 1].point_.z_ - input_via_points_[i].point_.z_) 
                / (via_points_time_new_[i + 1] - via_points_time_new_[i]) 
                - (input_via_points_[i].point_.z_ - input_via_points_[i - 1].point_.z_) 
                / (via_points_time_new_[i] - via_points_time_new_[i - 1])
                );
        }
    }
    if (!inverse(H, via_points_cnt_, inv_H))
    {
        return false;
    }

    //m = inv(H) * Y;
    for (int i = 0; i < via_points_cnt_; ++i)
    {
        for (int j = 0; j < via_points_cnt_; ++j)
        {
            inv_H_px_Y[i] += inv_H[i * via_points_cnt_ + j] * px_Y[j];
            inv_H_py_Y[i] += inv_H[i * via_points_cnt_ + j] * py_Y[j];
            inv_H_pz_Y[i] += inv_H[i * via_points_cnt_ + j] * pz_Y[j];
        }
    }

    vector<double> fa0_x, fa0_y, fa0_z;
    vector<double> fa1_x, fa1_y, fa1_z;
    vector<double> fa2_x, fa2_y, fa2_z;
    vector<double> fa3_x, fa3_y, fa3_z;

    for (int i = 0; i < via_points_cnt_ - 1; ++i)
    {
        fa0_x.push_back(input_via_points_[i].point_.x_);
        fa0_y.push_back(input_via_points_[i].point_.y_);
        fa0_z.push_back(input_via_points_[i].point_.z_);

        fa1_x.push_back((input_via_points_[i + 1].point_.x_ - input_via_points_[i].point_.x_) 
            / (via_points_time_new_[i + 1] - via_points_time_new_[i])
            - inv_H_px_Y[i] * (via_points_time_new_[i + 1] - via_points_time_new_[i]) / 2 
            - (inv_H_px_Y[i + 1] - inv_H_px_Y[i]) * (via_points_time_new_[i + 1] - via_points_time_new_[i]) / 6);

        fa1_y.push_back((input_via_points_[i + 1].point_.y_ - input_via_points_[i].point_.y_)
            / (via_points_time_new_[i + 1] - via_points_time_new_[i])
            - inv_H_py_Y[i] * (via_points_time_new_[i + 1] - via_points_time_new_[i]) / 2
            - (inv_H_py_Y[i + 1] - inv_H_py_Y[i]) * (via_points_time_new_[i + 1] - via_points_time_new_[i]) / 6);

        fa1_z.push_back((input_via_points_[i + 1].point_.z_ - input_via_points_[i].point_.z_)
            / (via_points_time_new_[i + 1] - via_points_time_new_[i])
            - inv_H_pz_Y[i] * (via_points_time_new_[i + 1] - via_points_time_new_[i]) / 2
            - (inv_H_pz_Y[i + 1] - inv_H_pz_Y[i]) * (via_points_time_new_[i + 1] - via_points_time_new_[i]) / 6);

        fa2_x.push_back(inv_H_px_Y[i] / 2);
        fa2_y.push_back(inv_H_py_Y[i] / 2);
        fa2_z.push_back(inv_H_pz_Y[i] / 2);

        fa3_x.push_back((inv_H_px_Y[i + 1] - inv_H_px_Y[i]) 
            / (6 * (via_points_time_new_[i + 1] - via_points_time_new_[i])));

        fa3_y.push_back((inv_H_py_Y[i + 1] - inv_H_py_Y[i])
            / (6 * (via_points_time_new_[i + 1] - via_points_time_new_[i])));

        fa3_z.push_back((inv_H_pz_Y[i + 1] - inv_H_pz_Y[i])
            / (6 * (via_points_time_new_[i + 1] - via_points_time_new_[i])));
    }
    int index = 0;
    PoseEuler tmp_pos;
    for (double t = 0; t <= traj_time_set_; t += sampling_freq_)
    {
        for (int i = 0; ; ++i)
        {
            if (via_points_time_new_[i] > t)
            {
                index = i - 1;
                break;
            }
        }
        tmp_pos.point_.x_ = fa3_x[index] * pow(t - via_points_time_new_[index], 3)
            + fa2_x[index] * pow(t - via_points_time_new_[index], 2)
            + fa1_x[index] * (t - via_points_time_new_[index])
            + fa0_x[index];

        tmp_pos.point_.y_ = fa3_y[index] * pow(t - via_points_time_new_[index], 3)
            + fa2_y[index] * pow(t - via_points_time_new_[index], 2)
            + fa1_y[index] * (t - via_points_time_new_[index])
            + fa0_y[index];

        tmp_pos.point_.z_ = fa3_z[index] * pow(t - via_points_time_new_[index], 3)
            + fa2_z[index] * pow(t - via_points_time_new_[index], 2)
            + fa1_z[index] * (t - via_points_time_new_[index])
            + fa0_z[index];

        resampled_traj_.push_back(tmp_pos);
    }

    delete[] px_Y;
    delete[] py_Y;
    delete[] pz_Y;
    delete[] H;
    delete[] inv_H;
    delete[] inv_H_px_Y;
    delete[] inv_H_py_Y;
    delete[] inv_H_pz_Y;

    return true;
}


bool GivenVelocityPlanner::abcTrajResampling(void)
{
    double dq = 0.0;
    vector<Quaternion> resampled_quat;
    vector<Quaternion> tmp_quat_vec_ = via_points_quat_;
    for (int i = 0; i < tmp_quat_vec_.size() - 1; ++i)
    {
        dq = tmp_quat_vec_[i].w_ * tmp_quat_vec_[i + 1].w_
            + tmp_quat_vec_[i].x_ * tmp_quat_vec_[i + 1].x_
            + tmp_quat_vec_[i].y_ * tmp_quat_vec_[i + 1].y_
            + tmp_quat_vec_[i].z_ * tmp_quat_vec_[i + 1].z_;

        if (dq < 0)
        {
            tmp_quat_vec_[i + 1].w_ = -tmp_quat_vec_[i + 1].w_;
            tmp_quat_vec_[i + 1].x_ = -tmp_quat_vec_[i + 1].x_;
            tmp_quat_vec_[i + 1].y_ = -tmp_quat_vec_[i + 1].y_;
            tmp_quat_vec_[i + 1].z_ = -tmp_quat_vec_[i + 1].z_;
        }
    }
    via_points_quat_ = tmp_quat_vec_;
    if (!abcSpline()) return false;

    return true;
}

bool GivenVelocityPlanner::abcSpline(void)
{
    double v_start = 0.0, v_end = 0.0;
    double* qw_Y = new double[via_points_cnt_];
    double* qx_Y = new double[via_points_cnt_];
    double* qy_Y = new double[via_points_cnt_];
    double* qz_Y = new double[via_points_cnt_];
    double* H = new double[via_points_cnt_ * via_points_cnt_];
    double* inv_H = new double[via_points_cnt_ * via_points_cnt_];
    double* inv_H_pw_Y = new double[via_points_cnt_];
    double* inv_H_px_Y = new double[via_points_cnt_];
    double* inv_H_py_Y = new double[via_points_cnt_];
    double* inv_H_pz_Y = new double[via_points_cnt_];

    memset(qw_Y, 0, sizeof(double) * via_points_cnt_);
    memset(qx_Y, 0, sizeof(double) * via_points_cnt_);
    memset(qy_Y, 0, sizeof(double) * via_points_cnt_);
    memset(qz_Y, 0, sizeof(double) * via_points_cnt_);
    memset(H, 0, sizeof(double) * via_points_cnt_ * via_points_cnt_);
    memset(inv_H, 0, sizeof(double) * via_points_cnt_ * via_points_cnt_);
    memset(inv_H_pw_Y, 0, sizeof(double) * via_points_cnt_);
    memset(inv_H_px_Y, 0, sizeof(double) * via_points_cnt_);
    memset(inv_H_py_Y, 0, sizeof(double) * via_points_cnt_);
    memset(inv_H_pz_Y, 0, sizeof(double) * via_points_cnt_);

    for (int i = 0; i < via_points_cnt_; ++i)
    {
        if (i == 0)
        {
            H[0] = 2 * (via_points_time_new_[1] - via_points_time_new_[0]);
            H[1] = via_points_time_new_[1] - via_points_time_new_[0];

            qw_Y[0] = 6 * (
                (via_points_quat_[1].w_ - via_points_quat_[0].w_)
                / (via_points_time_new_[1] - via_points_time_new_[0]) - v_start
                );

            qx_Y[0] = 6 * (
                (via_points_quat_[1].x_ - via_points_quat_[0].x_)
                / (via_points_time_new_[1] - via_points_time_new_[0]) - v_start
                );

            qy_Y[0] = 6 * (
                (via_points_quat_[1].y_ - via_points_quat_[0].y_)
                / (via_points_time_new_[1] - via_points_time_new_[0]) - v_start
                );

            qz_Y[0] = 6 * (
                (via_points_quat_[1].z_ - via_points_quat_[0].z_)
                / (via_points_time_new_[1] - via_points_time_new_[0]) - v_start
                );
        }
        else if (i == via_points_cnt_ - 1)
        {
            H[i * via_points_cnt_ + i - 1] = via_points_time_new_[i] - via_points_time_new_[i - 1];
            H[i * via_points_cnt_ + i - 0] = 2 * (via_points_time_new_[i] - via_points_time_new_[i - 1]);

            //H[via_points_cnt_ * via_points_cnt_ - 1] = via_points_time_new_[i + 1] - via_points_time_new_[i];
            //H[via_points_cnt_ * via_points_cnt_] = 2 * (via_points_time_new_[i + 1] - via_points_time_new_[i]);

            //Y(i) = 6 * (diff(p(i:i + 1)) / diff(tnew(i:i + 1)) - v0);
            qw_Y[i] = 6 * (v_end
                - (via_points_quat_[i].w_ - via_points_quat_[i - 1].w_)
                / (via_points_time_new_[i] - via_points_time_new_[i - 1])
                );

            qx_Y[i] = 6 * (v_end
                - (via_points_quat_[i].x_ - via_points_quat_[i - 1].x_)
                / (via_points_time_new_[i] - via_points_time_new_[i - 1])
                );

            qy_Y[i] = 6 * (v_end
                - (via_points_quat_[i].y_ - via_points_quat_[i - 1].y_)
                / (via_points_time_new_[i] - via_points_time_new_[i - 1])
                );

            qz_Y[i] = 6 * (v_end
                - (via_points_quat_[i].z_ - via_points_quat_[i - 1].z_)
                / (via_points_time_new_[i] - via_points_time_new_[i - 1])
                );
        }
        else
        {
            H[i * via_points_cnt_ + i - 1] = via_points_time_new_[i] - via_points_time_new_[i - 1];
            /*a[i] - a[i-1] + a[i+1] - a[i] --> a[i+1] - a[i-1]*/
            H[i * via_points_cnt_ + i - 0] = 2 * (via_points_time_new_[i + 1] - via_points_time_new_[i - 1]);
            H[i * via_points_cnt_ + i + 1] = via_points_time_new_[i + 1] - via_points_time_new_[i];
            //Y(i) = 6 * (diff(p(i:i + 1)) / diff(tnew(i:i + 1)) - diff(p(i - 1:i) / diff(tnew(i - 1:i))));
            qw_Y[i] = 6 * (
                (via_points_quat_[i + 1].w_ - via_points_quat_[i].w_)
                / (via_points_time_new_[i + 1] - via_points_time_new_[i])
                - (via_points_quat_[i].w_ - via_points_quat_[i - 1].w_)
                / (via_points_time_new_[i] - via_points_time_new_[i - 1])
                );
            qx_Y[i] = 6 * (
                (via_points_quat_[i + 1].x_ - via_points_quat_[i].x_)
                / (via_points_time_new_[i + 1] - via_points_time_new_[i])
                - (via_points_quat_[i].x_ - via_points_quat_[i - 1].x_)
                / (via_points_time_new_[i] - via_points_time_new_[i - 1])
                );

            qy_Y[i] = 6 * (
                (via_points_quat_[i + 1].y_ - via_points_quat_[i].y_)
                / (via_points_time_new_[i + 1] - via_points_time_new_[i])
                - (via_points_quat_[i].y_ - via_points_quat_[i - 1].y_)
                / (via_points_time_new_[i] - via_points_time_new_[i - 1])
                );

            qz_Y[i] = 6 * (
                (via_points_quat_[i + 1].z_ - via_points_quat_[i].z_)
                / (via_points_time_new_[i + 1] - via_points_time_new_[i])
                - (via_points_quat_[i].z_ - via_points_quat_[i - 1].z_)
                / (via_points_time_new_[i] - via_points_time_new_[i - 1])
                );
        }
    }
    if (!inverse(H, via_points_cnt_, inv_H))
    {
        return false;
    }

    //m = inv(H) * Y;
    for (int i = 0; i < via_points_cnt_; ++i)
    {
        for (int j = 0; j < via_points_cnt_; ++j)
        {
            inv_H_pw_Y[i] += inv_H[i * via_points_cnt_ + j] * qw_Y[j];
            inv_H_px_Y[i] += inv_H[i * via_points_cnt_ + j] * qx_Y[j];
            inv_H_py_Y[i] += inv_H[i * via_points_cnt_ + j] * qy_Y[j];
            inv_H_pz_Y[i] += inv_H[i * via_points_cnt_ + j] * qz_Y[j];
        }
    }

    vector<double> fa0_w, fa0_x, fa0_y, fa0_z;
    vector<double> fa1_w, fa1_x, fa1_y, fa1_z;
    vector<double> fa2_w, fa2_x, fa2_y, fa2_z;
    vector<double> fa3_w, fa3_x, fa3_y, fa3_z;

    for (int i = 0; i < via_points_cnt_ - 1; ++i)
    {
        fa0_w.push_back(via_points_quat_[i].w_);
        fa0_x.push_back(via_points_quat_[i].x_);
        fa0_y.push_back(via_points_quat_[i].y_);
        fa0_z.push_back(via_points_quat_[i].z_);

        fa1_w.push_back((via_points_quat_[i + 1].w_ - via_points_quat_[i].w_)
            / (via_points_time_new_[i + 1] - via_points_time_new_[i])
            - inv_H_pw_Y[i] * (via_points_time_new_[i + 1] - via_points_time_new_[i]) / 2
            - (inv_H_pw_Y[i + 1] - inv_H_pw_Y[i]) * (via_points_time_new_[i + 1] - via_points_time_new_[i]) / 6);

        fa1_x.push_back((via_points_quat_[i + 1].x_ - via_points_quat_[i].x_)
            / (via_points_time_new_[i + 1] - via_points_time_new_[i])
            - inv_H_px_Y[i] * (via_points_time_new_[i + 1] - via_points_time_new_[i]) / 2
            - (inv_H_px_Y[i + 1] - inv_H_px_Y[i]) * (via_points_time_new_[i + 1] - via_points_time_new_[i]) / 6);

        fa1_y.push_back((via_points_quat_[i + 1].y_ - via_points_quat_[i].y_)
            / (via_points_time_new_[i + 1] - via_points_time_new_[i])
            - inv_H_py_Y[i] * (via_points_time_new_[i + 1] - via_points_time_new_[i]) / 2
            - (inv_H_py_Y[i + 1] - inv_H_py_Y[i]) * (via_points_time_new_[i + 1] - via_points_time_new_[i]) / 6);

        fa1_z.push_back((via_points_quat_[i + 1].z_ - via_points_quat_[i].z_)
            / (via_points_time_new_[i + 1] - via_points_time_new_[i])
            - inv_H_pz_Y[i] * (via_points_time_new_[i + 1] - via_points_time_new_[i]) / 2
            - (inv_H_pz_Y[i + 1] - inv_H_pz_Y[i]) * (via_points_time_new_[i + 1] - via_points_time_new_[i]) / 6);

        fa2_w.push_back(inv_H_pw_Y[i] / 2);
        fa2_x.push_back(inv_H_px_Y[i] / 2);
        fa2_y.push_back(inv_H_py_Y[i] / 2);
        fa2_z.push_back(inv_H_pz_Y[i] / 2);

        fa3_w.push_back((inv_H_pw_Y[i + 1] - inv_H_pw_Y[i])
            / (6 * (via_points_time_new_[i + 1] - via_points_time_new_[i])));

        fa3_x.push_back((inv_H_px_Y[i + 1] - inv_H_px_Y[i])
            / (6 * (via_points_time_new_[i + 1] - via_points_time_new_[i])));

        fa3_y.push_back((inv_H_py_Y[i + 1] - inv_H_py_Y[i])
            / (6 * (via_points_time_new_[i + 1] - via_points_time_new_[i])));

        fa3_z.push_back((inv_H_pz_Y[i + 1] - inv_H_pz_Y[i])
            / (6 * (via_points_time_new_[i + 1] - via_points_time_new_[i])));
    }
    int index = 0;
    Quaternion tmp_quat;
    int traj_index = 0;
    for (double t = 0; t <= traj_time_set_; t += sampling_freq_, ++traj_index)
    {
        for (int i = 0; ; ++i)
        {
            if (via_points_time_new_[i] > t)
            {
                index = i - 1;
                break;
            }
        }
        tmp_quat.w_ = fa3_w[index] * pow(t - via_points_time_new_[index], 3)
            + fa2_w[index] * pow(t - via_points_time_new_[index], 2)
            + fa1_w[index] * (t - via_points_time_new_[index])
            + fa0_w[index];

        tmp_quat.x_ = fa3_x[index] * pow(t - via_points_time_new_[index], 3)
            + fa2_x[index] * pow(t - via_points_time_new_[index], 2)
            + fa1_x[index] * (t - via_points_time_new_[index])
            + fa0_x[index];

        tmp_quat.y_ = fa3_y[index] * pow(t - via_points_time_new_[index], 3)
            + fa2_y[index] * pow(t - via_points_time_new_[index], 2)
            + fa1_y[index] * (t - via_points_time_new_[index])
            + fa0_y[index];

        tmp_quat.z_ = fa3_z[index] * pow(t - via_points_time_new_[index], 3)
            + fa2_z[index] * pow(t - via_points_time_new_[index], 2)
            + fa1_z[index] * (t - via_points_time_new_[index])
            + fa0_z[index];

        resampled_traj_[traj_index].euler_ = Quaternion2Euler(tmp_quat);
    }

    delete[] qw_Y;
    delete[] qx_Y;
    delete[] qy_Y;
    delete[] qz_Y;
    delete[] H;
    delete[] inv_H;
    delete[] inv_H_pw_Y;
    delete[] inv_H_px_Y;
    delete[] inv_H_py_Y;
    delete[] inv_H_pz_Y;

    return true;
}

vector<Quaternion> GivenVelocityPlanner::testQuatSmooth(const vector<Quaternion>& quat_in)
{
    via_points_quat_ = quat_in;
    abcQuatSmooth(abc_smooth_window_);
    return via_points_quat_;
}

vector<PoseEuler> GivenVelocityPlanner::testFitSmooth(const vector<PoseEuler>& pos_in)
{
    return input_via_points_;
}
