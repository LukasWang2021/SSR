/*************************************************************************
	> File Name: motion_plan_manual_teach.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年03月26日 星期一 09时17分06秒
 ************************************************************************/

#include <math.h>
#include <iostream>
#include <string.h>
#include <vector>

#include <motion_plan_manual_teach.h>
#include <motion_plan_variable.h>
#include <motion_plan_basic_function.h>
#include <motion_plan_reuse.h>

using namespace std;
using namespace fst_controller;
using namespace fst_algorithm;

ManualTeach::ManualTeach()
{
};

ManualTeach::~ManualTeach()
{};

void ManualTeach::setMode(const ManualMode mode)
{
    manual_mode_ = mode;
}

void ManualTeach::setFrame(const ManualFrame frame)
{
    manual_frame_ = frame;
}

void ManualTeach::setDirection(const ManualDirection *directions)
{
    manual_direction_[0] = directions[0];
    manual_direction_[1] = directions[1];
    manual_direction_[2] = directions[2];
    manual_direction_[3] = directions[3];
    manual_direction_[4] = directions[4];
    manual_direction_[5] = directions[5];
}

ErrorCode ManualTeach::stepTeach(void)
{
    if (manual_frame_ == JOINT)
    {
        return manualJoint();
    }
    else
    {
        return manualCartesian();
    }
}

ErrorCode ManualTeach::stepTeach(const ManualDirection *directions)
{
    setDirection(directions);
    return stepTeach();
}

ErrorCode ManualTeach::manualJoint(void)
{
    double speed_ratio  = g_manual_top_speed_ratio * g_manual_sub_speed_ratio;
    double acc_ratio    = g_manual_top_acc_ratio * g_manual_sub_acc_ratio;

    if (manual_mode_ == STEP)
    {
        FST_INFO("manualJoint (step): directions = %d %d %d %d %d %d, planning trajectory ...",
                 manual_direction_[0], manual_direction_[1], manual_direction_[2],
                 manual_direction_[3], manual_direction_[4], manual_direction_[5]);
        FST_INFO("  step_angle=%.2frad, speed_ratio=%.0f%%, acc_ratio=%.0f%%",
                 g_manual_step_joint, speed_ratio * 100, acc_ratio * 100);

        JointConstraint &cons = g_soft_constraint;
        double trips[6];
        double omega[6] = {cons.j1.max_omega, cons.j2.max_omega, cons.j3.max_omega,
                           cons.j4.max_omega, cons.j5.max_omega, cons.j6.max_omega};
        double alpha[6] = {cons.j1.max_alpha * 3, cons.j2.max_alpha * 3, cons.j3.max_alpha * 3,
                           cons.j4.max_alpha * 3, cons.j5.max_alpha * 3, cons.j6.max_alpha * 3};;
        double t_min[6];

        for (size_t i = 0; i < 6; i++)
        {
            trips[i] = manual_direction_[i] == STANDBY ? 0 : g_manual_step_joint;
            omega[i] = omega[i] * speed_ratio;
            alpha[i] = alpha[i] * acc_ratio;

            
            if (omega[i] * omega[i] / alpha[i] >= trips[i])
            {
                //FST_INFO("trip=%f, omega=%f, alpha=%f  w*w/a > trip", trips[i], omega[i], alpha[i]);
                t_min[i] = sqrt(trips[i] / alpha[i]) * 2;
            }
            else
            {
                //FST_INFO("trip=%f, omega=%f, alpha=%f  w*w/a < trip", trips[i], omega[i], alpha[i]);
                t_min[i] = omega[i] / alpha[i] + trips[i] / omega[i];
            }
        }

        double duration = t_min[0];
        duration = duration < t_min[1] ? t_min[1] : duration;
        duration = duration < t_min[2] ? t_min[2] : duration;
        duration = duration < t_min[3] ? t_min[3] : duration;
        duration = duration < t_min[4] ? t_min[4] : duration;
        duration = duration < t_min[5] ? t_min[5] : duration;

        //FST_INFO("t1=%f, t2=%f, t3=%f, t4=%f, t5=%f, t6=%f",\
                 t_min[0], t_min[1], t_min[2], t_min[3], t_min[4], t_min[5]);
        FST_INFO("  duration=%.4f, create trajectory", duration);

        double det;
        for (size_t i = 0; i < 6; i++)
        {
            det = duration * duration - trips[i] / alpha[i] * 4;
            if (det > MINIMUM_E9)
            {
                det = sqrt(det);
                g_manual_joint_coeff[i].duration_1 = (duration - det) / 2;
                g_manual_joint_coeff[i].duration_2 = g_manual_joint_coeff[i].duration_1 + det;
                g_manual_joint_coeff[i].duration_3 = duration;
                g_manual_joint_coeff[i].alpha_1 = manual_direction_[i] == STANDBY ? 0 : (manual_direction_[i] == INCREASE ? alpha[i] : -alpha[i]);
                g_manual_joint_coeff[i].alpha_3 = - g_manual_joint_coeff[i].alpha_1;
            }
            else
            {
                g_manual_joint_coeff[i].duration_1 = duration / 2;
                g_manual_joint_coeff[i].duration_2 = g_manual_joint_coeff[i].duration_1;
                g_manual_joint_coeff[i].duration_3 = duration;
                //g_manual_joint_coeff[i].alpha_1 = trips[i] * 4 / duration / duration;
                g_manual_joint_coeff[i].alpha_1 = manual_direction_[i] == STANDBY ? 0 : (manual_direction_[i] == INCREASE ? alpha[i] : -alpha[i]);
                g_manual_joint_coeff[i].alpha_3 = - g_manual_joint_coeff[i].alpha_1;
            }
            //FST_INFO("d1=%f, d2=%f, d3=%f", g_manual_joint_coeff[i].duration_1, g_manual_joint_coeff[i].duration_2, g_manual_joint_coeff[i].duration_3);
        }
        
    }

    FST_INFO("Success !");

    g_manual_mode = manual_mode_;
    g_manual_frame = manual_frame_;
    memcpy(g_manual_direction, manual_direction_, 6 * sizeof(ManualDirection));
    memcpy(&g_manual_joint_start, &g_start_joint, sizeof(Joint));

    return SUCCESS;
}

ErrorCode ManualTeach::manualCartesian(void)
{
    double speed_ratio  = g_manual_top_speed_ratio * g_manual_sub_speed_ratio;
    double acc_ratio    = g_manual_top_acc_ratio * g_manual_sub_acc_ratio;

    if (manual_mode_ == STEP)
    {
        FST_INFO("manualCartesian (step): directions = %d %d %d %d %d %d, planning trajectory ...",
                 manual_direction_[0], manual_direction_[1], manual_direction_[2],
                 manual_direction_[3], manual_direction_[4], manual_direction_[5]);
        FST_INFO("  step_length=%.2fmm step_angle=%.2frad, speed_ratio=%f%%, acc_ratio=%f%%",
                 g_manual_step_position, g_manual_step_orientation, speed_ratio * 100, acc_ratio * 100);
        
        double trips[6];
        double spd = g_cart_vel_reference * speed_ratio;
        double acc = g_cart_acc_reference * acc_ratio;
        double omega = g_orientation_omega_reference * speed_ratio;
        double alpha = g_orientation_alpha_reference * acc_ratio;


    }


    return SUCCESS;
}



