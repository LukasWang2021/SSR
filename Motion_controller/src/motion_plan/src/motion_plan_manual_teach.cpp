/*************************************************************************
	> File Name: motion_plan_manual_teach.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年03月26日 星期一 09时17分06秒
 ************************************************************************/

#include <math.h>
#include <float.h>
#include <iostream>
#include <string.h>
#include <vector>

#include <motion_plan_manual_teach.h>
#include <motion_plan_variable.h>
#include <motion_plan_basic_function.h>
#include <motion_plan_kinematics.h>
#include <motion_plan_reuse.h>

using namespace std;
using namespace fst_controller;
using namespace fst_algorithm;

ManualTeach::ManualTeach()
{
};

ManualTeach::~ManualTeach()
{};

ManualMode ManualTeach::getMode(void)
{
    return manual_mode_;
}

ManualFrame ManualTeach::getFrame(void)
{
    return manual_frame_;
}

void ManualTeach::setMode(const ManualMode mode)
{
    manual_mode_ = mode;
}

void ManualTeach::setFrame(const ManualFrame frame)
{
    manual_frame_ = frame;
}

void ManualTeach::setManualDirection(const ManualDirection *directions)
{
    manual_direction_[0] = directions[0];
    manual_direction_[1] = directions[1];
    manual_direction_[2] = directions[2];
    manual_direction_[3] = directions[3];
    manual_direction_[4] = directions[4];
    manual_direction_[5] = directions[5];
}

void ManualTeach::setManualTarget(const Joint &target)
{
    manual_target_joint_ = target;
}

ErrorCode ManualTeach::stepTeach(void)
{
    FST_INFO("Manual request received, mode=%d, frame=%d", manual_mode_, manual_frame_);

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
    setManualDirection(directions);
    return stepTeach();
}

ErrorCode ManualTeach::stepTeach(const Joint &target)
{
    setManualTarget(target);
    return stepTeach();
}

ErrorCode ManualTeach::stopTeach(MotionTime time)
{
    time += g_cycle_time * 50;

    if (manual_mode_ == CONTINUOUS)
    {
        if (manual_frame_ == JOINT)
        {
            return stopJointContinuous(time);
        }
        else
        {
            return stopCartesianContinuous(time);
        }
    }
    else if (manual_mode_ == APOINT)
    {
        // TODO
    }
    else
    {
        return SUCCESS;
    }
}

ErrorCode ManualTeach::manualJoint(void)
{
    ErrorCode err;
    
    if (manual_mode_ == STEP)
    {
        err = manualJointStep();
    }
    else if (manual_mode_ == CONTINUOUS)
    {
        err = manualJointContinuous();
    }
    else
    {
        err = manualJointAPoint();
    }

    if (err == SUCCESS)
    {
        g_manual_mode = manual_mode_;
        g_manual_frame = manual_frame_;
        memcpy(&g_manual_joint_start, &g_start_joint, sizeof(Joint));
    }

    return err;
}

ErrorCode ManualTeach::manualJointStep(void)
{
    double speed_ratio  = g_global_vel_ratio;
    double acc_ratio    = g_global_acc_ratio;

    FST_INFO("manual-Joint-Step: directions = %d %d %d %d %d %d, planning trajectory ...",
             manual_direction_[0], manual_direction_[1], manual_direction_[2],
             manual_direction_[3], manual_direction_[4], manual_direction_[5]);
    FST_INFO("  step_angle=%.2frad, speed_ratio=%.0f%%, acc_ratio=%.0f%%",
             g_manual_step_joint, speed_ratio * 100, acc_ratio * 100);
    FST_INFO("  start  joint=%.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
             g_start_joint.j1, g_start_joint.j2, g_start_joint.j3, 
             g_start_joint.j4, g_start_joint.j5, g_start_joint.j6);

    Joint target = g_start_joint;
    JointConstraint &cons = g_soft_constraint;

    if      (manual_direction_[0] == INCREASE) target.j1 += g_manual_step_joint;
    else if (manual_direction_[0] == DECREASE) target.j1 -= g_manual_step_joint;
    if      (manual_direction_[1] == INCREASE) target.j2 += g_manual_step_joint;
    else if (manual_direction_[1] == DECREASE) target.j2 -= g_manual_step_joint;
    if      (manual_direction_[2] == INCREASE) target.j3 += g_manual_step_joint;
    else if (manual_direction_[2] == DECREASE) target.j3 -= g_manual_step_joint;
    if      (manual_direction_[3] == INCREASE) target.j4 += g_manual_step_joint;
    else if (manual_direction_[3] == DECREASE) target.j4 -= g_manual_step_joint;
    if      (manual_direction_[4] == INCREASE) target.j5 += g_manual_step_joint;
    else if (manual_direction_[4] == DECREASE) target.j5 -= g_manual_step_joint;
    if      (manual_direction_[5] == INCREASE) target.j6 += g_manual_step_joint;
    else if (manual_direction_[5] == DECREASE) target.j7 -= g_manual_step_joint;

    FST_INFO("  target joint=%.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
             target.j1, target.j2, target.j3, 
             target.j4, target.j5, target.j6);

    if (!isJointInConstraint(target, cons))
    {
        FST_ERROR("Target out of soft constraint.");
        return TARGET_OUT_OF_CONSTRAINT;
    }

    double trips[6];
    double omega[6] = {cons.j1.max_omega, cons.j2.max_omega, cons.j3.max_omega,
                       cons.j4.max_omega, cons.j5.max_omega, cons.j6.max_omega};
    double alpha[6] = {cons.j1.max_alpha * 3, cons.j2.max_alpha * 3, cons.j3.max_alpha * 3,
                       cons.j4.max_alpha * 3, cons.j5.max_alpha * 3, cons.j6.max_alpha * 3};;
    double t_min[6];
    double delta[6];
    

    for (size_t i = 0; i < 6; i++)
    {
        trips[i] = manual_direction_[i] == STANDBY ? 0 : g_manual_step_joint;
        omega[i] = omega[i] * speed_ratio;
        alpha[i] = alpha[i] * acc_ratio;
        
        delta[i] = trips[i] - omega[i] * omega[i] / alpha[i];
        t_min[i] = delta[i] > 0 ? (omega[i] / alpha[i] + trips[i] / omega[i]) : (sqrt(trips[i] / alpha[i]) * 2);
        FST_INFO("  J%d: trip=%.4f omega=%f, alpha=%f, delta=%f, tmin=%f", i + 1, trips[i], omega[i], alpha[i], delta[i], t_min[i]);
    }

    double duration = t_min[0];
    duration = duration < t_min[1] ? t_min[1] : duration;
    duration = duration < t_min[2] ? t_min[2] : duration;
    duration = duration < t_min[3] ? t_min[3] : duration;
    duration = duration < t_min[4] ? t_min[4] : duration;
    duration = duration < t_min[5] ? t_min[5] : duration;

    FST_INFO("  t1=%f, t2=%f, t3=%f, t4=%f, t5=%f, t6=%f",\
             t_min[0], t_min[1], t_min[2], t_min[3], t_min[4], t_min[5]);
    FST_INFO("  duration=%.4f, create trajectory", duration);

    for (size_t i = 0; i < 6; i++)
    {
        if (manual_direction_[i] == STANDBY)
        {
            g_manual_joint_coeff[i].duration_1 = 0;
            g_manual_joint_coeff[i].duration_2 = duration;
            g_manual_joint_coeff[i].duration_3 = duration;
            g_manual_joint_coeff[i].alpha_1 = 0;
            g_manual_joint_coeff[i].alpha_3 = 0;
        }
        else
        {
            if (duration < t_min[i] + MINIMUM_E12)
            {
                g_manual_joint_coeff[i].duration_1 = delta[i] > 0 ? omega[i] / alpha[i] : duration / 2;
                g_manual_joint_coeff[i].duration_2 = delta[i] > 0 ? duration - omega[i] / alpha[i] : duration / 2;
                g_manual_joint_coeff[i].duration_3 = duration;
            }
            else
            {
                // replan traj using duration
                double t2 = sqrt(duration * duration - trips[i] / alpha[i] * 4);
                g_manual_joint_coeff[i].duration_1 = (duration - t2) / 2;
                g_manual_joint_coeff[i].duration_2 = (duration + t2) / 2;
                g_manual_joint_coeff[i].duration_3 = duration;
            }

            g_manual_joint_coeff[i].alpha_1 = manual_direction_[i] == INCREASE ? alpha[i] : -alpha[i];
            g_manual_joint_coeff[i].alpha_3 = -g_manual_joint_coeff[i].alpha_1;
        }
        FST_INFO("  J%d: d1=%f, d2=%f, d3=%f, alpha1=%f, alpha3=%f", i + 1, g_manual_joint_coeff[i].duration_1,
                 g_manual_joint_coeff[i].duration_2, g_manual_joint_coeff[i].duration_3,
                 g_manual_joint_coeff[i].alpha_1, g_manual_joint_coeff[i].alpha_3);
    }

    memcpy(g_manual_direction, manual_direction_, 6 * sizeof(ManualDirection));
    FST_INFO("Success !");
    return SUCCESS;
}

ErrorCode ManualTeach::manualJointContinuous(void)
{
    double speed_ratio  = g_global_vel_ratio;
    double acc_ratio    = g_global_acc_ratio;

    FST_INFO("manual-Joint-Continuous: directions = %d %d %d %d %d %d, planning trajectory ...",
             manual_direction_[0], manual_direction_[1], manual_direction_[2],
             manual_direction_[3], manual_direction_[4], manual_direction_[5]);
    FST_INFO("  speed_ratio=%.0f%%, acc_ratio=%.0f%%", speed_ratio * 100, acc_ratio * 100);
    FST_INFO("  start joint=%.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
             g_start_joint.j1, g_start_joint.j2, g_start_joint.j3, 
             g_start_joint.j4, g_start_joint.j5, g_start_joint.j6);

    JointConstraint &cons = g_soft_constraint;

    // TODO
    // check soft constraint

    double omega[6] = {cons.j1.max_omega * speed_ratio, cons.j2.max_omega * speed_ratio,
                       cons.j3.max_omega * speed_ratio, cons.j4.max_omega * speed_ratio,
                       cons.j5.max_omega * speed_ratio, cons.j6.max_omega * speed_ratio};
    double alpha[6] = {cons.j1.max_alpha * 3 * acc_ratio, cons.j2.max_alpha * 3 * acc_ratio,
                       cons.j3.max_alpha * 3 * acc_ratio, cons.j4.max_alpha * 3 * acc_ratio,
                       cons.j5.max_alpha * 3 * acc_ratio, cons.j6.max_alpha * 3 * acc_ratio};

    for (size_t i = 0; i < 6; i++)
    {
        if (manual_direction_[i] == STANDBY)
        {
            g_manual_joint_coeff[i].alpha_1 = 0;
            g_manual_joint_coeff[i].alpha_3 = 0;
            g_manual_joint_coeff[i].duration_1 = 0;
            g_manual_joint_coeff[i].duration_2 = 99999;
            g_manual_joint_coeff[i].duration_3 = 99999;
        }
        else if (manual_direction_[i] == INCREASE)
        {
            g_manual_joint_coeff[i].alpha_1 = alpha[i];
            g_manual_joint_coeff[i].alpha_3 = -alpha[i];
            g_manual_joint_coeff[i].duration_1 = omega[i] / alpha[i];
            g_manual_joint_coeff[i].duration_2 = 99999 - g_manual_joint_coeff[i].duration_1;
            g_manual_joint_coeff[i].duration_3 = 99999;
        }
        else
        {
            g_manual_joint_coeff[i].alpha_1 = -alpha[i];
            g_manual_joint_coeff[i].alpha_3 = alpha[i];
            g_manual_joint_coeff[i].duration_1 = omega[i] / alpha[i];
            g_manual_joint_coeff[i].duration_2 = 99999 - g_manual_joint_coeff[i].duration_1;
            g_manual_joint_coeff[i].duration_3 = 99999;
        }
        FST_INFO("  J%d: d1=%f, d2=%f, d3=%f, alpha1=%f, alpha3=%f", i + 1, g_manual_joint_coeff[i].duration_1,
                 g_manual_joint_coeff[i].duration_2, g_manual_joint_coeff[i].duration_3,
                 g_manual_joint_coeff[i].alpha_1, g_manual_joint_coeff[i].alpha_3);
    }

    memcpy(g_manual_direction, manual_direction_, 6 * sizeof(ManualDirection));
    FST_INFO("Success !");
    return SUCCESS;
}

ErrorCode ManualTeach::manualJointAPoint(void)
{
    double speed_ratio  = g_global_vel_ratio;
    double acc_ratio    = g_global_acc_ratio;

    FST_INFO("manual-Joint-APOINT: planning trajectory ...");
    FST_INFO("  speed_ratio=%.0f%%, acc_ratio=%.0f%%", speed_ratio * 100, acc_ratio * 100);
    FST_INFO("  start joint=%.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
                g_start_joint.j1, g_start_joint.j2, g_start_joint.j3,
                g_start_joint.j4, g_start_joint.j5, g_start_joint.j6);
    FST_INFO("  target joint: %.4f %.4f %.4f %.4f %.4f %.4f",
                manual_target_joint_.j1, manual_target_joint_.j2, manual_target_joint_.j3, 
                manual_target_joint_.j4, manual_target_joint_.j5, manual_target_joint_.j6);

    JointConstraint &cons = g_soft_constraint;

    if (!isJointInConstraint(manual_target_joint_, cons))
    {
        FST_ERROR("Target out of soft constraint.");
        return TARGET_OUT_OF_CONSTRAINT;
    }

    double trips[6] = { fabs(manual_target_joint_.j1 - g_start_joint.j1),
                        fabs(manual_target_joint_.j2 - g_start_joint.j2),
                        fabs(manual_target_joint_.j3 - g_start_joint.j3),
                        fabs(manual_target_joint_.j4 - g_start_joint.j4),
                        fabs(manual_target_joint_.j5 - g_start_joint.j5),
                        fabs(manual_target_joint_.j6 - g_start_joint.j6)
                      };
    double omega[6] = { cons.j1.max_omega * speed_ratio, cons.j2.max_omega * speed_ratio,
                        cons.j3.max_omega * speed_ratio, cons.j4.max_omega * speed_ratio,
                        cons.j5.max_omega * speed_ratio, cons.j6.max_omega * speed_ratio
                      };
    double alpha[6] = { cons.j1.max_alpha * 3 * acc_ratio, cons.j2.max_alpha * 3 * acc_ratio,
                        cons.j3.max_alpha * 3 * acc_ratio, cons.j4.max_alpha * 3 * acc_ratio,
                        cons.j5.max_alpha * 3 * acc_ratio, cons.j6.max_alpha * 3 * acc_ratio
                      };

    double t_min[6];

    double delta[6];
    for (size_t i = 0; i < 6; i++)
    {
        delta[i] = trips[i] - omega[i] * omega[i] / alpha[i];
        t_min[i] = delta[i] > 0 ? (omega[i] / alpha[i] + trips[i] / omega[i]) : (sqrt(trips[i] / alpha[i]) * 2);
        FST_INFO("  J%d: trip=%.8f omega=%f, alpha=%f, delta=%f, tmin=%f", i + 1, trips[i], omega[i], alpha[i], delta[i], t_min[i]);
        FST_INFO("  J%d: t=%f", i + 1, sqrt(trips[i] / alpha[i]) * 2);
    }

    double duration = t_min[0];
    duration = duration < t_min[1] ? t_min[1] : duration;
    duration = duration < t_min[2] ? t_min[2] : duration;
    duration = duration < t_min[3] ? t_min[3] : duration;
    duration = duration < t_min[4] ? t_min[4] : duration;
    duration = duration < t_min[5] ? t_min[5] : duration;

    FST_INFO("t1=%f, t2=%f, t3=%f, t4=%f, t5=%f, t6=%f",\
             t_min[0], t_min[1], t_min[2], t_min[3], t_min[4], t_min[5]);
    FST_INFO("  duration=%.4f, create trajectory", duration);

    double *start  = &g_start_joint.j1;
    double *target = &manual_target_joint_.j1;

    for (size_t i = 0; i < 6; i++)
    {
        if (trips[i] < MINIMUM_E9)
        {
            g_manual_joint_coeff[i].duration_1 = 0;
            g_manual_joint_coeff[i].duration_2 = duration;
            g_manual_joint_coeff[i].duration_3 = duration;
            g_manual_joint_coeff[i].alpha_1 = 0;
            g_manual_joint_coeff[i].alpha_3 = 0;
        }
        else
        {
            if (duration < t_min[i] + MINIMUM_E12)
            {
                g_manual_joint_coeff[i].duration_1 = delta[i] > 0 ? omega[i] / alpha[i] : duration / 2;
                g_manual_joint_coeff[i].duration_2 = delta[i] > 0 ? duration - omega[i] / alpha[i] : duration / 2;
                g_manual_joint_coeff[i].duration_3 = duration;
            }
            else
            {
                // replan traj using duration
                double t2 = sqrt(duration * duration - trips[i] / alpha[i] * 4);
                g_manual_joint_coeff[i].duration_1 = (duration - t2) / 2;
                g_manual_joint_coeff[i].duration_2 = (duration + t2) / 2;
                g_manual_joint_coeff[i].duration_3 = duration;
            }

            g_manual_joint_coeff[i].alpha_1 = *target > *start ? alpha[i] : -alpha[i];
            g_manual_joint_coeff[i].alpha_3 = -g_manual_joint_coeff[i].alpha_1;
        }

        ++ start;
        ++ target;

        FST_INFO("  J%d: d1=%f, d2=%f, d3=%f, alpha1=%f, alpha3=%f", i + 1, g_manual_joint_coeff[i].duration_1,
                 g_manual_joint_coeff[i].duration_2, g_manual_joint_coeff[i].duration_3,
                 g_manual_joint_coeff[i].alpha_1, g_manual_joint_coeff[i].alpha_3);
    }

    memcpy(&g_manual_joint_target, &manual_target_joint_, sizeof(Joint));
    FST_INFO("Success !");
    return SUCCESS;
}

ErrorCode ManualTeach::manualCartesian(void)
{
    ErrorCode err = SUCCESS;
    
    if (manual_mode_ == STEP)
    {
        err = manualCartesianStep();
    }
    else if (manual_mode_ == CONTINUOUS)
    {
        err = manualCartesianContinuous();
    }
    else
    {
        err = MOTION_INTERNAL_FAULT;
    }

    if (err == SUCCESS)
    {
        g_manual_mode = manual_mode_;
        g_manual_frame = manual_frame_;
        memcpy(g_manual_direction, manual_direction_, 6 * sizeof(ManualDirection));
        forwardKinematics(g_start_joint, g_manual_cartesian_start);
    }

    return err;
}

ErrorCode ManualTeach::manualCartesianStep(void)
{
    double speed_ratio  = g_global_vel_ratio;
    double acc_ratio    = g_global_acc_ratio;

    PoseEuler start  = forwardKinematics2PoseEuler(g_start_joint);
    PoseEuler target = start;

    if      (manual_direction_[0] == INCREASE) {target.position.x += g_manual_step_position;}
    else if (manual_direction_[0] == DECREASE) {target.position.x -= g_manual_step_position;}
    if      (manual_direction_[1] == INCREASE) {target.position.y += g_manual_step_position;}
    else if (manual_direction_[1] == DECREASE) {target.position.y -= g_manual_step_position;}
    if      (manual_direction_[2] == INCREASE) {target.position.z += g_manual_step_position;}
    else if (manual_direction_[2] == DECREASE) {target.position.z -= g_manual_step_position;}
    if      (manual_direction_[3] == INCREASE) {target.orientation.a += g_manual_step_orientation;}
    else if (manual_direction_[3] == DECREASE) {target.orientation.a -= g_manual_step_orientation;}
    if      (manual_direction_[4] == INCREASE) {target.orientation.b += g_manual_step_orientation;}
    else if (manual_direction_[4] == DECREASE) {target.orientation.b -= g_manual_step_orientation;}
    if      (manual_direction_[5] == INCREASE) {target.orientation.c += g_manual_step_orientation;}
    else if (manual_direction_[5] == DECREASE) {target.orientation.c -= g_manual_step_orientation;}

    FST_INFO("manual-Cartesian-STEP: directions = %d %d %d %d %d %d, planning trajectory ...",
             manual_direction_[0], manual_direction_[1], manual_direction_[2],
             manual_direction_[3], manual_direction_[4], manual_direction_[5]);
    FST_INFO("  step_postion = %.2fmm, step_angle = %.2frad, speed_ratio = %.0f%%, acc_ratio=%.0f%%",
             g_manual_step_position, g_manual_step_orientation, speed_ratio * 100, acc_ratio * 100);
    FST_INFO("  start  pose  = %.2f %.2f %.2f - %.4f %.4f %.4f",
             start.position.x, start.position.y, start.position.z,
             start.orientation.a, start.orientation.b, start.orientation.c);
    FST_INFO("  target pose  = %.2f %.2f %.2f - %.4f %.4f %.4f",
             target.position.x, target.position.y, target.position.z,
             target.orientation.a, target.orientation.b, target.orientation.c);
    FST_INFO("  start joint  = %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
             g_start_joint.j1, g_start_joint.j2, g_start_joint.j3, 
             g_start_joint.j4, g_start_joint.j5, g_start_joint.j6);

    ErrorCode err = SUCCESS;

    if (!isJointInConstraint(g_start_joint, g_soft_constraint))
    {
        FST_ERROR("start joint out of constraint, manual cartesian is disabled.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    Joint target_joint;
    err = inverseKinematics(target, g_start_joint, target_joint);
    if (err != SUCCESS)
    {
        FST_ERROR("Cannot find a valid inverse result of the target pose, err=%0xllx", err);
        return err;
    }
    FST_INFO("  target joint = %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
             target_joint.j1, target_joint.j2, target_joint.j3, 
             target_joint.j4, target_joint.j5, target_joint.j6);
    if (!isJointInConstraint(target_joint, g_soft_constraint))
    {
        err = JOINT_OUT_OF_CONSTRAINT;
        FST_ERROR("Target joint out of constraint, err=0x%llx", err);
        return err;
    }

    double dis = (manual_direction_[0] != STANDBY || manual_direction_[1] != STANDBY ||
                  manual_direction_[2] != STANDBY) ? g_manual_step_position : 0;
    double spd = g_cart_vel_max * speed_ratio;
    double acc = g_cart_acc_max * acc_ratio;

    double angle = (manual_direction_[3] != STANDBY || manual_direction_[4] != STANDBY ||
                    manual_direction_[5] != STANDBY) ? g_manual_step_orientation : 0;
    double omega = g_orientation_omega_reference * speed_ratio;
    double alpha = g_orientation_alpha_reference * acc_ratio;

    //FST_WARN("  spd=%f acc=%f omega=%f alpha=%f", spd, acc, omega, alpha);

    double delta = dis - spd * spd / acc;
    double t_pos = delta > 0 ? (spd / acc * 2 + delta / spd) : (sqrt(dis / acc) * 2);
    double theta = angle - omega * omega / alpha;
    double t_ort = theta > 0 ? (omega / alpha * 2 + theta / omega) : (sqrt(angle / alpha) * 2);

    double duration = t_pos > t_ort ? t_pos : t_ort;

    //FST_WARN("  delta=%f t_pos=%f, theta=%f, t_ort=%f, duration=%f", delta, t_pos, theta, t_ort, duration);

    for (size_t i = 0; i < 3; i++)
    {
        if (manual_direction_[i] == STANDBY)
        {
            g_manual_cartesian_coeff[i].duration_1 = 0;
            g_manual_cartesian_coeff[i].duration_2 = duration;
            g_manual_cartesian_coeff[i].duration_3 = duration;
            g_manual_cartesian_coeff[i].alpha_1 = 0;
            g_manual_cartesian_coeff[i].alpha_3 = 0;
        }
        else
        {
            if (duration < t_pos + MINIMUM_E12)
            {
                g_manual_cartesian_coeff[i].duration_1 = delta > 0 ? spd / acc : duration / 2;
                g_manual_cartesian_coeff[i].duration_2 = delta > 0 ? duration - spd / acc : duration / 2;
                g_manual_cartesian_coeff[i].duration_3 = duration;
            }
            else
            {
                // replan position traj
                g_manual_cartesian_coeff[i].duration_2 = duration - sqrt(g_manual_step_position / acc) * 2;
                g_manual_cartesian_coeff[i].duration_1 = (duration - g_manual_cartesian_coeff[i].duration_2) / 2;
                g_manual_cartesian_coeff[i].duration_3 = g_manual_cartesian_coeff[i].duration_1;
            }

            g_manual_cartesian_coeff[i].alpha_1 = manual_direction_[i] == INCREASE ? acc : -acc;
            g_manual_cartesian_coeff[i].alpha_3 = -g_manual_cartesian_coeff[i].alpha_1;
        }
    }

    for (size_t i = 3; i < 6; i++)
    {
        if (manual_direction_[i] == STANDBY)
        {
            g_manual_cartesian_coeff[i].duration_1 = 0;
            g_manual_cartesian_coeff[i].duration_2 = duration;
            g_manual_cartesian_coeff[i].duration_3 = duration;
            g_manual_cartesian_coeff[i].alpha_1 = 0;
            g_manual_cartesian_coeff[i].alpha_3 = 0;
        }
        else
        {
            if (duration < t_ort + MINIMUM_E12)
            {
                g_manual_cartesian_coeff[i].duration_1 = theta > 0 ? omega / alpha : duration / 2;
                g_manual_cartesian_coeff[i].duration_2 = theta > 0 ? duration - omega / alpha : duration / 2;
                g_manual_cartesian_coeff[i].duration_3 = duration;
            }
            else
            {
                // replan orientation traj
                g_manual_cartesian_coeff[i].duration_2 = duration - sqrt(g_manual_step_orientation / alpha) * 2;
                g_manual_cartesian_coeff[i].duration_1 = (duration - g_manual_cartesian_coeff[i].duration_2) / 2;
                g_manual_cartesian_coeff[i].duration_3 = g_manual_cartesian_coeff[i].duration_1;
            }

            g_manual_cartesian_coeff[i].alpha_1 = manual_direction_[i] == INCREASE ? alpha : -alpha;
            g_manual_cartesian_coeff[i].alpha_3 = -g_manual_cartesian_coeff[i].alpha_1;
        }
    }

    g_manual_mode = manual_mode_;
    g_manual_frame = manual_frame_;
    g_manual_cartesian_start = start;
    memcpy(g_manual_direction, manual_direction_, 6 * sizeof(ManualDirection));

    FST_INFO("Success !");
    return SUCCESS;
}

ErrorCode ManualTeach::manualCartesianContinuous(void)
{
    ErrorCode err = SUCCESS;

    double speed_ratio  = g_global_vel_ratio;
    double acc_ratio    = g_global_acc_ratio;

    PoseEuler start  = forwardKinematics2PoseEuler(g_start_joint);

    FST_INFO("manual-Cartesian-CONTINUOUS: directions = %d %d %d %d %d %d, planning trajectory ...",
             manual_direction_[0], manual_direction_[1], manual_direction_[2],
             manual_direction_[3], manual_direction_[4], manual_direction_[5]);
    FST_INFO("  speed_ratio = %.0f%%, acc_ratio=%.0f%%",
             speed_ratio * 100, acc_ratio * 100);
    FST_INFO("  start  pose  = %.2f %.2f %.2f - %.4f %.4f %.4f",
             start.position.x, start.position.y, start.position.z,
             start.orientation.a, start.orientation.b, start.orientation.c);

    double spd, acc;
    double duration;
    size_t i;

    for (i = 0; i < 6; i++)
    {
        if (manual_direction_[i] != STANDBY)
        {
            if (i < 3)
            {
                spd = g_cart_vel_max * speed_ratio;
                acc = g_cart_acc_max * acc_ratio;
            }
            else
            {
                spd = g_orientation_omega_reference * speed_ratio;
                acc = g_orientation_alpha_reference * acc_ratio;
            }

            g_manual_cartesian_coeff[i].duration_1 = spd / acc;
            g_manual_cartesian_coeff[i].duration_2 = 99999 - g_manual_cartesian_coeff[i].duration_1;
            g_manual_cartesian_coeff[i].duration_3 = 99999;

            g_manual_cartesian_coeff[i].alpha_1 = manual_direction_[i] == INCREASE ? acc : -acc;
            g_manual_cartesian_coeff[i].alpha_3 = -g_manual_cartesian_coeff[i].alpha_1;
            
            break;
        }
        else
        {
            g_manual_cartesian_coeff[i].duration_1 = 0;
            g_manual_cartesian_coeff[i].duration_2 = 99999;
            g_manual_cartesian_coeff[i].duration_3 = 99999;
            g_manual_cartesian_coeff[i].alpha_1 = 0;
            g_manual_cartesian_coeff[i].alpha_3 = 0;
        }
    }

    for (i = i + 1; i < 6; i++)
    {
        g_manual_cartesian_coeff[i].duration_1 = 0;
        g_manual_cartesian_coeff[i].duration_2 = 99999;
        g_manual_cartesian_coeff[i].duration_3 = 99999;
        g_manual_cartesian_coeff[i].alpha_1 = 0;
        g_manual_cartesian_coeff[i].alpha_3 = 0;
    }

    g_manual_mode = manual_mode_;
    g_manual_frame = manual_frame_;
    g_manual_cartesian_start = start;
    memcpy(g_manual_direction, manual_direction_, 6 * sizeof(ManualDirection));

    FST_INFO("Success !");
    return SUCCESS;
}

ErrorCode ManualTeach::stopJointContinuous(MotionTime time)
{
    FST_INFO("stopJointContinuous: time = %.4f", time);

    double omega[6] = {0, 0, 0, 0, 0, 0};
    double alpha[6] = {g_manual_joint_coeff[0].alpha_1, g_manual_joint_coeff[1].alpha_1,
                       g_manual_joint_coeff[2].alpha_1, g_manual_joint_coeff[3].alpha_1,
                       g_manual_joint_coeff[4].alpha_1, g_manual_joint_coeff[5].alpha_1};
    double duration[6] = {0, 0, 0, 0, 0, 0};
    double duration_max = 0;

    for (size_t i = 0; i < 6; i++)
    {
        if (manual_direction_[i] != STANDBY)
        {
            if (time < g_manual_joint_coeff[i].duration_1)
            {
                duration[i] = time * 2;
            }
            else
            {
                duration[i] = time + g_manual_joint_coeff[i].duration_1;
            }
        }
        duration_max = duration[i] > duration_max ? duration[i] : duration_max;
    }

    for (size_t i = 0; i < 6; i++)
    {
        if (duration[i] + MINIMUM_E12 < duration_max)
        {
            if (time < g_manual_joint_coeff[i].duration_1)
            {
                g_manual_joint_coeff[i].duration_1 = time;
                g_manual_joint_coeff[i].duration_2 = duration_max - time;
                g_manual_joint_coeff[i].duration_3 = duration_max;
                g_manual_joint_coeff[i].alpha_3 = -g_manual_joint_coeff[i].alpha_1;
            }
            else
            {
                g_manual_joint_coeff[i].duration_2 = duration_max - g_manual_joint_coeff[i].duration_1;
                g_manual_joint_coeff[i].duration_3 = duration_max;
                g_manual_joint_coeff[i].alpha_3 = -g_manual_joint_coeff[i].alpha_1;
            }
        }
        else
        {
            if (time < g_manual_joint_coeff[i].duration_1)
            {
                g_manual_joint_coeff[i].duration_1 = time;
                g_manual_joint_coeff[i].duration_2 = time;
                g_manual_joint_coeff[i].duration_3 = duration_max;
                g_manual_joint_coeff[i].alpha_3 = -g_manual_joint_coeff[i].alpha_1;
            }
            else
            {
                g_manual_joint_coeff[i].duration_2 = time;
                g_manual_joint_coeff[i].duration_3 = duration_max;
                g_manual_joint_coeff[i].alpha_3 = -g_manual_joint_coeff[i].alpha_1;
            }
        }

        FST_INFO("  J%d: d1=%.4f, d2=%.4f, d3=%.4f, a1=%.6f, a3=%.6f", i + 1, g_manual_joint_coeff[i].duration_1,
                 g_manual_joint_coeff[i].duration_2, g_manual_joint_coeff[i].duration_3,
                 g_manual_joint_coeff[i].alpha_1, g_manual_joint_coeff[i].alpha_3);
    }

    FST_INFO("stopJointContinuous: success");
    return SUCCESS;
}

ErrorCode ManualTeach::stopCartesianContinuous(MotionTime time)
{
    FST_INFO("stopCartesianContinuous: time = %.4f", time);

    size_t index;

    for (index = 0; index < 6; index++)
    {
        if (manual_direction_[index] != STANDBY)
        {
            if (time < g_manual_cartesian_coeff[index].duration_1)
            {
                g_manual_cartesian_coeff[index].duration_1 = time;
                g_manual_cartesian_coeff[index].duration_2 = time;
                g_manual_cartesian_coeff[index].duration_3 = time * 2;
                g_manual_cartesian_coeff[index].alpha_3 = -g_manual_cartesian_coeff[index].alpha_1;
            }
            else
            {
                g_manual_cartesian_coeff[index].duration_2 = time;
                g_manual_cartesian_coeff[index].duration_3 = time + g_manual_cartesian_coeff[index].duration_1;
                g_manual_cartesian_coeff[index].alpha_3 = -g_manual_cartesian_coeff[index].alpha_1;
            }

            break;
        }
    }

    for (size_t i = 0; i < 6; i++)
    {
        if (i != index)
        {
            g_manual_cartesian_coeff[i].duration_1 = 0;
            g_manual_cartesian_coeff[i].duration_2 = g_manual_cartesian_coeff[index].duration_3;
            g_manual_cartesian_coeff[i].duration_3 = g_manual_cartesian_coeff[index].duration_3;
            g_manual_cartesian_coeff[i].alpha_3 = 0;
        }

        FST_INFO("  P%d: t1=%.4f, t2=%.4f, t3=%.4f, a1=%.4f, a3=%.4f", i + 1, g_manual_cartesian_coeff[index].duration_1,
                 g_manual_cartesian_coeff[index].duration_2, g_manual_cartesian_coeff[index].duration_3,
                 g_manual_cartesian_coeff[index].alpha_1, g_manual_cartesian_coeff[index].alpha_3);
    }

    FST_INFO("stopCartesianContinuous: success");
    return SUCCESS;
}


