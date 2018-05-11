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

ErrorCode ManualTeach::stepTeach(const ManualDirection *directions, MotionTime time, ManualTrajectory &traj)
{
    ErrorCode err = SUCCESS;
    FST_INFO("Manual request received, mode=%d, frame=%d", traj.mode, traj.frame);

    if (!isJointInConstraint(traj.joint_start, g_soft_constraint))
    {
        if (traj.frame != JOINT)
        {
            err = INVALID_SEQUENCE;
            FST_ERROR("start-joint is out of soft constraint, manual-frame-cartesian is disabled.");
            return err;
        }
        if (traj.mode == APOINT)
        {
            err = INVALID_SEQUENCE;
            FST_ERROR("start-joint is out of soft constraint, manual-mode-apoint is disabled.");
            return err;
        }
        
        double *start = &traj.joint_start.j1;
        JointLimit *limit = &g_soft_constraint.j1;
        for (size_t i = 0; i < 6; i++)
        {
            if (start[i] > limit[i].upper + MINIMUM_E6 && directions[i] == INCREASE)
            {
                err = INVALID_PARAMETER;
                FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (increase).",
                          i + 1, start[i], limit[i].lower, limit[i].upper);
                return err;
            }
            else if (start[i] < limit[i].lower - MINIMUM_E6 && directions[i] == DECREASE)
            {
                err = INVALID_PARAMETER;
                FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (decrease).",
                          i + 1, start[i], limit[i].lower, limit[i].upper);
                return err;
            }
        }
    }

    switch (traj.frame)
    {
        case JOINT:
            err = manualJoint(directions, time, traj);
            break;
        case WORLD:
        case USER:
        case TOOL:
            err = manualCartesian(directions, time, traj);
            break;
        default:
            err = MOTION_INTERNAL_FAULT;
            break;
    }

    if (err != SUCCESS)
    {
        FST_ERROR("Unsupported manual frame: %d", traj.frame);
    }
}

ErrorCode ManualTeach::stepTeach(const Joint &target, MotionTime time, ManualTrajectory &traj)
{
    return manualJointAPoint(target, time, traj);
}

ErrorCode ManualTeach::stopTeach(MotionTime time, ManualTrajectory &traj)
{
    FST_INFO("stopTeach: curr-time=%.4f, total-time=%.4f", time, traj.duration);
    if (traj.mode == CONTINUOUS)
    {
        FST_WARN("manual-mode is continuous, stop motion by change direction input");
        return SUCCESS;
    }
    else if (time + 0.05 > traj.duration)
    {
        FST_INFO("robot motion will stop soon, do nothing");
        return SUCCESS;
    }
    
    for (size_t i = 0; i < 6; i++)
    {
        if (time < traj.coeff[i].start_time)
        {
            traj.coeff[i].start_time = time;
            traj.coeff[i].stable_time = time;
            traj.coeff[i].brake_time = time;
            traj.coeff[i].stop_time = time;
            traj.coeff[i].start_alpha = 0;
            traj.coeff[i].brake_alpha = 0;
            traj.direction[i] = STANDBY;
            if (traj.frame == JOINT)
                *(&traj.joint_ending.j1 + i) = *(&traj.joint_start.j1 + i);
            else
                *(&traj.cart_ending.position.x + i) = *(&traj.cart_start.position.x + i);
        }
        else if (time < traj.coeff[i].stable_time)
        {
            traj.coeff[i].stable_time = time;
            traj.coeff[i].brake_time = time;
            traj.coeff[i].stop_time = time + traj.coeff[i].stable_time - traj.coeff[i].start_time;
            traj.direction[i] = STANDBY;
            double tm = time - traj.coeff[i].start_time;
            if (traj.frame == JOINT)
                *(&traj.joint_ending.j1 + i) = *(&traj.joint_start.j1 + i) + traj.coeff[i].start_alpha * tm * tm;
            else
                *(&traj.cart_ending.position.x + i) = *(&traj.cart_start.position.x + i) + traj.coeff[i].start_alpha * tm * tm;
        }
        else if (time < traj.coeff[i].brake_time)
        {
            traj.coeff[i].brake_time = time;
            traj.coeff[i].stop_time = time + traj.coeff[i].stable_time - traj.coeff[i].start_time;
            traj.direction[i] = STANDBY;
            double tim = traj.coeff[i].stable_time - traj.coeff[i].start_time;
            double omg = tim * traj.coeff[i].start_alpha;
            if (traj.frame == JOINT)
            {
                *(&traj.joint_ending.j1 + i) = *(&traj.joint_start.j1 + i) + omg * tim;
                tim = time - traj.coeff[i].stable_time;
                *(&traj.joint_ending.j1 + i) += omg * tim;
            }
            else
            {
                *(&traj.cart_ending.position.x + i) = *(&traj.cart_start.position.x + i) + omg * tim;
                tim = time - traj.coeff[i].stable_time;
                *(&traj.cart_ending.position.x + i) += omg * tim;
            }
        }
        else
        {
            // already in axis-slow-down trajectory, do nothing
            FST_WARN("  J%d/Dir%d: Standby or in slow-down trajectory, will stop on soft constraint", i + 1, i + 1);
            continue;
        }

        FST_INFO("  J%d/Dir%d: t1=%.4f,t2=%.4f,t3=%.4f,t4=%.4f,alpha1-2=%.4f,alpha3-4=%.4f",
                 i + 1, i + 1, traj.coeff[i].start_time, traj.coeff[i].stable_time,
                 traj.coeff[i].brake_time, traj.coeff[i].stop_time,
                 traj.coeff[i].start_alpha, traj.coeff[i].brake_alpha);
    }

    double duration = 0;
    for (size_t i = 0; i < 6; i++)
    {
        if (traj.coeff[i].stop_time > duration)
        {
            duration = traj.coeff[i].stop_time;
        }
    }
    traj.duration = duration;
    FST_INFO("  total duration=%.4f", traj.duration);
    FST_INFO("Success !");
    return SUCCESS;
}

ErrorCode ManualTeach::manualJoint(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj)
{
    ErrorCode err = SUCCESS;
    
    switch (traj.mode)
    {
        case STEP:
            err = manualJointStep(dir, time, traj);
            break;
        case CONTINUOUS:
            err = manualJointContinuous(dir, time, traj);
            break;
        case APOINT:
            FST_ERROR("manualJoint: manual-mode=%d, but given 6 directions");
            err = MOTION_INTERNAL_FAULT;
            break;
        default:
            FST_ERROR("Unsupported manual mode: %d", traj.mode);
            err = MOTION_INTERNAL_FAULT;
            break;
    }

    if (err != SUCCESS)
    {
        FST_ERROR("manualJoint failed, err=0x%llx", err);
    }

    return err;
}

ErrorCode ManualTeach::manualJointStep(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj)
{
    double speed_ratio  = g_global_vel_ratio;
    double acc_ratio    = g_global_acc_ratio;

    FST_INFO("manual-Joint-Step: directions = %d %d %d %d %d %d, planning trajectory ...",
             dir[0], dir[1], dir[2], dir[3], dir[4], dir[5]);
    FST_INFO("  step_angle=%.4frad, speed_ratio=%.0f%%, acc_ratio=%.0f%%",
             g_manual_step_joint, speed_ratio * 100, acc_ratio * 100);
    FST_INFO("  start  joint=%.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
             traj.joint_start.j1, traj.joint_start.j2, traj.joint_start.j3, 
             traj.joint_start.j4, traj.joint_start.j5, traj.joint_start.j6);

    Joint target  = traj.joint_start;
    double *angle = &target.j1;

    for (size_t i = 0; i < 6; i++)
    {
        if      (dir[i] == INCREASE) *angle += g_manual_step_joint;
        else if (dir[i] == DECREASE) *angle -= g_manual_step_joint;

        angle ++;
    }

    FST_INFO("  target joint=%.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
             target.j1, target.j2, target.j3, target.j4, target.j5, target.j6);

    if (!isJointInConstraint(target, g_soft_constraint))
    {
        FST_ERROR("Target out of soft constraint.");
        return TARGET_OUT_OF_CONSTRAINT;
    }

    double omega[6], alpha[6], trips[6], delta[6];
    double duration = 0;
    double t_min[6];


    for (size_t i = 0; i < 6; i++)
    {
        trips[i] = dir[i] == STANDBY ? 0 : g_manual_step_joint;
        omega[i] = g_omega_limit[i] * speed_ratio;
        alpha[i] = g_alpha_limit[i] * acc_ratio;
        
        delta[i] = trips[i] - omega[i] * omega[i] / alpha[i];
        t_min[i] = delta[i] > 0 ? (omega[i] / alpha[i] + trips[i] / omega[i]) : (sqrt(trips[i] / alpha[i]) * 2);
        FST_INFO("  J%d: trip=%.4f omega=%f, alpha=%f, delta=%f, tmin=%f", i + 1, trips[i], omega[i], alpha[i], delta[i], t_min[i]);
        if (t_min[i] > duration) duration = t_min[i];
    }

    //FST_INFO("  t1=%f, t2=%f, t3=%f, t4=%f, t5=%f, t6=%f",\
    //         t_min[0], t_min[1], t_min[2], t_min[3], t_min[4], t_min[5]);
    FST_INFO("  duration=%.4f, create trajectory ...", duration);

    traj.duration = duration;
    traj.joint_ending = target;

    for (size_t i = 0; i < 6; i++)
    {
        traj.direction[i] = dir[i];
        if (dir[i] == STANDBY)
        {
            traj.coeff[i].start_time = time;
            traj.coeff[i].stable_time = time;
            traj.coeff[i].brake_time = time;
            traj.coeff[i].stop_time = time;
            traj.coeff[i].start_alpha = 0;
            traj.coeff[i].brake_alpha = 0;
        }
        else
        {
            if (duration < t_min[i] + MINIMUM_E12)
            {
                traj.coeff[i].start_time = time;
                traj.coeff[i].stable_time = time + delta[i] > 0 ? omega[i] / alpha[i] : duration / 2;
                traj.coeff[i].brake_time = time + delta[i] > 0 ? duration - omega[i] / alpha[i] : duration / 2;
                traj.coeff[i].stop_time = time + duration;
            }
            else
            {
                // replan traj using duration
                double t_stable = sqrt(duration * duration - trips[i] / alpha[i] * 4);
                traj.coeff[i].start_time = time;
                traj.coeff[i].stable_time = time + (duration - t_stable) / 2;
                traj.coeff[i].brake_time = time + (duration + t_stable) / 2;
                traj.coeff[i].stop_time = time + duration;
            }

            traj.coeff[i].start_alpha = dir[i] == INCREASE ? alpha[i] : -alpha[i];
            traj.coeff[i].brake_alpha = -traj.coeff[i].start_alpha;
        }

        FST_INFO("  J%d: t1=%.4f,t2=%.4f,t3=%.4f,t4=%.4f,alpha1-2=%.4f,alpha3-4=%.4f",
                 i + 1, traj.coeff[i].start_time, traj.coeff[i].stable_time,
                 traj.coeff[i].brake_time, traj.coeff[i].stop_time,
                 traj.coeff[i].start_alpha, traj.coeff[i].brake_alpha);
    }

    FST_INFO("Success !");
    return SUCCESS;
}

ErrorCode ManualTeach::manualJointContinuous(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj)
{
    double speed_ratio  = g_global_vel_ratio;
    double acc_ratio    = g_global_acc_ratio;

    FST_INFO("manual-Joint-Continuous: directions = %d %d %d %d %d %d, planning trajectory ...",
             dir[0], dir[1], dir[2], dir[3], dir[4], dir[5]);
    FST_INFO("  speed_ratio=%.0f%%, acc_ratio=%.0f%%", speed_ratio * 100, acc_ratio * 100);
    FST_INFO("  start joint=%.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
             traj.joint_start.j1, traj.joint_start.j2, traj.joint_start.j3, 
             traj.joint_start.j4, traj.joint_start.j5, traj.joint_start.j6);

    double alpha[6] = {g_alpha_limit[0] * acc_ratio, g_alpha_limit[1] * acc_ratio,
                       g_alpha_limit[2] * acc_ratio, g_alpha_limit[3] * acc_ratio,
                       g_alpha_limit[4] * acc_ratio, g_alpha_limit[5] * acc_ratio};
    double omega[6] = {g_omega_limit[0] * speed_ratio, g_omega_limit[1] * speed_ratio,
                       g_omega_limit[2] * speed_ratio, g_omega_limit[3] * speed_ratio,
                       g_omega_limit[4] * speed_ratio, g_omega_limit[5] * speed_ratio};

    for (size_t i = 0; i < 6; i++)
    {
        if (traj.direction[i] == dir[i])
        {
            // keep standby or motion
            // do nothing
            FST_INFO("  J%d: given direction same as current motion, running along the original trajectory", i + 1);
            continue;
        }
        else if (dir[i] == STANDBY && traj.direction[i] != STANDBY)
        {
            // stop joint motion
            if (time < traj.coeff[i].start_time)
            {
                traj.coeff[i].start_time = time;
                traj.coeff[i].stable_time = time;
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time;
                traj.coeff[i].start_alpha = 0;
                traj.coeff[i].brake_alpha = 0;
                traj.direction[i] = STANDBY;
                *(&traj.joint_ending.j1 + i) = *(&traj.joint_start.j1 + i);
            }
            else if (time < traj.coeff[i].stable_time)
            {
                traj.coeff[i].stable_time = time;
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time + traj.coeff[i].stable_time - traj.coeff[i].start_time;
                traj.direction[i] = STANDBY;
                double tm = time - traj.coeff[i].start_time;
                *(&traj.joint_ending.j1 + i) = *(&traj.joint_start.j1 + i) + traj.coeff[i].start_alpha * tm * tm;
            }
            else if (time < traj.coeff[i].brake_time)
            {
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time + traj.coeff[i].stable_time - traj.coeff[i].start_time;
                traj.direction[i] = STANDBY;
                double tim = traj.coeff[i].stable_time - traj.coeff[i].start_time;
                double omg = tim * traj.coeff[i].start_alpha;
                *(&traj.joint_ending.j1 + i) = *(&traj.joint_start.j1 + i) + omg * tim;
                tim = time - traj.coeff[i].stable_time;
                *(&traj.joint_ending.j1 + i) += omg * tim;
            }
            else
            {
                // already in axis-slow-down trajectory, do nothing
                FST_WARN("  J%d: Standby or in slow-down trajectory, will stop on soft constraint", i + 1);
                continue;
            }
        }
        else if (dir[i] != STANDBY && traj.direction[i] == STANDBY)
        {
            if (time >= traj.coeff[i].stop_time)
            {
                // start joint motion
                // stop until soft constraint
                JointLimit *limit = &g_soft_constraint.j1;
                double trip = dir[i] == INCREASE ? limit[i].upper - *(&traj.joint_start.j1 + i) :
                                limit[i].lower - *(&traj.joint_start.j1 + i);
               
                trip = fabs(trip);
                if (trip > MINIMUM_E6)
                {
                    double delta = trip - omega[i] * omega[i] / alpha[i];
                    if (delta > 0)
                    {
                        traj.coeff[i].start_time = time;
                        traj.coeff[i].stable_time = traj.coeff[i].start_time + omega[i] / alpha[i];
                        traj.coeff[i].brake_time = traj.coeff[i].stable_time + delta / omega[i];
                        traj.coeff[i].stop_time = traj.coeff[i].brake_time + omega[i] / alpha[i];
                    }
                    else
                    {
                        double t_startup = sqrt(trip / alpha[i]);
                        traj.coeff[i].start_time = time;
                        traj.coeff[i].stable_time = traj.coeff[i].start_time + t_startup;
                        traj.coeff[i].brake_time = traj.coeff[i].stable_time;
                        traj.coeff[i].stop_time = traj.coeff[i].brake_time + t_startup;

                    }
                    traj.coeff[i].start_alpha = dir[i] == INCREASE ? alpha[i] : -alpha[i];
                    traj.coeff[i].brake_alpha = -traj.coeff[i].start_alpha;
                    traj.direction[i] = dir[i];
                    *(&traj.joint_ending.j1 + i) = dir[i] == INCREASE ? limit[i].upper : limit[i].lower;
                }
                else
                {
                    FST_WARN("  J%d: near soft constraint, cannot manual move", i + 1);
                    continue;
                }
            }
            else
            {
                // axis-slow-down trajectory havn't finished
                FST_WARN("  J%d: in slow-down trajectory, cannot startup until standby", i + 1);
                continue;
            }
        }
        else
        {
            // prev-direction = INCREASE && input-direction = DECREASE or
            // prev-direction = DECREASE && input-direction = INCREASE,
            // stop joint motion
            traj.direction[i] = STANDBY;
            if (time < traj.coeff[i].start_time)
            {
                traj.coeff[i].start_time = time;
                traj.coeff[i].stable_time = time;
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time;
                traj.coeff[i].start_alpha = 0;
                traj.coeff[i].brake_alpha = 0;
                *(&traj.joint_ending.j1 + i) = *(&traj.joint_start.j1 + i);
            }
            else if (time < traj.coeff[i].stable_time)
            {
                traj.coeff[i].stable_time = time;
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time + traj.coeff[i].stable_time - traj.coeff[i].start_time;
                double tm = time - traj.coeff[i].start_time;
                *(&traj.joint_ending.j1 + i) = *(&traj.joint_start.j1 + i) + traj.coeff[i].start_alpha * tm * tm;
            }
            else if (time < traj.coeff[i].brake_time)
            {
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time + traj.coeff[i].stable_time - traj.coeff[i].start_time;
                double tim = traj.coeff[i].stable_time - traj.coeff[i].start_time;
                double omg = tim * traj.coeff[i].start_alpha;
                *(&traj.joint_ending.j1 + i) = *(&traj.joint_start.j1 + i) + omg * tim;
                tim = time - traj.coeff[i].stable_time;
                *(&traj.joint_ending.j1 + i) += omg * tim;
            }
            else
            {
                // already in axis-slow-down trajectory, do nothing
                FST_WARN("  J%d: Standby or in slow-down trajectory, will stop on soft constraint", i + 1);
                continue;
            }
        }

        FST_INFO("  J%d: t1=%.4f,t2=%.4f,t3=%.4f,t4=%.4f,alpha1-2=%.4f,alpha3-4=%.4f",
                 i + 1, traj.coeff[i].start_time, traj.coeff[i].stable_time,
                 traj.coeff[i].brake_time, traj.coeff[i].stop_time,
                 traj.coeff[i].start_alpha, traj.coeff[i].brake_alpha);
    }

    double duration = 0;
    for (size_t i = 0; i < 6; i++)
    {
        if (traj.coeff[i].stop_time > duration)
        {
            duration = traj.coeff[i].stop_time;
        }
    }
    traj.duration = duration;
    FST_INFO("  total duration=%.4f", traj.duration);
    FST_INFO("Success !");
    return SUCCESS;
}

ErrorCode ManualTeach::manualJointAPoint(const Joint &target, MotionTime time, ManualTrajectory &traj)
{
    double speed_ratio  = g_global_vel_ratio;
    double acc_ratio    = g_global_acc_ratio;

    FST_INFO("manual-Joint-APOINT: planning trajectory ...");
    FST_INFO("  speed_ratio=%.0f%%, acc_ratio=%.0f%%", speed_ratio * 100, acc_ratio * 100);
    FST_INFO("  start joint=%.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
                traj.joint_start.j1, traj.joint_start.j2, traj.joint_start.j3, 
                traj.joint_start.j4, traj.joint_start.j5, traj.joint_start.j6);
    FST_INFO("  target joint: %.4f %.4f %.4f %.4f %.4f %.4f",
                target.j1, target.j2, target.j3, target.j4, target.j5, target.j6);

    JointConstraint &cons = g_soft_constraint;

    double trips[6] = { fabs(target.j1 - traj.joint_start.j1), fabs(target.j2 - traj.joint_start.j2),
                        fabs(target.j3 - traj.joint_start.j3), fabs(target.j4 - traj.joint_start.j4),
                        fabs(target.j5 - traj.joint_start.j5), fabs(target.j6 - traj.joint_start.j6)};
    double alpha[6] = {g_alpha_limit[0] * acc_ratio, g_alpha_limit[1] * acc_ratio,
                       g_alpha_limit[2] * acc_ratio, g_alpha_limit[3] * acc_ratio,
                       g_alpha_limit[4] * acc_ratio, g_alpha_limit[5] * acc_ratio};
    double omega[6] = {g_omega_limit[0] * speed_ratio, g_omega_limit[1] * speed_ratio,
                       g_omega_limit[2] * speed_ratio, g_omega_limit[3] * speed_ratio,
                       g_omega_limit[4] * speed_ratio, g_omega_limit[5] * speed_ratio};

    double t_min[6], delta[6];
    double duration = 0;

    for (size_t i = 0; i < 6; i++)
    {
        delta[i] = trips[i] - omega[i] * omega[i] / alpha[i];
        t_min[i] = delta[i] > 0 ? (omega[i] / alpha[i] + trips[i] / omega[i]) : (sqrt(trips[i] / alpha[i]) * 2);
        FST_INFO("  J%d: trip=%.8f omega=%f, alpha=%f, delta=%f, tmin=%f", i + 1, trips[i], omega[i], alpha[i], delta[i], t_min[i]);
        //FST_INFO("  J%d: t=%f", i + 1, sqrt(trips[i] / alpha[i]) * 2);
        if (t_min[i] > duration) duration = t_min[i];
    }

    FST_INFO("  duration=%.4f, create trajectory", duration);

    double *start = &traj.joint_start.j1;
    const double *tar = &target.j1;

    for (size_t i = 0; i < 6; i++)
    {
        if (trips[i] < MINIMUM_E6)
        {
            traj.coeff[i].start_time = time;
            traj.coeff[i].stable_time = time;
            traj.coeff[i].brake_time = time;
            traj.coeff[i].stop_time = time;
            traj.coeff[i].start_alpha = 0;
            traj.coeff[i].brake_alpha = 0;
        }
        else
        {
            if (duration < t_min[i] + MINIMUM_E12)
            {
                traj.coeff[i].start_time = time;
                traj.coeff[i].stable_time = time + delta[i] > 0 ? omega[i] / alpha[i] : duration / 2;;
                traj.coeff[i].brake_time = time + delta[i] > 0 ? duration - omega[i] / alpha[i] : duration / 2;
                traj.coeff[i].stop_time = time + duration;
            }
            else
            {
                // replan traj using duration
                double t2 = sqrt(duration * duration - trips[i] / alpha[i] * 4);
                traj.coeff[i].start_time = time;
                traj.coeff[i].stable_time = time + (duration - t2) / 2;
                traj.coeff[i].brake_time = time + (duration + t2) / 2;
                traj.coeff[i].stop_time = time + duration;
            }

            traj.coeff[i].start_alpha = *tar > *start ? alpha[i] : -alpha[i];
            traj.coeff[i].brake_alpha = -traj.coeff[i].start_alpha;
        }

        *(&traj.joint_ending.j1 + i) = *tar;

        ++ start;
        ++ tar;

        FST_INFO("  J%d: t1=%.4f,t2=%.4f,t3=%.4f,t4=%.4f,alpha1-2=%.4f,alpha3-4=%.4f",
                 i + 1, traj.coeff[i].start_time, traj.coeff[i].stable_time,
                 traj.coeff[i].brake_time, traj.coeff[i].stop_time,
                 traj.coeff[i].start_alpha, traj.coeff[i].brake_alpha);
    }

    traj.duration = duration;
    FST_INFO("  total duration=%.4f", traj.duration);
    FST_INFO("Success !");
    return SUCCESS;
}

ErrorCode ManualTeach::manualCartesian(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj)
{
    ErrorCode err = SUCCESS;
    
    if (traj.mode == STEP)
    {
        err = manualCartesianStep(dir, time, traj);
    }
    else if (traj.mode == CONTINUOUS)
    {
        err = manualCartesianContinuous(dir, time, traj);
    }
    else
    {
        FST_ERROR("manualJoint: manual-mode=%d, but given 6 directions");
        err = MOTION_INTERNAL_FAULT;
    }

    if (err != SUCCESS)
    {
        FST_ERROR("manualJoint failed, err=0x%llx", err);
    }

    return err;
}

ErrorCode ManualTeach::manualCartesianStep(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj)
{
    double speed_ratio  = g_global_vel_ratio;
    double acc_ratio    = g_global_acc_ratio;

    forwardKinematics(traj.joint_start, traj.cart_start);
    PoseEuler &start = traj.cart_start;
    PoseEuler target = traj.cart_start;

    if      (dir[0] == INCREASE) {target.position.x += g_manual_step_position;}
    else if (dir[0] == DECREASE) {target.position.x -= g_manual_step_position;}
    if      (dir[1] == INCREASE) {target.position.y += g_manual_step_position;}
    else if (dir[1] == DECREASE) {target.position.y -= g_manual_step_position;}
    if      (dir[2] == INCREASE) {target.position.z += g_manual_step_position;}
    else if (dir[2] == DECREASE) {target.position.z -= g_manual_step_position;}
    if      (dir[3] == INCREASE) {target.orientation.a += g_manual_step_orientation;}
    else if (dir[3] == DECREASE) {target.orientation.a -= g_manual_step_orientation;}
    if      (dir[4] == INCREASE) {target.orientation.b += g_manual_step_orientation;}
    else if (dir[4] == DECREASE) {target.orientation.b -= g_manual_step_orientation;}
    if      (dir[5] == INCREASE) {target.orientation.c += g_manual_step_orientation;}
    else if (dir[5] == DECREASE) {target.orientation.c -= g_manual_step_orientation;}
    traj.cart_ending = target;

    FST_INFO("manual-Cartesian-STEP: directions = %d %d %d %d %d %d, planning trajectory ...",
             dir[0], dir[1], dir[2], dir[3], dir[4], dir[5]);
    FST_INFO("  step_postion = %.2fmm, step_angle = %.2frad, speed_ratio = %.0f%%, acc_ratio=%.0f%%",
             g_manual_step_position, g_manual_step_orientation, speed_ratio * 100, acc_ratio * 100);
    FST_INFO("  start  pose  = %.2f %.2f %.2f - %.4f %.4f %.4f",
             start.position.x, start.position.y, start.position.z,
             start.orientation.a, start.orientation.b, start.orientation.c);
    FST_INFO("  target pose  = %.2f %.2f %.2f - %.4f %.4f %.4f",
             target.position.x, target.position.y, target.position.z,
             target.orientation.a, target.orientation.b, target.orientation.c);
    FST_INFO("  start joint  = %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
             traj.joint_start.j1, traj.joint_start.j2, traj.joint_start.j3, 
             traj.joint_start.j4, traj.joint_start.j5, traj.joint_start.j6);

    ErrorCode err = SUCCESS;

    if (!isJointInConstraint(traj.joint_start, g_soft_constraint))
    {
        FST_ERROR("start joint out of constraint, manual cartesian is disabled.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    Joint target_joint;
    err = inverseKinematics(target, traj.joint_start, traj.joint_ending);
    if (err != SUCCESS)
    {
        FST_ERROR("Cannot find a valid inverse result of the target pose, err=%0xllx", err);
        return err;
    }
    FST_INFO("  target joint = %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
             traj.joint_ending.j1, traj.joint_ending.j2, traj.joint_ending.j3, 
             traj.joint_ending.j4, traj.joint_ending.j5, traj.joint_ending.j6);
    if (!isJointInConstraint(traj.joint_ending, g_soft_constraint))
    {
        err = JOINT_OUT_OF_CONSTRAINT;
        FST_ERROR("Target joint out of constraint, err=0x%llx", err);
        return err;
    }

    double dis = (dir[0] != STANDBY || dir[1] != STANDBY || dir[2] != STANDBY) ? g_manual_step_position : 0;
    double spd = g_cart_vel_max * speed_ratio;
    double acc = g_cart_acc_max * acc_ratio;

    double angle = (dir[3] != STANDBY || dir[4] != STANDBY || dir[5] != STANDBY) ? g_manual_step_orientation : 0;
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
        if (dir[i] == STANDBY)
        {
            traj.coeff[i].start_time = time;
            traj.coeff[i].stable_time = time;
            traj.coeff[i].brake_time = time;
            traj.coeff[i].stop_time = time;
            traj.coeff[i].start_alpha = 0;
            traj.coeff[i].brake_alpha = 0;
        }
        else
        {
            if (duration < t_pos + MINIMUM_E12)
            {
                traj.coeff[i].start_time = time;
                traj.coeff[i].stable_time = time + delta > 0 ? spd / acc : duration / 2;
                traj.coeff[i].brake_time = time + delta > 0 ? duration - spd / acc : duration / 2;
                traj.coeff[i].stop_time = duration;
            }
            else
            {
                // replan position traj
                double t_stable = sqrt(duration * duration - dis / acc * 4);
                traj.coeff[i].start_time = time;
                traj.coeff[i].stable_time = time + (duration - t_stable) / 2;
                traj.coeff[i].brake_time = time + (duration + t_stable) / 2;
                traj.coeff[i].stop_time = time + duration;
            }

            traj.coeff[i].start_alpha = dir[i] == INCREASE ? acc : -acc;
            traj.coeff[i].brake_alpha = -traj.coeff[i].start_alpha;
        }

        traj.direction[i] = dir[i];
        FST_INFO("  Dir-%d: t1=%.4f,t2=%.4f,t3=%.4f,t4=%.4f,alpha1-2=%.4f,alpha3-4=%.4f",
                 i + 1, traj.coeff[i].start_time, traj.coeff[i].stable_time,
                 traj.coeff[i].brake_time, traj.coeff[i].stop_time,
                 traj.coeff[i].start_alpha, traj.coeff[i].brake_alpha);
    }

    for (size_t i = 3; i < 6; i++)
    {
        if (dir[i] == STANDBY)
        {
            traj.coeff[i].start_time = time;
            traj.coeff[i].stable_time = time;
            traj.coeff[i].brake_time = time;
            traj.coeff[i].stop_time = time;
            traj.coeff[i].start_alpha = 0;
            traj.coeff[i].brake_alpha = 0;
        }
        else
        {
            if (duration < t_ort + MINIMUM_E12)
            {
                traj.coeff[i].start_time = time;
                traj.coeff[i].stable_time = time + theta > 0 ? omega / alpha : duration / 2;
                traj.coeff[i].brake_time = time + theta > 0 ? duration - omega / alpha : duration / 2;
                traj.coeff[i].stop_time = time + duration;
            }
            else
            {
                // replan orientation traj
                double t_stable = sqrt(duration * duration - angle / alpha * 4);
                traj.coeff[i].start_time = time;
                traj.coeff[i].stable_time = time + (duration - t_stable) / 2;
                traj.coeff[i].brake_time = time + (duration + t_stable) / 2;
                traj.coeff[i].stop_time = time + duration;
            }

            traj.coeff[i].start_alpha = dir[i] == INCREASE ? alpha : -alpha;
            traj.coeff[i].brake_alpha = -traj.coeff[i].start_alpha;
        }

        traj.direction[i] = dir[i];
        FST_INFO("  Dir-%d: t1=%.4f,t2=%.4f,t3=%.4f,t4=%.4f,alpha1-2=%.4f,alpha3-4=%.4f",
                 i + 1, traj.coeff[i].start_time, traj.coeff[i].stable_time,
                 traj.coeff[i].brake_time, traj.coeff[i].stop_time,
                 traj.coeff[i].start_alpha, traj.coeff[i].brake_alpha);
    }

    traj.duration = duration;
    FST_INFO("  total duration=%.4f", traj.duration);
    FST_INFO("Success !");
    return SUCCESS;
}

ErrorCode ManualTeach::manualCartesianContinuous(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj)
{
    ErrorCode err = SUCCESS;

    double speed_ratio  = g_global_vel_ratio;
    double acc_ratio    = g_global_acc_ratio;

    forwardKinematics(traj.joint_start, traj.cart_start);
    PoseEuler &start = traj.cart_start;
    PoseEuler target = traj.cart_start;

    FST_INFO("manual-Cartesian-CONTINUOUS: directions = %d %d %d %d %d %d, planning trajectory ...",
             dir[0], dir[1], dir[2], dir[3], dir[4], dir[5]);
    FST_INFO("  speed_ratio = %.0f%%, acc_ratio=%.0f%%", speed_ratio * 100, acc_ratio * 100);
    FST_INFO("  start  pose  = %.2f %.2f %.2f - %.4f %.4f %.4f",
             start.position.x, start.position.y, start.position.z,
             start.orientation.a, start.orientation.b, start.orientation.c);

    double spd, acc;
    size_t i;

    for (i = 0; i < 6; i++)
    {
        if (traj.direction[i] == dir[i])
        {
            // keep standby or motion direction
            // do nothing
            if (traj.direction[i] == STANDBY)
            {
                *(&traj.cart_ending.position.x + i) = *(&traj.cart_start.position.x + i);
            }
            FST_INFO("  Dir-%d: given direction same as current motion, running along the current trajectory", i + 1);
            continue;
        }
        else if (dir[i] == STANDBY && traj.direction[i] != STANDBY)
        {
            // stop motion in this direction
            if (time < traj.coeff[i].start_time)
            {
                traj.coeff[i].start_time = time;
                traj.coeff[i].stable_time = time;
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time;
                traj.coeff[i].start_alpha = 0;
                traj.coeff[i].brake_alpha = 0;
                traj.direction[i] = STANDBY;
                *(&traj.cart_ending.position.x + i) = *(&traj.cart_start.position.x + i);
            }
            else if (time < traj.coeff[i].stable_time)
            {
                double tm = time - traj.coeff[i].start_time;
                traj.coeff[i].stable_time = time;
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time + tm;
                traj.direction[i] = STANDBY;
                *(&traj.cart_ending.position.x + i) = *(&traj.cart_start.position.x + i) + traj.coeff[i].start_alpha * tm * tm;
            }
            else if (time < traj.coeff[i].brake_time)
            {
                double tim = traj.coeff[i].stable_time - traj.coeff[i].start_time;
                double omg = tim * traj.coeff[i].start_alpha;
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time + tim;
                traj.direction[i] = STANDBY;
                *(&traj.cart_ending.position.x + i) = *(&traj.cart_start.position.x + i) + omg * tim;
                tim = time - traj.coeff[i].stable_time;
                *(&traj.cart_ending.position.x + i) += omg * tim;
            }
            else
            {
                // already in axis-slow-down trajectory, do nothing
                FST_WARN("  Dir-%d: Standby or in slow-down trajectory, will stop soon", i + 1);
                continue;
            }
        }
        else if (dir[i] != STANDBY && traj.direction[i] == STANDBY)
        {
            if (time >= traj.coeff[i].stop_time)
            {
                // start motion in this direction
                spd = i < 3 ? g_cart_vel_reference * speed_ratio : g_orientation_omega_reference * speed_ratio;
                acc = i < 3 ? g_cart_acc_reference * acc_ratio : g_orientation_alpha_reference * acc_ratio;

                traj.coeff[i].start_time = time;
                traj.coeff[i].stable_time = time + spd / acc;
                traj.coeff[i].brake_time = 32 - spd / acc;
                traj.coeff[i].stop_time = 32;
                traj.coeff[i].start_alpha = dir[i] == INCREASE ? acc : -acc;
                traj.coeff[i].brake_alpha = -traj.coeff[i].start_alpha;
                traj.direction[i] = dir[i];
                double trip = spd * spd / acc + (32 - spd / acc * 2) * spd;
                *(&traj.cart_ending.position.x + i) = *(&traj.cart_start.position.x + i) + 
                                                        (dir[i] == INCREASE ? trip : -trip);
            }
            else
            {
                // axis-slow-down trajectory havn't finished
                FST_WARN("  Dir-%d: in slow-down trajectory, cannot startup until standby", i + 1);
                continue;
            }
        }
        else
        {
            // prev-direction = INCREASE && input-direction = DECREASE or
            // prev-direction = DECREASE && input-direction = INCREASE,
            // stop motion in this direction
            traj.direction[i] = STANDBY;
            if (time < traj.coeff[i].start_time)
            {
                traj.coeff[i].start_time = time;
                traj.coeff[i].stable_time = time;
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time;
                traj.coeff[i].start_alpha = 0;
                traj.coeff[i].brake_alpha = 0;
                *(&traj.cart_ending.position.x + i) = *(&traj.cart_start.position.x + i);
            }
            else if (time < traj.coeff[i].stable_time)
            {
                double tm = time - traj.coeff[i].start_time;
                traj.coeff[i].stable_time = time;
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time + tm;
                *(&traj.cart_ending.position.x + i) = *(&traj.cart_start.position.x + i) + traj.coeff[i].start_alpha * tm * tm;
            }
            else if (time < traj.coeff[i].brake_time)
            {
                double tim = traj.coeff[i].stable_time - traj.coeff[i].start_time;
                double omg = tim * traj.coeff[i].start_alpha;
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time + tim;
                *(&traj.cart_ending.position.x + i) = *(&traj.cart_start.position.x + i) + omg * tim;
                tim = time - traj.coeff[i].stable_time;
                *(&traj.cart_ending.position.x + i) += omg * tim;
            }
            else
            {
                // already in axis-slow-down trajectory, do nothing
                FST_WARN("  Dir-%d: Standby or in slow-down trajectory, will stop soon", i + 1);
                continue;
            }
        }

        FST_INFO("  Dir-%d: t1=%.4f,t2=%.4f,t3=%.4f,t4=%.4f,alpha1-2=%.4f,alpha3-4=%.4f",
                 i + 1, traj.coeff[i].start_time, traj.coeff[i].stable_time,
                 traj.coeff[i].brake_time, traj.coeff[i].stop_time,
                 traj.coeff[i].start_alpha, traj.coeff[i].brake_alpha);
    }

    double duration = 0;
    for (size_t i = 0; i < 6; i++)
    {
        if (traj.coeff[i].stop_time > duration)
        {
            duration = traj.coeff[i].stop_time;
        }
    }
    traj.duration = duration;
    FST_INFO("  total duration=%.4f", traj.duration);
    FST_INFO("Success !");
    return SUCCESS;
}

ErrorCode ManualTeach::repairCartesianContinuous(MotionTime time, MotionTime front, ManualTrajectory &traj)
{
    PoseEuler   cart_ending;
    MotionTime  tim_total = time + front;
    FST_INFO("repairCartContinuous: repair cart trajectory");

    double *value = &cart_ending.position.x;
    double *start = &traj.cart_start.position.x;
    double *target = &traj.cart_ending.position.x;
    double tim, vel;

    for (size_t i = 0; i < 6; i++)
    {
        if (tim_total < traj.coeff[i].start_time)
        {
            *value = *start;
        }
        else if (tim_total < traj.coeff[i].stable_time)
        {
            tim = tim_total - traj.coeff[i].start_time;
            *value = *start + traj.coeff[i].start_alpha * tim * tim / 2;
        }
        else if (tim_total < traj.coeff[i].brake_time)
        {
            tim = traj.coeff[i].stable_time - traj.coeff[i].start_time;
            vel = traj.coeff[i].start_alpha * tim;
            *value = *start + vel * tim / 2;
            tim = tim_total - traj.coeff[i].stable_time;
            *value = *value + vel * tim;
        }
        else if (tim_total < traj.coeff[i].stop_time)
        {
            tim = traj.coeff[i].stable_time - traj.coeff[i].start_time;
            vel = traj.coeff[i].start_alpha * tim;
            *value = *start + vel * tim / 2;
            tim = traj.coeff[i].brake_time - traj.coeff[i].stable_time;
            *value = *value + vel * tim;
            tim = tim_total - traj.coeff[i].brake_time;
            *value = *value + vel * tim + traj.coeff[i].brake_alpha * tim * tim / 2;
        }
        else
        {
            *value = *target;
        }

        ++ value;
        ++ start;
        ++ target;
    }

    FST_INFO("  stop-pose=%.2f,%.2f,%.2f - %.4f,%.4f,%.4f",
              cart_ending.position.x, cart_ending.position.y, cart_ending.position.z,
              cart_ending.orientation.a, cart_ending.orientation.b, cart_ending.orientation.c);

    // TODO 
    for (size_t i = 0; i < 6; i++)
    {
        traj.direction[i] = STANDBY;
        if (time < traj.coeff[i].start_time)
        {
            traj.coeff[i].start_time = time;
            traj.coeff[i].stable_time = time;
            traj.coeff[i].brake_time = time;
            traj.coeff[i].stop_time = time;
            traj.coeff[i].start_alpha = 0;
            traj.coeff[i].brake_alpha = 0;
            *(&traj.cart_ending.position.x + i) = *(&traj.cart_start.position.x + i);
        }
        else if (time < traj.coeff[i].stable_time)
        {
            tim = time - traj.coeff[i].start_time;
            traj.coeff[i].stable_time = time;
            traj.coeff[i].brake_time = time;
            traj.coeff[i].stop_time = time + tim;
            *(&traj.cart_ending.position.x + i) = *(&traj.cart_start.position.x + i) + traj.coeff[i].start_alpha * tim * tim;
        }
        else if (time < traj.coeff[i].brake_time)
        {
            tim = traj.coeff[i].stable_time - traj.coeff[i].start_time;
            vel = tim * traj.coeff[i].start_alpha;
            traj.coeff[i].brake_time = time;
            traj.coeff[i].stop_time = time + tim;
            *(&traj.cart_ending.position.x + i) = *(&traj.cart_start.position.x + i) + vel * tim;
            tim = time - traj.coeff[i].stable_time;
            *(&traj.cart_ending.position.x + i) += vel * tim;
        }
        else
        {
            // already in axis-slow-down trajectory, do nothing
            FST_WARN("  Dir-%d: Standby or in slow-down trajectory, will stop soon", i + 1);
            continue;
        }
        FST_INFO("  Dir-%d: t1=%.4f,t2=%.4f,t3=%.4f,t4=%.4f,alpha1-2=%.4f,alpha3-4=%.4f",
                 i + 1, traj.coeff[i].start_time, traj.coeff[i].stable_time,
                 traj.coeff[i].brake_time, traj.coeff[i].stop_time,
                 traj.coeff[i].start_alpha, traj.coeff[i].brake_alpha);
    }

    double duration = 0;
    for (size_t i = 0; i < 6; i++)
    {
        if (traj.coeff[i].stop_time > duration)
        {
            duration = traj.coeff[i].stop_time;
        }
    }
    traj.duration = duration;
    FST_INFO("  total duration=%.4f", traj.duration);
    FST_INFO("Success !");
    return SUCCESS;
}




