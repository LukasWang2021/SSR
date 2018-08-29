/*************************************************************************
	> File Name: motion_control_manual_teach.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 15时48分54秒
 ************************************************************************/

#include <math.h>
#include <float.h>
#include <iostream>
#include <sstream>
#include <string>
#include <string.h>
#include <vector>

#include <motion_control_manual_teach.h>

#define FST_LOG(fmt, ...)       log_ptr_->log(fmt, ##__VA_ARGS__)
#define FST_INFO(fmt, ...)      log_ptr_->info(fmt, ##__VA_ARGS__)
#define FST_WARN(fmt, ...)      log_ptr_->warn(fmt, ##__VA_ARGS__)
#define FST_ERROR(fmt, ...)     log_ptr_->error(fmt, ##__VA_ARGS__)

using namespace std;

namespace fst_mc
{

ManualTeach::ManualTeach(void)
{
    joint_num_ = 0;
    log_ptr_ = NULL;
    joint_constraint_ptr_ = NULL;

    vel_ratio_ = 0;
    acc_ratio_ = 0;
    step_joint_ = 0;
    step_position_ = 0;
    step_orientation_ = 0;
}

ManualTeach::~ManualTeach(void)
{}

ErrorCode ManualTeach::init(size_t joint_num, Constraint *pcons, fst_log::Logger *plog)
{
    if (joint_num > 0 && joint_num <= NUM_OF_JOINT && pcons && plog)
    {
        joint_num_ = joint_num;
        log_ptr_ = plog;
        joint_constraint_ptr_ = pcons;
    }
    else
    {
        return MOTION_INTERNAL_FAULT;
    }

    vel_ratio_ = 1;
    acc_ratio_ = 1;
    step_joint_ = 0.05;
    step_position_ = 0.1;
    step_orientation_ = 0.05;

    return SUCCESS;
}

ErrorCode ManualTeach::manualStepByDirect(const ManualDirection *directions, MotionTime time, ManualTrajectory &traj)
{
    ErrorCode err = SUCCESS;
    FST_INFO("Manual step request received, frame=%d", traj.frame);

    switch (traj.frame)
    {
        case JOINT:
            err = manualJointStep(directions, time, traj);
            break;
        case WORLD:
        case USER:
        case TOOL:
            //err = manualCartesian(directions, time, traj);
            //break;
        default:
            err = MOTION_INTERNAL_FAULT;
            FST_ERROR("Unsupported manual frame: %d", traj.frame);
            break;
    }

    if (err == SUCCESS)
    {
        FST_INFO("Manual trajectory ready.");
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Manual failed, err = 0x%llx", err);
        return err;
    }
}

ErrorCode ManualTeach::manualJointStep(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj)
{
    char buffer[LOG_TEXT_SIZE];

    FST_INFO("manual-Joint-Step: directions = %s, planning trajectory ...", printDBLine((int*)dir, buffer, LOG_TEXT_SIZE));
    FST_INFO("  step_angle = %.4frad, vel_ratio = %.0f%%, acc_ratio = %.0f%%", step_joint_, vel_ratio_ * 100, acc_ratio_ * 100);
    FST_INFO("  start  joint = %s", printDBLine(&traj.joint_start[0], buffer, LOG_TEXT_SIZE));

    Joint target  = traj.joint_start;

    for (size_t i = 0; i < joint_num_; i++)
    {
        if      (dir[i] == INCREASE) target[i] += step_joint_;
        else if (dir[i] == DECREASE) target[i] -= step_joint_;
    }

    FST_INFO("  target joint = %s", printDBLine(&target[0], buffer, LOG_TEXT_SIZE));

    if (!joint_constraint_ptr_->isJointInConstraint(target))
    {
        FST_ERROR("Target out of soft constraint.");
        return TARGET_OUT_OF_CONSTRAINT;
    }

    double omega_limit[NUM_OF_JOINT] = {5.81, 4.66, 5.81, 7.85, 7.07, 10.56, 0, 0, 0};
    double alpha_limit[NUM_OF_JOINT] = {11.62, 9.32, 16.6, 22.44, 28.27, 42.24, 0, 0, 0};
    double omega[NUM_OF_JOINT], alpha[NUM_OF_JOINT], trips[NUM_OF_JOINT], delta[NUM_OF_JOINT];
    double duration = 0;
    double t_min[NUM_OF_JOINT];

    for (size_t i = 0; i < joint_num_; i++)
    {
        trips[i] = dir[i] == STANDING ? 0 : step_joint_;
        omega[i] = omega_limit[i] * vel_ratio_;
        alpha[i] = alpha_limit[i] * acc_ratio_;

        delta[i] = trips[i] - omega[i] * omega[i] / alpha[i];
        t_min[i] = delta[i] > 0 ? (omega[i] / alpha[i] + trips[i] / omega[i]) : (sqrt(trips[i] / alpha[i]) * 2);
        FST_INFO("  J%d: trip=%.4f omega=%f, alpha=%f, delta=%f, tmin=%f", i + 1, trips[i], omega[i], alpha[i], delta[i], t_min[i]);
        if (t_min[i] > duration) duration = t_min[i];
    }

    //FST_INFO("  t1=%f, t2=%f, t3=%f, t4=%f, t5=%f, t6=%f",
    //         t_min[0], t_min[1], t_min[2], t_min[3], t_min[4], t_min[5]);
    FST_INFO("  duration=%.4f, create trajectory ...", duration);

    traj.duration = duration;
    traj.joint_ending = target;

    for (size_t i = 0; i < joint_num_; i++)
    {
        traj.direction[i] = dir[i];
        if (dir[i] == STANDING)
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
            if (duration < t_min[i] + MINIMUM_E6)
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





ErrorCode ManualTeach::manualContinuousByDirect(const ManualDirection *directions, MotionTime time, ManualTrajectory &traj)
{
    ErrorCode err = SUCCESS;
    FST_INFO("Manual continuous request received, frame=%d", traj.frame);

    switch (traj.frame)
    {
        case JOINT:
            err = manualJointContinuous(directions, time, traj);
            break;

        case BASE:
        case USER:
        case WORLD:
            err = manualCartesianContinuous(directions, time, traj);
            break;
        case TOOL:
            //err = manualCartesianContinuousInToolFrame(directions, time, traj);
            break;
        default:
            err = MOTION_INTERNAL_FAULT;
            FST_ERROR("Unsupported manual frame: %d", traj.frame);
            break;
    }

    if (err == SUCCESS)
    {
        FST_INFO("Manual trajectory ready.");
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Manual failed, err = 0x%llx", err);
        return err;
    }
}

ErrorCode ManualTeach::manualJointContinuous(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj)
{
    // FIXME
    double omega_limit[NUM_OF_JOINT] = {5.81, 4.66, 5.81, 7.85, 7.07, 10.56, 0, 0, 0};
    double alpha_limit[NUM_OF_JOINT] = {11.62, 9.32, 16.6, 22.44, 28.27, 42.24, 0, 0, 0};

    double alpha, omega;
    char buffer[LOG_TEXT_SIZE];

    FST_INFO("manual-Joint-Continuous: directions = %s, planning trajectory ...", printDBLine((int*)dir, buffer, LOG_TEXT_SIZE));
    FST_INFO("  manual time = %.4f, vel_ratio = %.0f%%, acc_ratio = %.0f%%", time, vel_ratio_ * 100, acc_ratio_ * 100);
    FST_INFO("  start  joint = %s", printDBLine(&traj.joint_start[0], buffer, LOG_TEXT_SIZE));

    if (traj.duration < MINIMUM_E6)
    {
        // begin to manual-continuous, update ending joint
        traj.joint_ending = traj.joint_start;
    }

    for (size_t i = 0; i < joint_num_; i++)
    {
        alpha = alpha_limit[i] * acc_ratio_;
        omega = omega_limit[i] * vel_ratio_;

        if (traj.direction[i] == dir[i])
        {
            // keep STANDING or motion
            // do nothing
            FST_INFO("  J%d: given direction same as current motion, running along the original trajectory", i + 1);
            continue;
        }
        else if (dir[i] == STANDING && traj.direction[i] != STANDING)
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
                traj.direction[i] = STANDING;
                *(&traj.joint_ending.j1 + i) = *(&traj.joint_start.j1 + i);
            }
            else if (time < traj.coeff[i].stable_time)
            {
                traj.coeff[i].stable_time = time;
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time + traj.coeff[i].stable_time - traj.coeff[i].start_time;
                traj.direction[i] = STANDING;
                double tm = time - traj.coeff[i].start_time;
                *(&traj.joint_ending.j1 + i) = *(&traj.joint_start.j1 + i) + traj.coeff[i].start_alpha * tm * tm;
            }
            else if (time < traj.coeff[i].brake_time)
            {
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time + traj.coeff[i].stable_time - traj.coeff[i].start_time;
                traj.direction[i] = STANDING;
                double tim = traj.coeff[i].stable_time - traj.coeff[i].start_time;
                double omg = tim * traj.coeff[i].start_alpha;
                *(&traj.joint_ending.j1 + i) = *(&traj.joint_start.j1 + i) + omg * tim;
                tim = time - traj.coeff[i].stable_time;
                *(&traj.joint_ending.j1 + i) += omg * tim;
            }
            else
            {
                // already in axis-slow-down trajectory, do nothing
                FST_WARN("  J%d: STANDING or in slow-down trajectory, will stop on soft constraint", i + 1);
                continue;
            }
        }
        else if (dir[i] != STANDING && traj.direction[i] == STANDING)
        {
            if (time >= traj.coeff[i].stop_time)
            {
                // start joint motion
                // stop until soft constraint
                double trip;

                if (joint_constraint_ptr_->isJointMasked(i))
                {
                    trip = 99.99;
                }
                else
                {
                    trip = dir[i] == INCREASE ? joint_constraint_ptr_->upper()[i] - traj.joint_start[i] :
                           traj.joint_start[i] - joint_constraint_ptr_->lower()[i];
                }

                if (trip > MINIMUM_E6)
                {
                    double delta = trip - omega * omega / alpha;
                    if (delta > 0)
                    {
                        traj.coeff[i].start_time = time;
                        traj.coeff[i].stable_time = traj.coeff[i].start_time + omega / alpha;
                        traj.coeff[i].brake_time = traj.coeff[i].stable_time + delta / omega;
                        traj.coeff[i].stop_time = traj.coeff[i].brake_time + omega / alpha;
                    }
                    else
                    {
                        double t_startup = sqrt(trip / alpha);
                        traj.coeff[i].start_time = time;
                        traj.coeff[i].stable_time = traj.coeff[i].start_time + t_startup;
                        traj.coeff[i].brake_time = traj.coeff[i].stable_time;
                        traj.coeff[i].stop_time = traj.coeff[i].brake_time + t_startup;

                    }
                    traj.coeff[i].start_alpha = dir[i] == INCREASE ? alpha : -alpha;
                    traj.coeff[i].brake_alpha = -traj.coeff[i].start_alpha;
                    traj.direction[i] = dir[i];

                    if (joint_constraint_ptr_->isJointMasked(i))
                    {
                        traj.joint_ending[i] = dir[i] == INCREASE ? traj.joint_start[i] + trip :
                                               traj.joint_start[i] - trip;
                    }
                    else
                    {
                        traj.joint_ending[i] = dir[i] == INCREASE ? joint_constraint_ptr_->upper()[i] :
                                               joint_constraint_ptr_->lower()[i];
                    }
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
                FST_WARN("  J%d: in slow-down trajectory, cannot startup until STANDING", i + 1);
                continue;
            }
        }
        else
        {
            // prev-direction = INCREASE && input-direction = DECREASE or
            // prev-direction = DECREASE && input-direction = INCREASE,
            // stop joint motion
            traj.direction[i] = STANDING;
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
                FST_WARN("  J%d: STANDING or in slow-down trajectory, will stop on soft constraint", i + 1);
                continue;
            }
        }

        FST_INFO("  J%d: t1=%.4f,t2=%.4f,t3=%.4f,t4=%.4f,alpha1-2=%.4f,alpha3-4=%.4f",
                 i + 1, traj.coeff[i].start_time, traj.coeff[i].stable_time,
                 traj.coeff[i].brake_time, traj.coeff[i].stop_time,
                 traj.coeff[i].start_alpha, traj.coeff[i].brake_alpha);
    }

    double duration = 0;

    for (size_t i = 0; i < joint_num_; i++)
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

ErrorCode ManualTeach::manualCartesianContinuous(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj)
{
    char buffer[LOG_TEXT_SIZE];

    FST_INFO("manual-Cartesian-Continuous: directions = %s, planning trajectory ...", printDBLine((int*)dir, buffer, LOG_TEXT_SIZE));
    FST_INFO("  manual time = %.4f, vel_ratio = %.0f%%, acc_ratio = %.0f%%", time, vel_ratio_ * 100, acc_ratio_ * 100);
    FST_INFO("  start  joint = %s", printDBLine(&traj.joint_start[0], buffer, LOG_TEXT_SIZE));

    //forwardKinematics(traj.joint_start, traj.cart_start);
    PoseEuler &start = traj.cart_start;
    PoseEuler target = traj.cart_start;

    FST_INFO("manual-Cartesian-CONTINUOUS: directions = %d %d %d %d %d %d, planning trajectory ...",
             dir[0], dir[1], dir[2], dir[3], dir[4], dir[5]);
    FST_INFO("  speed_ratio = %.0f%%, acc_ratio=%.0f%%", vel_ratio_ * 100, acc_ratio_ * 100);
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
            if (traj.direction[i] == STANDING)
            {
                *(&traj.cart_ending.position.x + i) = *(&traj.cart_start.position.x + i);
            }
            FST_INFO("  Dir-%d: given direction same as current motion, running along the current trajectory", i + 1);
            continue;
        }
        else if (dir[i] == STANDING && traj.direction[i] != STANDING)
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
                traj.direction[i] = STANDING;
                *(&traj.cart_ending.position.x + i) = *(&traj.cart_start.position.x + i);
            }
            else if (time < traj.coeff[i].stable_time)
            {
                double tm = time - traj.coeff[i].start_time;
                traj.coeff[i].stable_time = time;
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time + tm;
                traj.direction[i] = STANDING;
                *(&traj.cart_ending.position.x + i) = *(&traj.cart_start.position.x + i) + traj.coeff[i].start_alpha * tm * tm;
            }
            else if (time < traj.coeff[i].brake_time)
            {
                double tim = traj.coeff[i].stable_time - traj.coeff[i].start_time;
                double omg = tim * traj.coeff[i].start_alpha;
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time + tim;
                traj.direction[i] = STANDING;
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
        else if (dir[i] != STANDING && traj.direction[i] == STANDING)
        {
            if (time >= traj.coeff[i].stop_time)
            {
                // start motion in this direction
                spd = i < 3 ? position_vel_reference_ * vel_ratio_ : orientation_omega_reference_ * vel_ratio_;
                acc = i < 3 ? position_acc_reference_ * acc_ratio_ : orientation_alpha_reference_ * acc_ratio_;

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
            traj.direction[i] = STANDING;
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


ErrorCode ManualTeach::manualByTarget(const Joint &target, MotionTime time, ManualTrajectory &traj)
{
    FST_INFO("Manual request received, frame=%d", traj.frame);
    return manualJointAPoint(target, time, traj);
}

ErrorCode ManualTeach::manualJointAPoint(const Joint &target, MotionTime time, ManualTrajectory &traj)
{
    // FIXME
    double omega_limit[NUM_OF_JOINT] = {5.81, 4.66, 5.81, 7.85, 7.07, 10.56, 0, 0, 0};
    double alpha_limit[NUM_OF_JOINT] = {11.62, 9.32, 16.6, 22.44, 28.27, 42.24, 0, 0, 0};

    char buffer[LOG_TEXT_SIZE];

    FST_INFO("manual-Joint-APOINT: planning trajectory ...");
    FST_INFO("  vel-ratio = %.0f%%, acc-ratio = %.0f%%", vel_ratio_ * 100, acc_ratio_ * 100);
    FST_INFO("  start  joint = %s", printDBLine(&traj.joint_start[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  target joint = %s", printDBLine(&target[0], buffer, LOG_TEXT_SIZE));

    double trips[NUM_OF_JOINT], alpha[NUM_OF_JOINT], omega[NUM_OF_JOINT], t_min[NUM_OF_JOINT], delta[NUM_OF_JOINT];
    double duration = 0;

    for (size_t i = 0; i < joint_num_; i++)
    {
        trips[i] = fabs(target[i] - traj.joint_start[i]);
        alpha[i] = alpha_limit[i] * acc_ratio_;
        omega[i] = omega_limit[i] * vel_ratio_;
        delta[i] = trips[i] - omega[i] * omega[i] / alpha[i];
        t_min[i] = delta[i] > 0 ? (omega[i] / alpha[i] + trips[i] / omega[i]) : (sqrt(trips[i] / alpha[i]) * 2);
        FST_INFO("  J%d: trip=%.8f omega=%f, alpha=%f, delta=%f, tmin=%f", i + 1, trips[i], omega[i], alpha[i], delta[i], t_min[i]);
        //FST_INFO("  J%d: t=%f", i + 1, sqrt(trips[i] / alpha[i]) * 2);
        if (t_min[i] > duration) duration = t_min[i];
    }

    FST_INFO("  duration=%.4f, create trajectory", duration);

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
            if (duration < t_min[i] + MINIMUM_E6)
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

            traj.coeff[i].start_alpha = target[i] > traj.joint_start[i] ? alpha[i] : -alpha[i];
            traj.coeff[i].brake_alpha = -traj.coeff[i].start_alpha;
        }

        traj.joint_ending[i] = target[i];

        FST_INFO("  J%d: t1=%.4f,t2=%.4f,t3=%.4f,t4=%.4f,alpha1-2=%.4f,alpha3-4=%.4f",
                 i + 1, traj.coeff[i].start_time, traj.coeff[i].stable_time,
                 traj.coeff[i].brake_time, traj.coeff[i].stop_time,
                 traj.coeff[i].start_alpha, traj.coeff[i].brake_alpha);
    }

    traj.duration = duration;
    FST_INFO("Success !");
    return SUCCESS;
}



ErrorCode ManualTeach::manualStop(MotionTime time, ManualTrajectory &traj)
{
    /*
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
            traj.direction[i] = STANDING;
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
            traj.direction[i] = STANDING;
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
            traj.direction[i] = STANDING;
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
            FST_WARN("  J%d/Dir%d: STANDING or in slow-down trajectory, will stop on soft constraint", i + 1, i + 1);
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
    */
    return SUCCESS;
}

char* ManualTeach::printDBLine(const int *data, char *buffer, size_t length)
{
    int len = 0;

    for (size_t i = 0; i < joint_num_; i++)
    {
        len += snprintf(buffer + len, length - len, "%d ", data[i]);
    }

    if (len > 0)
    {
        len --;
    }

    buffer[len] = '\0';
    return buffer;
}

char* ManualTeach::printDBLine(const double *data, char *buffer, size_t length)
{
    int len = 0;

    for (size_t i = 0; i < joint_num_; i++)
    {
        len += snprintf(buffer + len, length - len, "%.6f ", data[i]);
    }

    if (len > 0)
    {
        len --;
    }

    buffer[len] = '\0';
    return buffer;
}


}


