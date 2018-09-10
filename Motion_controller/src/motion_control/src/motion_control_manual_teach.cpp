/*************************************************************************
	> File Name: motion_control_manual_teach.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 15时48分54秒
 ************************************************************************/

#include <math.h>
#include <float.h>
#include <iostream>
#include <string>
#include <string.h>
#include <vector>

#include <motion_control_manual_teach.h>
#include <common_log.h>
#include <parameter_manager/parameter_manager_param_group.h>


using namespace std;
using namespace fst_parameter;

namespace fst_mc
{

ManualTeach::ManualTeach(void)
{
    joint_num_ = 0;
    vel_ratio_ = 0;
    acc_ratio_ = 0;
    step_joint_ = 0;
    step_position_ = 0;
    step_orientation_ = 0;
    position_vel_reference_ = 0;
    position_acc_reference_ = 0;
    orientation_omega_reference_ = 0;
    orientation_alpha_reference_ = 0;

    memset(axis_vel_, 0, sizeof(axis_vel_));
    memset(axis_acc_, 0, sizeof(axis_acc_));

    log_ptr_ = NULL;
    joint_constraint_ptr_ = NULL;
}

ManualTeach::~ManualTeach(void)
{}

ErrorCode ManualTeach::init(BaseKinematics *pkinematics, Constraint *pcons, fst_log::Logger *plog, const string &config_file)
{
    if (pkinematics && pcons && plog)
    {
        log_ptr_ = plog;
        kinematics_ptr_ = pkinematics;
        joint_constraint_ptr_ = pcons;
    }
    else
    {
        return MOTION_INTERNAL_FAULT;
    }

    int joint_num;
    ParamGroup config;
    manual_config_file_ = config_file;

    if (!config.loadParamFile(manual_config_file_))
    {
        FST_ERROR("Fail to load config file: %s", manual_config_file_.c_str());
        return config.getLastError();
    }

    if (config.getParam("number_of_axis", joint_num))
    {
        if (joint_num > 0 && joint_num <= NUM_OF_JOINT)
        {
            joint_num_ = joint_num;
            FST_INFO("Number of axis: %d", joint_num_);
        }
        else
        {
            FST_ERROR("Invalid number of joint in config file, num-of-joint = %d", joint_num);
            return INVALID_PARAMETER;
        }
    }
    else
    {
        FST_ERROR("Fail to load manual configuration.");
        return config.getLastError();
    }

    if (config.getParam("step/axis", step_joint_) &&
        config.getParam("step/position", step_position_) &&
        config.getParam("step/orientation", step_orientation_) &&
        config.getParam("reference/position/velocity", position_vel_reference_) &&
        config.getParam("reference/position/acceleration", position_acc_reference_) &&
        config.getParam("reference/orientation/omega", orientation_omega_reference_) &&
        config.getParam("reference/orientation/alpha", orientation_alpha_reference_))
    {
        FST_INFO("Step: axis=%.4f, position=%.4f, orientation=%.4f", step_joint_, step_position_, step_orientation_);
        FST_INFO("Reference: position-vel=%.4f, position-acc=%.4f, orientation-omega=%.4f, orientation-alpha=%.4f",
                 position_vel_reference_, position_acc_reference_, orientation_omega_reference_, orientation_alpha_reference_);
    }
    else
    {
        FST_ERROR("Fail to load manual configuration.");
        return config.getLastError();
    }

    vector<double> vel; vel.resize(joint_num_);
    vector<double> acc; acc.resize(joint_num_);

    if (config.getParam("reference/axis/velocity", vel) && config.getParam("reference/axis/acceleration", acc))
    {
        char buffer[LOG_TEXT_SIZE];
        memcpy(axis_vel_, &vel[0], joint_num_ * sizeof(double));
        memcpy(axis_acc_, &acc[0], joint_num_ * sizeof(double));
        FST_INFO("Axis vel : %s", printDBLine(axis_vel_, buffer, LOG_TEXT_SIZE));
        FST_INFO("Axis acc : %s", printDBLine(axis_acc_, buffer, LOG_TEXT_SIZE));
    }

    return SUCCESS;
}


double ManualTeach::getGlobalVelRatio(void)
{
    return vel_ratio_;
}

double ManualTeach::getGlobalAccRatio(void)
{
    return acc_ratio_;
}

double ManualTeach::getManualStepAxis(void)
{
    return step_joint_;
}

double ManualTeach::getManualStepPosition(void)
{
    return step_position_;
}

double ManualTeach::getManualStepOrientation(void)
{
    return step_orientation_;
}

ErrorCode ManualTeach::setGlobalVelRatio(double ratio)
{
    vel_ratio_ = ratio;
    return SUCCESS;
}

ErrorCode ManualTeach::setGlobalAccRatio(double ratio)
{
    acc_ratio_ = ratio;
    return SUCCESS;
}

ErrorCode ManualTeach::setManualStepAxis(double step)
{
    if (step > MINIMUM_E6 && step < 1)
    {
        ParamGroup param;

        if (param.loadParamFile(manual_config_file_) && param.setParam("step/axis", step) && param.dumpParamFile())
        {
            step_joint_ = step;
            return SUCCESS;
        }
        else
        {
            return param.getLastError();
        }
    }
    else
    {
        return INVALID_PARAMETER;
    }
}

ErrorCode ManualTeach::setManualStepPosition(double step)
{
    if (step > MINIMUM_E3 && step < 100)
    {
        ParamGroup param;

        if (param.loadParamFile(manual_config_file_) && param.setParam("step/position", step) && param.dumpParamFile())
        {
            step_position_ = step;
            return SUCCESS;
        }
        else
        {
            return param.getLastError();
        }
    }
    else
    {
        return INVALID_PARAMETER;
    }
}

ErrorCode ManualTeach::setManualStepOrientation(double step)
{
    if (step > MINIMUM_E6 && step < 1)
    {
        ParamGroup param;

        if (param.loadParamFile(manual_config_file_) && param.setParam("step/orientation", step) && param.dumpParamFile())
        {
            step_orientation_ = step;
            return SUCCESS;
        }
        else
        {
            return param.getLastError();
        }
    }
    else
    {
        return INVALID_PARAMETER;
    }
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

        case BASE:
        case USER:
        case WORLD:
            err = manualCartesianStep(directions, time, traj);
            break;

        case TOOL:
            //
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
    FST_INFO("  step-angle = %.4frad, vel-ratio = %.0f%%, acc-ratio = %.0f%%", step_joint_, vel_ratio_ * 100, acc_ratio_ * 100);
    FST_INFO("  start-joint = %s", printDBLine(&traj.joint_start[0], buffer, LOG_TEXT_SIZE));

    Joint target  = traj.joint_start;

    for (size_t i = 0; i < joint_num_; i++)
    {
        if      (dir[i] == INCREASE) target[i] += step_joint_;
        else if (dir[i] == DECREASE) target[i] -= step_joint_;
    }

    FST_INFO("  target-joint = %s", printDBLine(&target[0], buffer, LOG_TEXT_SIZE));

    if (!joint_constraint_ptr_->isJointInConstraint(target))
    {
        FST_ERROR("Target out of soft constraint.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    double omega[NUM_OF_JOINT], alpha[NUM_OF_JOINT], trips[NUM_OF_JOINT], delta[NUM_OF_JOINT];
    double duration = 0;
    double t_min[NUM_OF_JOINT];

    for (size_t i = 0; i < joint_num_; i++)
    {
        trips[i] = dir[i] == STANDING ? 0 : step_joint_;
        omega[i] = axis_vel_[i] * vel_ratio_;
        alpha[i] = axis_acc_[i] * acc_ratio_;

        delta[i] = trips[i] - omega[i] * omega[i] / alpha[i];
        t_min[i] = delta[i] > 0 ? (omega[i] / alpha[i] + trips[i] / omega[i]) : (sqrt(trips[i] / alpha[i]) * 2);
        FST_INFO("  J%d: trip=%.4f omega=%f, alpha=%f, delta=%f, tmin=%f", i + 1, trips[i], omega[i], alpha[i], delta[i], t_min[i]);
        if (t_min[i] > duration) duration = t_min[i];
    }

    traj.duration = duration;
    traj.joint_ending = target;
    FST_INFO("  duration=%.4f, create trajectory ...", duration);

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

        /*FST_INFO("  J%d: t1=%.4f,t2=%.4f,t3=%.4f,t4=%.4f,alpha1-2=%.4f,alpha3-4=%.4f",
                 i + 1, traj.coeff[i].start_time, traj.coeff[i].stable_time,
                 traj.coeff[i].brake_time, traj.coeff[i].stop_time,
                 traj.coeff[i].start_alpha, traj.coeff[i].brake_alpha);*/
    }

    FST_INFO("Success !");
    return SUCCESS;
}

ErrorCode ManualTeach::manualCartesianStep(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj)
{
    char buffer[LOG_TEXT_SIZE];
    PoseEuler &target = traj.cart_ending;
    PoseEuler &start = traj.cart_start;

    FST_INFO("manual-Cartesian-Step: directions = %s, planning trajectory ...", printDBLine((int*)dir, buffer, LOG_TEXT_SIZE));
    FST_INFO("  step-postion = %.4fmm, step-orientation = %.4frad", step_position_, step_orientation_);
    FST_INFO("  position: speed = %.4f, acceleration = %.4f", position_vel_reference_, position_acc_reference_);
    FST_INFO("  orientation: omega = %.4f, alpha = %.4f", orientation_omega_reference_, orientation_alpha_reference_);
    FST_INFO("  vel-ratio = %.0f%%, acc-ratio = %.0f%%", vel_ratio_ * 100, acc_ratio_ * 100);
    FST_INFO("  start-joint = %s", printDBLine(&traj.joint_start[0], buffer, LOG_TEXT_SIZE));

    target.position[0] = dir[0] == STANDING ? start.position[0] : (dir[0] == INCREASE ? start.position[0] + step_position_ : start.position[0] - step_position_);
    target.position[1] = dir[1] == STANDING ? start.position[1] : (dir[1] == INCREASE ? start.position[1] + step_position_ : start.position[1] - step_position_);
    target.position[2] = dir[2] == STANDING ? start.position[2] : (dir[2] == INCREASE ? start.position[2] + step_position_ : start.position[2] - step_position_);
    target.orientation[0] = dir[3] == STANDING ? start.orientation[0] : (dir[3] == INCREASE ? start.orientation[0] + step_orientation_ : start.orientation[0] - step_orientation_);
    target.orientation[1] = dir[4] == STANDING ? start.orientation[1] : (dir[4] == INCREASE ? start.orientation[1] + step_orientation_ : start.orientation[1] - step_orientation_);
    target.orientation[2] = dir[5] == STANDING ? start.orientation[2] : (dir[5] == INCREASE ? start.orientation[2] + step_orientation_ : start.orientation[2] - step_orientation_);

    FST_INFO("  start-pose  = %.4f %.4f %.4f - %.4f %.4f %.4f",
             start.position.x, start.position.y, start.position.z, start.orientation.a, start.orientation.b, start.orientation.c);
    FST_INFO("  target-pose = %.4f %.4f %.4f - %.4f %.4f %.4f",
             target.position.x, target.position.y, target.position.z, target.orientation.a, target.orientation.b, target.orientation.c);

    ErrorCode err;
    Joint target_joint;

    switch (traj.frame)
    {
        case BASE:
            err = kinematics_ptr_->inverseKinematicsInBase(target, traj.joint_start, target_joint);
            break;
        case USER:
            err = kinematics_ptr_->inverseKinematicsInUser(target, traj.joint_start, target_joint);
            break;
        case WORLD:
            err = kinematics_ptr_->inverseKinematicsInWorld(target, traj.joint_start, target_joint);
            break;
        default:
            err = MOTION_INTERNAL_FAULT;
            break;
    }

    if (err != SUCCESS)
    {
        FST_ERROR("Cannot find a valid inverse result of the target pose, err=%0xllx", err);
        return err;
    }

    FST_INFO("  target joint = %s", printDBLine(&target_joint[0], buffer, LOG_TEXT_SIZE));

    if (joint_constraint_ptr_->isJointInConstraint(traj.joint_ending))
    {
        traj.joint_ending = target_joint;
    }
    else
    {
        err = JOINT_OUT_OF_CONSTRAINT;
        FST_ERROR("Target joint out of constraint, err=0x%llx", err);
        return err;
    }

    double dis = (dir[0] != STANDING || dir[1] != STANDING || dir[2] != STANDING) ? step_position_ : 0;
    double spd = position_vel_reference_ * vel_ratio_;
    double acc = position_acc_reference_ * acc_ratio_;

    double angle = (dir[3] != STANDING || dir[4] != STANDING || dir[5] != STANDING) ? step_orientation_ : 0;
    double omega = orientation_omega_reference_ * vel_ratio_;
    double alpha = orientation_alpha_reference_ * acc_ratio_;

    //FST_WARN("  spd=%f acc=%f omega=%f alpha=%f", spd, acc, omega, alpha);

    double delta = dis - spd * spd / acc;
    double t_pos = delta > 0 ? (spd / acc * 2 + delta / spd) : (sqrt(dis / acc) * 2);
    double theta = angle - omega * omega / alpha;
    double t_ort = theta > 0 ? (omega / alpha * 2 + theta / omega) : (sqrt(angle / alpha) * 2);

    double duration = t_pos > t_ort ? t_pos : t_ort;

    //FST_WARN("  delta=%f t_pos=%f, theta=%f, t_ort=%f, duration=%f", delta, t_pos, theta, t_ort, duration);

    for (size_t i = 0; i < 3; i++)
    {
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
            if (duration < t_pos + MINIMUM_E6)
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
        FST_INFO("  Position[%d]: t1=%.4f,t2=%.4f,t3=%.4f,t4=%.4f,alpha1-2=%.4f,alpha3-4=%.4f",
                 i, traj.coeff[i].start_time, traj.coeff[i].stable_time,
                 traj.coeff[i].brake_time, traj.coeff[i].stop_time,
                 traj.coeff[i].start_alpha, traj.coeff[i].brake_alpha);
    }

    for (size_t i = 3; i < 6; i++)
    {
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
            if (duration < t_ort + MINIMUM_E6)
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
        FST_INFO("  Orientation[%d]: t1=%.4f,t2=%.4f,t3=%.4f,t4=%.4f,alpha1-2=%.4f,alpha3-4=%.4f",
                 i - 3, traj.coeff[i].start_time, traj.coeff[i].stable_time,
                 traj.coeff[i].brake_time, traj.coeff[i].stop_time,
                 traj.coeff[i].start_alpha, traj.coeff[i].brake_alpha);
    }

    traj.duration = duration;
    FST_INFO("Success, total duration=%.4f", traj.duration);
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
    double alpha, omega;
    char buffer[LOG_TEXT_SIZE];

    FST_INFO("manual-Joint-Continuous: directions = %s, planning trajectory ...", printDBLine((int*)dir, buffer, LOG_TEXT_SIZE));
    FST_INFO("  manual-time = %.4f, vel-ratio = %.0f%%, acc-ratio = %.0f%%", time, vel_ratio_ * 100, acc_ratio_ * 100);
    FST_INFO("  start-joint = %s", printDBLine(&traj.joint_start[0], buffer, LOG_TEXT_SIZE));

    if (traj.duration < MINIMUM_E6)
    {
        // begin to manual-continuous, update ending joint
        traj.joint_ending = traj.joint_start;
    }

    for (size_t i = 0; i < joint_num_; i++)
    {
        alpha = axis_acc_[i] * acc_ratio_;
        omega = axis_vel_[i] * vel_ratio_;

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
    FST_INFO("Succes, total duration=%.4f", traj.duration);
    return SUCCESS;
}

ErrorCode ManualTeach::manualCartesianContinuous(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj)
{
    char buffer[LOG_TEXT_SIZE];

    FST_INFO("manual-Cartesian-Continuous: directions = %s, planning trajectory ...", printDBLine((int*)dir, buffer, LOG_TEXT_SIZE));
    FST_INFO("  manual-time = %.4f, vel-ratio = %.0f%%, acc-ratio = %.0f%%", time, vel_ratio_ * 100, acc_ratio_ * 100);
    FST_INFO("  start-joint = %s", printDBLine(&traj.joint_start[0], buffer, LOG_TEXT_SIZE));

    PoseEuler &start = traj.cart_start;
    PoseEuler target = traj.cart_start;

    FST_INFO("  start-pose  = %.2f %.2f %.2f - %.4f %.4f %.4f",
             start.position.x, start.position.y, start.position.z, start.orientation.a, start.orientation.b, start.orientation.c);

    double spd, acc;

    for (size_t i = 0; i < 3; i++)
    {
        if (traj.direction[i] == dir[i])
        {
            // keep standby or motion direction
            // do nothing
            if (traj.direction[i] == STANDING)
            {
                traj.cart_ending.position[i] = traj.cart_start.position[i];
            }

            FST_INFO("  Position[%d]: given direction same as current motion, running along the current trajectory", i);
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
                traj.cart_ending.position[i] = traj.cart_start.position[i];
            }
            else if (time < traj.coeff[i].stable_time)
            {
                double tm = time - traj.coeff[i].start_time;
                traj.coeff[i].stable_time = time;
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time + tm;
                traj.direction[i] = STANDING;
                traj.cart_ending.position[i] = traj.cart_start.position[i] + traj.coeff[i].start_alpha * tm * tm;
            }
            else if (time < traj.coeff[i].brake_time)
            {
                double tim = traj.coeff[i].stable_time - traj.coeff[i].start_time;
                double omg = tim * traj.coeff[i].start_alpha;
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time + tim;
                traj.direction[i] = STANDING;
                traj.cart_ending.position[i] = traj.cart_start.position[i] + omg * tim;
                tim = time - traj.coeff[i].stable_time;
                traj.cart_ending.position[i] += omg * tim;
            }
            else
            {
                // already in axis-slow-down trajectory, do nothing
                FST_WARN("  Position[%d]: Standby or in slow-down trajectory, will stop soon", i);
                continue;
            }
        }
        else if (dir[i] != STANDING && traj.direction[i] == STANDING)
        {
            if (time >= traj.coeff[i].stop_time)
            {
                // start motion in this direction
                //spd = i < 3 ? position_vel_reference_ * vel_ratio_ : orientation_omega_reference_ * vel_ratio_;
                //acc = i < 3 ? position_acc_reference_ * acc_ratio_ : orientation_alpha_reference_ * acc_ratio_;
                spd = position_vel_reference_ * vel_ratio_;
                acc = position_acc_reference_ * acc_ratio_;

                traj.coeff[i].start_time = time;
                traj.coeff[i].stable_time = time + spd / acc;
                traj.coeff[i].brake_time = 32 - spd / acc;
                traj.coeff[i].stop_time = 32;
                traj.coeff[i].start_alpha = dir[i] == INCREASE ? acc : -acc;
                traj.coeff[i].brake_alpha = -traj.coeff[i].start_alpha;
                traj.direction[i] = dir[i];
                double trip = spd * spd / acc + (32 - spd / acc * 2) * spd;
                traj.cart_ending.position[i] = traj.cart_start.position[i] + (dir[i] == INCREASE ? trip : -trip);
            }
            else
            {
                // axis-slow-down trajectory havn't finished
                FST_WARN("  Position[%d]: in slow-down trajectory, cannot startup until standby", i);
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
                traj.cart_ending.position[i] = traj.cart_start.position[i];
            }
            else if (time < traj.coeff[i].stable_time)
            {
                double tm = time - traj.coeff[i].start_time;
                traj.coeff[i].stable_time = time;
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time + tm;
                traj.cart_ending.position[i] = traj.cart_start.position[i] + traj.coeff[i].start_alpha * tm * tm;
            }
            else if (time < traj.coeff[i].brake_time)
            {
                double tim = traj.coeff[i].stable_time - traj.coeff[i].start_time;
                double omg = tim * traj.coeff[i].start_alpha;
                traj.coeff[i].brake_time = time;
                traj.coeff[i].stop_time = time + tim;
                traj.cart_ending.position[i] = traj.cart_start.position[i] + omg * tim;
                tim = time - traj.coeff[i].stable_time;
                traj.cart_ending.position[i] += omg * tim;
            }
            else
            {
                // already in axis-slow-down trajectory, do nothing
                FST_WARN("  Position[%d]: Standby or in slow-down trajectory, will stop soon", i);
                continue;
            }
        }

        FST_INFO("  Position[%d]: t1=%.4f,t2=%.4f,t3=%.4f,t4=%.4f,alpha1-2=%.4f,alpha3-4=%.4f",
                 i, traj.coeff[i].start_time, traj.coeff[i].stable_time, traj.coeff[i].brake_time, traj.coeff[i].stop_time,
                 traj.coeff[i].start_alpha, traj.coeff[i].brake_alpha);
    }

    for (size_t i = 0; i < 3; i++)
    {
        size_t j = i + 3;
        if (traj.direction[j] == dir[j])
        {
            // keep standby or motion direction
            // do nothing
            if (traj.direction[j] == STANDING)
            {
                traj.cart_ending.orientation[i] = traj.cart_start.orientation[i];
            }

            FST_INFO("  Orientation[%d]: given direction same as current motion, running along the current trajectory", i);
            continue;
        }
        else if (dir[j] == STANDING && traj.direction[j] != STANDING)
        {
            // stop motion in this direction
            if (time < traj.coeff[j].start_time)
            {
                traj.coeff[j].start_time = time;
                traj.coeff[j].stable_time = time;
                traj.coeff[j].brake_time = time;
                traj.coeff[j].stop_time = time;
                traj.coeff[j].start_alpha = 0;
                traj.coeff[j].brake_alpha = 0;
                traj.direction[j] = STANDING;
                traj.cart_ending.orientation[i] = traj.cart_start.orientation[i];
            }
            else if (time < traj.coeff[j].stable_time)
            {
                double tm = time - traj.coeff[j].start_time;
                traj.coeff[j].stable_time = time;
                traj.coeff[j].brake_time = time;
                traj.coeff[j].stop_time = time + tm;
                traj.direction[j] = STANDING;
                traj.cart_ending.orientation[i] = traj.cart_start.orientation[i] + traj.coeff[j].start_alpha * tm * tm;
            }
            else if (time < traj.coeff[j].brake_time)
            {
                double tim = traj.coeff[j].stable_time - traj.coeff[j].start_time;
                double omg = tim * traj.coeff[j].start_alpha;
                traj.coeff[j].brake_time = time;
                traj.coeff[j].stop_time = time + tim;
                traj.direction[j] = STANDING;
                traj.cart_ending.orientation[i] = traj.cart_start.orientation[i] + omg * tim;
                tim = time - traj.coeff[j].stable_time;
                traj.cart_ending.orientation[i] += omg * tim;
            }
            else
            {
                // already in axis-slow-down trajectory, do nothing
                FST_WARN("  Orientation[%d]: Standby or in slow-down trajectory, will stop soon", i);
                continue;
            }
        }
        else if (dir[j] != STANDING && traj.direction[j] == STANDING)
        {
            if (time >= traj.coeff[j].stop_time)
            {
                // start motion in this direction
                //spd = i < 3 ? position_vel_reference_ * vel_ratio_ : orientation_omega_reference_ * vel_ratio_;
                //acc = i < 3 ? position_acc_reference_ * acc_ratio_ : orientation_alpha_reference_ * acc_ratio_;
                spd = orientation_omega_reference_ * vel_ratio_;
                acc = orientation_alpha_reference_ * acc_ratio_;

                traj.coeff[j].start_time = time;
                traj.coeff[j].stable_time = time + spd / acc;
                traj.coeff[j].brake_time = 32 - spd / acc;
                traj.coeff[j].stop_time = 32;
                traj.coeff[j].start_alpha = dir[j] == INCREASE ? acc : -acc;
                traj.coeff[j].brake_alpha = -traj.coeff[j].start_alpha;
                traj.direction[j] = dir[j];
                double trip = spd * spd / acc + (32 - spd / acc * 2) * spd;
                traj.cart_ending.orientation[i] = traj.cart_start.orientation[i] + (dir[j] == INCREASE ? trip : -trip);
            }
            else
            {
                // axis-slow-down trajectory havn't finished
                FST_WARN("  Orientation[%d]: in slow-down trajectory, cannot startup until standby", i);
                continue;
            }
        }
        else
        {
            // prev-direction = INCREASE && input-direction = DECREASE or
            // prev-direction = DECREASE && input-direction = INCREASE,
            // stop motion in this direction
            traj.direction[j] = STANDING;

            if (time < traj.coeff[j].start_time)
            {
                traj.coeff[j].start_time = time;
                traj.coeff[j].stable_time = time;
                traj.coeff[j].brake_time = time;
                traj.coeff[j].stop_time = time;
                traj.coeff[j].start_alpha = 0;
                traj.coeff[j].brake_alpha = 0;
                traj.cart_ending.orientation[i] = traj.cart_start.orientation[i];
            }
            else if (time < traj.coeff[j].stable_time)
            {
                double tm = time - traj.coeff[j].start_time;
                traj.coeff[j].stable_time = time;
                traj.coeff[j].brake_time = time;
                traj.coeff[j].stop_time = time + tm;
                traj.cart_ending.orientation[i] = traj.cart_start.orientation[i] + traj.coeff[j].start_alpha * tm * tm;
            }
            else if (time < traj.coeff[j].brake_time)
            {
                double tim = traj.coeff[j].stable_time - traj.coeff[j].start_time;
                double omg = tim * traj.coeff[j].start_alpha;
                traj.coeff[j].brake_time = time;
                traj.coeff[j].stop_time = time + tim;
                traj.cart_ending.orientation[i] = traj.cart_start.orientation[i] + omg * tim;
                tim = time - traj.coeff[j].stable_time;
                traj.cart_ending.orientation[i] += omg * tim;
            }
            else
            {
                // already in axis-slow-down trajectory, do nothing
                FST_WARN("  Orientation[%d]: Standby or in slow-down trajectory, will stop soon", i);
                continue;
            }
        }

        FST_INFO("  Orientation[%d]: t1=%.4f,t2=%.4f,t3=%.4f,t4=%.4f,alpha1-2=%.4f,alpha3-4=%.4f",
                 i, traj.coeff[i].start_time, traj.coeff[i].stable_time, traj.coeff[i].brake_time, traj.coeff[i].stop_time,
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
    FST_INFO("Success, total duration=%.4f", traj.duration);
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


