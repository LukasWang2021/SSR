/*************************************************************************
	> File Name: motion_control_manual_teach.cpp
	> Author: 冯赟
	> Mail: yun.feng@foresight-robotics.com
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
using namespace basic_alg;
using namespace fst_parameter;

namespace fst_mc
{

ManualTeach::ManualTeach(void)
{
    joint_num_ = 0;
    vel_ratio_ = 0;
    acc_ratio_ = 0;
    step_position_ = 0;
    step_orientation_ = 0;
    position_vel_reference_ = 0;
    position_acc_reference_ = 0;
    orientation_omega_reference_ = 0;
    orientation_alpha_reference_ = 0;
    time_multiplier_in_step_mode_ = 0;

    memset(step_axis_, 0, sizeof(step_axis_));
    memset(axis_vel_, 0, sizeof(axis_vel_));
    memset(axis_acc_, 0, sizeof(axis_acc_));

    log_ptr_ = NULL;
    joint_constraint_ptr_ = NULL;
}

ManualTeach::~ManualTeach(void)
{}

ErrorCode ManualTeach::init(Kinematics *kinematics_ptr, Constraint *pcons, fst_log::Logger *plog, const string &config_file)
{
    if (kinematics_ptr && pcons && plog)
    {
        log_ptr_ = plog;
        kinematics_ptr_ = kinematics_ptr;
        joint_constraint_ptr_ = pcons;
    }
    else
    {
        return MC_INTERNAL_FAULT;
    }

    int  joint_num;
    char buffer[LOG_TEXT_SIZE];
    double data[NUM_OF_JOINT];
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
            FST_ERROR("Invalid number of axis in config file, num-of-joint = %d", joint_num);
            return INVALID_PARAMETER;
        }
    }
    else
    {
        FST_ERROR("Fail to load number of manual axis, code = 0x%llx", config.getLastError());
        return config.getLastError();
    }

    if (!config.getParam("step/axis", data, joint_num_))
    {
        FST_ERROR("Fail to load manual step, code = 0x%llx", config.getLastError());
        return config.getLastError();
    }

    memcpy(step_axis_, data, joint_num_ * sizeof(double));
    FST_INFO("Axis step: %s", printDBLine(step_axis_, buffer, LOG_TEXT_SIZE));

    if (config.getParam("step/position", step_position_) &&
        config.getParam("step/orientation", step_orientation_) &&
        config.getParam("reference/position/velocity", position_vel_reference_) &&
        config.getParam("reference/position/acceleration", position_acc_reference_) &&
        config.getParam("reference/position/jerk", position_jerk_reference_) &&
        config.getParam("reference/orientation/omega", orientation_omega_reference_) &&
        config.getParam("reference/orientation/alpha", orientation_alpha_reference_) &&
        config.getParam("reference/orientation/beta", orientation_beta_reference_) &&
        config.getParam("time_multiplier", time_multiplier_in_step_mode_))
    {
        FST_INFO("time multiplier in step mode: %.4f", time_multiplier_in_step_mode_);
        FST_INFO("Step: position=%.4f, orientation=%.4f", step_position_, step_orientation_);
        FST_INFO("Reference: position-vel=%.4f, position-acc=%.4f, position-jerk=%.4f, orientation-omega=%.4f, orientation-alpha=%.4f, orientation-beta=%.4f",
                 position_vel_reference_, position_acc_reference_, position_jerk_reference_, orientation_omega_reference_, orientation_alpha_reference_, orientation_beta_reference_);
    }
    else
    {
        FST_ERROR("Fail to load manual configuration.");
        return config.getLastError();
    }

    if (!config.getParam("reference/axis/move_to_point_velocity", data, joint_num_))
    {
        FST_ERROR("Fail to load reference velocity in move-to-point of each axis, code = 0x%llx", config.getLastError());
        return config.getLastError();
    }

    memcpy(move_to_point_vel_, data, joint_num_ * sizeof(double));
    FST_INFO("Axis vel in move-to-point: %s", printDBLine(move_to_point_vel_, buffer, LOG_TEXT_SIZE));

    if (!config.getParam("reference/axis/velocity", data, joint_num_))
    {
        FST_ERROR("Fail to load reference velocity of each axis, code = 0x%llx", config.getLastError());
        return config.getLastError();
    }

    memcpy(axis_vel_, data, joint_num_ * sizeof(double));
    FST_INFO("Axis vel: %s", printDBLine(axis_vel_, buffer, LOG_TEXT_SIZE));
    
    if(!config.getParam("reference/axis/acceleration", data, joint_num_))
    {
        FST_ERROR("Fail to load reference acceleration of each axis, code = 0x%llx", config.getLastError());
        return config.getLastError();
    }

    memcpy(axis_acc_, data, joint_num_ * sizeof(double));
    FST_INFO("Axis acc : %s", printDBLine(axis_acc_, buffer, LOG_TEXT_SIZE));

    if(!config.getParam("reference/axis/jerk", data, joint_num_))
    {
        FST_ERROR("Fail to load reference jerk of each axis, code = 0x%llx", config.getLastError());
        return config.getLastError();
    }

    memcpy(axis_jerk_, data, joint_num_ * sizeof(double));
    FST_INFO("Axis jerk : %s", printDBLine(axis_jerk_, buffer, LOG_TEXT_SIZE));

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

void ManualTeach::getManualStepAxis(double *steps)
{
    for (size_t i = 0; i < joint_num_; i++)
    {
        steps[i] = step_axis_[i];
    }
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

ErrorCode ManualTeach::setManualStepAxis(const double *steps)
{
    vector<double> data;

    for (size_t i = 0; i < joint_num_; i++)
    {
        data.push_back(steps[i]);
    }

    ParamGroup param;

    if (param.loadParamFile(manual_config_file_) && param.setParam("step/axis", data) && param.dumpParamFile())
    {
        memcpy(step_axis_, steps, joint_num_ * sizeof(double));
        return SUCCESS;
    }
    else
    {
        return param.getLastError();
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
        case TOOL:
            err = manualCartesianStep(directions, time, traj);
            break;

        default:
            err = MC_FAIL_MANUAL_STEP;
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
    FST_INFO("  step-axis = %s", printDBLine(step_axis_, buffer, LOG_TEXT_SIZE));
    FST_INFO("  start-joint = %s", printDBLine(&traj.joint_start[0], buffer, LOG_TEXT_SIZE));

    Joint target  = traj.joint_start;

    for (size_t i = 0; i < joint_num_; i++)
    {
        if      (dir[i] == INCREASE) target[i] += step_axis_[i];
        else if (dir[i] == DECREASE) target[i] -= step_axis_[i];
    }

    FST_INFO("  target-joint = %s", printDBLine(&target[0], buffer, LOG_TEXT_SIZE));

    if (!joint_constraint_ptr_->isJointInConstraint(target))
    {
        FST_ERROR("Target out of soft constraint.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    double omega, alpha, delta, trip, axis_duration;
    double total_duration = 0;

    for (size_t i = 0; i < joint_num_; i++)
    {
        trip = dir[i] == STANDING ? 0 : step_axis_[i];
        omega = axis_vel_[i];
        alpha = axis_acc_[i];

        delta = trip - omega * omega / alpha;
        axis_duration = delta > 0 ? (omega / alpha + trip / omega) : (sqrt(trip / alpha) * 2);
        // FST_INFO("  J%d: trip=%.4f omega=%f, alpha=%f, delta=%f, t=%f", i, trips, omega, alpha, delta, axis_duration);
        if (axis_duration > total_duration) total_duration = axis_duration;
    }

    total_duration = total_duration * time_multiplier_in_step_mode_;
    traj.ending_time = total_duration;
    traj.joint_ending = target;
    FST_INFO("  duration=%.4f, create trajectory ...", total_duration);

    for (size_t i = 0; i < joint_num_; i++)
    {
        traj.direction[i] = dir[i];
        traj.axis[i].time_stamp[0] = time;
        traj.axis[i].time_stamp[1] = time;
        traj.axis[i].time_stamp[2] = time;
        traj.axis[i].time_stamp[3] = time;
        traj.axis[i].time_stamp[4] = time;
        traj.axis[i].time_stamp[5] = time;
        traj.axis[i].time_stamp[6] = time;
        traj.axis[i].time_stamp[7] = total_duration;
        getQuinticSplineCoefficients(traj.joint_start[i], 0, 0, target[i], 0, 0, total_duration, traj.axis[i].coeff[6].data);
    }

    FST_INFO("  Create manual trajectory success !");
    return SUCCESS;
}

void ManualTeach::getQuinticSplineCoefficients(double start_pos, double start_vel, double start_acc, double end_pos, double end_vel, double end_acc, double duration, double *coeffs)
{
    if (duration < 0.000001)
    {
        coeffs[0] = end_pos;
		coeffs[1] = end_vel;
		coeffs[2] = end_acc/2;
		coeffs[3] = 0;
		coeffs[4] = 0;
		coeffs[5] = 0;
        return;
    }

    double t[6];
    t[0] = 1;
    t[1] = duration;
    t[2] = t[1] * duration;
    t[3] = t[2] * duration;
    t[4] = t[3] * duration;
    t[5] = t[4] * duration;
    coeffs[0] = start_pos;
    coeffs[1] = start_vel;
    coeffs[2] = start_acc/2;
    coeffs[3] = (end_pos * 20 - start_pos * 20 + end_acc * t[2] - start_acc * t[2] * 3 - start_vel * t[1] * 12 - end_vel * t[1] * 8) / (t[3] * 2);
    coeffs[4] = (start_pos * 30 - end_pos * 30 + start_vel * t[1] * 16 + end_vel * t[1] * 14 + start_acc * t[2] * 3 - end_acc * t[2] * 2) / (t[4] * 2);
    coeffs[5] = (end_pos * 12 - start_pos * 12 - start_vel * t[1] * 6 - end_vel * t[1] * 6 - start_acc * t[2] + end_acc * t[2]) / (t[5] * 2);
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
    FST_INFO("  start-joint = %s", printDBLine(&traj.joint_start[0], buffer, LOG_TEXT_SIZE));

    target.point_.x_ = dir[0] == STANDING ? start.point_.x_ : (dir[0] == INCREASE ? start.point_.x_ + step_position_ : start.point_.x_ - step_position_);
    target.point_.y_ = dir[1] == STANDING ? start.point_.y_ : (dir[1] == INCREASE ? start.point_.y_ + step_position_ : start.point_.y_ - step_position_);
    target.point_.z_ = dir[2] == STANDING ? start.point_.z_ : (dir[2] == INCREASE ? start.point_.z_ + step_position_ : start.point_.z_ - step_position_);
    target.euler_.a_ = dir[3] == STANDING ? start.euler_.a_ : (dir[3] == INCREASE ? start.euler_.a_ + step_orientation_ : start.euler_.a_ - step_orientation_);
    target.euler_.b_ = dir[4] == STANDING ? start.euler_.b_ : (dir[4] == INCREASE ? start.euler_.b_ + step_orientation_ : start.euler_.b_ - step_orientation_);
    target.euler_.c_ = dir[5] == STANDING ? start.euler_.c_ : (dir[5] == INCREASE ? start.euler_.c_ + step_orientation_ : start.euler_.c_ - step_orientation_);

    FST_INFO("  start-pose = %.4f %.4f %.4f - %.4f %.4f %.4f", start.point_.x_, start.point_.y_, start.point_.z_, start.euler_.a_, start.euler_.b_, start.euler_.c_);
    FST_INFO("  target-pose = %.4f %.4f %.4f - %.4f %.4f %.4f", target.point_.x_, target.point_.y_, target.point_.z_, target.euler_.a_, target.euler_.b_, target.euler_.c_);

    double dis = (dir[0] != STANDING || dir[1] != STANDING || dir[2] != STANDING) ? step_position_ : 0;
    double spd = position_vel_reference_;
    double acc = position_acc_reference_;

    double angle = (dir[3] != STANDING || dir[4] != STANDING || dir[5] != STANDING) ? step_orientation_ : 0;
    double omega = orientation_omega_reference_;
    double alpha = orientation_alpha_reference_;
    //FST_WARN("  spd=%f acc=%f omega=%f alpha=%f", spd, acc, omega, alpha);
    double delta = dis - spd * spd / acc;
    double t_pos = delta > 0 ? (spd / acc * 2 + delta / spd) : (sqrt(dis / acc) * 2);
    double theta = angle - omega * omega / alpha;
    double t_ort = theta > 0 ? (omega / alpha * 2 + theta / omega) : (sqrt(angle / alpha) * 2);
    double duration = (t_pos > t_ort ? t_pos : t_ort) * time_multiplier_in_step_mode_;
    //FST_WARN("  delta=%f t_pos=%f, theta=%f, t_ort=%f, duration=%f", delta, t_pos, theta, t_ort, duration);

    for (size_t i = 0; i < 6; i++)
    {
        traj.direction[i] = dir[i];
        traj.axis[i].time_stamp[0] = time;
        traj.axis[i].time_stamp[1] = time;
        traj.axis[i].time_stamp[2] = time;
        traj.axis[i].time_stamp[3] = time;
        traj.axis[i].time_stamp[4] = time;
        traj.axis[i].time_stamp[5] = time;
        traj.axis[i].time_stamp[6] = time;
        traj.axis[i].time_stamp[7] = duration;
        getQuinticSplineCoefficients(*(&start.point_.x_ + i), 0, 0, *(&target.point_.x_ + i), 0, 0, duration, traj.axis[i].coeff[6].data);
    }

    traj.ending_time = duration;
    FST_INFO("  Create manual trajectory success !");
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
        case TOOL:
            err = manualCartesianContinuous(directions, time, traj);
            break;
        //case TOOL:
        //    err = manualCartesianContinuousInToolFrame(directions, time, traj);
        //    break;
        default:
            err = MC_FAIL_MANUAL_CONTINUOUS;
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

void ManualTeach::getAxisCoefficient(double time_to_start, double start_position, double total_trip, double absolute_jerk, double expect_omega, double expect_alpha, ManualAxisBlock &axis_block)
{
    double absolute_alpha = expect_alpha >= 0 ? expect_alpha : -expect_alpha;
    double absolute_omega = expect_omega >= 0 ? expect_omega : -expect_omega;
    
    double t1, t2, t4;
    double theta = absolute_omega - absolute_alpha * absolute_alpha / absolute_jerk;
    double start_pos, start_vel, start_acc, end_pos, end_vel, end_acc;

    if (theta < 0)
    {
        // 目标速度Ｖ不足以加速度达到Ａ，没有匀加速阶段，４段或５段
        t1 = sqrt(absolute_omega / absolute_jerk);
        t2 = 0;
        double delta = total_trip - absolute_jerk * t1 * t1 * t1 * 2;

        if (delta < 0)
        {
            // 目标距离不足以速度达到Ｖ，没有匀速阶段，４段
            t1 = pow(total_trip / absolute_jerk / 2, 1.0 / 3.0);
            t4 = 0;
            double jerk = expect_alpha < 0 ? -absolute_jerk : absolute_jerk;
            double alpha = jerk * t1;
            double omega = jerk * t1 * t1;
            axis_block.time_stamp[0] = time_to_start;
            axis_block.time_stamp[1] = axis_block.time_stamp[0] + t1;
            axis_block.time_stamp[2] = axis_block.time_stamp[1] + t2;
            axis_block.time_stamp[3] = axis_block.time_stamp[2] + t1;
            axis_block.time_stamp[4] = axis_block.time_stamp[3] + t4;
            axis_block.time_stamp[5] = axis_block.time_stamp[4] + t1;
            axis_block.time_stamp[6] = axis_block.time_stamp[5] + t2;
            axis_block.time_stamp[7] = axis_block.time_stamp[6] + t1;
            
            start_pos = start_position;
            start_vel = 0;
            start_acc = 0;
            end_pos = start_pos + jerk * t1 * t1 * t1 / 6;
            end_vel = alpha * t1 / 2;
            end_acc = alpha;
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[0].data);
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t2, axis_block.coeff[1].data);
            start_pos = end_pos;
            start_vel = end_vel;
            start_acc = end_acc;
            end_pos = start_pos + jerk * t1 * t1 * t1 * 5 / 6;
            end_vel = omega;
            end_acc = 0;
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[2].data);
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t4, axis_block.coeff[3].data);
            start_pos = end_pos;
            start_vel = end_vel;
            start_acc = end_acc;
            end_pos = start_pos + jerk * t1 * t1 * t1 * 5 / 6;
            end_vel = jerk * t1 * t1 / 2;
            end_acc = -alpha;
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[4].data);
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t2, axis_block.coeff[5].data);
            start_pos = end_pos;
            start_vel = end_vel;
            start_acc = end_acc;
            end_pos = start_pos + jerk * t1 * t1 * t1 / 6;
            end_vel = 0;
            end_acc = 0;
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[6].data);
        }
        else
        {
            // 目标距离足以速度达到Ｖ，有匀速阶段，5段
            t4 = delta / absolute_omega;
            double jerk = expect_alpha < 0 ? -absolute_jerk : absolute_jerk;
            double alpha = jerk * t1;
            double omega = expect_omega;
            axis_block.time_stamp[0] = time_to_start;
            axis_block.time_stamp[1] = axis_block.time_stamp[0] + t1;
            axis_block.time_stamp[2] = axis_block.time_stamp[1] + t2;
            axis_block.time_stamp[3] = axis_block.time_stamp[2] + t1;
            axis_block.time_stamp[4] = axis_block.time_stamp[3] + t4;
            axis_block.time_stamp[5] = axis_block.time_stamp[4] + t1;
            axis_block.time_stamp[6] = axis_block.time_stamp[5] + t2;
            axis_block.time_stamp[7] = axis_block.time_stamp[6] + t1;
            
            start_pos = start_position;
            start_vel = 0;
            start_acc = 0;
            end_pos = start_pos + jerk * t1 * t1 * t1 / 6;
            end_vel = omega / 2;
            end_acc = alpha;
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[0].data);
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t2, axis_block.coeff[1].data);
            start_pos = end_pos;
            start_vel = end_vel;
            start_acc = end_acc;
            end_pos = start_pos + jerk * t1 * t1 * t1 * 5 / 6;
            end_vel = omega;
            end_acc = 0;
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[2].data);
            start_pos = end_pos;
            start_vel = end_vel;
            start_acc = end_acc;
            end_pos = start_pos + omega * t4;
            end_vel = omega;
            end_acc = 0;
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t4, axis_block.coeff[3].data);
            start_pos = end_pos;
            start_vel = end_vel;
            start_acc = end_acc;
            end_pos = start_pos + jerk * t1 * t1 * t1 * 5 / 6;
            end_vel = omega / 2;
            end_acc = -alpha;
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[4].data);
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t2, axis_block.coeff[5].data);
            start_pos = end_pos;
            start_vel = end_vel;
            start_acc = end_acc;
            end_pos = start_pos + jerk * t1 * t1 * t1 / 6;
            end_vel = 0;
            end_acc = 0;
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[6].data);
        }
    }
    else
    {
        // 目标速度Ｖ足以加速度达到Ａ，有匀加速阶段，６段或７段
        double delta = total_trip - absolute_omega * (absolute_omega / absolute_alpha + absolute_alpha / absolute_jerk);

        if (delta > 0)
        {
            // 目标距离足以使速度达到Ｖ，有匀速阶段，7段
            t1 = absolute_alpha / absolute_jerk;
            t2 = absolute_omega / absolute_alpha - t1;
            t4 = delta / absolute_omega;
            double jerk = expect_alpha < 0 ? -absolute_jerk : absolute_jerk;
            double alpha = expect_alpha;
            double omega = expect_omega;
            axis_block.time_stamp[0] = time_to_start;
            axis_block.time_stamp[1] = axis_block.time_stamp[0] + t1;
            axis_block.time_stamp[2] = axis_block.time_stamp[1] + t2;
            axis_block.time_stamp[3] = axis_block.time_stamp[2] + t1;
            axis_block.time_stamp[4] = axis_block.time_stamp[3] + t4;
            axis_block.time_stamp[5] = axis_block.time_stamp[4] + t1;
            axis_block.time_stamp[6] = axis_block.time_stamp[5] + t2;
            axis_block.time_stamp[7] = axis_block.time_stamp[6] + t1;
            
            start_pos = start_position;
            start_vel = 0;
            start_acc = 0;
            end_pos = start_pos + jerk * t1 * t1 * t1 / 6;
            end_vel = jerk * t1 * t1 / 2;
            end_acc = alpha;
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[0].data);
            start_pos = end_pos;
            start_vel = end_vel;
            start_acc = end_acc;
            end_pos = start_pos + jerk * t1 * t1 * t2 / 2 + jerk * t1 * t2 * t2 / 2;
            end_vel = jerk * t1 * t1 / 2 + jerk * t1 * t2;
            end_acc = alpha;
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t2, axis_block.coeff[1].data);
            start_pos = end_pos;
            start_vel = end_vel;
            start_acc = end_acc;
            end_pos = start_pos + omega * t1 - jerk * t1 * t1 * t1 / 6;
            end_vel = omega;
            end_acc = 0;
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[2].data);
            start_pos = end_pos;
            start_vel = end_vel;
            start_acc = end_acc;
            end_pos = start_pos + omega * t4;
            end_vel = omega;
            end_acc = 0;
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t4, axis_block.coeff[3].data);
            start_pos = end_pos;
            start_vel = end_vel;
            start_acc = end_acc;
            end_pos = start_pos + omega * t1 - jerk * t1 * t1 * t1 / 6;
            end_vel = jerk * t1 * t1 / 2 + jerk * t1 * t2;
            end_acc = -alpha;
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[4].data);
            start_pos = end_pos;
            start_vel = end_vel;
            start_acc = end_acc;
            end_pos = start_pos + omega * t2 / 2;
            end_vel = jerk * t1 * t1 / 2;
            end_acc = -alpha;
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t2, axis_block.coeff[5].data);
            start_pos = end_pos;
            start_vel = end_vel;
            start_acc = end_acc;
            end_pos = start_pos + jerk * t1 * t1 * t1 / 6;
            end_vel = 0;
            end_acc = 0;
            getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[6].data);
        }
        else
        {
            // 目标距离不足以速度达到Ｖ，没有匀速阶段，6段或４段
            t1 = absolute_alpha / absolute_jerk;
            delta = total_trip - absolute_jerk * t1 * t1 * t1 * 2;

            if (delta > 0)
            {
                // 目标距离足以使加速度达到A，有匀加速阶段，6段
                t2 = (sqrt(t1 * t1 + total_trip / absolute_alpha * 4) - t1 * 3) / 2;
                t4 = 0;
                double jerk = expect_alpha < 0 ? -absolute_jerk : absolute_jerk;
                double alpha = expect_alpha;
                double omega = jerk * t1 * t1 + jerk * t1 * t2;
                axis_block.time_stamp[0] = time_to_start;
                axis_block.time_stamp[1] = axis_block.time_stamp[0] + t1;
                axis_block.time_stamp[2] = axis_block.time_stamp[1] + t2;
                axis_block.time_stamp[3] = axis_block.time_stamp[2] + t1;
                axis_block.time_stamp[4] = axis_block.time_stamp[3] + t4;
                axis_block.time_stamp[5] = axis_block.time_stamp[4] + t1;
                axis_block.time_stamp[6] = axis_block.time_stamp[5] + t2;
                axis_block.time_stamp[7] = axis_block.time_stamp[6] + t1;
                
                start_pos = start_position;
                start_vel = 0;
                start_acc = 0;
                end_pos = start_pos + jerk * t1 * t1 * t1 / 6;
                end_vel = jerk * t1 * t1 / 2;
                end_acc = alpha;
                getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[0].data);
                start_pos = end_pos;
                start_vel = end_vel;
                start_acc = end_acc;
                end_pos = start_pos + omega * t2 / 2;
                end_vel = jerk * t1 * t1 / 2 + jerk * t1 * t2;
                end_acc = alpha;
                getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t2, axis_block.coeff[1].data);
                start_pos = end_pos;
                start_vel = end_vel;
                start_acc = end_acc;
                end_pos = start_pos + omega * t1 - jerk * t1 * t1 * t1 / 6;
                end_vel = omega;
                end_acc = 0;
                getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[2].data);
                getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t4, axis_block.coeff[3].data);
                start_pos = end_pos;
                start_vel = end_vel;
                start_acc = end_acc;
                end_pos = start_pos + omega * t1 - jerk * t1 * t1 * t1 / 6;
                end_vel = jerk * t1 * t1 / 2 + jerk * t1 * t2;
                end_acc = -alpha;
                getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[4].data);
                start_pos = end_pos;
                start_vel = end_vel;
                start_acc = end_acc;
                end_pos = start_pos + omega * t2 / 2;
                end_vel = jerk * t1 * t1 / 2;
                end_acc = -alpha;
                getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t2, axis_block.coeff[5].data);
                start_pos = end_pos;
                start_vel = end_vel;
                start_acc = end_acc;
                end_pos = start_pos + jerk * t1 * t1 * t1 / 6;
                end_vel = 0;
                end_acc = 0;
                getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[6].data);
            }
            else
            {
                // 目标距离不足以使加速度达到A，无匀加速阶段，4段
                t1 = pow(total_trip / absolute_jerk / 2, 1.0 / 3.0);
                t2 = 0;
                t4 = 0;
                double jerk = expect_alpha < 0 ? -absolute_jerk : absolute_jerk;
                double alpha = t1 * jerk;
                double omega = jerk * t1 * t1;
                axis_block.time_stamp[0] = time_to_start;
                axis_block.time_stamp[1] = axis_block.time_stamp[0] + t1;
                axis_block.time_stamp[2] = axis_block.time_stamp[1] + t2;
                axis_block.time_stamp[3] = axis_block.time_stamp[2] + t1;
                axis_block.time_stamp[4] = axis_block.time_stamp[3] + t4;
                axis_block.time_stamp[5] = axis_block.time_stamp[4] + t1;
                axis_block.time_stamp[6] = axis_block.time_stamp[5] + t2;
                axis_block.time_stamp[7] = axis_block.time_stamp[6] + t1;

                start_pos = start_position;
                start_vel = 0;
                start_acc = 0;
                end_pos = start_pos + jerk * t1 * t1 * t1 / 6;
                end_vel = jerk * t1 * t1 / 2;
                end_acc = alpha;
                getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[0].data);
                getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t2, axis_block.coeff[1].data);
                start_pos = end_pos;
                start_vel = end_vel;
                start_acc = end_acc;
                end_pos = start_pos + jerk * t1 * t1 * t1 * 5 / 6;
                end_vel = omega;
                end_acc = 0;
                getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[2].data);
                getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t4, axis_block.coeff[3].data);
                start_pos = end_pos;
                start_vel = end_vel;
                start_acc = end_acc;
                end_pos = start_pos + jerk * t1 * t1 * t1 * 5 / 6;
                end_vel = jerk * t1 * t1 / 2;
                end_acc = -alpha;
                getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[4].data);
                getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t2, axis_block.coeff[5].data);
                start_pos = end_pos;
                start_vel = end_vel;
                start_acc = end_acc;
                end_pos = start_pos + jerk * t1 * t1 * t1 / 6;
                end_vel = 0;
                end_acc = 0;
                getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[6].data);
            }
        }
    }
}

void ManualTeach::getAxisCoefficientWithDuration(double time_to_start, double duration, double start_position, double end_postion, 
                                                 double expect_jerk, double expect_omega, double expect_alpha, ManualAxisBlock &axis_block)
{
    double t1, t2, t4;
    double start_pos, start_vel, start_acc, end_pos, end_vel, end_acc;

    // 假设使用4段式轨迹
    t1 = duration / 4;
    double jerk = (end_postion - start_position) / t1 / t1 / t1 / 2;
    double alpha = jerk * t1;
    double omega = jerk * t1 * t1;

    if (fabs(jerk) <= fabs(expect_jerk) && fabs(alpha) <= fabs(expect_alpha) && fabs(omega) <= fabs(expect_omega))
    {
        //FST_INFO("4 segment -> t1: %.6f, jerk: %.6f, alpha: %.6f, omega: %.6f, trip: %.6f", t1, jerk, alpha, omega, end_postion - start_position);
        t2 = 0;
        t4 = 0;
        axis_block.time_stamp[0] = time_to_start;
        axis_block.time_stamp[1] = axis_block.time_stamp[0] + t1;
        axis_block.time_stamp[2] = axis_block.time_stamp[1] + t2;
        axis_block.time_stamp[3] = axis_block.time_stamp[2] + t1;
        axis_block.time_stamp[4] = axis_block.time_stamp[3] + t4;
        axis_block.time_stamp[5] = axis_block.time_stamp[4] + t1;
        axis_block.time_stamp[6] = axis_block.time_stamp[5] + t2;
        axis_block.time_stamp[7] = axis_block.time_stamp[6] + t1;

        start_pos = start_position;
        start_vel = 0;
        start_acc = 0;
        end_pos = start_pos + jerk * t1 * t1 * t1 / 6;
        end_vel = jerk * t1 * t1 / 2;
        end_acc = alpha;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[0].data);
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t2, axis_block.coeff[1].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + jerk * t1 * t1 * t1 * 5 / 6;
        end_vel = omega;
        end_acc = 0;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[2].data);
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t4, axis_block.coeff[3].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + jerk * t1 * t1 * t1 * 5 / 6;
        end_vel = jerk * t1 * t1 / 2;
        end_acc = -alpha;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[4].data);
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t2, axis_block.coeff[5].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + jerk * t1 * t1 * t1 / 6;
        end_vel = 0;
        end_acc = 0;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[6].data);
        return;
    }

    if (fabs(omega) > fabs(expect_omega))
    {
        // 使用4段式轨迹将导致速度超出限制，使用５段式轨迹
        t4 = (end_postion - start_position) * 2 / expect_omega - duration;
        t2 = 0;
        t1 = (duration - t4) / 4;
        jerk = expect_omega / t1 / t1;
        alpha = jerk * t1;
        omega = expect_omega;
        //FST_INFO("4 segment -> t1: %.6f, t4: %.6f, jerk: %.6f, alpha: %.6f, omega: %.6f", t1, t4, jerk, alpha, omega);
        axis_block.time_stamp[0] = time_to_start;
        axis_block.time_stamp[1] = axis_block.time_stamp[0] + t1;
        axis_block.time_stamp[2] = axis_block.time_stamp[1] + t2;
        axis_block.time_stamp[3] = axis_block.time_stamp[2] + t1;
        axis_block.time_stamp[4] = axis_block.time_stamp[3] + t4;
        axis_block.time_stamp[5] = axis_block.time_stamp[4] + t1;
        axis_block.time_stamp[6] = axis_block.time_stamp[5] + t2;
        axis_block.time_stamp[7] = axis_block.time_stamp[6] + t1;
        
        start_pos = start_position;
        start_vel = 0;
        start_acc = 0;
        end_pos = start_pos + jerk * t1 * t1 * t1 / 6;
        end_vel = omega / 2;
        end_acc = alpha;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[0].data);
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t2, axis_block.coeff[1].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + jerk * t1 * t1 * t1 * 5 / 6;
        end_vel = omega;
        end_acc = 0;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[2].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + omega * t4;
        end_vel = omega;
        end_acc = 0;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t4, axis_block.coeff[3].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + jerk * t1 * t1 * t1 * 5 / 6;
        end_vel = omega / 2;
        end_acc = -alpha;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[4].data);
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t2, axis_block.coeff[5].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + jerk * t1 * t1 * t1 / 6;
        end_vel = 0;
        end_acc = 0;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[6].data);
        return;
    }

    else
    {
        FST_ERROR("t1: %.6f, jerk: %.6f, alpha: %.6f, omega: %.6f, trip: %.6f", t1, jerk, alpha, omega, end_postion - start_position);
        FST_ERROR("duration: %.6f, jerk: %.6f, alpha: %.6f, omega: %.6f", duration, expect_jerk, expect_alpha, expect_omega);
    }
}

bool ManualTeach::getStopCoefficient(double time_ready_to_stop, double jerk, ManualAxisBlock &axis_block)
{
    if (time_ready_to_stop < axis_block.time_stamp[0])
    {
        axis_block.time_stamp[0] = 0;
        axis_block.time_stamp[1] = 0;
        axis_block.time_stamp[2] = 0;
        axis_block.time_stamp[3] = 0;
        axis_block.time_stamp[4] = 0;
        axis_block.time_stamp[5] = 0;
        axis_block.time_stamp[6] = 0;
        axis_block.time_stamp[7] = 0;
        double start_pos, start_vel, start_acc;
        sampleQuinticPolynomial(0, axis_block.coeff[0].data, start_pos, start_vel, start_acc);
        getQuinticSplineCoefficients(start_pos, 0, 0, start_pos, 0, 0, 0, axis_block.coeff[6].data);
        return true;
    }
    else if (time_ready_to_stop < axis_block.time_stamp[1])
    {
        // 加加速阶段，共计４段停止
        double start_pos, start_vel, start_acc, end_pos, end_vel, end_acc;
        double t1 = time_ready_to_stop - axis_block.time_stamp[0];
        axis_block.time_stamp[1] = axis_block.time_stamp[0] + t1;
        axis_block.time_stamp[2] = axis_block.time_stamp[1];
        axis_block.time_stamp[3] = axis_block.time_stamp[2] + t1;
        axis_block.time_stamp[4] = axis_block.time_stamp[3];
        axis_block.time_stamp[5] = axis_block.time_stamp[4] + t1;
        axis_block.time_stamp[6] = axis_block.time_stamp[5];
        axis_block.time_stamp[7] = axis_block.time_stamp[6] + t1;
        sampleQuinticPolynomial(0, axis_block.coeff[0].data, start_pos, start_vel, start_acc);
        sampleQuinticPolynomial(t1, axis_block.coeff[0].data, end_pos, end_vel, end_acc);

        if (start_pos > end_pos)
        {
            // 负向运动
            jerk = -jerk;
        }

        double alpha = jerk * t1;
        double omega = jerk * t1 * t1;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, 0, axis_block.coeff[1].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = end_pos + omega * t1 - jerk * t1 * t1 * t1 / 6;
        end_vel = omega;
        end_acc = 0;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[2].data);
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, 0, axis_block.coeff[3].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = end_pos + omega * t1 - jerk * t1 * t1 * t1 / 6;
        end_vel = omega / 2;
        end_acc = -alpha;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[4].data);
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, 0, axis_block.coeff[5].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = end_pos + jerk * t1 * t1 * t1 / 6;
        end_vel = 0;
        end_acc = 0;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[6].data);
        return true;
    }
    else if (time_ready_to_stop < axis_block.time_stamp[2])
    {
        // 匀加速阶段，共计６段停止
        double start_pos, start_vel, start_acc, end_pos, end_vel, end_acc;
        double t1 = axis_block.time_stamp[1] - axis_block.time_stamp[0];
        double t2 = time_ready_to_stop - axis_block.time_stamp[1];
        axis_block.time_stamp[2] = axis_block.time_stamp[1] + t2;
        axis_block.time_stamp[3] = axis_block.time_stamp[2] + t1;
        axis_block.time_stamp[4] = axis_block.time_stamp[3];
        axis_block.time_stamp[5] = axis_block.time_stamp[4] + t1;
        axis_block.time_stamp[6] = axis_block.time_stamp[5] + t2;
        axis_block.time_stamp[7] = axis_block.time_stamp[6] + t1;
        sampleQuinticPolynomial(0, axis_block.coeff[0].data, start_pos, start_vel, start_acc);
        sampleQuinticPolynomial(t1, axis_block.coeff[0].data, end_pos, end_vel, end_acc);
        
        if (start_pos > end_pos)
        {
            // 负向运动
            jerk = -jerk;
        }

        double alpha = jerk * t1;
        double omega = jerk * t1 * t1 + jerk * t1 * t2;
        sampleQuinticPolynomial(t2, axis_block.coeff[1].data, start_pos, start_vel, start_acc);
        end_pos = start_pos + omega * t1 - jerk * t1 * t1 * t1 / 6;
        end_vel = omega;
        end_acc = 0;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[2].data);
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, 0, axis_block.coeff[3].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + omega * t1 - jerk * t1 * t1 * t1 / 6;
        end_vel = jerk * t1 * t1 / 2 + jerk * t1 * t2;
        end_acc = -alpha;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[4].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + omega * t2 / 2;
        end_vel = jerk * t1 * t1 / 2;
        end_acc = -alpha;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t2, axis_block.coeff[5].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + jerk * t1 * t1 * t1 / 6;
        end_vel = 0;
        end_acc = 0;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[6].data);
        return true;
    }
    else if (time_ready_to_stop < axis_block.time_stamp[3])
    {
        // 减加速阶段，共计６段停止
        double start_pos, start_vel, start_acc, end_pos, end_vel, end_acc;
        double t1 = axis_block.time_stamp[1] - axis_block.time_stamp[0];
        double t2 = axis_block.time_stamp[2] - axis_block.time_stamp[1];
        axis_block.time_stamp[4] = axis_block.time_stamp[3];
        axis_block.time_stamp[5] = axis_block.time_stamp[4] + t1;
        axis_block.time_stamp[6] = axis_block.time_stamp[5] + t2;
        axis_block.time_stamp[7] = axis_block.time_stamp[6] + t1;
        sampleQuinticPolynomial(0, axis_block.coeff[2].data, start_pos, start_vel, start_acc);
        sampleQuinticPolynomial(t1, axis_block.coeff[2].data, end_pos, end_vel, end_acc);
        
        if (start_pos > end_pos)
        {
            // 负向运动
            jerk = -jerk;
        }

        double alpha = jerk * t1;
        double omega = jerk * t1 * t1 + jerk * t1 * t2;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, 0, axis_block.coeff[3].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + omega * t1 - jerk * t1 * t1 * t1 / 6;
        end_vel = jerk * t1 * t1 / 2 + jerk * t1 * t2;
        end_acc = -alpha;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[4].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + omega * t2 / 2;
        end_vel = jerk * t1 * t1 / 2;
        end_acc = -alpha;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t2, axis_block.coeff[5].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + jerk * t1 * t1 * t1 / 6;
        end_vel = 0;
        end_acc = 0;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[6].data);
        return true;
    }
    else if (time_ready_to_stop < axis_block.time_stamp[4])
    {
        // 匀速阶段，共计7段停止
        double start_pos, start_vel, start_acc, end_pos, end_vel, end_acc;
        double t1 = axis_block.time_stamp[1] - axis_block.time_stamp[0];
        double t2 = axis_block.time_stamp[2] - axis_block.time_stamp[1];
        double t4 = time_ready_to_stop - axis_block.time_stamp[3];
        axis_block.time_stamp[4] = axis_block.time_stamp[3] + t4;
        axis_block.time_stamp[5] = axis_block.time_stamp[4] + t1;
        axis_block.time_stamp[6] = axis_block.time_stamp[5] + t2;
        axis_block.time_stamp[7] = axis_block.time_stamp[6] + t1;
        sampleQuinticPolynomial(0, axis_block.coeff[3].data, start_pos, start_vel, start_acc);
        sampleQuinticPolynomial(t4, axis_block.coeff[3].data, end_pos, end_vel, end_acc);

        if (start_pos > end_pos)
        {
            // 负向运动
            jerk = -jerk;
        }

        double alpha = jerk * t1;
        double omega = jerk * t1 * t1 + jerk * t1 * t2;
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + omega * t1 - jerk * t1 * t1 * t1 / 6;
        end_vel = jerk * t1 * t1 / 2 + jerk * t1 * t2;
        end_acc = -alpha;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[4].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + omega * t2 / 2;
        end_vel = jerk * t1 * t1 / 2;
        end_acc = -alpha;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t2, axis_block.coeff[5].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + jerk * t1 * t1 * t1 / 6;
        end_vel = 0;
        end_acc = 0;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[6].data);
        return true;
    }
    else
    {
        // 已经进入减速阶段，不再重新规划
        return false;
    }
}

void ManualTeach::getStopCoefficientInPointMode(double time_ready_to_stop, double jerk, ManualAxisBlock &axis_block)
{
    if (time_ready_to_stop < axis_block.time_stamp[0])
    {
        axis_block.time_stamp[0] = 0;
        axis_block.time_stamp[1] = 0;
        axis_block.time_stamp[2] = 0;
        axis_block.time_stamp[3] = 0;
        axis_block.time_stamp[4] = 0;
        axis_block.time_stamp[5] = 0;
        axis_block.time_stamp[6] = 0;
        axis_block.time_stamp[7] = 0;
        double start_pos, start_vel, start_acc;
        sampleQuinticPolynomial(0, axis_block.coeff[0].data, start_pos, start_vel, start_acc);
        getQuinticSplineCoefficients(start_pos, 0, 0, start_pos, 0, 0, 0, axis_block.coeff[6].data);
        return;
    }
    
    if (time_ready_to_stop < axis_block.time_stamp[1])
    {
        // 加加速阶段，共计４段停止
        double start_pos, start_vel, start_acc, end_pos, end_vel, end_acc;
        double t1 = time_ready_to_stop - axis_block.time_stamp[0];
        sampleQuinticPolynomial(0, axis_block.coeff[0].data, start_pos, start_vel, start_acc);
        sampleQuinticPolynomial(t1, axis_block.coeff[0].data, end_pos, end_vel, end_acc);
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, 0, axis_block.coeff[1].data);
        double t3 = fabs(end_acc) / jerk;

        if (start_pos > end_pos)
        {
            // 负向运动
            jerk = -jerk;
        }

        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + start_vel * t3 + start_acc * t3 * t3 / 2 - jerk * t3 * t3 * t3 / 6;
        end_vel = start_vel + start_acc * t3 - jerk * t3 * t3 / 2;
        end_acc = 0;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t3, axis_block.coeff[2].data);
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, 0, axis_block.coeff[3].data);
        double t5 = sqrt(fabs(end_vel / jerk));
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + jerk * t5 * t5 * t5 * 5 / 6;
        end_vel = jerk * t5 * t5 / 2;
        end_acc = -jerk * t5;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t5, axis_block.coeff[4].data);
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, 0, axis_block.coeff[5].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + jerk * t5 * t5 * t5 / 6;
        end_vel = 0;
        end_acc = 0;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[6].data);

        axis_block.time_stamp[1] = axis_block.time_stamp[0] + t1;
        axis_block.time_stamp[2] = axis_block.time_stamp[1];
        axis_block.time_stamp[3] = axis_block.time_stamp[2] + t3;
        axis_block.time_stamp[4] = axis_block.time_stamp[3];
        axis_block.time_stamp[5] = axis_block.time_stamp[4] + t5;
        axis_block.time_stamp[6] = axis_block.time_stamp[5];
        axis_block.time_stamp[7] = axis_block.time_stamp[6] + t5;
        return;
    }

    if (time_ready_to_stop < axis_block.time_stamp[3])
    {
        // 减加速阶段，共计4段停止
        double start_pos, start_vel, start_acc, end_pos, end_vel, end_acc;
        double t1 = axis_block.time_stamp[1] - axis_block.time_stamp[0];
        double t3 = time_ready_to_stop - axis_block.time_stamp[2];
        sampleQuinticPolynomial(0, axis_block.coeff[2].data, start_pos, start_vel, start_acc);
        sampleQuinticPolynomial(t3, axis_block.coeff[2].data, end_pos, end_vel, end_acc);
        t3 = fabs(end_acc) / jerk;

        if (start_pos > end_pos)
        {
            // 负向运动
            jerk = -jerk;
        }

        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + start_vel * t3 + start_acc * t3 * t3 / 2 - jerk * t3 * t3 * t3 / 6;
        end_vel = start_vel + jerk * t3 * t3 / 2;
        end_acc = 0;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t3, axis_block.coeff[2].data);
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, 0, axis_block.coeff[3].data);
        double t5 = sqrt(fabs(end_vel / jerk));
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + jerk * t5 * t5 * t5 * 5 / 6;
        end_vel = jerk * t5 * t5 / 2;
        end_acc = -jerk * t5;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t5, axis_block.coeff[4].data);
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, 0, axis_block.coeff[5].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + jerk * t5 * t5 * t5 / 6;
        end_vel = 0;
        end_acc = 0;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t5, axis_block.coeff[6].data);

        axis_block.time_stamp[2] = time_ready_to_stop;
        axis_block.time_stamp[3] = axis_block.time_stamp[2] + t3;
        axis_block.time_stamp[4] = axis_block.time_stamp[3];
        axis_block.time_stamp[5] = axis_block.time_stamp[4] + t5;
        axis_block.time_stamp[6] = axis_block.time_stamp[5];
        axis_block.time_stamp[7] = axis_block.time_stamp[6] + t5;
        return;
    }

    if (time_ready_to_stop < axis_block.time_stamp[4])
    {
        // 匀速阶段，共计5段停止
        double start_pos, start_vel, start_acc, end_pos, end_vel, end_acc;
        double t1 = axis_block.time_stamp[1] - axis_block.time_stamp[0];
        double t4 = time_ready_to_stop - axis_block.time_stamp[3];
        sampleQuinticPolynomial(0, axis_block.coeff[3].data, start_pos, start_vel, start_acc);
        sampleQuinticPolynomial(t4, axis_block.coeff[3].data, end_pos, end_vel, end_acc);
        double t5 = sqrt(fabs(end_vel / jerk));

        if (start_pos > end_pos)
        {
            // 负向运动
            jerk = -jerk;
        }

        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + jerk * t5 * t5 * t5 * 5 / 6;
        end_vel = jerk * t5 * t5 / 2;
        end_acc = -jerk * t5;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[4].data);
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, 0, axis_block.coeff[5].data);
        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + jerk * t1 * t1 * t1 / 6;
        end_vel = 0;
        end_acc = 0;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t1, axis_block.coeff[6].data);

        axis_block.time_stamp[4] = axis_block.time_stamp[3] + t4;
        axis_block.time_stamp[5] = axis_block.time_stamp[4] + t5;
        axis_block.time_stamp[6] = axis_block.time_stamp[5];
        axis_block.time_stamp[7] = axis_block.time_stamp[6] + t5;
        return;
    }

    if (time_ready_to_stop < axis_block.time_stamp[5])
    {
        // 加减速阶段，共计4段或5段停止
        double start_pos, start_vel, start_acc, end_pos, end_vel, end_acc;
        double t5 = axis_block.time_stamp[5] - axis_block.time_stamp[4];
        sampleQuinticPolynomial(0, axis_block.coeff[4].data, start_pos, start_vel, start_acc);
        sampleQuinticPolynomial(t5, axis_block.coeff[4].data, end_pos, end_vel, end_acc);
        double t7 = sqrt(fabs(end_vel * 2 / jerk));

        if (start_pos > end_pos)
        {
            // 负向运动
            jerk = -jerk;
        }

        start_pos = end_pos;
        start_vel = end_vel;
        start_acc = end_acc;
        end_pos = start_pos + jerk * t7 * t7 * t7 / 6;
        end_vel = 0;
        end_acc = 0;
        getQuinticSplineCoefficients(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, t7, axis_block.coeff[6].data);
        axis_block.time_stamp[7] = axis_block.time_stamp[6] + t7;
        return;
    }
}

void ManualTeach::sampleQuinticPolynomial(double sample_time, const double *coefficients, double &pos, double &vel, double &acc)
{
    double t1 = sample_time;
    double t2 = t1 * sample_time;
    double t3 = t2 * sample_time;
    double t4 = t3 * sample_time;
    double t5 = t4 * sample_time;
    pos = coefficients[5] * t5 + coefficients[4] * t4 + coefficients[3] * t3 + coefficients[2] * t2 + coefficients[1] * t1 + coefficients[0];
    vel = coefficients[5] * t4 * 5 + coefficients[4] * t3 * 4 + coefficients[3] * t2 * 3 + coefficients[2] * t1 * 2 + coefficients[1];
    acc = coefficients[5] * t3 * 20 + coefficients[4] * t2 * 12 + coefficients[3] * t1 * 6 + coefficients[2] * 2;
}

ErrorCode ManualTeach::manualJointContinuous(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj)
{
    double jerk, alpha, omega;
    char buffer[LOG_TEXT_SIZE];

    FST_INFO("manual-Joint-Continuous: directions = %s, planning trajectory ...", printDBLine((int*)dir, buffer, LOG_TEXT_SIZE));
    FST_INFO("  manual-time = %.4f, vel-ratio = %.0f%%, acc-ratio = %.0f%%", time, vel_ratio_ * 100, acc_ratio_ * 100);
    FST_INFO("  start-joint = %s", printDBLine(&traj.joint_start[0], buffer, LOG_TEXT_SIZE));

    if (traj.ending_time < MINIMUM_E6)
    {
        // begin to manual-continuous, update ending joint
        traj.joint_ending = traj.joint_start;
    }

    for (size_t i = 0; i < joint_num_; i++)
    {
        jerk = axis_jerk_[i];
        alpha = axis_acc_[i] * acc_ratio_;
        omega = axis_vel_[i] * vel_ratio_;

        if (traj.direction[i] == dir[i])
        {
            // keep STANDING or motion
            // do nothing
            // FST_INFO("  J%d: given direction same as current motion, running along the original trajectory", i + 1);
            continue;
        }
        else if (dir[i] == STANDING && traj.direction[i] != STANDING)
        {
            // stop joint motion
            if (getStopCoefficient(time, jerk, traj.axis[i]))
            {
                double pos, vel, acc;
                sampleQuinticPolynomial(traj.axis[i].time_stamp[7] - traj.axis[i].time_stamp[6], traj.axis[i].coeff[6].data, pos, vel, acc);
                traj.joint_ending[i] = pos;
                traj.direction[i] = STANDING;
            }
        }
        else if (dir[i] != STANDING && traj.direction[i] == STANDING)
        {
            // startup motion in this axis
            if (time >= traj.axis[i].time_stamp[7])
            {
                // start joint motion
                // stop until soft constraint
                double trip = 99.99;

                if (!joint_constraint_ptr_->isJointMasked(i))
                {
                    trip = (dir[i] == INCREASE) ? (joint_constraint_ptr_->upper()[i] - traj.joint_start[i]) :
                           (traj.joint_start[i] - joint_constraint_ptr_->lower()[i]);
                }

                if (trip < MINIMUM_E6)
                {
                    FST_WARN("  J%d: near soft constraint, cannot manual move", i + 1);
                    continue;
                }
                
                memset(traj.axis[i].coeff, 0, sizeof(traj.axis[i].coeff));
                if (dir[i] == INCREASE)
                {
                    getAxisCoefficient(time, traj.joint_start[i], trip, jerk, omega, alpha, traj.axis[i]);
                    //getIncreaseCoefficient(time, traj.joint_start[i], trip, omega, alpha, jerk, traj.axis[i]);
                }
                else
                {
                    getAxisCoefficient(time, traj.joint_start[i], trip, jerk, -omega, -alpha, traj.axis[i]);
                    //getDecreaseCoefficient(time, traj.joint_start[i], trip, omega, alpha, jerk, traj.axis[i]);
                }
                
                
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

            if (getStopCoefficient(time, jerk, traj.axis[i]))
            {
                double pos, vel, acc;
                sampleQuinticPolynomial(traj.axis[i].time_stamp[7] - traj.axis[i].time_stamp[6], traj.axis[i].coeff[6].data, pos, vel, acc);
                traj.joint_ending[i] = pos;
            }
            else
            {
                // already in axis-slow-down trajectory, do nothing
                FST_WARN("  J%d: STANDING or in slow-down trajectory, will stop on soft constraint", i + 1);
                continue;
            }
        }
    }

    double duration = 0;

    for (size_t i = 0; i < joint_num_; i++)
    {
        if (traj.axis[i].time_stamp[7] > duration)
        {
            duration = traj.axis[i].time_stamp[7];
        }
    }

    traj.ending_time = duration;
    FST_INFO("Create manual trajectory success, end-time: %.4f", duration);
    return SUCCESS;
}

ErrorCode ManualTeach::manualCartesianContinuous(const ManualDirection *dir, MotionTime time, ManualTrajectory &traj)
{
    char buffer[LOG_TEXT_SIZE];
    PoseEuler &start = traj.cart_start;
    PoseEuler &target = traj.cart_ending;
    FST_INFO("manual-Cartesian-Continuous: directions = %s, planning trajectory ...", printDBLine((int*)dir, buffer, LOG_TEXT_SIZE));
    FST_INFO("  manual-time = %.4f, vel-ratio = %.0f%%, acc-ratio = %.0f%%", time, vel_ratio_ * 100, acc_ratio_ * 100);
    FST_INFO("  start-joint = %s", printDBLine(&traj.joint_start[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  start-pose  = %.2f %.2f %.2f - %.4f %.4f %.4f", start.point_.x_, start.point_.y_, start.point_.z_, start.euler_.a_, start.euler_.b_, start.euler_.c_);

    if (traj.ending_time < MINIMUM_E6)
    {
        // begin to manual-continuous, update ending joint
        traj.joint_ending = traj.joint_start;
    }

    double spd, acc, jerk;
    
    for (size_t i = 0; i < 6; i++)
    {
        spd = (i < 3 ? position_vel_reference_ : orientation_omega_reference_) * vel_ratio_;
        acc = (i < 3 ? position_acc_reference_ : orientation_alpha_reference_) * acc_ratio_;
        jerk = i < 3 ? position_jerk_reference_ : orientation_beta_reference_;

        if (traj.direction[i] == dir[i])
        {
            // keep standby or motion direction
            // do nothing
            //if (traj.direction[i] == STANDING)
            //{
            //    *(&traj.cart_ending.point_.x_ + i) = *(&traj.cart_start.point_.x_ + i);
            //}

            //FST_INFO("  PoseEuler[%d]: given direction same as current motion, running along the current trajectory", i);
            continue;
        }
        else if (dir[i] == STANDING && traj.direction[i] != STANDING)
        {
            // stop motion in this direction
            if (getStopCoefficient(time, jerk, traj.axis[i]))
            {
                double p, v, a;
                sampleQuinticPolynomial(traj.axis[i].time_stamp[7] - traj.axis[i].time_stamp[6], traj.axis[i].coeff[6].data, p, v, a);
                traj.joint_ending[i] = p;
                *(&target.point_.x_ + i) = p;
                traj.direction[i] = STANDING;
            }
        }
        else if (dir[i] != STANDING && traj.direction[i] == STANDING)
        {
            // startup motion in this axis
            if (time >= traj.axis[i].time_stamp[7])
            {
                // start motion
                double trip = 2000;
                memset(traj.axis[i].coeff, 0, sizeof(traj.axis[i].coeff));

                if (dir[i] == INCREASE)
                {
                    getAxisCoefficient(time, *(&start.point_.x_ + i), trip, jerk, spd, acc, traj.axis[i]);
                }
                else
                {
                    getAxisCoefficient(time, *(&start.point_.x_ + i), trip, jerk, -spd, -acc, traj.axis[i]);
                }
                
                traj.direction[i] = dir[i];
                *(&target.point_.x_ + i) = *(&start.point_.x_ + i) + (dir[i] == INCREASE ? trip : -trip);
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
            // stop motion in this direction
            traj.direction[i] = STANDING;

            if (getStopCoefficient(time, jerk, traj.axis[i]))
            {
                double p, v, a;
                sampleQuinticPolynomial(traj.axis[i].time_stamp[7] - traj.axis[i].time_stamp[6], traj.axis[i].coeff[6].data, p, v, a);
                *(&target.point_.x_ + i) = p;
            }
            else
            {
                // already in axis-slow-down trajectory, do nothing
                FST_WARN("  PoseEuler[%d]: Standby or in slow-down trajectory, will stop soon", i);
                continue;
            }
        }
    }

    double duration = 0;

    for (size_t i = 0; i < 6; i++)
    {
        if (traj.axis[i].time_stamp[7] > duration)
        {
            duration = traj.axis[i].time_stamp[7];
        }
    }

    traj.ending_time = duration;
    FST_INFO("Create manual trajectory success, end-time: %.4f", duration);
    return SUCCESS;
}

ErrorCode ManualTeach::manualByTarget(const Joint &target, MotionTime time, ManualTrajectory &traj)
{
    FST_INFO("Manual request received, frame=%d", traj.frame);
    return manualJointAPoint(target, time, traj);
}

ErrorCode ManualTeach::manualJointAPoint(const Joint &target, MotionTime time, ManualTrajectory &traj)
{
    char buffer[LOG_TEXT_SIZE];

    FST_INFO("manual-Joint-APOINT: planning trajectory ...");
    FST_INFO("  axis-vel = %s", printDBLine(&move_to_point_vel_[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  start joint = %s", printDBLine(&traj.joint_start[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  target joint = %s", printDBLine(&target[0], buffer, LOG_TEXT_SIZE));

    double trips[NUM_OF_JOINT], alpha[NUM_OF_JOINT], omega[NUM_OF_JOINT], beta[NUM_OF_JOINT];
    double max_duration = 0;
    size_t index_of_max_duration = 0;

    // 单轴预规划
    for (size_t i = 0; i < joint_num_; i++)
    {
        trips[i] = target[i] - traj.joint_start[i];
        alpha[i] = trips[i] < 0 ? -axis_acc_[i] : axis_acc_[i];
        omega[i] = trips[i] < 0 ? -move_to_point_vel_[i] : move_to_point_vel_[i];
        beta[i] = trips[i] < 0 ? -axis_jerk_[i] : axis_jerk_[i];
        getAxisCoefficient(0, traj.joint_start[i], fabs(trips[i]), axis_jerk_[i], omega[i], alpha[i], traj.axis[i]);

        if (traj.axis[i].time_stamp[7] > max_duration)
        {
            max_duration = traj.axis[i].time_stamp[7];
            index_of_max_duration = i;
        }
    }

    if (max_duration < 0.000001)
    {
        FST_INFO("Target joint neer start joint, do nothing!");
        traj.ending_time = 0;
        return SUCCESS;
    }

    FST_INFO("Max-duration: %.4f, index: %d, create trajectory ...", max_duration, index_of_max_duration);

    for (size_t i = 0; i < joint_num_; i++)
    {
        traj.joint_ending[i] = target[i];
        // 预规划时间最长的轴不需要重新规划，其他轴重新规划
        if (i == index_of_max_duration) continue;

        getAxisCoefficientWithDuration(0, max_duration, traj.joint_start[i], target[i], beta[i], omega[i], alpha[i], traj.axis[i]);
    }

    traj.ending_time = max_duration;
    FST_INFO("Create manual trajectory success !");
    return SUCCESS;
}

ErrorCode ManualTeach::manualStop(MotionTime time, ManualTrajectory &traj)
{
    FST_INFO("stopTeach: curr-time=%.4f, total-time=%.4f", time, traj.ending_time);
    
    if (traj.mode == CONTINUOUS)
    {
        FST_WARN("manual-mode is continuous, stop motion by change direction input");
        return SUCCESS;
    }

    if (traj.mode == STEP)
    {
        FST_WARN("manual-mode is STEP, stop motion by trajectory");
        return SUCCESS;
    }
    
    for (size_t i = 0; i < joint_num_; i++)
    {
        getStopCoefficientInPointMode(time, axis_jerk_[i], traj.axis[i]);
        double pos, vel, acc;
        sampleQuinticPolynomial(traj.axis[i].time_stamp[7] - traj.axis[i].time_stamp[6], traj.axis[i].coeff[6].data, pos, vel, acc);
        traj.joint_ending[i] = pos;
    }

    double max_duration = 0;

    for (size_t i = 0; i < joint_num_; i++)
    {
        if (traj.axis[i].time_stamp[7] > max_duration)
        {
            max_duration = traj.axis[i].time_stamp[7];
        }
    }

    traj.ending_time = max_duration;
    FST_INFO("Success, total duration=%.4f", traj.ending_time);
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


